#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Low-level async chassis client.

Wraps the raw WebSocket + protobuf protocol behind a clean class so higher-level
code (controllers, navigators, planners) can drive the robot programmatically:

    async with ChassisClient("ws://192.168.1.230:8439") as bot:
        await bot.initialize()
        await bot.set_velocity(0.1, 0.0, 0.0)
        ...
        await bot.brake()

Notes on the protocol (see proto-public-api/public_api_types.proto):
- There is NO server-side navigation command. The only base motion primitives
  are XyzSpeed (vx, vy, omega), brake, and zero_resistance. Higher-level
  navigation must be built on top using odometry feedback.
- The robot expects a continuous stream of SimpleBaseMoveCommand messages
  while api_control is initialized; this class keeps a heartbeat send loop
  running so callers only set the *target* velocity and forget.
"""

import sys
import os
import asyncio
import time
from dataclasses import dataclass
from typing import Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import websockets
from hex_device.generated import (
    public_api_down_pb2,
    public_api_up_pb2,
    public_api_types_pb2,
)
from hex_device.generated.version import (
    CURRENT_PROTOCOL_MAJOR_VERSION,
    CURRENT_PROTOCOL_MINOR_VERSION,
)


@dataclass
class Odometry:
    pos_x: float   # m
    pos_y: float   # m
    yaw: float     # rad  (proto field pos_z)
    vx: float      # m/s
    vy: float      # m/s
    omega: float   # rad/s
    stamp: float   # monotonic seconds when received


class ChassisClientError(RuntimeError):
    pass


class NotConnectedError(ChassisClientError):
    pass


class ChassisClient:
    """Async, thread-unsafe (single event loop) low-level chassis client."""

    def __init__(
        self,
        url: str,
        send_hz: float = 50.0,
        report_frequency=public_api_types_pb2.Rf100Hz,
        recv_timeout: float = 3.0,
    ):
        self.url = url
        self.send_hz = send_hz
        self.report_frequency = report_frequency
        self.recv_timeout = recv_timeout

        self._ws = None
        self._recv_task: Optional[asyncio.Task] = None
        self._send_task: Optional[asyncio.Task] = None
        self._running = False
        self._initialized = False

        # Target command (read by send loop)
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_omega = 0.0
        self._brake_latched = False  # if True, send brake instead of xyz

        # Latest state (written by recv loop)
        self._latest_base_status = None
        self._latest_odometry: Optional[Odometry] = None
        self._session_id: Optional[int] = None
        self._robot_type: Optional[int] = None
        self._protocol_version: Tuple[int, int] = (0, 0)
        self._first_status = asyncio.Event()

    # ── Properties ──────────────────────────────────────────────────────────

    @property
    def is_connected(self) -> bool:
        return self._ws is not None and self._running

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def session_id(self) -> Optional[int]:
        return self._session_id

    @property
    def robot_type(self) -> Optional[int]:
        return self._robot_type

    @property
    def protocol_version(self) -> Tuple[int, int]:
        return self._protocol_version

    # ── Message builders (private) ──────────────────────────────────────────

    def _stamp(self, msg):
        msg.protocol_major_version = CURRENT_PROTOCOL_MAJOR_VERSION
        msg.protocol_minor_version = CURRENT_PROTOCOL_MINOR_VERSION
        return msg

    def _build_speed(self, vx: float, vy: float, omega: float) -> bytes:
        msg = public_api_down_pb2.APIDown()
        xyz = public_api_types_pb2.XyzSpeed()
        xyz.speed_x = vx
        xyz.speed_y = vy
        xyz.speed_z = omega
        msg.base_command.simple_move_command.xyz_speed.CopyFrom(xyz)
        return self._stamp(msg).SerializeToString()

    def _build_brake(self) -> bytes:
        msg = public_api_down_pb2.APIDown()
        msg.base_command.simple_move_command.brake = True
        return self._stamp(msg).SerializeToString()

    def _build_init(self, initialize: bool) -> bytes:
        msg = public_api_down_pb2.APIDown()
        msg.base_command.api_control_initialize = initialize
        return self._stamp(msg).SerializeToString()

    def _build_clear_parking_stop(self) -> bytes:
        msg = public_api_down_pb2.APIDown()
        msg.base_command.clear_parking_stop = True
        return self._stamp(msg).SerializeToString()

    def _build_report_frequency(self, freq) -> bytes:
        msg = public_api_down_pb2.APIDown()
        msg.set_report_frequency = freq
        return self._stamp(msg).SerializeToString()

    # ── Connection lifecycle ────────────────────────────────────────────────

    async def connect(self) -> None:
        """Open WebSocket, start background loops, set report frequency."""
        if self._ws is not None:
            return
        self._ws = await websockets.connect(
            self.url, ping_interval=20, ping_timeout=60, close_timeout=5
        )
        self._running = True
        self._recv_task = asyncio.create_task(self._recv_loop())
        self._send_task = asyncio.create_task(self._send_loop())

        # Wait for first APIUp so robot_type / protocol / session are populated
        try:
            await asyncio.wait_for(self._first_status.wait(), timeout=5.0)
        except asyncio.TimeoutError:
            await self.close()
            raise ChassisClientError("No initial APIUp from robot within 5 s")

        await self._ws.send(self._build_report_frequency(self.report_frequency))

    async def initialize(self, warmup_seconds: float = 0.5) -> None:
        """Take API control. Sends zero-velocity heartbeat first (protocol requirement)."""
        if not self.is_connected:
            raise NotConnectedError("call connect() first")
        # Heartbeat is already running (zeros). Just wait the warmup window.
        await asyncio.sleep(warmup_seconds)
        await self._ws.send(self._build_init(True))
        self._initialized = True

    async def deinitialize(self) -> None:
        """Release API control."""
        if not self.is_connected:
            return
        try:
            await self._ws.send(self._build_speed(0.0, 0.0, 0.0))
            await self._ws.send(self._build_init(False))
        except Exception:
            pass
        self._initialized = False

    async def close(self) -> None:
        """Stop loops, deinit, close socket."""
        if self._initialized:
            await self.deinitialize()
        self._running = False
        for task in (self._recv_task, self._send_task):
            if task is not None:
                task.cancel()
                try:
                    await task
                except (asyncio.CancelledError, Exception):
                    pass
        if self._ws is not None:
            try:
                await self._ws.close()
            except Exception:
                pass
            self._ws = None

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        await self.close()

    # ── Command methods (public API for higher-level controllers) ──────────

    def set_velocity(self, vx: float, vy: float, omega: float) -> None:
        """Set the *target* body-frame velocity. Heartbeat loop sends it at send_hz.

        Non-async on purpose: it just updates shared state. Call as often as
        you like from your control loop.
        """
        self._target_vx = float(vx)
        self._target_vy = float(vy)
        self._target_omega = float(omega)
        self._brake_latched = False

    def stop(self) -> None:
        """Set target velocity to zero (does NOT engage brake)."""
        self.set_velocity(0.0, 0.0, 0.0)

    def brake(self) -> None:
        """Latch brake mode — heartbeat will send SimpleBaseMoveCommand.brake."""
        self._brake_latched = True

    def release_brake(self) -> None:
        """Leave brake mode; resume sending xyz_speed (currently zero)."""
        self._brake_latched = False
        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_omega = 0.0

    async def clear_parking_stop(self) -> None:
        if not self.is_connected:
            raise NotConnectedError()
        await self._ws.send(self._build_clear_parking_stop())

    async def set_report_frequency(self, freq) -> None:
        if not self.is_connected:
            raise NotConnectedError()
        await self._ws.send(self._build_report_frequency(freq))

    # ── State accessors ────────────────────────────────────────────────────

    def get_odometry(self) -> Optional[Odometry]:
        return self._latest_odometry

    def get_base_status(self):
        """Return the raw protobuf BaseStatus (or None if not received yet)."""
        return self._latest_base_status

    async def wait_for_odometry(self, timeout: float = 2.0) -> Odometry:
        """Block until at least one odometry sample is available."""
        deadline = time.monotonic() + timeout
        while self._latest_odometry is None:
            if time.monotonic() > deadline:
                raise ChassisClientError("Timed out waiting for odometry")
            await asyncio.sleep(0.02)
        return self._latest_odometry

    # ── Background loops ───────────────────────────────────────────────────

    async def _recv_loop(self) -> None:
        while self._running:
            try:
                raw = await asyncio.wait_for(self._ws.recv(), timeout=self.recv_timeout)
            except asyncio.TimeoutError:
                continue
            except websockets.exceptions.ConnectionClosed:
                self._running = False
                return

            if not isinstance(raw, bytes):
                continue

            api_up = public_api_up_pb2.APIUp()
            try:
                api_up.ParseFromString(raw)
            except Exception:
                continue

            self._session_id = api_up.session_id
            self._robot_type = api_up.robot_type
            self._protocol_version = (
                api_up.protocol_major_version,
                api_up.protocol_minor_version,
            )

            if api_up.HasField("base_status"):
                bs = api_up.base_status
                self._latest_base_status = bs
                if bs.HasField("estimated_odometry"):
                    o = bs.estimated_odometry
                    self._latest_odometry = Odometry(
                        pos_x=o.pos_x,
                        pos_y=o.pos_y,
                        yaw=o.pos_z,
                        vx=o.speed_x,
                        vy=o.speed_y,
                        omega=o.speed_z,
                        stamp=time.monotonic(),
                    )

            if not self._first_status.is_set():
                self._first_status.set()

    async def _send_loop(self) -> None:
        period = 1.0 / self.send_hz
        while self._running:
            try:
                if self._brake_latched:
                    payload = self._build_brake()
                else:
                    payload = self._build_speed(
                        self._target_vx, self._target_vy, self._target_omega
                    )
                await self._ws.send(payload)
            except websockets.exceptions.ConnectionClosed:
                self._running = False
                return
            except Exception:
                pass
            await asyncio.sleep(period)


# ── Smoke test ──────────────────────────────────────────────────────────────

async def _smoke_test(url: str):
    async with ChassisClient(url) as bot:
        print(f"Connected. robot_type={bot.robot_type} proto={bot.protocol_version}")
        await bot.initialize()
        odom = await bot.wait_for_odometry()
        print(f"Start odom: x={odom.pos_x:.3f} y={odom.pos_y:.3f} yaw={odom.yaw:.3f}")
        bot.set_velocity(0.05, 0.0, 0.0)
        await asyncio.sleep(1.0)
        bot.stop()
        await asyncio.sleep(0.2)
        odom = bot.get_odometry()
        print(f"End odom:   x={odom.pos_x:.3f} y={odom.pos_y:.3f} yaw={odom.yaw:.3f}")


if __name__ == "__main__":
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--url", default="ws://192.168.1.230:8439")
    args = p.parse_args()
    asyncio.run(_smoke_test(args.url))
