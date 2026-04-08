#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
Raw WebSocket chassis control — bypasses the HexDeviceApi SDK (and its version check).

Usage:
    python3 tests/raw_chassis_control.py --url ws://192.168.1.230:8439

Interactive commands (type in terminal):
    vx vy omega   — set velocity, e.g. "0.1 0.0 0.0" (m/s, m/s, rad/s)
    stop          — zero velocity
    status        — print latest robot status
    quit          — graceful shutdown
    Ctrl+C        — same as quit
"""

import sys
import os
import argparse
import asyncio
import time
import signal

# Allow importing hex_device from the repo root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import websockets
from hex_device.generated import public_api_down_pb2, public_api_up_pb2, public_api_types_pb2
from hex_device.generated.version import CURRENT_PROTOCOL_MAJOR_VERSION, CURRENT_PROTOCOL_MINOR_VERSION

# ── Helpers to build protobuf messages ──────────────────────────────────────

def make_report_frequency_msg(freq: int) -> bytes:
    """Build a serialised APIDown that sets the report frequency."""
    msg = public_api_down_pb2.APIDown()
    msg.set_report_frequency = freq
    msg.protocol_major_version = CURRENT_PROTOCOL_MAJOR_VERSION
    msg.protocol_minor_version = CURRENT_PROTOCOL_MINOR_VERSION
    return msg.SerializeToString()


def make_init_msg(initialize: bool = True) -> bytes:
    """Build a serialised APIDown that sends api_control_initialize."""
    msg = public_api_down_pb2.APIDown()
    msg.base_command.api_control_initialize = initialize
    msg.protocol_major_version = CURRENT_PROTOCOL_MAJOR_VERSION
    msg.protocol_minor_version = CURRENT_PROTOCOL_MINOR_VERSION
    return msg.SerializeToString()


def make_speed_msg(vx: float, vy: float, omega: float) -> bytes:
    """Build a serialised APIDown with a SimpleBaseMoveCommand / XyzSpeed."""
    msg = public_api_down_pb2.APIDown()
    xyz = public_api_types_pb2.XyzSpeed()
    xyz.speed_x = vx
    xyz.speed_y = vy
    xyz.speed_z = omega
    msg.base_command.simple_move_command.xyz_speed.CopyFrom(xyz)
    msg.protocol_major_version = CURRENT_PROTOCOL_MAJOR_VERSION
    msg.protocol_minor_version = CURRENT_PROTOCOL_MINOR_VERSION
    return msg.SerializeToString()


def make_brake_msg() -> bytes:
    """Build a serialised APIDown with a SimpleBaseMoveCommand / brake."""
    msg = public_api_down_pb2.APIDown()
    msg.base_command.simple_move_command.brake = True
    msg.protocol_major_version = CURRENT_PROTOCOL_MAJOR_VERSION
    msg.protocol_minor_version = CURRENT_PROTOCOL_MINOR_VERSION
    return msg.SerializeToString()


# ── Pretty-print helpers ───────────────────────────────────────────────────

BASE_STATE_NAMES = {
    0: "BsParked",
    1: "BsAlgrithmControl",
    2: "BsOvertakeSpeedControl",
    3: "BsOvertakeZeroResistanceControl",
    4: "BsEmergencyStop",
}

PARKING_CATEGORY_NAMES = {
    0: "EmergencyStopButton",
    1: "MotorHasError",
    2: "BatteryProtection",
    3: "GamepadTriggered",
    4: "Unknown",
    5: "APICommunicationTimeout",
    6: "LimitSwitchTriggered",
    7: "BmsTimeout",
}


def format_base_status(bs) -> str:
    state = BASE_STATE_NAMES.get(bs.state, str(bs.state))
    parts = [
        f"state={state}",
        f"api_init={bs.api_control_initialized}",
        f"session_holder={bs.session_holder}",
        f"battery={bs.battery_voltage:.1f}V ({bs.battery_thousandth/10:.1f}%)",
        f"motors={len(bs.motor_status)}",
    ]
    if bs.HasField("estimated_odometry"):
        odom = bs.estimated_odometry
        parts.append(
            f"odom=({odom.pos_x:.3f}, {odom.pos_y:.3f}, {odom.pos_z:.3f})"
        )
    if bs.HasField("parking_stop_detail"):
        psd = bs.parking_stop_detail
        cat = PARKING_CATEGORY_NAMES.get(psd.category, str(psd.category))
        parts.append(f"PARKING_STOP: {cat} — {psd.reason} (clearable={psd.is_remotely_clearable})")
    return " | ".join(parts)


# ── Main controller ────────────────────────────────────────────────────────

class RawChassisController:
    def __init__(self, url: str, max_speed: float, max_omega: float, send_hz: float):
        self.url = url
        self.max_speed = max_speed
        self.max_omega = max_omega
        self.send_hz = send_hz

        # Shared state (written from stdin reader, read from send loop)
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_omega = 0.0
        self.last_cmd_time = time.monotonic()

        # Latest status from robot
        self.latest_base_status = None
        self.session_id = None
        self.robot_type = None
        self.protocol_version = (0, 0)

        self._ws = None
        self._running = True
        self._initialized = False
        self._cmd_timeout = 2.0  # auto-zero after 2s of no input

    # ── Clamp helper ──

    def _clamp_velocity(self, vx, vy, omega):
        vx = max(-self.max_speed, min(self.max_speed, vx))
        vy = max(-self.max_speed, min(self.max_speed, vy))
        omega = max(-self.max_omega, min(self.max_omega, omega))
        return vx, vy, omega

    # ── Receive loop ──

    async def _recv_loop(self):
        """Receive APIUp messages, parse, and store latest status."""
        while self._running:
            try:
                raw = await asyncio.wait_for(self._ws.recv(), timeout=3.0)
            except asyncio.TimeoutError:
                print("[warn] No data from robot for 3 s")
                continue
            except websockets.exceptions.ConnectionClosed:
                print("[error] WebSocket connection closed by robot")
                self._running = False
                return

            if not isinstance(raw, bytes):
                continue

            api_up = public_api_up_pb2.APIUp()
            try:
                api_up.ParseFromString(raw)
            except Exception as e:
                print(f"[warn] Failed to parse APIUp: {e}")
                continue

            self.session_id = api_up.session_id
            self.robot_type = api_up.robot_type
            self.protocol_version = (
                api_up.protocol_major_version,
                api_up.protocol_minor_version,
            )

            if api_up.HasField("base_status"):
                bs = api_up.base_status
                self.latest_base_status = bs

                # Print parking stop warnings
                if bs.HasField("parking_stop_detail"):
                    psd = bs.parking_stop_detail
                    cat = PARKING_CATEGORY_NAMES.get(psd.category, str(psd.category))
                    print(f"\n[!] PARKING STOP: {cat} — {psd.reason} (clearable={psd.is_remotely_clearable})")

    # ── Send loop ──

    async def _send_loop(self):
        """Continuously send velocity commands at send_hz."""
        period = 1.0 / self.send_hz
        while self._running:
            now = time.monotonic()

            # Auto-zero on command timeout
            if now - self.last_cmd_time > self._cmd_timeout:
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.target_omega = 0.0

            vx, vy, omega = self._clamp_velocity(
                self.target_vx, self.target_vy, self.target_omega
            )

            try:
                await self._ws.send(make_speed_msg(vx, vy, omega))
            except websockets.exceptions.ConnectionClosed:
                self._running = False
                return

            await asyncio.sleep(period)

    # ── Stdin reader (runs in executor to avoid blocking) ──

    async def _stdin_loop(self):
        """Read commands from stdin."""
        loop = asyncio.get_event_loop()
        while self._running:
            try:
                line = await loop.run_in_executor(None, sys.stdin.readline)
            except EOFError:
                break
            if not line:
                break
            line = line.strip()
            if not line:
                continue

            if line.lower() == "quit":
                print("Shutting down...")
                self._running = False
                return
            elif line.lower() == "stop":
                self.target_vx = 0.0
                self.target_vy = 0.0
                self.target_omega = 0.0
                self.last_cmd_time = time.monotonic()
                print("-> velocity set to (0, 0, 0)")
            elif line.lower() == "status":
                if self.latest_base_status is not None:
                    print(f"[status] {format_base_status(self.latest_base_status)}")
                    print(f"         robot_type={self.robot_type}  proto={self.protocol_version}  session={self.session_id}")
                else:
                    print("[status] No base_status received yet")
            elif line.lower() == "init":
                # Re-send api_control_initialize
                try:
                    await self._ws.send(make_init_msg(True))
                    print("-> sent api_control_initialize = True")
                except websockets.exceptions.ConnectionClosed:
                    self._running = False
            elif line.lower() == "brake":
                try:
                    await self._ws.send(make_brake_msg())
                    print("-> sent brake command")
                except websockets.exceptions.ConnectionClosed:
                    self._running = False
            else:
                # Try to parse as "vx vy omega"
                parts = line.split()
                if len(parts) == 3:
                    try:
                        vx, vy, omega = float(parts[0]), float(parts[1]), float(parts[2])
                        vx, vy, omega = self._clamp_velocity(vx, vy, omega)
                        self.target_vx = vx
                        self.target_vy = vy
                        self.target_omega = omega
                        self.last_cmd_time = time.monotonic()
                        print(f"-> velocity set to ({vx:.3f}, {vy:.3f}, {omega:.3f})")
                    except ValueError:
                        print("[?] Could not parse. Use: vx vy omega  (e.g. 0.1 0.0 0.0)")
                else:
                    print("[?] Commands: vx vy omega | stop | status | init | brake | quit")

    # ── Main entry ──

    async def run(self):
        print(f"Connecting to {self.url} ...")
        try:
            self._ws = await websockets.connect(
                self.url,
                ping_interval=20,
                ping_timeout=60,
                close_timeout=5,
            )
        except Exception as e:
            print(f"[error] Could not connect: {e}")
            return

        # Receive first message to learn about the robot
        try:
            raw = await asyncio.wait_for(self._ws.recv(), timeout=5.0)
        except Exception as e:
            print(f"[error] No response from robot: {e}")
            await self._ws.close()
            return

        api_up = public_api_up_pb2.APIUp()
        api_up.ParseFromString(raw)
        self.robot_type = api_up.robot_type
        self.protocol_version = (api_up.protocol_major_version, api_up.protocol_minor_version)
        self.session_id = api_up.session_id

        print(f"Connected!  robot_type={self.robot_type}  protocol={self.protocol_version}  session={self.session_id}")

        if api_up.HasField("base_status"):
            self.latest_base_status = api_up.base_status
            print(f"  {format_base_status(api_up.base_status)}")
        else:
            print("  (first message had no base_status — this robot may not be a chassis type)")

        # Set report frequency to 100 Hz
        print("Setting report frequency to 100 Hz ...")
        await self._ws.send(make_report_frequency_msg(public_api_types_pb2.Rf100Hz))

        # Start sending zero-velocity commands (proto says: send these first, then init)
        print("Sending zero-velocity commands for 0.5 s before initializing ...")
        for _ in range(25):  # ~0.5s at 50 Hz
            await self._ws.send(make_speed_msg(0.0, 0.0, 0.0))
            await asyncio.sleep(0.02)

        # Send api_control_initialize
        print("Sending api_control_initialize = True ...")
        await self._ws.send(make_init_msg(True))
        self._initialized = True

        print()
        print("Ready. Enter commands:")
        print("  vx vy omega   — e.g. '0.1 0.0 0.0' (m/s m/s rad/s)")
        print("  stop          — zero velocity")
        print("  status        — print robot status")
        print("  init          — re-send api_control_initialize")
        print("  brake         — send brake")
        print("  quit / Ctrl+C — shutdown")
        print()

        # Run recv, send, and stdin loops concurrently
        tasks = [
            asyncio.create_task(self._recv_loop()),
            asyncio.create_task(self._send_loop()),
            asyncio.create_task(self._stdin_loop()),
        ]

        # Wait until any task finishes (usually stdin quit or connection close)
        try:
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        except asyncio.CancelledError:
            pass

        self._running = False

        # Graceful shutdown: zero velocity, deinit
        try:
            await self._ws.send(make_speed_msg(0.0, 0.0, 0.0))
            await self._ws.send(make_init_msg(False))
        except Exception:
            pass

        for t in tasks:
            t.cancel()

        try:
            await self._ws.close()
        except Exception:
            pass

        print("Connection closed. Bye!")


# ── CLI ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Raw WebSocket chassis control (bypasses SDK version check)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--url",
        default="ws://192.168.1.230:8439",
        help="WebSocket URL (default: ws://192.168.1.230:8439)",
    )
    parser.add_argument(
        "--max-speed",
        type=float,
        default=0.5,
        help="Max linear speed in m/s (default: 0.5)",
    )
    parser.add_argument(
        "--max-omega",
        type=float,
        default=1.0,
        help="Max angular speed in rad/s (default: 1.0)",
    )
    parser.add_argument(
        "--send-hz",
        type=float,
        default=50.0,
        help="Command send rate in Hz (default: 50)",
    )
    args = parser.parse_args()

    ctrl = RawChassisController(
        url=args.url,
        max_speed=args.max_speed,
        max_omega=args.max_omega,
        send_hz=args.send_hz,
    )

    loop = asyncio.new_event_loop()

    def _signal_handler():
        print("\nCtrl+C received, shutting down ...")
        ctrl._running = False

    loop.add_signal_handler(signal.SIGINT, _signal_handler)

    try:
        loop.run_until_complete(ctrl.run())
    except KeyboardInterrupt:
        ctrl._running = False
    finally:
        loop.close()


if __name__ == "__main__":
    main()