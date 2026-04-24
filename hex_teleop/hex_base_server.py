#!/usr/bin/env python3
"""
RPC server wrapping ChassisClient for the hex fellow chassis.

Analogous to base_server.py but for the hex chassis over WebSocket/protobuf.
Runs the async ChassisClient in a background thread and exposes a synchronous
RPC interface so other processes (hex_teleop.py) can call set_velocity /
get_odometry without dealing with asyncio.

Usage:
    python hex_base_server.py [--url ws://192.168.1.230:8439]
                              [--host localhost] [--port 50002]
"""

import argparse
import asyncio
import threading
import time
from multiprocessing.managers import BaseManager as MPBaseManager

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hex_device.chassis_client import ChassisClient

# Defaults
HEX_BASE_RPC_HOST = "localhost"
HEX_BASE_RPC_PORT = 50002
HEX_RPC_AUTHKEY = b"hex secret"
DEFAULT_CHASSIS_URL = "ws://192.168.1.230:8439"


class HexBase:
    """Synchronous wrapper around the async ChassisClient.

    A dedicated asyncio event-loop runs in a daemon thread.  Public methods
    are ordinary blocking calls that schedule coroutines on that loop.
    """

    def __init__(self, url=DEFAULT_CHASSIS_URL):
        self.url = url
        self._client: ChassisClient | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None

    # -- helpers to run coroutines from sync land --------------------------

    def _run(self, coro):
        """Submit *coro* to the background event loop and block for the result."""
        if self._loop is None or self._loop.is_closed():
            raise RuntimeError("Event loop not running – call reset() first")
        future = asyncio.run_coroutine_threadsafe(coro, self._loop)
        return future.result(timeout=10.0)

    def _start_loop(self):
        """Spin up the background asyncio thread (idempotent)."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(
            target=self._loop.run_forever, daemon=True
        )
        self._thread.start()

    # -- public API (called via RPC from hex_teleop) -----------------------

    def reset(self):
        """(Re-)connect to the chassis and take API control."""
        # Tear down previous connection if any
        if self._client is not None:
            try:
                self._run(self._client.close())
            except Exception:
                pass
            self._client = None

        self._start_loop()

        # ChassisClient.__init__ constructs asyncio primitives (e.g.
        # asyncio.Event), which on Python 3.9 require a current event loop
        # in the calling thread. Build it *on* the background loop thread.
        async def _build():
            return ChassisClient(self.url)

        self._client = self._run(_build())
        self._run(self._client.connect())
        self._run(self._client.initialize())
        print(
            f"HexBase connected  robot_type={self._client.robot_type}  "
            f"proto={self._client.protocol_version}"
        )

    def set_velocity(self, vx, vy, omega):
        """Set body-frame velocity (m/s, m/s, rad/s).  Non-blocking."""
        if self._client is None:
            raise RuntimeError("Not connected – call reset() first")
        self._client.set_velocity(float(vx), float(vy), float(omega))

    def brake(self):
        if self._client is None:
            return
        self._client.brake()

    def release_brake(self):
        if self._client is None:
            return
        self._client.release_brake()

    def get_odometry(self):
        """Return (pos_x, pos_y, yaw, vx, vy, omega) or None."""
        if self._client is None:
            return None
        odom = self._client.get_odometry()
        if odom is None:
            return None
        return {
            "pos_x": odom.pos_x,
            "pos_y": odom.pos_y,
            "yaw": odom.yaw,
            "vx": odom.vx,
            "vy": odom.vy,
            "omega": odom.omega,
        }

    def get_state(self):
        """Return state dict compatible with the tidybot obs format."""
        import numpy as np
        odom = self.get_odometry()
        if odom is None:
            return {"base_pose": np.zeros(3)}
        return {
            "base_pose": np.array([odom["pos_x"], odom["pos_y"], odom["yaw"]])
        }

    def close(self):
        if self._client is not None:
            try:
                self._run(self._client.close())
            except Exception:
                pass
            self._client = None
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
            self._loop = None


# -- RPC plumbing ----------------------------------------------------------

class HexBaseManager(MPBaseManager):
    pass


def main():
    parser = argparse.ArgumentParser(description="Hex chassis RPC server")
    parser.add_argument("--url", default=DEFAULT_CHASSIS_URL,
                        help="ChassisClient WebSocket URL")
    parser.add_argument("--host", default=HEX_BASE_RPC_HOST)
    parser.add_argument("--port", type=int, default=HEX_BASE_RPC_PORT)
    parser.add_argument("--authkey", default=HEX_RPC_AUTHKEY,
                        type=lambda s: s.encode() if isinstance(s, str) else s)
    args = parser.parse_args()

    # Register HexBase with the URL from CLI so proxies created by clients
    # use the server-configured chassis URL (not the hard-coded default).
    HexBaseManager.register("HexBase", callable=lambda: HexBase(url=args.url))

    manager = HexBaseManager(
        address=(args.host, args.port), authkey=args.authkey
    )
    server = manager.get_server()
    print(f"HexBase RPC server listening on {args.host}:{args.port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
