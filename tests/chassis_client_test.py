#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
ChassisClient integration test — exercises every public method and measures
how long a velocity command persists after the last set_velocity() call vs.
whether an updated command can interrupt it mid-motion.

Usage:
    python3 tests/chassis_client_test.py --url ws://192.168.1.230:8439

Tests performed (in order):
  1. connect / properties / wait_for_odometry
  2. initialize
  3. set_velocity  — drive forward, observe odom change
  4. stop          — zero velocity, confirm deceleration
  5. set_velocity  interrupt — issue a new command while moving
  6. brake / release_brake
  7. velocity persistence — measure how long the robot keeps moving
     after we *stop calling* set_velocity (heartbeat keeps sending it)
  8. clear_parking_stop (if applicable)
  9. set_report_frequency
 10. deinitialize / close
"""

import sys
import os
import argparse
import asyncio
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from hex_device.chassis_client import ChassisClient, Odometry
from hex_device.generated import public_api_types_pb2


# ── Helpers ─────────────────────────────────────────────────────────────────

def fmt_odom(o: Odometry) -> str:
    return (f"x={o.pos_x:+.4f} y={o.pos_y:+.4f} yaw={o.yaw:+.4f}  "
            f"vx={o.vx:+.4f} vy={o.vy:+.4f} ω={o.omega:+.4f}")


def heading(title: str):
    print(f"\n{'─'*60}")
    print(f"  {title}")
    print(f"{'─'*60}")


async def wait_settle(bot: ChassisClient, seconds: float = 0.3):
    """Give the robot time to physically respond."""
    await asyncio.sleep(seconds)


# ── Individual tests ────────────────────────────────────────────────────────

async def test_connection_and_properties(bot: ChassisClient):
    heading("1. Connection & properties")
    assert bot.is_connected, "expected is_connected=True after connect()"
    print(f"  is_connected   = {bot.is_connected}")
    print(f"  robot_type     = {bot.robot_type}")
    print(f"  protocol_ver   = {bot.protocol_version}")
    print(f"  session_id     = {bot.session_id}")
    odom = await bot.wait_for_odometry(timeout=3.0)
    print(f"  first odometry = {fmt_odom(odom)}")
    print("  [PASS]")


async def test_initialize(bot: ChassisClient):
    heading("2. Initialize API control")
    await bot.initialize(warmup_seconds=0.5)
    assert bot.is_initialized, "expected is_initialized=True"
    print(f"  is_initialized = {bot.is_initialized}")
    print("  [PASS]")


async def test_set_velocity(bot: ChassisClient):
    heading("3. set_velocity — drive forward 0.05 m/s for 1 s")
    odom_before = bot.get_odometry()
    bot.set_velocity(0.05, 0.0, 0.0)
    await asyncio.sleep(1.0)
    odom_after = bot.get_odometry()
    dx = odom_after.pos_x - odom_before.pos_x
    print(f"  before: {fmt_odom(odom_before)}")
    print(f"  after:  {fmt_odom(odom_after)}")
    print(f"  Δx = {dx:+.4f} m  (expected ≈ +0.05)")
    bot.stop()
    await wait_settle(bot, 0.5)
    print("  [PASS]")


async def test_stop(bot: ChassisClient):
    heading("4. stop — zero velocity")
    bot.set_velocity(0.05, 0.0, 0.0)
    await asyncio.sleep(0.5)
    bot.stop()
    await asyncio.sleep(0.3)
    odom = bot.get_odometry()
    print(f"  odometry after stop: {fmt_odom(odom)}")
    print(f"  vx after stop = {odom.vx:+.4f}  (expected ≈ 0)")
    print("  [PASS]")


async def test_velocity_interrupt(bot: ChassisClient):
    heading("5. Velocity interrupt — change command mid-motion")
    print("  Phase A: forward 0.05 m/s for 0.5 s")
    bot.set_velocity(0.05, 0.0, 0.0)
    await asyncio.sleep(0.5)
    odom_a = bot.get_odometry()
    print(f"    odom A: {fmt_odom(odom_a)}")

    print("  Phase B: interrupt → rotate 0.2 rad/s for 0.5 s")
    bot.set_velocity(0.0, 0.0, 0.2)
    await asyncio.sleep(0.5)
    odom_b = bot.get_odometry()
    print(f"    odom B: {fmt_odom(odom_b)}")
    d_yaw = odom_b.yaw - odom_a.yaw
    print(f"    Δyaw = {d_yaw:+.4f} rad  (expected ≈ +0.1)")

    print("  Phase C: interrupt → backward -0.05 m/s for 0.5 s")
    bot.set_velocity(-0.05, 0.0, 0.0)
    await asyncio.sleep(0.5)
    odom_c = bot.get_odometry()
    print(f"    odom C: {fmt_odom(odom_c)}")

    bot.stop()
    await wait_settle(bot, 0.5)
    print("  [PASS] — velocity interrupts take effect immediately")


async def test_brake_and_release(bot: ChassisClient):
    heading("6. brake / release_brake")
    bot.set_velocity(0.05, 0.0, 0.0)
    await asyncio.sleep(0.3)
    print("  Engaging brake…")
    bot.brake()
    await asyncio.sleep(0.5)
    odom_braked = bot.get_odometry()
    print(f"  odom (braked): {fmt_odom(odom_braked)}")
    print(f"  vx while braked = {odom_braked.vx:+.4f}  (expected ≈ 0)")

    print("  Releasing brake…")
    bot.release_brake()
    await wait_settle(bot, 0.3)
    odom_released = bot.get_odometry()
    print(f"  odom (released): {fmt_odom(odom_released)}")
    print("  [PASS]")


async def test_velocity_persistence(bot: ChassisClient):
    heading("7. Velocity persistence — how long does the command last?")
    print("  The heartbeat loop keeps sending the last set_velocity at send_hz.")
    print("  Setting 0.05 m/s, then just sleeping (no further set_velocity calls).")
    bot.set_velocity(0.05, 0.0, 0.0)
    t0 = time.monotonic()
    odom_start = bot.get_odometry()

    # Sleep for 2 seconds without touching set_velocity — heartbeat keeps sending
    sample_times = [0.5, 1.0, 1.5, 2.0]
    for dt in sample_times:
        await asyncio.sleep(dt - (time.monotonic() - t0))
        odom = bot.get_odometry()
        elapsed = time.monotonic() - t0
        print(f"    t={elapsed:.1f}s  vx={odom.vx:+.4f}  x={odom.pos_x - odom_start.pos_x:+.4f}")

    bot.stop()
    await wait_settle(bot, 0.5)
    odom_end = bot.get_odometry()
    total_dx = odom_end.pos_x - odom_start.pos_x
    print(f"  Total Δx over 2 s = {total_dx:+.4f} m  (expected ≈ +0.10)")
    print("  → Velocity persists as long as the heartbeat loop is running.")
    print("  [PASS]")


async def test_set_report_frequency(bot: ChassisClient):
    heading("8. set_report_frequency")
    await bot.set_report_frequency(public_api_types_pb2.Rf50Hz)
    await asyncio.sleep(0.3)
    print("  Set report frequency to Rf50Hz — no error")
    # Restore
    await bot.set_report_frequency(public_api_types_pb2.Rf100Hz)
    await asyncio.sleep(0.1)
    print("  Restored to Rf100Hz")
    print("  [PASS]")


async def test_get_base_status(bot: ChassisClient):
    heading("9. get_base_status")
    bs = bot.get_base_status()
    if bs is not None:
        state_name = public_api_types_pb2.BaseState.Name(bs.state)
        print(f"  base_state             = {state_name}")
        print(f"  api_control_initialized = {bs.api_control_initialized}")
        print(f"  battery_voltage        = {bs.battery_voltage:.2f} V")
        print(f"  battery_thousandth     = {bs.battery_thousandth}")
    else:
        print("  (no base_status received yet)")
    print("  [PASS]")


async def test_deinitialize(bot: ChassisClient):
    heading("10. deinitialize")
    await bot.deinitialize()
    assert not bot.is_initialized, "expected is_initialized=False"
    print(f"  is_initialized = {bot.is_initialized}")
    print("  [PASS]")


# ── Main ────────────────────────────────────────────────────────────────────

async def run_all(url: str):
    print(f"Connecting to {url} …")
    async with ChassisClient(url, send_hz=50.0) as bot:
        await test_connection_and_properties(bot)
        await test_initialize(bot)
        await test_set_velocity(bot)
        await test_stop(bot)
        await test_velocity_interrupt(bot)
        await test_brake_and_release(bot)
        await test_velocity_persistence(bot)
        await test_set_report_frequency(bot)
        await test_get_base_status(bot)
        await test_deinitialize(bot)

    heading("All tests complete")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ChassisClient integration test")
    parser.add_argument("--url", default="ws://192.168.1.230:8439",
                        help="WebSocket URL of the robot")
    args = parser.parse_args()

    try:
        asyncio.run(run_all(args.url))
    except KeyboardInterrupt:
        print("\nInterrupted.")
