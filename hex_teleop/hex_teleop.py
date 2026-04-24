#!/usr/bin/env python3
"""
Phone teleop for the hex fellow chassis.

Serves the WebXR phone app (same index.html as tidybot), converts phone poses
into position targets via TeleopController, then uses Ruckig OTG to generate
smooth velocity commands sent to hex_base_server.py over RPC.

Architecture
~~~~~~~~~~~~
    Phone (WebXR)
      │  Socket.IO (port 5000)
      ▼
    Flask / SocketIO  ─►  TeleopController  ─►  position targets
                                                       │
    OTG control loop (50 Hz) ◄─────────────────────────┘
      │  velocity = OTG(target_pos, current_pos)
      ▼
    hex_base_server RPC  ─►  ChassisClient  ─►  hex chassis

Usage:
    # Terminal 1 – start the chassis RPC server
    python hex_base_server.py --url ws://192.168.1.230:8439

    # Terminal 2 – start this teleop script
    python hex_teleop.py

    # On your phone, open http://<host-ip>:5000 and tap "Start episode"
"""

import argparse
import logging
import math
import socket
import threading
import time
from queue import Queue

import numpy as np
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from ruckig import InputParameter, OutputParameter, Result, Ruckig, ControlInterface
from scipy.spatial.transform import Rotation as R

from hex_base_server import HexBaseManager, HEX_BASE_RPC_HOST, HEX_BASE_RPC_PORT, HEX_RPC_AUTHKEY

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

CONTROL_FREQ = 50       # Hz – matches ChassisClient send_hz
CONTROL_PERIOD = 1.0 / CONTROL_FREQ

# Velocity / acceleration limits (same as tidybot Vehicle defaults)
MAX_VEL = np.array([0.5, 0.5, 1.57])        # m/s, m/s, rad/s
MAX_ACCEL = np.array([0.25, 0.25, 0.79])     # m/s², m/s², rad/s²

TWO_PI = 2 * math.pi

# iPhone 14 Pro camera offset (same as original policies.py)
DEVICE_CAMERA_OFFSET = np.array([0.0, 0.02, -0.04])


# ---------------------------------------------------------------------------
# WebXR coordinate conversion  (copied from policies.py)
# ---------------------------------------------------------------------------

def convert_webxr_pose(pos, quat):
    """WebXR (+x right, +y up, +z back) → Robot (+x fwd, +y left, +z up)."""
    pos = np.array([-pos['z'], -pos['x'], pos['y']], dtype=np.float64)
    rot = R.from_quat([-quat['z'], -quat['x'], quat['y'], quat['w']])
    pos = pos + rot.apply(DEVICE_CAMERA_OFFSET)
    return pos, rot


# ---------------------------------------------------------------------------
# BaseTeleopController  (base-only subset of TeleopController)
# ---------------------------------------------------------------------------

class BaseTeleopController:
    """Processes WebXR phone messages and produces base position targets.

    This is a simplified version of TeleopController from policies.py that
    only handles base movement (ignores arm / gripper).
    """

    def __init__(self):
        self.primary_device_id = None
        self.enabled_counts = {}

        # Robot state
        self.base_pose = np.zeros(3)  # current pose from odometry

        # Targets
        self.targets_initialized = False
        self.base_target_pose = None

        # WebXR reference
        self.base_xr_ref_pos = None
        self.base_xr_ref_rot_inv = None
        self.base_ref_pose = None

    def update_base_pose(self, base_pose):
        """Called each control tick with latest odometry."""
        self.base_pose = base_pose
        if not self.targets_initialized:
            self.base_target_pose = base_pose.copy()
            self.targets_initialized = True

    def process_message(self, data):
        if not self.targets_initialized:
            return

        device_id = data['device_id']

        # Track enabled count (skip first 2 frames for WebXR pose latency)
        self.enabled_counts[device_id] = (
            self.enabled_counts.get(device_id, 0) + 1
            if 'teleop_mode' in data else 0
        )

        # Assign primary device
        if self.enabled_counts[device_id] > 2:
            if self.primary_device_id is None:
                self.primary_device_id = device_id
        elif self.enabled_counts[device_id] == 0:
            if device_id == self.primary_device_id:
                self.primary_device_id = None
                self.base_xr_ref_pos = None

        # Teleop enabled – process base movement
        if self.primary_device_id is not None and 'teleop_mode' in data:
            pos, rot = convert_webxr_pose(data['position'], data['orientation'])

            # Any touch region drives the base (no arm)
            if self.base_xr_ref_pos is None:
                self.base_ref_pose = self.base_pose.copy()
                self.base_xr_ref_pos = pos[:2]
                self.base_xr_ref_rot_inv = rot.inv()

            # Position
            self.base_target_pose[:2] = (
                self.base_ref_pose[:2] + (pos[:2] - self.base_xr_ref_pos)
            )

            # Orientation
            fwd = (rot * self.base_xr_ref_rot_inv).apply([1.0, 0.0, 0.0])
            target_theta = self.base_ref_pose[2] + math.atan2(fwd[1], fwd[0])
            # Unwrap angle
            self.base_target_pose[2] += (
                (target_theta - self.base_target_pose[2] + math.pi) % TWO_PI
                - math.pi
            )

        # Teleop disabled – track actual pose so target stays current
        elif self.primary_device_id is None:
            self.base_target_pose = self.base_pose.copy()

    @property
    def is_enabled(self):
        return self.primary_device_id is not None

    def get_target(self):
        """Return current base position target [x, y, theta]."""
        if self.base_target_pose is None:
            return None
        return self.base_target_pose.copy()


# ---------------------------------------------------------------------------
# Web server  (serves the phone WebXR UI)
# ---------------------------------------------------------------------------

class WebServer:
    def __init__(self, queue):
        self.app = Flask(__name__,
                         template_folder='templates',
                         static_folder='static')
        self.socketio = SocketIO(self.app)
        self.queue = queue

        @self.app.route('/')
        def index():
            return render_template('index.html')

        @self.socketio.on('message')
        def handle_message(data):
            emit('echo', data['timestamp'])
            self.queue.put(data)

        logging.getLogger('werkzeug').setLevel(logging.WARNING)

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0)
        try:
            s.connect(('8.8.8.8', 1))
            address = s.getsockname()[0]
        except Exception:
            address = '127.0.0.1'
        finally:
            s.close()
        print(f'Phone teleop UI at http://{address}:5000')
        self.socketio.run(self.app, host='0.0.0.0')


# ---------------------------------------------------------------------------
# Ruckig-based OTG velocity generator
# ---------------------------------------------------------------------------

class RuckigOTG:
    """Ruckig online trajectory generator for (x, y, yaw) position targets.

    Same formulation as tidybot2's base_controller.Vehicle: Ruckig runs in
    position-control mode, generates a smooth velocity profile that respects
    max_velocity and max_acceleration, and we read .new_velocity each tick.

    Yaw handling: Ruckig treats every dof as linear, so odometry yaw wrap
    (±π boundary) has to be removed before feeding in a current position.
    We "unwrap" the current yaw onto the same branch as the target so Ruckig
    always sees the shortest rotation.
    """

    def __init__(self, max_vel, max_accel, dt):
        self.num_dofs = 3
        self.dt = dt
        self.otg = Ruckig(self.num_dofs, dt)
        self.inp = InputParameter(self.num_dofs)
        self.out = OutputParameter(self.num_dofs)
        self.inp.max_velocity = np.asarray(max_vel, dtype=float)
        self.inp.max_acceleration = np.asarray(max_accel, dtype=float)
        self.inp.control_interface = ControlInterface.Position
        self.res = Result.Finished
        self._initialized = False

    def reset(self, current_pos):
        """Re-seed internal state to current_pos with zero velocity."""
        self.inp.current_position = current_pos.tolist()
        self.inp.current_velocity = [0.0, 0.0, 0.0]
        self.inp.current_acceleration = [0.0, 0.0, 0.0]
        self.inp.target_position = current_pos.tolist()
        self.inp.target_velocity = [0.0, 0.0, 0.0]
        self.res = Result.Working
        self._initialized = True

    def update(self, current_pos, target_pos):
        """Return the global-frame velocity to command this tick."""
        # Unwrap current yaw onto the same branch as target yaw so Ruckig
        # sees a continuous signal (no 2π jumps from odometry).
        current = current_pos.copy()
        yaw_err = (target_pos[2] - current[2] + math.pi) % TWO_PI - math.pi
        current[2] = target_pos[2] - yaw_err

        if not self._initialized:
            self.reset(current)

        self.inp.current_position = current.tolist()
        self.inp.target_position = target_pos.tolist()
        self.inp.target_velocity = [0.0, 0.0, 0.0]

        self.res = self.otg.update(self.inp, self.out)
        self.out.pass_to_input(self.inp)

        return np.array(self.out.new_velocity)


# ---------------------------------------------------------------------------
# Main control loop
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Hex chassis phone teleop")
    parser.add_argument("--rpc-host", default=HEX_BASE_RPC_HOST)
    parser.add_argument("--rpc-port", type=int, default=HEX_BASE_RPC_PORT)
    parser.add_argument("--rpc-authkey", default=HEX_RPC_AUTHKEY,
                        type=lambda s: s.encode() if isinstance(s, str) else s)
    args = parser.parse_args()

    # -- Connect to hex_base_server RPC ------------------------------------
    print(f"Connecting to hex_base_server at {args.rpc_host}:{args.rpc_port}...")
    manager = HexBaseManager(
        address=(args.rpc_host, args.rpc_port), authkey=args.rpc_authkey
    )
    manager.connect()
    base = manager.HexBase()
    print("Connected. Resetting chassis...")
    base.reset()

    # Wait for first odometry sample so the control loop never starts
    # with a zero-pose fallback.
    for _ in range(100):  # up to ~2 s
        if base.get_odometry() is not None:
            break
        time.sleep(0.02)
    else:
        print("WARNING: no odometry received from chassis yet")
    print("Chassis initialized.")

    # -- Start web server --------------------------------------------------
    msg_queue = Queue()
    web_server = WebServer(msg_queue)
    threading.Thread(target=web_server.run, daemon=True).start()

    # -- Teleop state machine ----------------------------------------------
    controller = BaseTeleopController()
    teleop_state = None  # episode_started / episode_ended / reset_env
    otg = RuckigOTG(MAX_VEL, MAX_ACCEL, CONTROL_PERIOD)

    def listener_loop():
        """Drain the web-server queue and feed messages to the controller."""
        nonlocal teleop_state
        while True:
            if not msg_queue.empty():
                data = msg_queue.get()
                if 'state_update' in data:
                    teleop_state = data['state_update']
                elif 1000 * time.time() - data['timestamp'] < 250:
                    controller.process_message(data)
            time.sleep(0.001)

    threading.Thread(target=listener_loop, daemon=True).start()

    # -- Wait for first episode --------------------------------------------
    print('Open the phone UI and press "Start episode" to begin.')

    try:
        while True:
            # Wait for episode start
            while teleop_state != 'episode_started':
                time.sleep(0.01)
            print("Episode started – teleop active")

            # Seed OTG with current pose so first velocity is zero
            odom0 = base.get_odometry()
            seed_pose = (
                np.array([odom0['pos_x'], odom0['pos_y'], odom0['yaw']])
                if odom0 is not None else np.zeros(3)
            )
            otg.reset(seed_pose)

            # -- 50 Hz control loop ----------------------------------------
            episode_running = True
            while episode_running:
                t0 = time.time()

                # 1. Read odometry
                odom = base.get_odometry()
                if odom is not None:
                    current_pose = np.array([
                        odom['pos_x'], odom['pos_y'], odom['yaw']
                    ])
                else:
                    current_pose = np.zeros(3)

                # 2. Feed odometry to teleop controller
                controller.update_base_pose(current_pose)

                # 3. Get position target from phone teleop
                target = controller.get_target()

                if target is not None and controller.is_enabled:
                    # 4. OTG: compute velocity in global frame
                    vel_global = otg.update(current_pose, target)

                    # 5. Transform global→body frame
                    yaw = current_pose[2]
                    c, s = math.cos(yaw), math.sin(yaw)
                    vx_body = c * vel_global[0] + s * vel_global[1]
                    vy_body = -s * vel_global[0] + c * vel_global[1]
                    omega = vel_global[2]

                    # 6. Send to chassis
                    base.set_velocity(vx_body, vy_body, omega)
                else:
                    # No teleop input – zero velocity, keep OTG in sync
                    base.set_velocity(0.0, 0.0, 0.0)
                    otg.reset(current_pose)

                # Check for episode end / reset
                if teleop_state == 'episode_ended':
                    print("Episode ended.")
                    base.set_velocity(0.0, 0.0, 0.0)
                    otg.reset(current_pose)
                    # Keep running until reset_env
                    while teleop_state != 'reset_env':
                        time.sleep(0.01)
                    episode_running = False

                # Maintain 50 Hz
                elapsed = time.time() - t0
                if elapsed < CONTROL_PERIOD:
                    time.sleep(CONTROL_PERIOD - elapsed)

            # Prepare for next episode
            teleop_state = None
            controller = BaseTeleopController()
            # Re-feed current pose
            odom = base.get_odometry()
            if odom is not None:
                controller.update_base_pose(
                    np.array([odom['pos_x'], odom['pos_y'], odom['yaw']])
                )
            print('Press "Start episode" for a new episode.')

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        base.set_velocity(0.0, 0.0, 0.0)
        base.close()
        print("Done.")


if __name__ == "__main__":
    main()
