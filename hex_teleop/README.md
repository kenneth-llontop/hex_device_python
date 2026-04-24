# Hex Fellow Chassis Teleop

Phone-based teleoperation for the hex fellow chassis using the TidyBot++ WebXR phone interface.

## Architecture

```
Phone (WebXR browser app)
  |  Socket.IO (port 5000)
  v
hex_teleop.py
  |  Flask/SocketIO + OTG velocity controller (50 Hz)
  |  RPC (port 50002)
  v
hex_base_server.py
  |  Async ChassisClient wrapper
  |  WebSocket + protobuf
  v
Hex Fellow Chassis (default: ws://192.168.1.230:8439)
```

## Prerequisites

```bash
pip install flask flask-socketio websockets scipy numpy protobuf
```

## Quick start

Open two terminals (or tmux panes):

```bash
# Terminal 1 - Start the chassis RPC server
cd hex_teleop
python hex_base_server.py --url ws://192.168.1.230:8439

# Terminal 2 - Start the phone teleop
cd hex_teleop
python hex_teleop.py
```

Then on your phone:

1. Connect to the same Wi-Fi network as the host machine
2. Open `http://<host-ip>:5000` in a WebXR-capable browser (Chrome on Android, Safari 15+ on iOS)
3. Tap **"Start episode"** to begin the AR session
4. Touch the screen and move the phone to drive the robot:
   - Phone translation controls robot translation (x/y)
   - Phone rotation controls robot rotation (yaw)
   - Touching anywhere on the screen activates base control
5. Tap **"End episode"** when done, then **"Reset env"** to start a new episode

## Configuration

**Chassis URL:**

```bash
python hex_base_server.py --url ws://<robot-ip>:8439
```

**RPC host/port:**

```bash
python hex_base_server.py --host localhost --port 50002
python hex_teleop.py --rpc-host localhost --rpc-port 50002
```

**Velocity and acceleration limits** - edit the constants at the top of `hex_teleop.py`:

```python
MAX_VEL   = np.array([0.5, 0.5, 1.57])    # m/s, m/s, rad/s
MAX_ACCEL = np.array([0.25, 0.25, 0.79])   # m/s^2, m/s^2, rad/s^2
```

## How it works

1. **hex_base_server.py** wraps the async `ChassisClient` (WebSocket + protobuf) in a synchronous RPC server using `multiprocessing.managers`. The asyncio event loop runs in a daemon thread. Other processes call `set_velocity()`, `get_odometry()`, `brake()`, and `close()` over RPC.

2. **hex_teleop.py** runs three concurrent threads:
   - **Flask/SocketIO web server** serving the WebXR phone UI
   - **Listener thread** draining phone messages into `BaseTeleopController`, which computes position targets using the same reference-pose logic as TidyBot++'s `TeleopController`
   - **50 Hz control loop** reading chassis odometry, computing position error to the phone target, running a trapezoidal OTG velocity profile generator (respecting max velocity and acceleration limits), transforming global-frame velocity to body-frame, and sending to the chassis via RPC

3. The OTG generates smooth acceleration/deceleration profiles to track position targets, the same behavior as TidyBot++'s Ruckig-based trajectory generator implemented as a lightweight trapezoidal profile.

## Troubleshooting

| Symptom | Fix |
|---|---|
| "No initial APIUp from robot within 5 s" | Check that the chassis is powered on and the WebSocket URL is correct |
| Phone can't load web UI | Ensure phone and host are on the same network; use `http://` not `https://` |
| WebXR not available | Use Chrome on Android or Safari 15+ on iOS |
| Robot not moving | Check RTT display on phone (~7 ms on 5 GHz Wi-Fi); verify hex_base_server.py prints "HexBase connected" |
| Jerky motion | Lower MAX_VEL / MAX_ACCEL in hex_teleop.py; check Wi-Fi signal quality |
