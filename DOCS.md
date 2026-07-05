# armsim — Python Library Documentation

`armsim` is a configurable **forward‑kinematics** simulator for robotic arms with
arbitrary revolute (rotating) joints. It computes joint and end‑effector
positions from Denavit–Hartenberg parameters, streams live state over
WebSocket, and serves a real‑time 2D web dashboard.

## Table of Contents

- [Installation](#installation)
- [Quickstart](#quickstart)
- [Core Concepts](#core-concepts)
  - [Joint](#joint)
  - [Arm](#arm)
  - [Config Files](#config-files)
- [API Reference](#api-reference)
  - [Arm](#arm-1)
  - [Joint](#joint-1)
  - [ArmState (snapshot)](#armstate-snapshot)
  - [Loading from config](#loading-from-config)
  - [Logging](#logging)
- [Controlling the Arm](#controlling-the-arm)
  - [Single-joint moves](#single-joint-moves)
  - [Multi-joint (parallel) moves](#multi-joint-parallel-moves)
  - [Blocking vs non-blocking](#blocking-vs-non-blocking)
  - [Velocity control](#velocity-control)
  - [Resetting](#resetting)
- [Reading Arm State](#reading-arm-state)
  - [End-effector position](#end-effector-position)
  - [Joint positions](#joint-positions)
  - [Full snapshot](#full-snapshot)
- [Server Modes](#server-modes)
  - [Local mode](#local-mode)
  - [LAN mode](#lan-mode)
  - [Health check](#health-check)
  - [WebSocket protocol](#websocket-protocol)
- [Trajectory Recording](#trajectory-recording)
- [CLI](#cli)
- [Customizability](#customizability)
- [Examples](#examples)

---

## Installation

```bash
# Install directly from GitHub
pip install git+https://github.com/Anirudh-Sohaney/Arm_Sim.git#subdirectory=backend

# Or clone + install locally
git clone https://github.com/Anirudh-Sohaney/Arm_Sim.git
cd Arm_Sim
pip install -e backend/
```

Requires Python ≥ 3.10. Dependencies: `numpy`, `pyyaml`, `fastapi`, `uvicorn`.

Optional dev deps (for WebSocket testing):

```bash
pip install -e "backend/[dev]"
```

---

## Quickstart

```python
import armsim

# Build a 3-joint arm
arm = armsim.Arm([
    armsim.Joint(name="base",     axis="z",     link_length=0),
    armsim.Joint(name="shoulder", axis="pitch", link_length=20),
    armsim.Joint(name="elbow",    axis="pitch", link_length=15),
])

# Start simulation + web server on localhost:8080
arm.start(mode="local", port=8080)

# Move two joints in parallel
arm.set_angles({"shoulder": 45, "elbow": 90})
arm.wait_until_idle()

# Read end-effector position
x, y, z = arm.get_end_effector_position()
print(f"EE: ({x:.1f}, {y:.1f}, {z:.1f})")

# Shut down
arm.stop_server()
```

---

## Core Concepts

### Joint

A `Joint` is a single **revolute** (rotating) degree of freedom. It is
defined by its name, DH parameters, angle limits, and velocity.

```python
Joint(
    name="elbow",           # Unique identifier (str)
    axis="pitch",           # Preset: "z" | "pitch" | "yaw"        (default "z")
    link_length=15.0,       # DH a — segment length                (default 0.0)
    link_offset=0.0,        # DH d — offset along parent Z         (default 0.0)
    dh_alpha=None,          # Explicit DH α override (degrees)     (default None)
    angle_min=0.0,          # Min rotation (degrees)               (default -180)
    angle_max=150.0,        # Max rotation (degrees)               (default 180)
    initial_angle=0.0,      # Starting angle                       (default 0.0)
    default_velocity=30.0,  # Angular speed (deg/s)                (default 30.0)
)
```

| Axis preset | DH α   | Typical use              |
|-------------|--------|--------------------------|
| `"z"`       | 0°     | Base rotation, yaw       |
| `"pitch"`   | 90°    | Shoulder, elbow (vertical arc) |
| `"yaw"`     | -90°   | Wrist twist              |

For any geometry not covered by the three presets, set `dh_alpha` directly:

```python
armsim.Joint(name="custom", dh_alpha=45.0, link_length=12)
# dh_alpha overrides axis entirely
```

### Arm

An `Arm` is an ordered chain of `Joint` objects from base → end‑effector.
**Order matters** — it defines the DH multiplication chain.

```python
arm = armsim.Arm(joints, tick_rate_hz=30)
```

`tick_rate_hz` controls how many simulation steps per second (default 30).

### Config Files

Arms can be defined in YAML or JSON for reproducibility:

```yaml
# six_axis.yaml
schema_version: 1
arm_name: "my_arm"
tick_rate_hz: 30
default_velocity: 45           # fallback for joints without explicit velocity
joints:
  - name: "base"
    axis: "z"
    link_length: 0
    link_offset: 10
    angle_min: -180
    angle_max: 180
  - name: "shoulder"
    axis: "pitch"
    link_length: 30
    angle_min: -90
    angle_max: 90
  # ... any number of joints
```

Load it:

```python
arm = armsim.load_arm_from_config("six_axis.yaml")
arm.start()
```

The config supports `.yaml`, `.yml`, and `.json` (auto‑detected by extension).

---

## API Reference

### Arm

| Method | Description |
|--------|-------------|
| `start(mode="local", port=8080, tick_rate_hz=None, record_to=None)` | Launch simulation + HTTP/WS server. `mode="lan"` binds 0.0.0.0. |
| `stop_server()` | Graceful shutdown — closes WS, flushes recorder, joins threads. |
| `set_angles(targets, velocity=None, blocking=False, timeout=None)` | Set multiple joint targets at once (validates all before applying any). |
| `wait_until_idle(timeout=None)` | Block until all joints reach their targets. |
| `get_end_effector_position()` → `(x, y, z)` | World-space EE coordinates. |
| `get_joint_positions()` → `[(x,y,z), ...]` | Ordered 3D positions of every joint. |
| `get_state()` → `ArmState` | Immutable full snapshot (angles, targets, positions, views). |
| `reset_to_initial()` | Return all joints to their configured `initial_angle`. |

### Joint

| Method | Description |
|--------|-------------|
| `set_angle(angle, velocity=None, blocking=False, timeout=None)` | Command toward target. Validates against [angle_min, angle_max]. |
| `wait(timeout=None)` | Block until joint reaches target (uses `threading.Event`, not busy‑poll). |
| `stop()` | Freeze at current angle. |
| `get_angle()` / `get_target_angle()` → `float` | Current / target angles in degrees. |
| `is_moving()` → `bool` | |
| `get_dh_parameters()` / `get_current_dh()` → `(θ, d, a, α)` | DH tuple in degrees/units. |

### ArmState (snapshot)

Returned by `arm.get_state()`:

```python
snapshot = arm.get_state()
# snapshot.tick                  — int
# snapshot.timestamp             — float (Unix time)
# snapshot.joints                — list of JointSnapshot
#   snapshot.joints[0].name      — str
#   snapshot.joints[0].angle     — float
#   snapshot.joints[0].target_angle — float
#   snapshot.joints[0].is_moving — bool
#   snapshot.joints[0].position  — (x, y, z)
# snapshot.end_effector_position  — (x, y, z)
# snapshot.front_view             — [(x, z), ...]  (2D projection)
# snapshot.top_view               — [(x, y), ...]  (2D projection)
```

### Loading from config

```python
arm = armsim.load_arm_from_config("path/to/arm.yaml")
# Returns a fully constructed Arm ready for .start()
```

### Logging

```python
armsim.set_log_level("DEBUG")   # or "INFO", "WARNING", "ERROR"
```

---

## Controlling the Arm

### Single-joint moves

```python
arm.joints["shoulder"].set_angle(45.0, blocking=True)
# Blocks until shoulder reaches 45°
```

### Multi-joint (parallel) moves

```python
arm.set_angles({
    "shoulder": 45.0,
    "elbow": 90.0,
    "wrist": 30.0,
})
# All three start moving simultaneously
arm.wait_until_idle()
```

All target angles are **validated** before any joint starts moving
(all‑or‑nothing semantics).

### Blocking vs non-blocking

```python
# Non-blocking — returns immediately
arm.set_angles({"shoulder": 45, "elbow": 90})  # blocking=False by default

# Blocking — returns when all joints are idle or timeout expires
arm.set_angles({"shoulder": 45, "elbow": 90}, blocking=True, timeout=5.0)
```

### Velocity control

Per‑joint default velocity is set at construction. Override per‑call:

```python
# Fast move
arm.joints["base"].set_angle(90, velocity=180.0)

# Slow move
arm.set_angles({"shoulder": 45, "elbow": 90}, velocity=10.0)
```

### Resetting

```python
arm.reset_to_initial()
# Returns all joints to their configured initial_angle (non-blocking)
arm.wait_until_idle()
```

---

## Reading Arm State

### End-effector position

```python
x, y, z = arm.get_end_effector_position()
```

### Joint positions

```python
positions = arm.get_joint_positions()
# [(base_x, base_y, base_z), (shoulder_x, shoulder_y, shoulder_z), ...]
```

### Full snapshot

```python
state = arm.get_state()
print(f"Tick: {state.tick}")
print(f"EE: {state.end_effector_position}")
for j in state.joints:
    print(f"  {j.name}: {j.angle:.1f}° → {j.target_angle:.1f}° {'moving' if j.is_moving else 'idle'}")
```

---

## Server Modes

The built‑in server serves the frontend dashboard and streams state over
WebSocket.

### Local mode

Binds to `127.0.0.1` — only accessible from the same machine.

```python
arm.start(mode="local", port=8080)
```

### LAN mode

Binds to `0.0.0.0` — accessible from any device on the local network.

```python
arm.start(mode="lan", port=8080)
```

Then open `http://<machine-ip>:8080` in a browser.

### Health check

```
GET /healthz → {"status": "ok", "tick_rate_hz": 30.0}
```

### WebSocket protocol

Connect to `ws://<host>:<port>/ws`. Two message types arrive:

1. **`config`** — sent once on connect (joint names, link lengths, total reach, etc.)
2. **`state`** — sent every tick (joint angles, EE position, 2D view projections)

The frontend (`/js/websocket_client.js`) is a reference consumer. Build
your own client by following the same JSON schema defined in
`backend/armsim/protocol.py`.

---

## Trajectory Recording

Pass `record_to=<path>` to `start()` to log every tick's joint angles
and EE position to a JSONL file:

```python
arm.start(mode="local", port=8080, record_to="trajectory.jsonl")

# ... run moves ...

arm.stop_server()
# trajectory.jsonl contains one JSON object per tick
```

Each line: `{"tick": 0, "angles": [0.0, 0.0, ...], "end_effector": [x, y, z]}`

---

## CLI

```bash
python -m armsim run path/to/config.yaml --mode lan --port 8080 --record-to trajectory.jsonl
```

Options:

| Flag | Description |
|------|-------------|
| `run <config>` | Load arm from YAML/JSON config file |
| `--mode local\|lan` | Bind address (default `local`) |
| `--port <n>` | TCP port (default `8080`) |
| `--record-to <path>` | Enable trajectory recording |
| `--log-level <level>` | DEBUG, INFO, WARNING, ERROR |

---

## Customizability

Every aspect of the arm is configurable — **no hard‑coded geometry**.

| What | How |
|------|-----|
| Number of joints | Any — 1 to N. Pass N `Joint` objects to `Arm()`. |
| Link lengths & offsets | Per‑joint `link_length` and `link_offset`. |
| Rotation axis (DOF) | Three presets (`z`, `pitch`, `yaw`) + `dh_alpha` escape hatch for any arbitrary axis. |
| Joint limits | Per‑joint `angle_min` / `angle_max`. |
| Velocity | Per‑joint `default_velocity`, per‑call `velocity` override. |
| Tick rate | `tick_rate_hz` on `Arm()` or `start()`. |
| Config storage | YAML or JSON — schema versioned for forward compat. |
| Recording | JSONL trajectory output — one line per tick. |

---

## Examples

### ML training data collection

```python
import armsim, random, json

arm = armsim.load_arm_from_config("six_axis.yaml")
arm.start(mode="local", port=8080, record_to="dataset.jsonl")

for _ in range(1000):
    targets = {
        j: random.uniform(arm.joints[j].angle_min, arm.joints[j].angle_max)
        for j in arm.joints
    }
    arm.set_angles(targets, blocking=True)
    ee = arm.get_end_effector_position()
    # Each tick is automatically logged to dataset.jsonl

arm.stop_server()
```

### Custom client consuming WebSocket state

```python
import asyncio, json, websockets

async def main():
    async with websockets.connect("ws://localhost:8080/ws") as ws:
        while True:
            msg = json.loads(await ws.recv())
            if msg["type"] == "state":
                ee = msg["end_effector"]
                print(f"EE: ({ee['x']:.1f}, {ee['y']:.1f}, {ee['z']:.1f})")

asyncio.run(main())
```
