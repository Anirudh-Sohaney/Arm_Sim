# armsim — Backend Package

Python package for configurable robotic arm forward‑kinematics simulation.

## Install

```bash
# From GitHub (anywhere on your system)
pip install git+https://github.com/Anirudh-Sohaney/Arm_Sim.git#subdirectory=backend

# Or from within this cloned repo
pip install -e .          # editable install
pip install -e ".[dev]"   # with development dependencies
```

Requires Python ≥ 3.10.

## Quickstart

```python
import armsim

arm = armsim.Arm([
    armsim.Joint(name="base",     axis="z",     link_length=0),
    armsim.Joint(name="shoulder", axis="pitch", link_length=20),
    armsim.Joint(name="elbow",    axis="pitch", link_length=15),
])

arm.start(mode="local", port=8080)
arm.set_angles({"shoulder": 45, "elbow": 90})
arm.wait_until_idle()
print(arm.get_end_effector_position())  # (x, y, z)
arm.stop_server()
```

## Load from config

```python
arm = armsim.load_arm_from_config("my_arm.yaml")
arm.start()
```

Config files use schema version 1:

```yaml
schema_version: 1
arm_name: "my_arm"
tick_rate_hz: 30
default_velocity: 45
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
```

## Package Structure

```
armsim/
├── __init__.py          # Public API: Joint, Arm, ArmState, errors, load_arm_from_config
├── joint.py             # Joint — DH parameters, angle limits, motion advance
├── arm.py               # Arm — lifecycle, simulation loop, control API
├── kinematics.py        # Denavit–Hartenberg forward kinematics
├── motion.py            # Motion profiles (linear interpolation with velocity)
├── simulation.py        # Tick loop (orchestrated from arm.py)
├── state.py             # SharedState (thread‑safe) + ArmState snapshot
├── server.py            # FastAPI app + WebSocket state streamer
├── protocol.py          # JSON message builders (config, state, error)
├── config.py            # YAML/JSON config loader
├── errors.py            # Custom exceptions (JointLimitError, etc.)
├── validation.py        # Input sanitizers
├── recorder.py          # JSONL trajectory recorder
├── logging_utils.py     # Logger factory
├── constants.py         # Defaults, axis-to-alpha mappings
└── cli.py               # Command‑line entry point
```

## Key Modules

### `joint.py`

A single revolute DOF. Holds DH parameters (`a`, `d`, `α`), angle limits,
current/target states, and a `threading.Event` for idle signalling.

### `arm.py`

Owns the full lifecycle:
- Validates the joint chain on construction
- Launches simulation + uvicorn server threads on `start()`
- Provides `set_angles()`, `wait_until_idle()`, `get_end_effector_position()`, etc.
- Graceful shutdown via `stop_server()`

### `kinematics.py`

Pure DH‑based forward kinematics:
- `forward_kinematics(dh_list)` → ordered list of 3D positions
- `get_end_effector_position(positions)` → last position
- `project_front(positions)` → 2D X‑Z view
- `project_top(positions)` → 2D X‑Y view
- `total_reach(link_lengths)` → sum of all link lengths

### `server.py`

FastAPI application with:
- `GET /` — serves `frontend/index.html`
- `GET /healthz` — liveness check
- `WS /ws` — streams `config` (once) + `state` (every tick)
- Static mounts for `/css`, `/js`, `/assets`

The state streamer starts as a background task when the first WebSocket
client connects (not via `@app.on_event`, since the server runs with
`lifespan="off"` for daemon‑thread compatibility).

### `protocol.py`

Single source of truth for JSON message shapes. Both the Python server
and the JavaScript frontend consume the same schema:

- `build_config_message(...)` → one‑time `{"type": "config", ...}`
- `build_state_message(snapshot, tick)` → per‑tick `{"type": "state", ...}`
- `build_error_message(code, msg, recoverable)` → error broadcasts

### `config.py`

Loads arm definitions from YAML or JSON:
- `load_arm_from_config(path)` → `Arm`
- Validates schema version, joint names, angle ranges, velocities
- Supports JSON via extension detection (`.json`)

## Axis Presets

| Preset | DH α   | Meaning                          |
|--------|--------|----------------------------------|
| `"z"`  | 0°     | Rotation around parent Z axis    |
| `"pitch"` | 90°   | Rotates parent Y → joint Z (vertical arc) |
| `"yaw"`  | -90°  | Rotates parent X → joint Z (wrist twist) |

For arbitrary geometry, set `dh_alpha` directly:

```python
armsim.Joint(name="custom", dh_alpha=45.0, link_length=12)
```

## CLI

```bash
python -m armsim run my_arm.yaml --mode lan --port 8080 --record-to trajectory.jsonl
```

| Flag | Default | Description |
|------|---------|-------------|
| `run <config>` | (required) | YAML/JSON config path |
| `--mode` | `local` | `local` (127.0.0.1) or `lan` (0.0.0.0) |
| `--port` | `8080` | TCP port |
| `--record-to` | (off) | JSONL trajectory output path |
| `--log-level` | `INFO` | `DEBUG`, `INFO`, `WARNING`, `ERROR` |

## Development

```bash
pip install -e ".[dev]"
```
