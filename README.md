# armsim — Robotic Arm Kinematic Simulator

A configurable forward-kinematics simulator for arbitrary kinematic chains defined
via **Denavit–Hartenberg parameters**. Supports live WebSocket state streaming
to a retro 2D frontend viewer.

Built for two primary use cases:
1. **ML training data generation** — consistent, deterministic joint/EE data
2. **IK algorithm comparison** — identical hardware configs for apples-to-apples evaluation

No physics engine, no 3D renderer. Pure kinematics with a lightweight web viewer.

## Quickstart

```bash
pip install -e robotics_sim/backend/
```

```python
import armsim

# Build any arm — arbitrary joints, link lengths, and DOF.
arm = armsim.Arm([
    armsim.Joint(name="base",     axis="z",     link_length=0,  angle_min=-180, angle_max=180),
    armsim.Joint(name="shoulder", axis="pitch", link_length=20, angle_min=-90,  angle_max=90),
    armsim.Joint(name="elbow",    axis="pitch", link_length=15, angle_min=0,    angle_max=150),
])

arm.start(mode="local", port=8080)

# Parallel multi-joint move
arm.set_angles({"shoulder": 45, "elbow": 90})
arm.wait_until_idle()

print(arm.get_end_effector_position())  # (x, y, z)
arm.stop_server()
```

Or load from YAML/JSON config — no code changes needed between arm shapes:

```python
arm = armsim.load_arm_from_config("configs/six_axis_industrial.yaml")
arm.start(mode="lan", port=8080)  # accessible from other devices on the network
```

## Customizability

Every aspect of the arm is configurable — no hard-coded geometry anywhere.

| Parameter | Per-joint? | Description |
|-----------|-----------|-------------|
| `link_length` (`a`) | Yes | Physical segment length in any unit (cm recommended) |
| `link_offset` (`d`) | Yes | Offset along previous joint's Z axis |
| `axis` preset | Yes | `"z"` (α=0°), `"pitch"` (α=90°), `"yaw"` (α=-90°) |
| `dh_alpha` escape | Yes | Explicit DH α in degrees — any arbitrary geometry |
| `angle_min` / `angle_max` | Yes | Joint rotation limits in degrees |
| `default_velocity` | Arm + Joint | Angular speed in deg/s |
| `tick_rate_hz` | Arm | Simulation update frequency (default 30) |
| Joint count | Arm | **Any number** — 1 to N, no limit |

### Joint definition (in-code)

```python
Joint(
    name="elbow",          # Unique identifier
    axis="pitch",          # "z" | "pitch" | "yaw"
    link_length=15.0,      # DH a — segment length
    link_offset=0.0,       # DH d
    dh_alpha=None,         # Override axis preset with arbitrary α
    angle_min=0.0,         # Min rotation (degrees)
    angle_max=150.0,       # Max rotation (degrees)
    initial_angle=0.0,     # Starting angle
    default_velocity=30.0, # deg/s
)
```

### Config file (YAML/JSON)

```yaml
schema_version: 1
arm_name: "my_custom_arm"
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
  # ... any number of joints
```

## Backend ↔ Frontend Protocol

State flows from backend → frontend over WebSocket at `/ws`. Two message types:

### `config` (sent once on connect)
```json
{
  "type": "config",
  "protocol_version": 1,
  "arm_name": "six_axis_industrial",
  "tick_rate_hz": 30,
  "joint_count": 6,
  "total_reach": 68.0,
  "joints": [
    {"name": "base", "link_length": 0, "link_offset": 10, "angle_min": -180, "angle_max": 180, "axis": "z"}
  ]
}
```

### `state` (sent every tick, default 30 Hz)
```json
{
  "type": "state",
  "protocol_version": 1,
  "tick": 4821,
  "timestamp": 1730822400.123,
  "joints": [
    {"name": "base", "angle": 12.4, "target_angle": 90.0, "is_moving": true}
  ],
  "end_effector": {"x": 33.2, "y": 12.0, "z": 15.4},
  "front_view": [{"x": 0.0, "z": 5.0}, ...],
  "top_view": [{"x": 0.0, "y": 0.0}, ...]
}
```

Both sides (Python `protocol.py` and JS `websocket_client.js`) consume the same schema.
Frontend is read-only — all control happens via the Python API, not the browser.

## Frontend Viewer

Static HTML/CSS/JS served by the built-in server. No build step, no bundler.

- **Retro terminal theme** — black background, gold `#FFD700` + white accents
- **Two orthographic views** — front (X-Z) and top (X-Y) on `<canvas>`
- **Live joint readouts** — per-joint angle, target indicator, moving/idle tags
- **Auto-scaling** — canvas adjusts to total reach; works for 10 cm desktop arms up to 2 m industrial arms
- **WebSocket reconnect** — exponential backoff on disconnect
- **Responsive** — stacks views vertically below 700px viewport

## API Reference

### `Arm`
| Method | Description |
|--------|-------------|
| `start(mode, port)` | Start simulation + server ("local" binds 127.0.0.1, "lan" binds 0.0.0.0) |
| `stop_server()` | Graceful shutdown — closes WS connections, flushes recorder, joins threads |
| `set_angles(targets)` | Parallel multi-joint move — validates all targets before applying any |
| `wait_until_idle()` | Block until all joints reach targets |
| `get_end_effector_position()` | → `(x, y, z)` |
| `get_joint_positions()` | → list of `(x, y, z)` per joint |
| `get_state()` | → `ArmState` frozen snapshot |
| `reset_to_initial()` | Set all joints back to configured `initial_angle` |

### `Joint`
| Method | Description |
|--------|-------------|
| `set_angle(angle, velocity, blocking, timeout)` | Command toward target |
| `wait(timeout)` | Block until target reached (uses `threading.Event`, not busy-poll) |
| `stop()` | Freeze at current angle |
| `get_angle()` / `get_target_angle()` | Current / target angles in degrees |
| `is_moving()` | → `bool` |
| `get_dh_parameters()` | → `(θ, d, a, α)` |

## Package

```
robotics_sim/
├── backend/
│   ├── armsim/          # Installable Python package
│   │   ├── __init__.py
│   │   ├── joint.py
│   │   ├── arm.py
│   │   ├── kinematics.py
│   │   ├── motion.py
│   │   ├── simulation.py
│   │   ├── server.py
│   │   ├── protocol.py
│   │   ├── config.py
│   │   ├── state.py
│   │   ├── errors.py
│   │   ├── validation.py
│   │   ├── logging_utils.py
│   │   ├── recorder.py
│   │   ├── constants.py
│   │   └── cli.py
│   ├── tests/           # 63 unit/integration tests
│   ├── examples/        # 5 scripts + 3 YAML configs
│   └── pyproject.toml
├── frontend/            # Static web UI
│   ├── index.html
│   ├── css/
│   └── js/
├── venv/                # Virtual environment
├── README.md
└── agent_log.md
```

## CLI

```bash
python -m armsim run configs/arm.yaml --mode lan --port 8080 --record-to trajectory.jsonl
```

## Development

```bash
pip install -e "robotics_sim/backend/[dev]"
pytest robotics_sim/backend/tests/ -v    # 63 tests
```

## License

MIT
