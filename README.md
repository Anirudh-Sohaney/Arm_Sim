# armsim — Robotic Arm Kinematic Simulator

A configurable forward‑kinematics simulator for arbitrary kinematic chains
defined via **Denavit–Hartenberg parameters**. Supports live WebSocket
state streaming to a retro 2D frontend viewer.

Built for two primary use cases:
1. **ML training data generation** — consistent, deterministic joint/EE data
2. **IK algorithm comparison** — identical hardware configs for apples‑to‑apples evaluation

No physics engine, no 3D renderer. Pure kinematics with a lightweight web viewer.

## Install

```bash
# Direct from GitHub
pip install git+https://github.com/Anirudh-Sohaney/Arm_Sim.git#subdirectory=backend

# Or clone + editable install
pip install -e backend/
```

Requires Python ≥ 3.10.

## Quickstart

```python
import armsim

arm = armsim.Arm([
    armsim.Joint(name="base",     axis="z",     link_length=0,  angle_min=-180, angle_max=180),
    armsim.Joint(name="shoulder", axis="pitch", link_length=20, angle_min=-90,  angle_max=90),
    armsim.Joint(name="elbow",    axis="pitch", link_length=15, angle_min=0,    angle_max=150),
])

arm.start(mode="lan", port=8080)
arm.set_angles({"shoulder": 45, "elbow": 90})
arm.wait_until_idle()

print(arm.get_end_effector_position())  # (x, y, z)
arm.stop_server()
```

Open `http://localhost:8080` in a browser to see the arm moving in real time.

## Documentation

| Document | Contents |
|----------|----------|
| **[DOCS.md](DOCS.md)** | Full Python library guide — API reference, control patterns, config files, trajectory recording, CLI, examples |
| **[backend/README.md](backend/README.md)** | Package structure, module descriptions, axis presets, development setup |
| **[frontend/README.md](frontend/README.md)** | Web dashboard — architecture, file map, WebSocket protocol, customisation |

## Customizability

Every aspect of the arm is configurable — **no hard‑coded geometry**.

| Parameter | Per‑joint? | Description |
|-----------|-----------|-------------|
| `link_length` (`a`) | Yes | Physical segment length in any unit (cm recommended) |
| `link_offset` (`d`) | Yes | Offset along previous joint's Z axis |
| `axis` preset | Yes | `"z"` (α=0°), `"pitch"` (α=90°), `"yaw"` (α=-90°) |
| `dh_alpha` escape | Yes | Explicit DH α in degrees — any arbitrary geometry |
| `angle_min` / `angle_max` | Yes | Joint rotation limits in degrees |
| `default_velocity` | Arm + Joint | Angular speed in deg/s |
| `tick_rate_hz` | Arm | Simulation update frequency (default 30) |
| Joint count | Arm | **Any number** — 1 to N, no limit |

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
```

## Frontend Viewer

Static HTML/CSS/JS served by the built‑in server. No build step, no bundler.

- **Retro terminal theme** — black background, gold `#FFD700` + white accents
- **Two orthographic views** — front (X‑Z) and top (X‑Y) on `<canvas>`
- **Live joint readouts** — per‑joint angle, target indicator, moving/idle tags
- **Auto‑scaling** — canvas adjusts to total reach; works for 10 cm desktop arms up to 2 m industrial arms
- **WebSocket reconnect** — exponential backoff on disconnect
- **Responsive** — stacks views vertically below 700px viewport

## Project Structure

```
robotics_sim/
├── backend/
│   ├── armsim/              # Installable Python package (16 modules)
│   ├── pyproject.toml       # Build config, dependencies, CLI entry point
│   └── README.md
├── frontend/                # Static web UI
│   ├── index.html
│   ├── css/
│   ├── js/
│   └── README.md
├── DOCS.md                  # Full Python library documentation
├── README.md                # This file
└── .gitignore
```

## CLI

```bash
python -m armsim run config.yaml --mode lan --port 8080 --record-to trajectory.jsonl
```

## Development

```bash
pip install -e "backend/[dev]"
```

## License

MIT
