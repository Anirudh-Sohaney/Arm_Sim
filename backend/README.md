# armsim — Backend Package

Python package for configurable robotic arm forward-kinematics simulation.

## Install

```bash
pip install -e .          # editable
pip install -e ".[dev]"   # with testing deps
```

## Quickstart

```python
import armsim

arm = armsim.Arm([
    armsim.Joint(name="base",     axis="z",     link_length=0),
    armsim.Joint(name="shoulder", axis="pitch", link_length=20),
    armsim.Joint(name="elbow",    axis="pitch", link_length=15),
])
arm.start(mode="local", port=8080)
arm.joints["shoulder"].set_angle(45, blocking=True)
print(arm.get_end_effector_position())
arm.stop_server()
```

## Load from config

```python
arm = armsim.load_arm_from_config("examples/configs/six_axis_industrial.yaml")
arm.start()
```

## CLI

```bash
python -m armsim run examples/configs/planar_2dof.yaml --mode local --port 8080
```

## Run Tests

```bash
pytest tests/ -v    # 63 tests
```
