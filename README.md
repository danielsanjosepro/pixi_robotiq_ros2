
# Robotiq 2F-85 Pixi

<img src="media/robotiq_visual.png" alt="Logo" width="160" align="right" z-index="10">

A [Pixi](https://pixi.sh)-based workspace for controlling the Robotiq 2F-85 gripper using ROS2 Jazzy and ros2_control.

> [!WARNING]
> It uses a fork https://github.com/danielsanjosepro/ros2_robotiq_gripper of the original repository https://github.com/PickNikRobotics/ros2_robotiq_gripper as it was required for a minimal change.
> You might want to consider using the orignal repository for future ROS2 versions.

### Usage 

Set the udev rules for the gripper:
```bash
# In scripts/99_grippers.rules
SUBSYSTEM=="tty", ATTRS{serial}=="TODO-CHECK-WITH-lsusb", SYMLINK+="robotiq_gripper", MODE="0666", ATTR{device/latency_timer}="1"
```

then run:
```bash
pixi run set-udev-rules
```



```bash
# Clone the ROS2 gripper driver
pixi run clone
# Build the workspace
pixi run build
# Launch the gripper
pixi run gripper
```

Launch arguments:

- `com_port` - Serial port (default: `/dev/robotiq_gripper`)
- `launch_rviz` - Launch RViz visualization (default: `false`)

### Test

**Sine Wave Test** - Moves the gripper in a continuous sine wave pattern:

```bash
pixi run sine-test
```

**Slider Control** - GUI slider for manual gripper control:

```bash
pixi shell
python scripts/slider_gripper_control.py
```

## License

See the individual package licenses in the `src/` directory.
