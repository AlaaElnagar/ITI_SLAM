# Low Speed Course

## Installation

```bash
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy
```

## Running

- Launch Gazebo.

```bash
ros2 launch simulation_demo magni_house.launch.py
```

- Launch Keyboard Control.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
