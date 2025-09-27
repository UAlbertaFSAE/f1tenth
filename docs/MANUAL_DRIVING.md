# Manual Control
Below is a guide to get the car running on manual control

## Wired Control

- Ensure the PS4 controller is plugged into the jetson via USB
- Plug in battery to VESC controller and check that VESC is powered (blue light is on)
- Run `colcon build` in the src directory of f1tenth_ws
- Source with `source install/setup.bash`
- Run `ros2 launch f1tenth_stack bringup_launch.py`
- Press the L1 button is pressed to move as it's the deadman's switch

## Wireless Control

TBD
