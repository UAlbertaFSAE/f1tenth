# Manual Driving

Driving the car by hand with a joystick. The teleop stack is a package in this workspace (`src/hardware/f1tenth_system`), so it builds with everything else -- there is no separate repository to clone and no submodule to initialise.

If the car has never been set up before, do the one-time Jetson steps in [JETSON_DEVELOPMENT.md](JETSON_DEVELOPMENT.md#hardware-installation) first. Without the udev rule the VESC has no stable device path and nothing below will work.

## Launch

```bash
make build
source ../install/setup.bash
ros2 launch f1tenth_stack bringup_launch.py
```

Two bringup launch files exist, and they differ by one node:

| Launch file | Brings up |
| --- | --- |
| `bringup_launch.py` | joystick, teleop, VESC driver, odometry, mux, static transforms, **and `urg_node`** |
| `no_lidar_bringup_launch.py` | the same, without `urg_node` |

`urg_node` is the driver for a Hokuyo LiDAR. Our car uses a Livox MID360, which is driven by `livox_ros_driver2` in the perception stack, not by `urg_node`. Use **`no_lidar_bringup_launch.py`** unless you have specifically attached a Hokuyo.

Either launch starts:

- `joy` and `joy_teleop` -- reads the controller, publishes to `/teleop`
- `vesc_driver_node` -- talks to the VESC over serial
- `ackermann_to_vesc_node` -- converts `AckermannDriveStamped` to VESC motor and servo commands
- `vesc_to_odom_node` -- publishes `/odom` from VESC telemetry
- `ackermann_mux` -- arbitrates between teleop and autonomous commands
- a static `base_link` -> `laser` transform

`throttle_interpolator` is defined in `bringup_launch.py` but its `add_action` line is commented out, so it does not run.

## The deadman's switch

The car only moves while a deadman button is held. Release it and commands stop.

Configured in `f1tenth_stack/config/joy_teleop.yaml`:

| Mode | Button index | Logitech F-710 | PS4 |
| --- | --- | --- | --- |
| `human_control` (teleop) | `4` | LB | L1 |
| `autonomous_control` | `5` | RB | R1 |

The `human_control` block also sets the scaling: `drive-speed` on axis 1 with `scale: 5.0`, and `drive-steering_angle` on axis 2 with `scale: 0.34`. Those are metres per second and radians at full deflection. Turn the speed scale down before driving somewhere tight.

There is a third block, `default`, on the same topic with both scales set to `0.0`. It is what publishes zeros when no deadman is held.

## How teleop and autonomy share the car

`ackermann_mux` subscribes to both command sources and forwards one of them to the VESC. From `f1tenth_stack/config/mux.yaml`:

| Input | Topic | Priority | Timeout |
| --- | --- | --- | --- |
| `joystick` | `teleop` | 100 | 0.2 s |
| `navigation` | `drive` | 10 | 0.2 s |

Higher priority wins, so **the joystick always overrides the autonomous stack**. That is the manual override: grab the controller, hold the deadman, and the car stops taking orders from `pure_pursuit`.

The 0.2 s timeout means an input that stops publishing is dropped after 200 ms. If teleop goes quiet, the mux falls back to `drive`; if both go quiet, nothing is forwarded.

## When the car will not move

Work down this list.

**Is the VESC powered?** Battery plugged in, blue light on. See [BATTERY_USAGE.md](BATTERY_USAGE.md) before handling the LiPo.

**Is the deadman held?** Nothing moves without it. Check the controller is the device `joy` opened -- `joy_teleop.yaml` sets `device_id: 0`, so a second connected input device can steal the slot.

**Is the controller publishing?**

```bash
ros2 topic echo /joy
```

Buttons should change as you press them. If the topic is silent, `joy` did not open the device.

**Are teleop commands being produced?**

```bash
ros2 topic echo /teleop
```

Should publish while the deadman is held. If `/joy` moves but `/teleop` does not, the button index in `joy_teleop.yaml` does not match your controller.

**Is the mux forwarding them?**

```bash
ros2 topic echo /drive
```

**Does the VESC have a device?** `vesc.yaml` sets `port: /dev/sensors/vesc`. Check it exists:

```bash
ls -l /dev/sensors/vesc
```

If it is missing, the udev rule is not installed or did not fire. See [JETSON_DEVELOPMENT.md](JETSON_DEVELOPMENT.md#hardware-installation).

**Is the VESC reporting?**

```bash
ros2 topic echo /sensors/core
```

Silence here with a device present means a serial permissions problem -- confirm your user is in the `dialout` group.

## Topics

Published by the driver stack:

- `/scan` -- `LaserScan`
- `/odom` -- `Odometry`
- `/sensors/imu/raw` -- `Imu`
- `/sensors/core` -- VESC telemetry

Subscribed to:

- `/drive` -- `AckermannDriveStamped`, the autonomous command input
- `/teleop` -- `AckermannDriveStamped`, the joystick command input

Per-package detail lives in the READMEs under `src/hardware/f1tenth_system/`.
