# NVIDIA Jetson Development Guide

Our onboard computer is an [NVIDIA Jetson Xavier Developer Kit](https://developer.nvidia.com/embedded/jetson-developer-kits) running NVIDIA Jetpack 5.1.1.

Much of the perception work has to happen on the Jetson: the Stereolabs ZED SDK and its ROS 2 wrapper need an NVIDIA GPU and the camera attached. This document covers working on the car, and the one-time hardware setup the Jetson image itself needs.

## Connection

Connect over [ssh](NETWORK_SETUP.md), or plug in a keyboard, mouse and monitor. Unlike local development you do not open a devcontainer -- you attach to a container that is already running on the Jetson.

**Be courteous:** try not to have two people connected at once. Development happens on separate branches and a shared checkout cannot be on two of them.

## Software startup

Open a terminal on the Jetson, reach the repository, and update it. There are no submodules; a plain pull is the whole update. Stash anything you do not want to lose before switching branches.

```bash
cd ~/fsd/f1tenth
git pull
```

Start the containers:

```bash
docker compose -f docker/compose.jetson.yml up -d
```

This brings up two services: `jetson` (the autonomy container, ROS 2 Humble) and `hardware` (the driver stack container). With the VS Code Remote Development extension you can now press `ctrl+shift+p`, type "attach to running container", and pick the one you want to work in.

Inside the `jetson` container, install dependencies, then build and run:

```bash
make deps
make build
make run_auto
```

`make package <pkg>` builds a single package -- see the [vscode usage guide](VSCODE_USAGE.md). Every Makefile recipe sources ROS and the venv itself, so there is nothing to source by hand first.

## Hardware installation

These steps run on the Jetson image, not in a container, and only need doing once per flashed image. Vendoring `f1tenth_system` into this workspace removed the *setup* step, not the *install* step -- the packages arrive with `git clone`, but the car still needs the host configured before it will drive.

### VESC udev rule

Without this the VESC's device node moves between reboots and `f1tenth_stack/config/vesc.yaml` (`port: /dev/sensors/vesc`) points at nothing.

The rule ships in the repo at `src/hardware/f1tenth_system/vesc/vesc_driver/scripts/99-vesc6.rules`. It matches the VESC by USB vendor and product id (`0483:5740`), runs `vesc_device_lookup` to read the device's UUID, and creates a symlink named after it, owned by the `dialout` group.

Install the rule and the lookup helper:

```bash
sudo cp src/hardware/f1tenth_system/vesc/vesc_driver/scripts/99-vesc6.rules /etc/udev/rules.d/
sudo cp src/hardware/f1tenth_system/vesc/vesc_driver/scripts/vesc_device_lookup /bin/
sudo chmod +x /bin/vesc_device_lookup
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the VESC, then confirm the symlink appeared:

```bash
ls -l /dev/vesc/
```

**Two known problems with the upstream helper**, both present on `f1tenth_system`'s `humble-devel` branch as shipped:

- `vesc_device_lookup` exports `LD_LIBRARY_PATH=/opt/ros/foxy/lib`. On a Humble image that path does not exist.
- The rule creates `/dev/vesc/<uuid>`, but `vesc.yaml` expects `/dev/sensors/vesc`. The two do not currently agree, so the port has to be set to whatever the rule actually produced.

### Serial permissions

The udev rule sets `GROUP="dialout"`, so your user must be in that group to open the VESC:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for it to take effect. `groups` should list `dialout`.

### Joystick

Plug the controller into the Jetson over USB. It appears as `/dev/input/jsN`. `f1tenth_stack/config/joy_teleop.yaml` selects it by index:

```yaml
joy:
  ros__parameters:
    # device_name: /dev/input/joypad-f710
    device_id: 0
```

`device_id: 0` means the first joystick device. If more than one input device is connected, the numbering is not stable across reboots -- uncomment `device_name` and point it at a fixed path instead.

Confirm the controller is being read with `ros2 topic echo /joy` once the stack is up. Button and axis mapping, and the deadman switch, are covered in [MANUAL_DRIVING.md](MANUAL_DRIVING.md).

### Flashing and first boot

Flashing the Jetson with Jetpack 5.1.1 is done from a separate Ubuntu host using NVIDIA SDK Manager, following NVIDIA's own instructions for the Xavier Developer Kit. After the first boot, what has to be installed on the image rather than in a container is: Docker with the NVIDIA container runtime (`runtime: nvidia` in `compose.jetson.yml` depends on it), the udev rule and serial permissions above, and the repository checkout at `~/fsd/f1tenth`. Everything else -- ROS, the build tooling, the Python environment -- lives in the containers.

## A note on the hardware container

`compose.jetson.yml` runs two containers. The `jetson` service is built from `docker/Dockerfile.jetson` and runs ROS 2 Humble, matching the rest of this workspace. The `hardware` service is built from `docker/Dockerfile.hardware`, which is `FROM f1tenth/focal-l4t-foxy:f1tenth-stack` -- an upstream prebuilt image running ROS 2 Foxy on Ubuntu 20.04.

They talk over DDS rather than sharing a ROS installation, so the distro mismatch does not stop the stack from working. It does mean anything you build inside `hardware` is built against Foxy, and the `f1tenth_system` sources in this workspace are not what that container is running.
