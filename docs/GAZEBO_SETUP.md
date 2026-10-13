# F1TENTH / UA26 Gazebo Simulation Setup

This simulation setup runs **ROS 2 Humble + Gazebo Harmonic** in a Docker container. The real-car / Jetson stack remains separate from the simulator.

The Gazebo model is intentionally kept outside the normal ROS source tree under `gazebo_sim/` so the simulation assets do not become part of the Jetson build.

## Repository layout

The expected layout is:

```text
f1tenth/
├── docker/
│   ├── Dockerfile.sim
│   └── compose.sim.yml
├── gazebo_sim/
│   └── models/
│       └── ua26_car/
│           ├── meshes/
│           ├── model.config
│           ├── model.sdf
│           └── world.sdf
├── start_sim.sh
└── ...
```

## Important: you must provide the UA26 SDF / meshes yourself

The large CAD / mesh files are not expected to be committed to normal Git.

Each team member must obtain the working UA26 Gazebo model separately and place it at:

```text
gazebo_sim/models/ua26_car/
```

The model directory **must be named**:

```text
ua26_car
```

The main model file should be named:

```text
model.sdf
```

The world file should be named:

```text
world.sdf
```

`world.sdf` is expected to include the vehicle with:

```xml
<include>
  <uri>model://ua26_car</uri>
</include>
```

Because `GZ_SIM_RESOURCE_PATH` points to `/sim/gazebo_sim/models`, Gazebo resolves that URI as:

```text
/sim/gazebo_sim/models/ua26_car
```

If your model folder has a different name, either rename it to `ua26_car`, or update the `model://...` URI in `world.sdf` to exactly match the folder name.

The provided `start_sim.sh` also supports an alternate folder name:

```bash
UA26_MODEL_NAME=my_model_name ./start_sim.sh
```

If you do this, `world.sdf` must also use:

```xml
<uri>model://my_model_name</uri>
```

## Starting the simulator

From the root of the repository:

```bash
chmod +x start_sim.sh
./start_sim.sh
```

The script will:

1. Start the `sim` Docker Compose service using the separate `f1tenth-sim` Compose project.
2. Source ROS 2 Humble inside the container.
3. Set the Gazebo resource path.
4. Use Ogre 1 as the Gazebo render engine.
5. Launch the UA26 `world.sdf`.

The equivalent manual commands are:

```bash
docker compose -p f1tenth-sim -f docker/compose.sim.yml up -d sim

docker compose -p f1tenth-sim -f docker/compose.sim.yml exec sim bash
```

Then inside the container:

```bash
source /opt/ros/humble/setup.bash
export GZ_SIM_RESOURCE_PATH=/sim/gazebo_sim/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RENDER_ENGINE=ogre
gz sim -v 4 /sim/gazebo_sim/models/ua26_car/world.sdf
```

## Why Ogre 1 is used

On the current Windows 11 + WSL2 setup, Ogre 2 reached the D3D12-backed GPU but crashed while allocating GPU resources. Ogre 1 worked correctly and was significantly smoother than software rendering, so the simulator currently defaults to:

```bash
GZ_SIM_RENDER_ENGINE=ogre
```

Do not remove this unless Ogre 2 has been tested successfully on your machine.

## GPU-accelerated WSL2 setup

For WSLg hardware acceleration, `compose.sim.yml` may contain entries similar to:

```yaml
environment:
  - DISPLAY=${DISPLAY}
  - LD_LIBRARY_PATH=/usr/lib/wsl/lib
  - QT_QPA_PLATFORM=xcb
  - GZ_SIM_RESOURCE_PATH=/sim/gazebo_sim/models
  - GZ_SIM_RENDER_ENGINE=ogre

devices:
  - /dev/dxg:/dev/dxg

volumes:
  - ../gazebo_sim:/sim/gazebo_sim
  - /tmp/.X11-unix:/tmp/.X11-unix
  - /usr/lib/wsl:/usr/lib/wsl:ro
```

To check whether Gazebo is using hardware acceleration, enter the container and run:

```bash
glxinfo -B
```

A hardware-accelerated WSL2 setup should show a real GPU / D3D12 renderer and should not report `llvmpipe` with `Accelerated: no`.

## If your machine does not have GPU acceleration

If `/dev/dxg` is unavailable, or you are intentionally using CPU software rendering, remove the WSL GPU-specific pieces from `compose.sim.yml`.

Remove:

```yaml
devices:
  - /dev/dxg:/dev/dxg
```

Remove this GPU-library mount:

```yaml
- /usr/lib/wsl:/usr/lib/wsl:ro
```

And remove:

```yaml
- LD_LIBRARY_PATH=/usr/lib/wsl/lib
```

Keep the display forwarding and Gazebo configuration, for example:

```yaml
environment:
  - DISPLAY=${DISPLAY}
  - QT_QPA_PLATFORM=xcb
  - GZ_SIM_RESOURCE_PATH=/sim/gazebo_sim/models
  - GZ_SIM_RENDER_ENGINE=ogre
  - LIBGL_ALWAYS_SOFTWARE=1

volumes:
  - ../gazebo_sim:/sim/gazebo_sim
  - /tmp/.X11-unix:/tmp/.X11-unix
```

`LIBGL_ALWAYS_SOFTWARE=1` explicitly tells Mesa to use software rendering. This is slower, but it allows Gazebo to run without GPU acceleration.

After changing `compose.sim.yml`, recreate the simulator container:

```bash
docker compose -p f1tenth-sim -f docker/compose.sim.yml down
docker compose -p f1tenth-sim -f docker/compose.sim.yml up -d --build sim
```

Then run:

```bash
./start_sim.sh
```

## Useful troubleshooting commands

Check whether the simulator container is running:

```bash
docker ps
```

Enter the simulator container:

```bash
docker compose -p f1tenth-sim -f docker/compose.sim.yml exec sim bash
```

Check the Gazebo model resource path:

```bash
echo $GZ_SIM_RESOURCE_PATH
```

Check the resource paths known by a running Gazebo server:

```bash
gz service \
  -s /gazebo/resource_paths/get \
  --reqtype gz.msgs.Empty \
  --reptype gz.msgs.StringMsg_V \
  --timeout 3000 \
  --req ''
```

Expected model resource path:

```text
/sim/gazebo_sim/models
```

Check graphics acceleration:

```bash
glxinfo -B
```

If Gazebo reports:

```text
Unable to find uri[model://ua26_car]
```

make sure this directory exists:

```text
gazebo_sim/models/ua26_car/
```

and that `world.sdf` uses exactly:

```xml
<uri>model://ua26_car</uri>
```

## Stopping the simulator

From the repository root:

```bash
docker compose -p f1tenth-sim -f docker/compose.sim.yml down
```

Using the explicit project name `f1tenth-sim` keeps the simulation Compose stack separate from the existing development / noVNC containers.
