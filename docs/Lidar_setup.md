# LiDAR Setup Guide

How to get the Livox MID360 publishing a point cloud on the Jetson and displayed in the stack's RViz window.

## How It Is Wired

The MID360 talks to the Jetson over the ethernet port `eno1`. The driver config is `src/perception/livox_ros_driver2/config/MID360_config.json`:

| | IP |
| --- | --- |
| Jetson (host) | `192.168.1.5` |
| MID360 | `192.168.1.133` |

The driver binds to the host IP, so if `eno1` does not have `192.168.1.5` the driver fails at startup with `bind failed` and `Init lds lidar fail!`.

## One-Time Network Setup

`eno1` defaults to DHCP, and there is no DHCP server on the car, so it never gets an address. Give it a static IP once. It persists across reboots. No gateway is set, so internet and ssh stay on WiFi.

```bash
sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.1.5/24 ipv6.method disabled
sudo nmcli con up "Wired connection 1"
```

Check it:

```bash
ip -4 -br addr show eno1      # should show 192.168.1.5/24
ping -c2 192.168.1.133        # the LiDAR should reply
```

## Finding the LiDAR's IP

A MID360's factory IP is `192.168.1.1XX`, where `XX` is the last two digits of its serial number. If the ping above fails, sweep the subnet and see what answers:

```bash
for i in $(seq 1 254); do ping -c1 -W1 192.168.1.$i >/dev/null 2>&1 & done; wait
ip neigh show dev eno1 | grep -v -E "INCOMPLETE|FAILED"
```

Put the address that answers in the `"ip"` field of `lidar_configs` in `MID360_config.json`.

## Running It

`launch_pkg/config/config.yaml` launches the driver with `msg_MID360_launch.py`, which publishes `sensor_msgs/PointCloud2` (`xfer_format = 0`) on `/livox/lidar` in the `livox_frame` frame. `lidar_cone_filtering` and RViz both need `PointCloud2`; the Livox `CustomMsg` format (`xfer_format = 1`) cannot be displayed in RViz. Do not use `rviz_MID360_launch.py` with the stack: it opens a second RViz window.

```bash
make run_auto
```

The stack's RViz window (`launch_pkg/config/visualizer.rviz`, fixed frame `base_link`) shows:

- **Raw LiDAR**: `/livox/lidar`
- **LiDAR No Ground**: `/lidar/no_ground`
- **LiDAR Cone Centroids**: `/lidar/cone_cluster_centroids`

`base_link -> livox_frame` is a static transform in `config.yaml`. It is currently all zeros; measure the LiDAR's mount position from the rear axle centre at ground level and fill it in.

## Testing the Driver on Its Own

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 topic hz /livox/lidar     # expect ~10 Hz
```
