# dashboard

The WebSocket bridge between the stack and the browser dashboard. A browser on the car's network can watch the stack without ROS installed and without DDS set up.

It brings up three things, all driven by `config/dashboard.yaml`:

| Process | Role |
| --- | --- |
| `dashboard_throttles` | A single `component_container` with one lazy `topic_tools` throttle per configured topic. It republishes each one at a fixed rate on `/dashboard/<name>`. |
| `rosbridge_websocket` | Serves `/dashboard/*` over a WebSocket on port 9090, and nothing else. |
| `dashboard_http` | Serves `web/` on port 8000, so the page loads from the car with no internet. |

## Run it

It comes up as part of the normal launch when the profile enables it: `launch.dashboard: true` in `config.yaml` (the car), or the `dashboard` entry in `sim_config.yaml`. To run it on its own:

```bash
ros2 launch dashboard dashboard.launch.py
ros2 launch dashboard dashboard.launch.py port:=9091 http_port:=8001
```

Then open `http://<host>:8000`. In the devcontainer, `<host>` is `localhost`. VS Code forwards ports 8000 and 9090 (check the Ports panel). On the car, `<host>` is the Jetson's address.

## The envelope

Every message is the [rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md) `publish` op, with the message as JSON:

```json
{"op": "publish", "topic": "/dashboard/odom", "msg": {"header": {...}, "pose": {...}, "twist": {...}}}
```

To receive a topic, send:

```json
{"op": "subscribe", "topic": "/dashboard/odom", "type": "nav_msgs/msg/Odometry", "queue_length": 1}
```

`web/index.html` wraps this in a `Bridge` class: `bridge.subscribe(topic, type, callback)`. It reconnects with backoff and re-subscribes after a reconnect. Views should build on that instead of opening their own sockets. The page reads the topic list from `topics.json`, which the launch file generates from `dashboard.yaml`.

## Adding a topic

Add an entry to `config/dashboard.yaml`:

```yaml
- name: lookahead       # served as /dashboard/lookahead
  source: /lookahead_waypoint
  type: geometry_msgs/msg/PointStamped
  rate: 10.0
```

Then add the message package as an `exec_depend` in `package.xml` if it isn't one already, since rosbridge has to import it. Keep rates low, and leave images, point clouds and raw scans out. The Jetson JSON-encodes every message rosbridge sends, so for those, send a summary computed on the car instead.

## Why only `/dashboard/*`

rosbridge's `topics_glob` controls publishing as well as subscribing. If the real topics were exposed, any browser on the network could advertise `/planned_path` or `/drive` and steer the car. Exposing only the throttled copies makes the stack read-only from the browser. A browser that tries to publish to a real topic gets `No match found for topic, cancelling advertisement`. Services and actions are disabled as well (`services_glob`/`actions_glob: []`).

## Why rosbridge

Issue #131 asked for `rosbridge_suite` to be evaluated before writing a custom node. It meets every requirement:
- browsers connect with a plain WebSocket and JSON
- the protocol is a stable envelope
- reconnecting clients don't affect the node
- it's already a dependency of `f1tenth_stack`

The throttles keep the Jetson's share of the work bounded: rosbridge only encodes messages at the configured rates, and the throttles subscribe to nothing until a browser does.

Measured in the devcontainer on an M-series Mac, with a client subscribed to everything while the sim was running: rosbridge used about 1.4% CPU and 64 MB, and the throttle container about 1% CPU and 15 MB. Expect a few times that on the Jetson.
