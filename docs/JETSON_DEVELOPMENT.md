# NVIDIA Jetson Development Guide
Our onboard computer for the RC car is currently an [NVIDIA Jetson Xavier Developer Kit](https://developer.nvidia.com/embedded/jetson-developer-kits). It is running NVIDIA Jetpack 5.1.1 (I think), which is NVIDIA's OS-Library-SDK thing for their Jetson series of embedded systems.

Much of the perception system development will have to take place directly on the jetson, as Stereolabs ZED SDK/ROS2 wrapper requires an nvidia GPU and the camera stereo camera connected. I imagine some other development will also have to happend directly on it as well. This document serves as a guide for doing so.

### Connection to Jetson
You can connect to our jetson either through [ssh](NETWORK_SETUP.md) or by plugging in a keyboard/mouse and monitor. Once connected to the jetson, you can start running our software stack. We have everything dockerized on it (much like our devcontainer setup for local development). This is to (hopefully) ensure that the code we develop locally runs fine on the jetson as well. The only difference is that you will not be able to connect to a devcontainer in the same way as you would with local development.

### Software Startup
The process is as follows:
1. open up a terminal on the jetson (either through ssh (vscode or pure terminal) or in person on monitor)
2. run `cd ~/fsd/f1tenth` to get to the repository on the jetson
3. ensure you are on the main branch (if not, stash changes of the branch that is currently checked out with `git stash` and then switch into main branch)
4. once on main, run `git pull` to ensure everything is up to date. You may also need to run `git submodule update --init --recursive` to ensure the submodules (ZED ROS2 wrapper stuff) is up to date.
5. run `docker compose -f docker/compose.jetson.yml up -d` to spin up the autonomy development container and the hardware container.
    - if using vscode remote development extension, you can now do `ctrl+shift+p` and type attach to container, then click the option that says "attach to running container". Now pick whatever container you are wanting to develop in.
6. if in the jetson container, run the following commands in the specified order:
```bash
bash scripts/install_extensions.sh
rc_home
apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
This will ensure vscode extensions are installed (if attached to container via vscode), and all ROS dependencies for our packages are installed.
7. if needing to rebuild packages, run `rc_build` to rebuild everything. Look at the [vscode usage guide](VSCODE_USAGE.md) for tips on how to build only specific packages. Then, source the ROS packages using `source install/setup.bash`

You should be good to run whatever you need at this point.

**Important**: For best experience, you should try to plan development times so that we don't have more than one person connected to the jetson at one time, as development will likely be happening on separate branches and git can't handle that.
- be courteous, don't accidentally destroy someone elses code pls
