# Ualberta F1Tenth

Welcome to our autonomous remote control car repository. We are developing an autonomy stack for an [RC car](https://f1tenth.org/build.html) as a testing ground for algorithms and design decisions that may be included in our future Autonomous Electric vehicle. If you are just starting out, check out the [Contributing Guidelines](docs/CONTRIBUTING.md) document for more information on how to proceed with helping us develop our system!! Make sure to check out the resources section of the guideline if you are new to ROS or a new UAlberta formula team member as there is information about onboarding and learning resources.

**Important**: In general, the `docs/` folder contains lots of info pertaining to our stack, and this will be where many sources of information get put in the future. If ever you are lost, check there first!

**Note**: As of right now, we are only accepting contributions from UofA students.

## Quick Start

Clone this repo into the `src/` directory of a ROS 2 workspace (e.g. `f1tenth_ws`), then run the Makefile from either the workspace root or `src/`:

```bash
mkdir -p f1tenth_ws/src
git clone git@github.com:UAlbertaFSAE/f1tenth.git f1tenth_ws/src

cd f1tenth_ws        # or: cd f1tenth_ws/src -- both work
make deps            # runs src/scripts/setup.sh: installs ROS 2, system/rosdep deps, Python venv
make build           # colcon build
```

Run `make help` for the full list of targets (`build_all`, `package <name>`, `clean`, `rebuild`, `test`, `run_auto`, `run_sim`).