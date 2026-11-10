# Scripts

This folder holds `setup.sh`, the environment bootstrapper. Run it through the
Makefile at the repo root rather than directly:

```bash
make deps
```

It installs ROS 2 Humble (Ubuntu 22.04 only), the system and rosdep
dependencies, CDT, and the Python venv at the workspace root.

Everything else — building, testing, linting and launching — is a Makefile
target, not a script. Run `make help` for the list.

There used to be three more scripts here, and they are gone on purpose:

- `formula_bashrc.sh` defined `rc_build`, `rc_clean`, `rc_source` and `rc_all`,
  plus a hardcoded list of ZED packages to skip. `make build`, `make clean`,
  `make rebuild` and `PACKAGES_IGNORE` replace it. It also resolved the
  workspace root from the caller, which is why `build/`, `install/` and `log/`
  kept landing inside the git tree; the Makefile resolves the root from its own
  location instead.
- `simulator_setup.sh` started the gym bridge in a tmux session against a
  separate `/sim_ws`. The gym is a package in this workspace now, so
  `make run_sim` launches it like anything else.
- `install_extensions.sh` scraped extension ids out of `devcontainer.json` and
  fed them to the `code` CLI. `.vscode/extensions.json` is the supported way to
  recommend extensions, and VS Code prompts for them on open.

If you are about to add a script here, check first whether it should be a
Makefile target. Four places that each knew how to build this workspace is the
problem the Makefile was written to solve.
