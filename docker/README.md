# Docker

This folder contains everything related to docker, like dependency list files, scripts for building and automating build processes, and dockerfile/compose files

### Running Instructions

**NOTE:** make sure to run `xhost +` in your terminal before starting this process

go to the root of the repository in your bash terminal and run `code .` to open vscode. When vscode loads, you should just have to press `ctrl+shift+p` to open vscode command palette, and then type `rebuild and reopen in container` and press enter. Once the container is done building and vscode says `Dev Container: f1tenth` in the bottom left corner, open up a new bash terminal in vscode and you should be good (you can tell you are inside the development environment by seeing that the user is `autonomous`).

There is no separate simulator image or service. `f1tenth_gym` is a package in this workspace, so it builds into the dev image with everything else and `make run_sim` launches it.
