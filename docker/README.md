# Docker

This folder contains everything related to docker, like dependency list files, scripts for building and automating build processes, and dockerfile/compose files

### Running Instructions

**NOTE:** make sure to run `xhost +` in your terminal before starting this process

go to the root of the repository in your bash terminal and run `code .` to open vscode. When vscode loads, you should just have to press `ctrl+shift+p` to open vscode command palette, and then type `rebuild and reopen in container` and press enter. Once the container is done building and vscode says `Dev Container: f1tenth` in the bottom left corner, open up a new bash terminal in vscode and you should be good. If all goes well, the simulator should start running and vscode should now be inside the development environment (you can tell by creating a new bash terminal in vscode and seeing that the user is `autonomous`)
