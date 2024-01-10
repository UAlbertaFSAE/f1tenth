# Docker
This folder contains everything related to docker, like dependency list files, scripts for building and automating build processes, and dockerfile/compose files

### Running Instructions
If you are using devcontainers in vscode, you should just have to press `ctrl+shift+p` to open vscode command palette, and then type `rebuild and reopen in container` and press enter. Once the container is done building and vscode says `Dev Container: f1tenth` in the bottom left corner, open up a new bash terminal in vscode (the user in the terminal should be `autonomous`), and then run `bash docker/setup.bash`. This will just do some final container setup things like sourcing ROS stuff in the bashrc and downloading any required dependencies from the ROS packages in our dev workspace.
