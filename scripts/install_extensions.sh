# grabs all the vscode extensions from the devcontainer.json file and installs them
# using the code cli (NOTE: for use only on the jetson as we have to attach to the
# running container and not create the container using vscode)
cd $FORMULA_HOME/.devcontainer
grep -E '"[a-zA-Z0-9-]+\.[a-zA-Z0-9-]+(@[a-zA-Z0-9-]+)?"' devcontainer.json | sed 's/[",]//g' | xargs -L 1 code --install-extension
