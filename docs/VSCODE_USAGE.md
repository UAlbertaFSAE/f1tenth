# VSCode Usage Guide
We use VSCode and various extensions to create an integrated development environment that contains all the tools you will need to develop code for our robot.

### Development Container
upon first installing vscode, you should've installed the vscode devcontainer extension. This allows vscode to attach it's GUI client to the development container we use and install any necessary extensions, allowing any code changes to be made directly in the container.

##### Steps
1. open up a new terminal in your linux instance and change into the `f1tenth/` repository (wherever it is on your system), and then run `code .` to open up vscode in the `f1tenth/` directory.
2. either:
    1. click the pop-up in the bottom right that says something like "open up in devcontainer" to start the dev container and attach vscode to it
    2. press `ctrl + shift + p` and type "rebuild", which should give you the option to run the devcontainer rebuild and reopen container command which will also start the devcontainer and attach vscode to it
3. press on the "open log" popup in the bottom right to see the container build/run process. Once new logs stop being produced you can (probably) safely open up a new terminal in the dev container by pressing the plus button at the top right of the vscode terminal.
    - you are in the devcontainer if the user in the terminal is `autonomous` and you are in the `/f1tenth` directory

**Note**: you may have to wait a sec for all the extensions to fully load in

You should be good to develop at this point.

### Code Linting and Formatting
We have linters and code formatters set up in vscode so that you will get annotations when your code isn't following some of the [style guidelines](CONTRIBUTING.md#code-style). Try to catch these errors while you are developing, as we have linter checks in our CI pipeline that will block your PR from being merged into main until they pass! Formatter checks are also included, and are really easy to stay on top of as you just need to explicitly save your code files (with `ctrl + s`) and vscode will format your files for you.

To see what an error/warning is and what tool is throwing it, just hover the annotation squiggle in vscode, or go to the problems pane in the terminal window. This should show you what checks are causing the error, and potential fixes. Sometimes fixes can be applied automatically by pressing `quick fix`, if a fix is available.

##### C++
We use `clang-tidy` for linting and `clang-format` for code formatting. Since the clangd language server requires build commands for processing your c++ files, you need to run the following for clang-tidy linting to work on your c++ package:
```bash
colcon build --packages-select <pkg-name> --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=On"
```

These tools will enforce the c++ style guides, and also ensure developers are following c++ best practies. If there is ever any lint checks that you think are not useful, don't hesitate to ask a lead about removing it.

If you need to ignore linting on a line for whatever reason (besides just to get CI checks to pass, try not to do that), put `// NOLINT` at the end of it and clang-tidy will ignore it. If you need to ignore an entire package (say you ported some open-source ROS package), you can throw a `.clang-tidy` file at the root of the package with the line `Checks: '-*'` for c++ packages. clang-tidy will ignore any files at the same level or lower than this file.

see the [clangd vscode extension](https://marketplace.visualstudio.com/items?itemName=llvm-vs-code-extensions.vscode-clangd) on the vscode extension marketplace for more information on how to use the extension to the fullest.

##### Python
We use `ruff` for linting and formatting, and `mypy` for static type checking. We also have a docstring generator set up, so if you type `"""` just under a function declaration, you can press enter and a docstring template will be generated.

These tools will enforce the pep8 style guidelines and ensure we have safe python code, since our system is fairly safety-critical. Again, if there is ever any lint checks that you think are not useful, don't hesitate to ask a lead about removing it.

Again, if you ever need to skip some linter check, use `# noqa` at the end of the line. If you need to skip linting on whole packages (say you ported someone's open source ROS package), you can add exclude paths in the `ruff.exclude` and `mypy-type-checker.ignorePatterns` settings in the `.vscode/settings.json` file to stop errors showing in vscode. Add them into the `FILTER_REGEX_EXCLUDE` regex string in the `.mega-linter.yml` file to ignore them in the CI linting check.
- you can also add them to the exclude section for each tool in the `pyproject.toml` file if you want, for completeness sake

here is the [mypy](https://mypy.readthedocs.io/en/stable/index.html) and [ruff](https://docs.astral.sh/ruff/) documentation if you want to learn more. The mypy vscode extension is [here](https://marketplace.visualstudio.com/items?itemName=ms-python.mypy-type-checker) and the ruff vscode extension is [here](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff)

### ROS extension
TODO

### Debugging
TODO
