# How to Contribute

Thanks for your interest in contributing to the UAlbertaFSAE software team! We very much appreciate the help, as building an autonomous vehicle is no easy task.

This document will help you with contributing issues, bug fixes, and new features to the f1tenth repository. You will also find guidelines for code style below, which are important for keeping the codebase clean and consistent. Following these guidelines helps to communicate that you respect the time of the developers managing and developing this open source project. In return, they should reciprocate that respect in addressing your issue, assessing changes, and helping you finalize your pull requests. Please be aware all contributions are subject to [the MIT license](../LICENSE).

## Issues

Issues on GitHub are the process by which you can report bugs or request new features. If these are new to you, check out this [overview](https://docs.github.com/en/issues/tracking-your-work-with-issues/about-issues). We have a few main types of issues:
1. bug
    - anything that is broken
2. feature
    - some new functionality that didn't exist before
3. performance
    - optimization type things, like increasing speed of processing or accuracy of something
4. documentation
    - improvements or additions to any of the documentation, like contributing guidelines and onboarding

They are listed in order of priority, but of course there are always outstanding circumstances. We have templates for each type of issue that will ensure all the necessary information is included for work to begin on it.

We use [github projects](https://docs.github.com/en/issues/planning-and-tracking-with-projects/learning-about-projects/about-projects) to organize and prioritize issues, so this may be a good place to check out what issues are most pressing.

If you want to work on an issue, please assign yourself to it so others can know who is working on an issue. You can leave updates on an issue to let others know your progress, and so that others can help if needed. If you are no longer working on an issue, please unassign yourself and leave a comment explaining why you are no longer working on it (it is fine if you don't have the time or get in over your head!).

## Pull Requests

Pull requests are the process by which you can submit modifications to the codebase. If you are not yet familiar with them, have a look [here](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests). Generally, pull requests are made to address an [issue](#issues).

pull requests should be reviewed by a lead or a member who is well acquanted with the part of the system you are trying to update. They will be merged into the codebase if they are approved. Please keep pull requests to a single feature or change. If you have multiple unrelated changes, please submit multiple pull requests.

Before submitting a pull request, please make sure that you have followed the [code style](#code-style) guidelines below. Ensure that any changes come with relevant updates to the documentation in the docstrings and Markdown files. Also, make sure that changes to the code have been tested, as we want to keep the `main` branch in a working state at all times.

Before getting started, first make sure you have latest code from the main branch by running the following command:
```bash
git pull origin main
```

Once you have done that you can create a new branch for your changes. If you are addressing a specific issue, your branch name should be `<issue number>-<name>/<modified>`. If you are not addressing a specific issue (generally an issue should be made about what needs to be changed before changes should happen), your branch name should be `<name>/<modified>`. `<name>` is your first initial and last name with no space between, and `<modified>` should be a brief 1-2 word description of what you are modifying such as `object-detector-node`, `plugin.xml`, or `docs`. To create a new branch run the following command while **on the main branch**:
```bash
git switch -b <branch-name>
```

Once you have created your branch you can make your changes in this branch. Changes should be mostly if not all complete before making a pull request, and if not complete make sure to attach a message as to why. Ensure that your commit messages and the title for merge request are descriptive of the changes you have made. Don't overthink your commit messages, they will be deleted when the branch is merged into the main branch. An example of a good commit message would be, `Add MPC controller to control package`.

When ready to make the pull request, ensure that you merge the main branch into your branch with
```bash
git fetch origin && git merge main
```
This will ensure whatever changes to the main branch that have occurred while you worked on your own branch can be combined into your changes.

You can push your changes to the remote repository. This will create a new branch on the remote repository with the same name as your local branch. You can then create a pull request on GitHub to merge your branch into the main branch. To push your branch to GitHub run the following command:
```bash
git push --set-upstream origin <branch-name>
```

Make sure to follow the pull request template when creating a pull request. If you are addressing a specific issue, please make sure that issue is linked in the pull request. Add a reviewer to your pull request, once it has been reviewed and approved, check one last time to make sure your branch has the most up to date main (as changes might have occurred while you were addressing feedback on the pull request), and then you can merge your code
- it is your job to click the merge button, we wouldn't want to rob you of that satisfaction ;) (but also it is your responsibility to ensure everything works before pressing that button)

## Documentation

Adding and updating documentation is an important part of developing our software. Good documentation allows others (sometimes even yourself) to understand what a piece of code does and how to use it. It is important to update relevant documentation when making changes, so that the documentation stays up to date. Our documentation mainly consists of 4 parts:
- Docstrings which describe the function of individual functions and classes. (See [Code Style](#code-style) for more details)
- README files in each package which explain the function and usage of the package.
- Markdown files in the `docs/` folder which explain other parts of the repository that are not directly related to a specific package, like the [simulator docs](SIMULATOR.md)

**NOTE**: we encourage contributors to add copyright notices as headers of every file containing code, as it is a means of (trying) to ensure you get credit for the code you contribute, and that the licensing information of the code can tag along if it ventures off into another project! You can just put the following at the top of the file:
`Copyright (c) YYYY, FirstName LastName. Subject to the MIT license.`
where YYYY indicates year.

## Code style

**IMPORTANT**: We try to follow the [ROS2 code style](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html) to ensure consistency and best practices. Within this they follow [Google C++ style](https://google.github.io/styleguide/cppguide.html#The__define_Guard) and [Python PEP8 style](https://peps.python.org/pep-0008/) for C++ and Python files respectively. It may be useful to have these handy while you are coding! Of course there will be times where deviation from this guide may be required, and that's okay. Just try your best to follow it! We will *hopefully* have linters and auto formatting set up soon to make your life easier while coding.
- the philosophy is to have fast commits and pushes to allow eyes on code, so formatting and linting won't be enforced until pull requests are made, at which point linter tests must be passed in order to merge code.

For all programming languages we use, please follow these common guidelines on top of the ROS guidelines:
- Have descriptive (ideally short) variable, function, and class names.
- All classes and functions should be documented with docstrings.
- Add comments describing what code is trying to accomplish at critical points, this is helpful both for yourself and for team members reviewing your code
- All class members should be private unless they need to be accessed outside of the class.
- License and copyright statements are needed in code files (source and headers).
    - see bottom of [documentation](#documentation) notes above
- Try to follow the advice of the linter.

**NOTE:** Try to include as much [type-hinting](https://mypy.readthedocs.io/en/stable/cheat_sheet_py3.html) as possible when writing python code, as we will also be trying to incorporate mypy static analysis at some point.

### File/Folder names:
snake_case:
- Python files
- C++ files
- XML files
- YAML files
- folders of any kind
- Markdown (**Note**: should be all caps)

CameCase:
- CMake files
- ROS Msg files
- ROS Action files
- ROS Service files

spinal-case:
- media files i.e. images, videos, etc.

### Configuration files

For most configuration files if they are already present elsewhere in the repository, try follow the same style as the other files. VSCode should have auto-formatting for most of these files, so make sure to use this if available to ensure consistency. If you are unsure about how to set up a configuration file, you can ask for help or just submit your pull request with your best guess and the file can be updated during review.

## Resources
For new members to the UAlberta Formula Student Racing Autonomous Subteam, or contributors new to ROS and programming in general, check out the [onboarding](ONBOARDING.md) document for steps to set up your development environment and learn all about the tools we use.

### General Resources
- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [ROS2 Code Guidelines](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- [C++ reference page](https://cplusplus.com/)
- [Google C++ style guide](https://google.github.io/styleguide/cppguide.html#The__define_Guard)
- [Python docs](https://www.python.org/doc/)
- [Python PEP8 style guide](https://peps.python.org/pep-0008/#introduction)
- [mypy python type-hints cheat sheet](https://mypy.readthedocs.io/en/stable/cheat_sheet_py3.html)
- [github docs](https://docs.github.com/en)
- [git docs](https://git-scm.com/doc)
- [f1tenth autonomous RC car resources](https://f1tenth.org/index.html)
