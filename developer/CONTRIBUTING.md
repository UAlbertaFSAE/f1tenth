# Contributing to the Autonomous Repository
For contributing to this repository, we **will not allow direct pushes to the main branch**, all code will be merged through pull requests where each pull request requires at least **one approval from a software lead**

## Steps to Merging Code
1. Create a branch for your code
```
git checkout -b <username>/branch-name
```
**Note:** for linking with linear, click on your issue and hit the button in the top right(or press `ctrl/cmd + shift + .` to copy the name) and paste that in after the `-b` as the branch name

2. Make your code changes and commit them
<br>
Please aim to keep your **PR's as small as possible**. We understand sometimes this isn't possible but please try to keep them manageable for reviewers

Ensure to commit changes in your vscode integrated terminal(`ctrl/cmd + j` to open)
```
git commit -am "<commit message>"
```

3. Merge main into your branch and resolve any conflicts
**Note:** Please use an external terminal for any of the following git commands: `fetch, pull, push`, this is because the dev containers cannot access the remote repository
```
git fetch origin && git merge main
```

4. Push your branch:
Again please do this in an external terminal
```
git push
```

5. Open Pull Request:
<br>
a) Go to https://github.com/UAlbertaFSAE/Autonomous-Driving/pulls and press "open pull request"
<br>
b) Give your PR a descriptive name and add a description for all your changes
<br>
c) On the right panel, look for "reviewers" and add one of the
leads to review your code
<br>
d) Once everything looks good, press "Create Pull Request"

6. Wait for a lead to review your code, if you get feedback,
address the feedback and push the relevant changes to the PR

7. Once approved from a lead press "merge" to merge your code into
the codebase
<br>

**Note: we will not merge the code for you**, you are responsible for merging the code when it is approved
**Also Note:** you should **always** make sure to merge main into your branch before merging, this is the command from **step 3**
