## Before You Start

### Formula Student Competition

Besides being a place to learn new things and hone your industry-applicable skills, the Formula Student Club competes at the [FSAE](https://www.sae.org/attend/student-events/formula-sae-knowledge/about) competition in Michigan. That North American competition is solely for **Driver-only** vehicles. Our expertise comes into play during the [Formula Student](https://www.imeche.org/events/formula-student) competition in Europe. Currently, only the European competitions have competition tasks for **Driverless** teams, so our goal for this year is to get a baseline autonomous driving stack set up so that in the next few years we can set our sights on those European challenges.

We will be hoping to achieve this goal by focusing on building an autonomous RC car as an MVP, based on the f1tenth car and the learning material to go along with it.

### Formula Student Handbook

Everything you need to know about the competition is in the [formula student handbook](<https://www.imeche.org/docs/default-source/1-oscar/formula-student/2023/rules/fsuk-2023-rules---v1-2-(released-version---mar-23).pdf?sfvrsn=2>). The autonomous related tasks are handy to read about to get a better idea of what our future goals will require. important sections:

- **Fully Driverless:(very important)** **T14, T15, IN6**
- **Events(important):** D2.5, D2.6, D2.7, D2.8, D4.3, D4.5, D5.3, D5.5, **D8**
- **Driverless Related:** A2.2.3, A6.7.4, T1.4.2, T2.6.1, T6.1.4, T11.9.1, T13.5.1, CV1.2.2, CV1.2.3, CV1.2.4, CV1.6.6, EV4.11.3, EV4.11.5, IN1.1.1, S3.4.4
  - **Note:** DV stands for driverless vehicle

### Our Software Stack

- ROS2 for middleware between sensors and the compute engine
- C++ and Python are the main programming languages. C++ will likely be used in all scenarios requiring performant software, and Python can be used for more high-level computation
- Linux-based Docker containers for running all of our code. If you are not familiar with Docker containers, they are basically light-weight virtual machines providing dependency isolation and consistency across machines. If you want to learn more, you can check out their [tutorials](https://docs.docker.com/get-started/overview/)

##### System Requirements

We recommend you have the following:

- A computer with at least 8Gb of ram (16Gb preferred), and 50-100Gb of free storage
  - Using Linux natively or through a dual boot is preferred, but our system should work (albeit with slight hiccups) on Linux, Windows, and Mac, as long as you follow the following instructions carefully and ensure you have everything set up correctly
  - **NOTE**: Mac support is iffy due to arm vs. x86 conflicts in docker containers. You can try, but having either a native linux system or a windows system with WSL2 is the best option (even if you have to remote desktop in)

## Software Setup

The prerequisite software setup process varies per operating system, so make sure you follow the right one!

**NOTE:** you may run into various issues and bugs along the way, do not hesitate to reach out to one of the co-leads for help troubleshooting, and then we can add the solution steps into this document to streamline the process for future new members

### Windows

If you are using windows you can use WSL2 instead of a virtual machine (WSL2 is essentially a lightweight linux kernel accessible inside your windows system). Install Windows Subsystem for Linux (WSL) from the microsoft store. If you already have it installed, run `wsl --update` in powershell to ensure you have the most up to date version. With WSL installed, you can follow the instructions [here](https://linuxconfig.org/ubuntu-22-04-on-wsl-windows-subsystem-for-linux) to install Ubuntu 22.04.X. Once you've done that, run `sudo apt update` and `sudo apt full-upgrade` inside of an Ubuntu bash shell (started from WSL) to make sure everything is up to date.

Now that you have WSL2 and Ubuntu setup, it's time for docker setup. This part should be relatively easy as the docker-desktop install should handle a lot of the nonsense, so just follow [this tutorial](https://docs.docker.com/desktop/install/windows-install/) for getting docker-desktop installed. Once you have it installed, open Docker Desktop and go to Settings > Resources > WSL Integration and enable WSL integration for Ubuntu, then restart Docker Desktop.

Finally, we need an X server to allow you to utilize GUI applications from within the Docker container. Install x-server for Windows from [here](https://sourceforge.net/projects/vcxsrv/). Once this is complete search “XLaunch” in the search bar and run it with the default options. You will not see anything after this, it is running in the background. Make sure you only launch one instance.
**NOTE**: you will need to make sure you have this running any time you want to use any GUI's in the containers

### Mac

The simplest virtual machine will be [UTM](https://mac.getutm.app/) (VirtualBox won't work with ARM-based systems, so if you aren't sure just go with UTM). After downloading and installing your virtual machine, you will need to download an Ubuntu ISO file and set it up following the instructions [here](https://docs.getutm.app/guides/ubuntu/) (**Note:** make sure to click "enable opengl and hardware acceleration")

- If your VM hangs on a black screen when restarting at the end, go to the top right and click on the drive icon > eject and then click the backwards play button in the top left to restart the VM. It should hopefully work after this.

Once you have your VM set up, install docker **inside** the virtual machine:

- Installation Guide: https://docs.docker.com/desktop/install/ubuntu/#install-docker-desktop
- If you have a M1/M2 mac, then you will need the ARM installation of docker, which currently does not support the docker desktop gui, so you will be stuck with just the docker engine, which will still work fine. Please follow [this](https://www.docker.com/blog/getting-started-with-docker-for-arm-on-linux/) tutorial to download and install the ARM version if needed

### Linux

If your running linux natively, or have a dual boot, this part should be easy enough. All you need to do is install the [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) (or [Docker Desktop](https://docs.docker.com/desktop/install/ubuntu/) if you prefer working with GUI's for handling docker things).

### Ensure Docker is Working

Run the following command in a bash terminal: `docker run hello-world`. If you see "Hello from Docker!" you're good to go. If you get a permission error, follow the post-install instructions [here](https://docs.docker.com/engine/install/linux-postinstall/) to make sure you can run docker as a normal user.

If you still do not see this after following the post-install instructions, and you are using native Linux, you may need to run `sudo systemctl start docker`.

**NOTE:** If using native Linux or a VM on Mac, you will need to be running X11 or install Xwayland if you wish to use wayland (when using linux or an ubuntu VM if on Mac) in order to run the necessary graphical applications from the docker container. Before running graphical applications from the docker container, run `xhost +` from your host system (or virtual machine) to allow graphical applications from the container to connect to the X server on your host system.

### VSCode Install and Setup

- If using Windows:
  1.  install vscode from [here](https://code.visualstudio.com/download)
  2.  install the WSL extension in vscode
- If using native Linux or VM Linux on Mac:

  1.  Download the `.deb` file
      - https://code.visualstudio.com/download (**Note:** again if you have an M1/M2 mac, then you will need to download the **_Arm64_** `.deb` file)
  2.  `cd` into the folder where the `.deb` file downloaded and run `sudo apt install ./code_<vscode-version>_arm64_.deb`

  **Important**: After installing vscode, install the Dev Containers extension in vscode

### SSH Setup

You will need to set up an SSH key to authenticate your computer with Github so you can clone or push code to our repository from your computer. Open up a linux terminal (For MacOS do this in your linux VM, in windows with WSL installed search “Ubuntu” in your search bar) and run `ssh-keygen`. You can hit enter for all the questions. Now print out your public ssh key with `cat ~/.ssh/id_rsa.pub` and copy the output.

Log into github and click on your profile icon in the top right corner > click settings > click SSH and GPG keys > click add new SSH key. Add an informative title such as "Ubuntu VM" or "WSL" (just giving information on where the key was generated basically, if you ever end up with more keys in the future), and then paste the public key you copied into the necessary area. You should be all set up for cloning and pushing code :)

## Onboarding

Now that you hopefully have everything set up (and again, if we missed any important steps in this document let us know) it's time for some fun!

**NOTE**: This onboarding task is catered towards those with basic understanding of the Linux bash terminal and Python (or C++ if you so please). If you are unfamiliar with either, check out the free Linux for Robotics and Python3 for Robotics classes on [The Construct](https://www.theconstructsim.com/robotigniteacademy_learnros/ros-courses-library/) before beginning.

Now assuming you are feeling confident enough with the basics of monkeying around in a terminal environment, you can do the following:

- Ensure you have Github setup under your UAlberta email (if you already have Github, you can just add your UAlberta address to your account)
- Give your UAlberta email address to one of the autonomous leads and we will grant you access to the RC car repository
  - Once you have access, you should be able to view this repository: https://github.com/UAlbertaFSAE/f1tenth

Once you have access to the f1tenth repository, follow the next steps:

1. Run `xhost +` in a bash terminal in your linux instance to ensure GUI's will work in the container
2. Make sure docker is installed and running in the background (either as docker desktop start it through terminal commands, you can check by running `docker --version` in a terminal in your linux instance)
3. Clone the repository into a directory in your **Linux** instance (either in the VM, WSL, or natively, depending on what setup steps you followed above. **NOTE:** you should have setup your ssh key on this linux instance as well)

```
git clone git@github.com:UAlbertaFSAE/f1tenth.git
```

4. Change into that directory with `cd f1tenth` and then run `code .` . This will open up a vscode instance in the Onboarding folder. **Note:** if a new vscode window did not open, open vscode, press `ctrl/cmd + shift + P` and run `Shell Command: Install 'code' command in PATH`, and then restart vscode and re-rerun the previous command

5. Press `ctrl/cmd + shift + P` and search `Dev Containers: (Re-)build and Reopen in Container` and click enter
6. Wait for a new vscode window to open inside the dev container as well as an RViz window to pop up with a little car and track in it. If these both happen, then you are good to go. If you don't see the RViz window, then you likely forgot to prep your xhost stuff from earlier (or something more sinister is going on). Nonetheless, if things are not working reach out to one of the co-leads for help

If everything is working, you can move on to the actual tasks (get used to that setup process, everything but the git clone step will have to happen every time you go to work on the software stack).

<!-- 6. Once in the container, run the following commands ***inside the container*** (press `ctrl/cmd + j` to open integrated terminal) to ensure the container is working properly:
	- If on Windows using WSL, run the following command before the next ones: `echo 'export DISPLAY=host.docker.internal:0.0' >> ~/.bashrc`
**Note:** you may need to run `sudo apt-get update` beforehand if the rviz2 installation fails

You should see a window open and not run into any errors. If you get any errors, let us know. If no errors, Congrats on setting up your ROS2 environment 🎉 -->

### ROS2 Learning

This is a great time to introduce git. You have a local instance of our repository on your computer, and so far you have probably been on the main branch. You will learn quickly to never be on the main branch, bad things happen often if your not careful. If you have never worked with git before, here are some resources to get you started:

- [official git documentation](https://git-scm.com/docs/gittutorial)
- [a nice interactive tutorial for git branching](https://learngitbranching.js.org/)

**NOTE:** all git commands should be run from a terminal **outside** of the devcontainer i.e. in another bash terminal instance (make sure the user in the terminal is your host computer user, not `autonomous`)

You will want to create your own branch by running `git switch -c <your initials>/onboarding_lab1`. Run `git branch` to see that you now have the branch `main` and your freshly created branch. The highlighted one is the one your currently on. You can now work to your hearts desire, committing your work as you go with `git add <files to stage>` and `git commit -m "<message attached to commit>"`. When you are done the lab, you can push your branch to the remote repository on github and make a pull request! Get a co-lead to review it, and if all looks good, you are done the onboarding task! Welcome aboard :)

**NOTE**: you should complete all of the tasks below while in the devcontainer

We will start by learning the core concepts of ROS2:

1. Review ROS Basic Concepts (focus on nodes, interfaces, topics, parameters, and launch for now)
   - https://docs.ros.org/en/humble/Concepts/Basic.html
2. Complete the beginner turtle sim tutorial
   - https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html

Once done those tutorials, you will be tasked with building your own [ros2 package](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html) and writing nodes, publishers, and subscribers, following the requirements in [this f1tenth lab](https://github.com/f1tenth/f1tenth_lab1_template/tree/24f7320ebf1b9a325a2039d4208204da4454d1ab) (start from section 3 and onwards, and switch any foxy instances with humble as we use humble for our ros2 distribution).

**NOTE:** Make sure you are on your own branch when doing this work, and do everything within the onboarding_ws/src folder!

When you are done the lab, you can push your branch to the remote repository on github and make a pull request! Get a co-lead to review it, and if all looks good, you are done the onboarding task! Welcome aboard :)

### Resources

If you are struggling at all during this process, do not sweat it. Software stuff is confusing, especially when you are first starting out. I encourage you to try and seek out information on your own, as that is the single most important skill you can gain as a software engineer. Nonetheless, if you are hard stuck, feel free to reach out to the co-leads for help, we don't bite (well, Ryder might sometimes, so watch out).

Here is a list of resources that may prove to be helpful during this process:

- [tutorial for python+cpp compatible package](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)
- [ros2 humble documentation (super good)](https://docs.ros.org/en/humble/index.html)
- [f1tenth learning modules](https://f1tenth.org/learn.html)

Let us know if you find any other helpful resources, we will add them to this list.

## Linear

Some notes on our project management tool Linear:

- it's an agile-based tool where we define our high level projects and tasks within those projects
  - this is where you will go to see what tasks are assigned for you to complete
- Access will be granted when you finish onboarding