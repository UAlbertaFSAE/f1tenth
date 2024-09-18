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

We recommend you have the following specs on a computer:

- 16Gb of ram
- 25-50Gb of free storage

**Note:** Using Linux natively or through a dual boot is preferred, but our system should work (albeit with slight hiccups) on Linux, Windows, and Mac, as long as you follow the following instructions carefully and ensure you have everything set up correctly

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

## Software Learning

If you don't have any experience with linux, programming, git, or the ROS framework, or just need a refresher listed below are some great resources that we highly advise each new member to watch/read through to get orientated with all the software we use. We **highly** recommend you be familiar with each of these tools, as it will make onboarding and future development a lot less daunting. Don't feel bad if you need to spend a long time getting familiar with these things, it'll save you (and the leads) much grief in the future.

### Linux

We use the Linux OS to interact with almost all of our software so we highly advise if you don't have experience using a terminal CLI more specifically within Linux then here is a great resource.

- [linux command line for beginners](https://ubuntu.com/tutorials/command-line-for-beginners#1-overview)

### Git

This is a great time to introduce git. If you have never worked with git before, here are some resources to get you started (the first link is an overview video, the other two are supplementary material if you are still confused):

- [Introduction to Git](https://www.youtube.com/watch?v=tRZGeaHPoaw)
- [official git documentation](https://git-scm.com/docs/gittutorial)
- [a nice interactive tutorial for git branching](https://learngitbranching.js.org/)

### Python

Listed below is an introduction to python, and in intro to object orientated programming in python, a concept we use a lot throughout our codebase:

- [Introduction to Python](https://www.codecademy.com/learn/learn-python-3)
- [Introduction to OOP](https://www.youtube.com/watch?v=Ej_02ICOIgs)

### ROS

Understanding the core principles and design patters of ROS will help immensely when developing code for our system:

- [ROS2 concepts and design patterns](https://osrf.github.io/ros2multirobotbook/ros2_design_patterns.html)

and understanding the command line tools will make your life much easier:

- [ROS2 command line interface](https://osrf.github.io/ros2multirobotbook/ros2_cli.html)

If you still feel like you need more introduction, [the construct sim](https://app.theconstruct.ai/courses/) has some free courses that may be of some use.

### Docker

Even though you will likely not have to touch any of the docker configuration during development, it is still good to have a high level understanding of how it works so you understand how our stack is set up better:

- [docker tutorial](https://docker-curriculum.com/)
- [docker docs](https://docs.docker.com/)

## Building the Software Container

Now assuming you are feeling confident enough with the basics of monkeying around in a terminal environment, you can do the following:

1. Clone the repository into a directory in your **Linux** instance (either in the VM, WSL, or natively, depending on what setup steps you followed above. **NOTE:** you should have setup your ssh key on this linux instance as well) with `git clone git@github.com:UAlbertaFSAE/f1tenth.git` and then change into it with `cd f1tenth`
2. Make sure docker is installed and running in the background (either as docker desktop or start it through terminal commands, you can check its running by running `docker --version` in a terminal in your linux instance)
3. Run `xhost +` in a bash terminal in your linux instance to ensure GUI's will work in the container
4. run `code .` to open up vscode inside the f1tenth directory (to ensure you are in the right directory beforehand, run `pwd` and you should see /f1tenth at the end of the output). **Note:** if a new vscode window did not open, open vscode, press `ctrl/cmd + shift + P` and run `Shell Command: Install 'code' command in PATH`, and then restart vscode and re-rerun the previous command
5. Press `ctrl/cmd + shift + P` and search `Dev Containers: (Re-)build and Reopen in Container` and click enter
6. Wait for a new vscode window to open inside the dev container (should say Dev Container in the bottom left corner of vscode).

**NOTE:** if you are on windows using WSL, you will need to run the following command when in the dev container: `export DISPLAY=host.docker.internal:0.0`. This ensures GUI's can be used properly from within the container. You must also do this in the simulator container if you want the simulator GUI (RViz) to open. You can do this by following the instructions in the [sim docs](SIMULATOR.md).

- this is just a temporary fix, we hope to have it automated in the future

If everything is working, you can move on to the actual onboarding task (get used to that setup process, everything but the git clone step will have to happen every time you go to work on the software stack). If you run into any issues feel free to reach out to one of the leads for help!

## Onboarding Task

Once done those tutorials, you will be tasked with building your own [ros2 package](https://docs.ros.org/en/humble/How-To-Guides/Developing-a-ROS-2-Package.html) and writing nodes, publishers, and subscribers. The goal is for you to get more comfortable with the ROS2 framework, as well as to build the skill of finding information and troubleshooting on your own. Lead's will be here to help, but try your best to find what you need online first (there are tons of resources out there that can help you complete this task, you just gotta channel your inner Sherlocke Holmes)

You will want to create your own branch (following the branch naming convention in the [contributing guidelines](CONTRIBUTING.md), with \<modified\> being "onboarding". Make sure you are in the `f1tenth/` directory or a child of it when running this. Run `git branch` to see that you now have the branch `main` and your freshly created branch. The highlighted one is the one your currently on. You can now work to your hearts desire, committing your work as you go with `git add <files to stage>` and `git commit -m "<message attached to commit>"`.

### The Task

You're going to learn ROS2 basics by creating your own ROS2 package and building some publisher and subscriber nodes to communicate information, along with a launch file to spin up your nodes.

#### Step 1: Create your package

Create a package name `onboarding` in the `src/` folder.

**requirements:**

- the package supports either `C++` or `Python`
    - if using `C++`, target `C++17` in your CMakeLists.txt file
- the package needs to have `ackermann_msgs` and `rclcpp` (or `rclpy` if using python) as dependencies

The goal is to have a working ROS2 package that can contain code for building a part of a robotic application. You will get experience with specifying dependencies for your package, which is code written by someone else (or maybe even you) that your code will require to run properly.

#### Step 2: Create a publisher node

Create a node called `odom_publisher` for publishing [odometry data](https://en.wikipedia.org/wiki/Odometry), which you will later subscribe to and manipulate. You will write this node in whatever language you specified your package to be in.

**requirements:**

- your publisher listens to two ROS parameters `v` and `d`.
- your publisher publishes an `AckermannDriveStamped` message to a topic named `drive` with the following fields set:
    - `speed` equal to the `v` parameter
    - `steering_angle` equal to the `d` parameter
    - `stamp` equal to the current time (hint: you can get this using a method on the parent Node class)
- your node publishes data at a rate of 1KHz.

If you want to go above and beyond, try setting floating point ranges on the two parameters `v` and `d`.

You can test your node by building your package with `colcon build --packages-select onboarding --symlink-install` (and then source the newly built code using `source install/setup.bash`) and running your node with `ros2 run onboarding odom_publisher`. Try updating the parameters `v` and `d` through the command line (find how in ROS2 Humble documentation) and echoing the `drive` topic to see changes. Try running `ros2 topic hz /drive` in a separate terminal to see if your data is being published near the 1KHz requirement.

The goal is to get you familiar with the conventions for writing ROS2 nodes and how to implement a publisher, which is used A LOT in robotics projects.

#### Step 3: Create a publisher/subscriber node

Create a node called `odom_relay` for subscribing to the odometry data that `odom_publisher` is publishing, manipulating that data, and publishing this new data. You will create this node in the same language you wrote your previous one in.

**requirements:**

- your subscriber node subscribes to the `drive` topic
- in the subscriber callback, take the speed and steering angle from the incoming message and multiply both by 3, then publish these new values via another `AckermannDriveStamped` message to a topic named `drive_relay`

You can test your node by spinning up the publisher you wrote previously, and then spinning up this new node and echoing the topics `drive` and `drive_relay`. If you did it right, `drive_relay` should have the speed and steering angles 3 times as large as `drive`.

- Make sure to build your package and re-source the ROS2 overlay again before testing!

The goal of this step is to get you experience writing a ROS2 node that has more than one component to it, which in this case is publishing **and** subscribing. This is another very common paradigm, where you have a node that is obtaining some data, manipulating it, and then sending it elsewhere.

#### Step 4: build a launch file to spin up your nodes

launch files are a tool for spinning up parts of the system that require each other, all at once. An example would be the safety node, which requires a few other nodes to be running with it for it to work.

**requirements:**

- create a launch file in a directory called `launch` just inside your package directory
- add the launch folder to your package configuration (CMakeLists.txt if using C++, setup.py if using Python)
    - the [ros2 docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html) on launch files may be of use here
- spin up your `odom_publisher` and your `odom_relay` nodes inside this launch file in such a way that you can set the `v` and `d` parameters in the command line when running the `ros2 launch ...` command given below

If you did it right, you should be able to build your package again and launch your nodes using `ros2 launch onboarding <launch_file> v:=<value1> d:=<value2>`. In another terminal, use the following commands (one at a time) to see if everything is working as expected

```
ros2 node list
ros2 node info <publisher node>
ros2 node info <publisher/subscriber node>
ros2 topic list
ros2 topic info drive
ros2 topic echo drive
ros2 topic info drive_manipulated
ros2 topic echo drive_manipulated
```

The goal is to get you used to writing launch files, as these are used everywhere to spin up specific parts of a robotic system, as well as for setting configuration details that nodes may need to run properly.

**Note**: if your simulator is running at at the same time as launching, you may see the car start moving. This is because the simulator is listening to the `drive` topic, and we are publishing to that with the `odom_publisher` node. If you don't want the car to move, specify a namespace for your launch file node actions. More info is in the docs linked above.

#### Step 5: Submitting your onboarding task

When you are done the onboarding task, you can push your branch to the remote repository on github and make a pull request! Get a co-lead to review it, and if all looks good, you are done the onboarding task! Welcome aboard :)

### Resources

Here is a list of resources that may prove to be helpful during this process:

- [ros2 humble documentation (super good)](https://docs.ros.org/en/humble/index.html)

## Github (Project Management)

Now that you've completed onboarding, you can head to our [project board](https://github.com/orgs/UAlbertaFSAE/projects/1) to see what tasks need to be worked on. There are various views you can play around with to see ToDo's per sub-component, as well as to see the workload of other team members and your own workload. Feel free to assign yourself to a task you are interested in (if a task with someone already assigned interests you, reach out to that person to see if you can collaborate on it).
