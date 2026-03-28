# Ualberta F1Tenth

Welcome to our autonomous remote control car repository. We are developing an autonomy stack for an [RC car](https://f1tenth.org/build.html) as a testing ground for algorithms and design decisions that may be included in our future Autonomous Electric vehicle. If you are just starting out, check out the [Contributing Guidelines](docs/CONTRIBUTING.md) document for more information on how to proceed with helping us develop our system!! Make sure to check out the resources section of the guideline if you are new to ROS or a new UAlberta formula team member as there is information about onboarding and learning resources.

**Important**: In general, the `docs/` folder contains lots of info pertaining to our stack, and this will be where many sources of information get put in the future. If ever you are lost, check there first!

**Note**: As of right now, we are only accepting contributions from UofA students.


First run the camera zed
then the detection node
then run the following command:
    ros2 run tf2_ros static_transform_publisher 0.2 0.0 0.5 0.0 0.0 0.0 base_link zed_camera_link
then run jayden cone_transfomer
then run run waypoint_new
finally pure-pursuit

run the f1tenth bringup stack

5.5 from base link to top of zed
1.5 cm from camera moving to the left to the base_link
