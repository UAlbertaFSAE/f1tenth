# Ualberta F1Tenth

Welcome to our autonomous remote control car repository. We are developing an autonomy stack for an [RC car](https://f1tenth.org/build.html) as a testing ground for algorithms and design decisions that may be included in our future Autonomous Electric vehicle. If you are just starting out, check out the [Contributing Guidelines](docs/CONTRIBUTING.md) document for more information on how to proceed with helping us develop our system!! Make sure to check out the resources section of the guideline if you are new to ROS or a new UAlberta formula team member as there is information about onboarding and learning resources.

**Important**: In general, the `docs/` folder contains lots of info pertaining to our stack, and this will be where many sources of information get put in the future. If ever you are lost, check there first!

**Note**: As of right now, we are only accepting contributions from UofA students.

For building the package please use: colcon build --packages-ignore livox_ros_driver2 --parallel-workers 1

cd fsae_f1tenth_ws/ && source install/setup.bash
First run the camera zed: ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
then the detection node: ros2 launch detection_camera camera_detection.launch.py
then run the following command:ros2
    ros2 run tf2_ros static_transform_publisher -0.05 -0.15 0.40 0.0 0.0 0.0 base_link zed_camera_link
    ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map base_link
then run cone_transfomer: ros2 run cone_transformer cone_transformer
then run waypoint_new: ros2 launch waypoint_new new_triangulator.launch.py or ros2 run waypoint_triangulation triangulator 
finally pure-pursuit: ros2 run pure_pursuit pure_pursuit

run the f1tenth bringup stack: ros2 launch f1tenth_stack bringup_launch.py

5.5 from base link to top of zed
1.5 cm from camera moving to the left to the base_link
0.5cm forward

x = 0.5cm = 0.05m
y = 1.5cm = 0.15m
z = -4.0cm = -0.4m
We swap the signs.

Even this may be required:
 ros2 run tf2_ros static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map base_link


Notes:
  ("min_lookahead", 0.8);    // Can be ignored.
  ("max_lookahead", 4.0);    // If this is too far, the smoothing is too much, and hence a very small correction will also not work.
  ("lookahead_ratio", 4.0);  // This was I think set to 8.0, which is the ratio between distance and speed.
        Higher lookahead ratio
            Larger lookahead distance at a given speed
            Smoother steering
            Better at high speed
            May cut corners or track less precisely
        Lower lookahead ratio
            Shorter lookahead distance
            Tighter path tracking
            Faster reactions
            Can become twitchy or oscillatory
  ("K_p", 0.30);    
        Lower value will correct less - smoother
        higher value will correct less - aggressive
  ("steering_limit", 25.0);     - Limit of vesc (which is 25.0)
  ("velocity_percentage", 1.0);  // 0.6 default
        Keep it as 1.0, going lower jitters the vechiles. (this is directly proportional to the currently set value in vesc.)


    Also vesc.yaml has a parameter of steering_angle which has a 0.543 offset, we set it to 0.5 so that it goes straight.
    no offset causes a problem to detection.
