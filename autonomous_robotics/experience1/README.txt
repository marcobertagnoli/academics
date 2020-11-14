https://elearning.dei.unipd.it/course/view.php?id=655

Esperienza 1

Given a map composed of nxm squares, make the robot go from a Start square to a Goal square while avoiding obstacles. You can see an example of map in Figure 1.

The robot can only go in the north, south, east and west squares, namely it cannot move diagonally. In the first scenario there are no obstacles, so the robot can freely move from one square to another. In the second scenario, the map is fixed so you know where the obstacles are. In the third scenario the map can change, so you cannot assume to know it in advance.  

At the end of the experience...
Objectives:
Make the robot reach the goal or report if it is not reachable
Use the light sensor to identify the map squares
Use the ultrasonic sensor to find obstacles
Create more then one node (at least three: Robot, Navigation, Mapping)
Read the map parameters from a file (.yaml) or the launcher (as args)
Understand the basic ROS concepts (node parameters, use of topics)

Plus:
Use the gyroscope to improve the robot performances
Use the third engine to move the ultrasonic sensor
Object Oriented (OO) approach
Draw the map (e.g. using OpenCV)
Assume that the obstacles are not fixed. That is, if the goal seem not to be reachable, try again paths already tested.

Challenges:
Solve a known a priori map, given the start and goal cells
The map is unknown (you don't know where the obstacles are, you only know the start and goal cells)

What we want:
Code (BitBucket)
Video (IASLabStudenti YouTube Channel)
Report (PDF using BitBucket) containing at least
The algorithm overview
At each step, how you choose the next cell to move in and why
The role of each node you developed
The three nodes interaction
The parameter file/launcher description


Explanation

Preliminary steps:
cd ~/workspace/ros/catkin/src
git clone https://<your_bb_username>@bitbucket.org/iaslab-unipd/nxt.git 
cd .. 
catkin_make --froce-cmake -G"Eclipse CDT4 - Unix Makefiles"
roslaunch nxt_unipd nxt_lab.launch
roslaunch nxt_unipd teleop_keyboard.launch
 
Robot configuration:
nxt_lab.yaml
 -  type: ultrasonic
    frame_id: ultrasonic_link
    name: ultrasonic_sensor
    port: PORT_4
    spread_angle: 0.2
    min_range: 0.01
    max_range: 2.5
    desired_frequency: 5.0

Program launcher:
nxt_lab.launch
  <group ns="base_parameters">
    <param name="r_wheel_joint" value="r_wheel_joint"/>
    <param name="l_wheel_joint" value="l_wheel_joint"/>
    <param name="wheel_radius" value="0.022"/>
    <param name="wheel_basis" value="0.055"/>
    <param name="vel_to_eff" value="0.2"/>
    <param name="l_vel_to_eff" value="0.1"/>
    <param name="r_vel_to_eff" value="0.1"/>
  </group>

Robot controller:
advanced_control.py

Robot teleoperation:
nxt_key.cpp
    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        angular_ = 2.0;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        angular_ = -2.0;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        linear_ = 0.15;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        linear_ = -0.15;
        break;
    }
 
teleop_keyboard.launch
<node pkg="nxt_unipd" type="nxt_teleop_key" name="nxt_teleop_key"  output="screen">
  <param name="scale_linear" value="1" type="double"/>
  <param name="scale_angular" value="1" type="double"/>
</node>
Sensors messages
nxt_msgs/Range Message
Header header
float64 range
float64 range_min
float64 range_max
float64 spread_angle
nxt_msgs/Color Message
Header header
float64 intensity
float64 r
float64 g
float64 b

