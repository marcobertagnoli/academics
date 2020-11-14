
Esperienza 3

During this experience you have to plan the motion of a NAO robot in a 2D simulated environment populated by obstacles. The robot has to walk through a prefix path by avoiding collisions with other objects around it.


At the end of the experience...
Objectives
Make the simulated robot walk around the environment without collision 
Understand the 2D map, and build a new map 
Move the real robot by using the path generated during the simulation 
Plus
Object Oriented (OO) approach 
Use a 3D map (see OctoMap) 
Use a dynamic map 
Challenges
Walk in a populated environment 
What we want
Code (BitBucket) 
Video (YouTube) 
Report (BitBucket) containing a short description of your attempts 

Explanation
Preliminary steps
sudo apt-get update

sudo apt-get install ros-indigo-nao-meshes ros-indigo-octomap-ros ros-indigo-octomap-msgs ros-indigo-map-server ros-indigo-fake-localization ros-indigo-sbpl ros-indigo-naoqi-libqi ros-indigo-naoqi-libqicore

cd ~/Downloads

git clone --branch 2.1.2.17 https://<your_bb_username>@bitbucket.org/iaslab-unipd/naoqi.git

sudo mv naoqi /opt/

echo "export PYTHONPATH=$PYTHONPATH:/opt/naoqi/pynaoqi-python2.7-2.1.2.17-linux64" >> ~/.bashrc
echo "export AL_DIR=/opt/naoqi/naoqi-sdk-2.1.2.17-linux64" >> ~/.bashrc

cd ~/workspace/ros/catkin/src
git clone https://github.com/michieletto/naoqi_driver.git
git clone https://github.com/ros-naoqi/naoqi_bridge.git
git clone https://github.com/ros-naoqi/naoqi_bridge_msgs.git
git clone https://github.com/michieletto/nao_robot.git
git clone https://github.com/ros-naoqi/nao_extras.git
git clone https://github.com/michieletto/humanoid_navigation.git
git clone https://github.com/ahornung/humanoid_msgs.git

cd ..
catkin_make
Now close all your terminal windows.
Make the NAO walk
In order to test the installed packages, you should run the NaoQi driver and then launch a preloaded example program.
As a first step you have to activate the simulated
cd $AL_DIR
./naoqi -b 127.0.0.1
In another terminal you can now launch the simulated environment
roslaunch footstep_planner footstep_navigation_nao_fake_loc.launch nao_ip:=127.0.0.1 network_interface:=lo
Move your NAO by using first 2D Pose Estimate and then 2D Nav Goal.


Experience
Now you can try to use your own maps in both simulated and real environments.
In order to be able to work in the real environment, you have to guarantee 2 constraints:
1. the motors have been activated (search for body_stiffness)
2. the robot is in standing position (search for body_pose action)

Plus: Octomap
Try to have a look at the Octomap package for ROS and use it to build the 3D map of a real environment.
This library implements a 3D occupancy grid mapping approach by providing data structures and mapping algorithms.


