
Esperienza 2

This lab experience aims to simulate a manipulator in Gazebo in order to place the robot end effector in a selected position with respect to an object. A simulated UR10 robot mounted on a table is provided in the Gazebo environment. A door model and another table sustaining it are also part of the scene. An object to be inspected is placed nearby the robot. A simulated pair of cameras forming a stereo couple are available for inspecting the object. Intrinsic and extrinsic camera parameters are provided.

At the end of the experience:

Objectives
Find a good template representing the object for both left and right cameras
Slightly change the object pose
Use a feature matching algorithm to estimate the new object pose
Use an inverse kinematics engine to find a valid robot configuration
Use MoveIt! to plan the robot motion by avoiding obstacles
Repeat until the pose of the cameras with respect to the object  are lower than a certain error in both position and orientation

Plus
Try different objects
Find the limitations of this approach
Apply a random error to the pose estimated before actually moving the robot
Add obstacles in the environment to limit the robot motion
Planner comparison

Challenges
Detect an object through feature matching
Reach a known robot pose (position + orientation) with respect to the object

What is requested
Code (BitBucket)
Video (YouTube) -- including a significant output of your computer vision algorithms
Report (PDF using BitBucket) containing at least 
The experience overview
The description of the feature matching algorithm
The description of the motion algorithm
The role of each node you developed
The nodes interaction

Explanation
Preliminary steps
sudo apt-get update
sudo apt-get install unzip liblapack-dev libxml2-dev libv4l-dev libzbar-dev libdc1394-22-dev libaria-dev libois-dev libdmtx-dev
sudo apt-get install ros-indigo-opencv-apps
sudo apt-get install ros-indigo-moveit-*
sudo apt-get install ros-indigo-gazebo-*
cd ~/Downloads/
wget https://github.com/Itseez/opencv/archive/2.4.12.zip -O opencv_2.4.12.zip
unzip opencv_2.4.12.zip
mv opencv-2.4.12 ~/workspace/opencv
wget http://gforge.inria.fr/frs/download.php/latestfile/475/visp-3.0.0.tar.gz
tar xzvf visp-3.0.0.tar.gz
mv visp-3.0.0 ~/workspace/visp
cd ~/workspace/opencv
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_OPENCL=OFF ..
make -j 8
sudo make install
cd ~/workspace/visp
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE ..
make -j 8
sudo make install
cd ~/workspace/ros/catkin/src
git clone https://github.com/lagadic/vision_visp.git
git clone https://github.com/lagadic/visp_ros.git
git clone https://github.com/ros-industrial/universal_robot
git clone https://<your_bb_username>@bitbucket.org/iaslab-unipd/door_assembly.git 
cd .. 
catkin_make
Launch Simulation
roslaunch door_assembly simulation.launch limited:=true
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch sim:=true
It is possible to load the UR10 robot model with both limited or not limited joints by using the parameter limited. In some cases, motion planning libraries could not work properly with the unlimited version of the model. The parameter is false as default.
You can now test the environment by using the MoveIt! plugin available in rviz.
roslaunch ur10_moveit_config moveit_rviz.launch config:=true
Check carefully that the size of the interactive marker is big enugh for controlling the robot end effector position (Motion Planning > Planning Request > Interactive Marker Size = 0.2). 

You can also add other information, like the image streamed from the cameras or TFs.


For adding the object you can use the following command once the system is running.
rosrun gazebo_ros spawn_model -database coke_can -sdf -model coke_can -y 0.3 -x -0.2 -z 1.0
In order to automatically control the robot motion you can use MoveIt!. The tutorial page will help you in your experience.
Pose Estimation Services
The pose estimation node offers you a way to compute the end-effector position with respect to the object. It is composed by three services. They have to be called as follow.
1. The InitCamera service allows you to initialize the system information. In particular, you have to provide left an right camera intrinsic parameters and the names of left and right camera optical frames, end effector frame, and robot base frame. No response is provided.
door_assembly/InitCameras
# Init Cameras service:
# request constants
string left_camera_frame
string right_camera_frame
string ee_frame
string base_frame
sensor_msgs/CameraInfo left_camera_info
sensor_msgs/CameraInfo right_camera_info
---
# response constants

2. The TrainPose service allows you to train the desired pose of the end effector with respect to the object. Data regarding the matched keypoints extracted from left and right cameras are expected as input. left_matched_keypoints and right_matched_keypoints will be filled with keypoints coming from left and right camera respectively. The match between left and right camera is be provided by using the same index in the structure (i.e. keypoint a will be at index 1 in both arrays). The response is composed by the reference 3D positions of the provided keypoints obtained through triangulation, and the desired end effector transformation computed from both left and right cameras.
door_assembly/TrainPose
# Train Pose service:
# request constants
opencv_apps/Point2DArray left_matched_keypoints
opencv_apps/Point2DArray right_matched_keypoints
---
# response constants
geometry_msgs/Transform left_ee_transform
geometry_msgs/Transform right_ee_transform
geometry_msgs/Point32[] left_points_3d
geometry_msgs/Point32[] right_points_3d
3. The QueryPose service gives back the new end effector pose starting from a series of keypoints matched with the ones provided during the training phase (TrainPose service). It is possible to use both left and right cameras. A map connecting novel keypoints with the one used for triangulation should also be provided, since the match is not guarantee.
door_assembly/QueryPose
# Query Pose service:
# request constants
bool use_camera_left
int64[] corr_map
opencv_apps/Point2DArray matched_keypoints
---
# response constants
geometry_msgs/Transform ee_transform



