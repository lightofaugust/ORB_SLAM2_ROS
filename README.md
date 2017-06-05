# ORB_SLAM2 
See original ORB_SLAM2 repository from https://github.com/raulmur/ORB_SLAM2. 

# ORB_SLAM2_ROS
The modifications of this repository are:
1. Change the old ros_build cmake tool chains to catkin tool chains, and integrate them into the root CMakeLists.txt. 
2. Add ros interface for the camera pose, publish it as tf2 tree, and convert from the left-hand coordinate system of the original algorithm to the ros convention right-hand coordinate system(x forward, y left, z up).

# Usage
Currently, opencv2.4 and Asus Xtion are used. And, only ros_rgbd is build. These can be easily modified from the CMakeLists.txt and launch file. 
1. Run buld.sh
2. catkin_make the package

The topic which gives the camera pose is "camera_rgb_optical_frame".
