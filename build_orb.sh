#!/bin/bash

cd ORB_SLAM2
chmod +x build.sh
./build.sh
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$(pwd)/Examples/ROS
chmod +x build_ros.sh
./build_ros.sh

#roscore
#rosrun pcl_ros pcd_to_pointcloud point/mapPoint.pcd	
#rosrun ORB_SLAM2 PCL_VIEWER
