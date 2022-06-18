#include "ParameterLoader.h"

ParameterLoader::ParameterLoader(){
    nh_.param<std::string>("study_slam/left_camera_topic", left_image_topic_, "cam0");
    nh_.param<std::string>("study_slam/right_camera_topic", right_image_topic_, "cam1");
    nh_.param<std::string>("study_slam/imu_topic", imu_topic_, "imu");
    
    usleep(100);

    ROS_INFO("=================Parameter Loader Result=================");
    ROS_INFO("Left Camera Topic : %s", left_image_topic_.c_str());
    ROS_INFO("Right Camera Topic : %s", right_image_topic_.c_str());
    ROS_INFO("IMU Camera Topic : %s", imu_topic_.c_str());
    ROS_INFO("=========================================================");
}