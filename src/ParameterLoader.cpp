#include "ParameterLoader.h"

ParameterLoader::ParameterLoader(){
    nh_.param<std::string>("study_slam/left_camera_topic", left_image_topic_, "cam0");
    nh_.param<std::string>("study_slam/right_camera_topic", right_image_topic_, "cam1");
    nh_.param<std::string>("study_slam/imu_topic", imu_topic_, "imu");

    ROS_INFO("=================Parameter Loader Result=================");
    ROS_INFO("Left Camera Topic : {}", left_image_topic_);
    ROS_INFO("Right Camera Topic : {}", right_image_topic_);
    ROS_INFO("IMU Camera Topic : {}", imu_topic_);
    ROS_INFO("=========================================================");
}