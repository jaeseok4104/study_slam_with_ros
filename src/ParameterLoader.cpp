#include "ParameterLoader.h"

ParameterLoader::ParameterLoader(){
    nh_.param<std::string>("study_slam/left_camera_topic", left_image_topic_, "cam0");
    nh_.param<std::string>("study_slam/right_camera_topic", right_image_topic_, "cam1");
    nh_.param<std::string>("study_slam/imu_topic", imu_topic_, "imu");

    nh_.param<std::vector<double>>("study_slam/extrinsic", extrinsic_mat_vector_, std::vector<double>());
    nh_.param<std::vector<double>>("study_slam/left_intrinsic", left_intrinsic_mat_vector_, std::vector<double>());
    nh_.param<std::vector<double>>("study_slam/right_intrinsic", right_intrinsic_mat_vector_, std::vector<double>());
    extrinsic_mat_ = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extrinsic_mat_vector_.data(), 3, 4);
    intrinsic_mat_.push_back(Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(left_intrinsic_mat_vector_.data(), 3, 3));
    intrinsic_mat_.push_back(Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(right_intrinsic_mat_vector_.data(), 3, 3));

    camera_projection_mat_.push_back(intrinsic_mat_[LEFT_CAM] * Eigen::MatrixXd::Identity(3,4));
    camera_projection_mat_.push_back(intrinsic_mat_[RIGHT_CAM] * extrinsic_mat_);
    
    usleep(100);

    ROS_INFO("=================Parameter Loader Result=================");
    ROS_INFO("Left Camera Topic : %s", left_image_topic_.c_str());
    ROS_INFO("Right Camera Topic : %s", right_image_topic_.c_str());
    ROS_INFO("IMU Camera Topic : %s", imu_topic_.c_str());
    ROS_INFO("=========================================================");
}