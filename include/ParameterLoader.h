#ifndef PARAMETER_LOARDER
#define PARAMETER_LOARDER
#include <string.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <Eigen/Eigen>

#define LEFT_CAM 0
#define RIGHT_CAM 1

class ParameterLoader {
protected:
    ros::NodeHandle nh_;

    std::string left_image_topic_;
    std::string right_image_topic_;
    std::string imu_topic_;
    std::vector<double> extrinsic_mat_vector_;
    Eigen::MatrixXd extrinsic_mat_;
    std::vector<double> left_intrinsic_mat_vector_;
    std::vector<double> right_intrinsic_mat_vector_;
    std::vector<Eigen::Matrix3d> intrinsic_mat_;
    std::vector<Eigen::MatrixXd> camera_projection_mat_;
    

    ParameterLoader();
};
#endif //PARAMETER_LOARDER