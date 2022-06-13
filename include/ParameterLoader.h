#ifndef PARAMETER_LOARDER
#define PARAMETER_LOARDER
#include <string.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class ParameterLoader {
protected:
    ros::NodeHandle nh_;

    std::string left_image_topic_;
    std::string right_image_topic_;
    std::string imu_topic_;

    ParameterLoader();
};
#endif //PARAMETER_LOARDER