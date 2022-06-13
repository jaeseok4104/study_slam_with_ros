#include "FeatureTracker.h"

FeatureTracker::FeatureTracker(){
    sub_left_image_ = nh_.subscribe<sensor_msgs::Image>(
        left_image_topic_, 1, &FeatureTracker::LeftImageHandler, this, ros::TransportHints().tcpNoDelay());
    sub_right_image_ = nh_.subscribe<sensor_msgs::Image>(
        right_image_topic_, 1, &FeatureTracker::RightImageHandler, this, ros::TransportHints().tcpNoDelay());
}

FeatureTracker::~FeatureTracker(){
}

void FeatureTracker::LeftImageHandler(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void FeatureTracker::RightImageHandler(const sensor_msgs::Image::ConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "study_slam");
    ROS_INFO("Feature Tracker Started");

    FeatureTracker feature_tracker;

    ros::spin();
    return 0;
}