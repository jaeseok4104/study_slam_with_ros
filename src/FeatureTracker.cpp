#include "FeatureTracker.h"

FeatureTracker::FeatureTracker(){

    orb_feature_detector_ = cv::ORB::create();
    orb_feature_detector_ = cv::DescriptorMatcher::create("FlannBased");

    sub_left_image_ = nh_.subscribe<sensor_msgs::Image>(
        left_image_topic_, 1, &FeatureTracker::LeftImageHandler, this, ros::TransportHints().tcpNoDelay());
    sub_right_image_ = nh_.subscribe<sensor_msgs::Image>(
        right_image_topic_, 1, &FeatureTracker::RightImageHandler, this, ros::TransportHints().tcpNoDelay());
}

FeatureTracker::~FeatureTracker(){
}

void FeatureTracker::LeftImageHandler(const sensor_msgs::Image::ConstPtr& msg){
    sync_mutex_.lock();
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        left_image_queue_.emplace(cv_ptr->image, msg->header.stamp.toSec());
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    sync_mutex_.unlock();
}

void FeatureTracker::RightImageHandler(const sensor_msgs::Image::ConstPtr& msg){
    sync_mutex_.lock();
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        right_image_queue_.emplace(cv_ptr->image, msg->header.stamp.toSec());
    } catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    sync_mutex_.unlock();
}

void FeatureTracker::SyncData(){
    while(1) {
        sync_mutex_.lock();
        cv::Mat left_image, right_image;

        if(!left_image_queue_.empty() && !right_image_queue_.empty()) {
           if(left_image_queue_.front().timestamp_ < right_image_queue_.front().timestamp_ - 0.003) {
            left_image_queue_.pop();
           }
           else if(left_image_queue_.front().timestamp_ > right_image_queue_.front().timestamp_ + 0.003) {
            right_image_queue_.pop();
           }
           else {
            left_image = left_image_queue_.front().image_.clone();
            ROS_DEBUG("Left Image timestamp: %lf", left_image_queue_.front().timestamp_);
            left_image_queue_.pop();
            right_image = right_image_queue_.front().image_.clone();
            ROS_DEBUG("Right Image timestamp: %lf", right_image_queue_.front().timestamp_);
            right_image_queue_.pop();
            sync_mutex_.unlock();

            Tracking(left_image, right_image);
           }
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void FeatureTracker::Tracking(const cv::Mat& left_image, const cv::Mat& right_image){
    std::vector<cv::KeyPoint> left_features;
    cv::Mat left_features_descriptor;
    orb_feature_detector_->detectAndCompute(left_image, cv::noArray(), left_features, left_features_descriptor);

    std::vector<cv::KeyPoint> right_features;
    cv::Mat right_features_descriptor;
    orb_feature_detector_->detectAndCompute(right_image, cv::noArray(), right_features, right_features_descriptor);

    std::vector<cv::DMatch> feature_correspondence;
    orb_feature_matcher_->match(left_features_descriptor, right_features_descriptor, feature_correspondence);

    
}

std::vector<Eigen::Vector3d> Triangulation(const std::vector<cv::KeyPoint> left_features, const std::vector<cv::KeyPoint> right_feature,
 const Eigen::Matrix4d projection_matrix) {

}

int main(int argc, char** argv){
    ros::init(argc, argv, "study_slam");
    ROS_INFO("Feature Tracker Started");

    FeatureTracker feature_tracker;
    std::thread sync_thread(&FeatureTracker::SyncData, &feature_tracker);
    ros::spin();
    return 0;
}