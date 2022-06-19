#include "FeatureTracker.h"

FeatureTracker::FeatureTracker(){

    orb_feature_detector_ = cv::ORB::create();
    orb_feature_matcher_ = cv::DescriptorMatcher::create("FlannBased");

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

std::vector<Eigen::Vector3d> FeatureTracker::Triangulation(const std::vector<cv::KeyPoint> left_features, const std::vector<cv::KeyPoint> right_features,
 const std::vector<Eigen::MatrixXd> projection_matrix) {
    std::vector<Eigen::Vector3d> x_1;
    for(auto iter : left_features)
        x_1.emplace_back(iter.pt.x, iter.pt.y, 1);
    std::vector<Eigen::Vector3d> x_2;
    for(auto iter : right_features)
        x_2.emplace_back(iter.pt.x, iter.pt.y, 1);

    // Calculate Initial Guess
    std::vector<Eigen::Vector3d> initial_guess;
    for(int i = 0; i < x_1.size(); i++){
        Eigen::Matrix4d A;
        A.row(0) = x_1[i].y() * projection_matrix[LEFT_CAM].row(2) - projection_matrix[LEFT_CAM].row(1);
        A.row(1) = projection_matrix[LEFT_CAM].row(0) - x_1[i].y() * projection_matrix[RIGHT_CAM].row(2);
        A.row(2) = x_2[i].y() * projection_matrix[RIGHT_CAM].row(2) - projection_matrix[RIGHT_CAM].row(1);
        A.row(3) = projection_matrix[RIGHT_CAM].row(0) - x_2[i].x() * projection_matrix[RIGHT_CAM].row(2);

        Eigen::JacobiSVD<Eigen::Matrix4d, Eigen::ComputeThinV> svd(A);
        Eigen::Vector4d hm_initial_guess = svd.matrixV().col(svd.matrixV().cols()-1);
        initial_guess.emplace_back(hm_initial_guess.x(), hm_initial_guess.y(), hm_initial_guess.z());
    }

    // // Calculate optimal value using Gauss Newton
    // for(int i = 0; i < x_1.size(); i++){
    //     Eigen::Vector3d x_p1 = projection_matrix[LEFT_CAM] * initial_guess[i];
    //     Eigen::Vector3d x_p2 = projection_matrix[RIGHT_CAM] * initial_guess[i];
    //     double r = 0;
    //     while(1){
    //         Eigen::MatrixXd jacobian(2,3);
    //         jacobian = 
    //         r = PointToPointDistance2D(Eigen::Vector2d(x_1[i].x(), x_1[i].y()), Eigen::Vector2d(x_p1.x(), x_p1.y())) + 
    //         PointToPointDistance2D(Eigen::Vector2d(x_2[i].x(), x_2[i].y()), Eigen::Vector2d(x_p2.x(), x_p2.y()));
    //     }
    // }
}

double FeatureTracker::PointToPointDistance2D(const Eigen::Vector2d& x_1, const Eigen::Vector2d& x_2){
    return 0;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "study_slam");
    ROS_INFO("Feature Tracker Started");

    FeatureTracker feature_tracker;
    std::thread sync_thread(&FeatureTracker::SyncData, &feature_tracker);
    ros::spin();
    return 0;
}