#include "FeatureTracker.h"

FeatureTracker::FeatureTracker() {

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
            ROS_INFO("Pop Left Image");
            left_image_queue_.pop();
           }
           else if(left_image_queue_.front().timestamp_ > right_image_queue_.front().timestamp_ + 0.003) {
            ROS_INFO("Pop Right Image");
            right_image_queue_.pop();
           }
           else {
            left_image = left_image_queue_.front().image_.clone();
            ROS_INFO("Left Image timestamp: %lf", left_image_queue_.front().timestamp_);
            left_image_queue_.pop();
            right_image = right_image_queue_.front().image_.clone();
            ROS_INFO("Right Image timestamp: %lf", right_image_queue_.front().timestamp_);
            right_image_queue_.pop();

            Tracking(left_image, right_image);
           }
        }
        sync_mutex_.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void FeatureTracker::Tracking(const cv::Mat& left_image, const cv::Mat& right_image){
    std::vector<cv::KeyPoint> left_features;
    std::vector<cv::KeyPoint> right_features;
    cv::Mat left_features_descriptor;
    cv::Mat right_features_descriptor;
    // GridFeatureExtract(left_image, right_image, 5, 5, &left_features, &right_features, &left_features_descriptor, &right_features_descriptor);
    orb_feature_detector_->detectAndCompute(left_image, cv::noArray(), left_features, left_features_descriptor);
    orb_feature_detector_->detectAndCompute(right_image, cv::noArray(), right_features, right_features_descriptor);

    //keypoint undistortion
    std::vector<cv::KeyPoint> undistorted_left_features;
    std::vector<cv::KeyPoint> undistorted_right_features;
    Undistortion(left_features, intrinsic_mat_[LEFT_CAM], distortion_coeff_[LEFT_CAM], &undistorted_left_features);
    Undistortion(right_features, intrinsic_mat_[RIGHT_CAM], distortion_coeff_[RIGHT_CAM], &undistorted_right_features);

    std::vector<cv::DMatch> feature_correspondence;
    std::vector<cv::KeyPoint> matched_left_features;
    std::vector<cv::KeyPoint> matched_right_features;
    orb_feature_matcher_->match(left_features_descriptor, right_features_descriptor, feature_correspondence);
    for(int i = 0; i < feature_correspondence.size(); i++) {
        matched_left_features.push_back(left_features[feature_correspondence[i].queryIdx]);
        matched_right_features.push_back(right_features[feature_correspondence[i].trainIdx]);
    }
    std::vector<Eigen::Vector3d> triangulated_points = Triangulation(matched_left_features, matched_right_features, camera_projection_mat_);
}

void FeatureTracker::Undistortion(const std::vector<cv::KeyPoint>& src, const Eigen::Matrix3d& intrinsic_mat, const Eigen::Vector4d& distortion_coeff, std::vector<cv::KeyPoint>* desc){
    for(auto iter : src){
        Eigen::Vector3d point(iter.pt.x, iter.pt.y, 1);
        Eigen::Vector3d p_normalized = intrinsic_mat.inverse() * point;

        double r_2 = std::pow(p_normalized.x(),2) + std::pow(p_normalized.y(),2);
        double theta = std::atan(std::sqrt(r_2));
        double theta_2 = std::pow(theta, 2);
        double theta_4 = std::pow(theta, 4);
        double theta_6 = std::pow(theta, 6);
        double theta_8 = std::pow(theta, 8);
        double theta_d = theta*(1 + distortion_coeff(0)*theta_2 + distortion_coeff(1)*theta_4 + distortion_coeff(2)*theta_6 + distortion_coeff(3)*theta_8);
        double x_undistorted = theta_d / std::sqrt(r_2) * p_normalized.x();
        double y_undistorted = theta_d / std::sqrt(r_2) + p_normalized.y();
        cv::Point2f pt_image(intrinsic_mat(0,0)*(x_undistorted + p_normalized.x() * y_undistorted) + intrinsic_mat(0,2),
                            intrinsic_mat(1,1)*y_undistorted + intrinsic_mat(1,2));
        desc->push_back(cv::KeyPoint(pt_image, iter.size, iter.angle, iter.response, iter.octave, iter.class_id));
    }
}

void FeatureTracker::GridFeatureExtract(const cv::Mat& left_image, const cv::Mat right_image, int x_grid_size, int y_grid_size,
                                        std::vector<cv::KeyPoint>* left_features, std::vector<cv::KeyPoint>* right_features, cv::Mat* left_features_descriptor, cv::Mat* right_features_descriptor) {
    cv::parallel_for_(cv::Range(0, x_grid_size * y_grid_size), LambdaBody([&](const cv::Range &range){
        std::vector<cv::KeyPoint> extracted_left_features;
        std::vector<cv::KeyPoint> extracted_right_features;
        cv::Mat extracted_left_features_descriptor;
        cv::Mat extracted_right_features_descriptor;
        orb_feature_detector_->detectAndCompute(left_image, cv::noArray(), extracted_left_features, extracted_left_features_descriptor);
        orb_feature_detector_->detectAndCompute(right_image, cv::noArray(), extracted_right_features, extracted_right_features_descriptor);

        for(auto iter : extracted_left_features){
            left_features->push_back(iter);
        }
        for(auto iter : extracted_right_features){
            right_features->push_back(iter);
        }
        //https://stackoverflow.com/questions/50464926/opencv-get-a-single-descriptor-from-a-descriptors-cvmat
    }));
}

std::vector<Eigen::Vector3d> FeatureTracker::Triangulation(const std::vector<cv::KeyPoint> left_features, const std::vector<cv::KeyPoint> right_features,
 const std::vector<Eigen::MatrixXd> projection_matrix) {
    std::vector<Eigen::Vector3d> x_1;
    for(auto iter : left_features)
        x_1.emplace_back(iter.pt.x, iter.pt.y, 1);
    std::vector<Eigen::Vector3d> x_2;
    for(auto iter : right_features)
        x_2.emplace_back(iter.pt.x, iter.pt.y, 1);

    std::vector<Eigen::Vector3d> estimated_points;
    estimated_points.reserve(left_features.size());
    // Calculate Initial Guess
    std::vector<Eigen::Vector3d> initial_guess;
    initial_guess.reserve(x_1.size());
    cv::parallel_for_(cv::Range(0, x_1.size()), LambdaBody([&](const cv::Range &range){
        for(int i = range.start; i < range.end; i++){
            Eigen::Matrix4d A;
            A.row(0) = x_1[i].x() * projection_matrix[LEFT_CAM].row(2) - projection_matrix[LEFT_CAM].row(0);
            A.row(1) = x_1[i].y() * projection_matrix[LEFT_CAM].row(2) - projection_matrix[LEFT_CAM].row(1);
            A.row(2) = x_2[i].x() * projection_matrix[RIGHT_CAM].row(2) - projection_matrix[RIGHT_CAM].row(0);
            A.row(3) = x_1[i].y() * projection_matrix[RIGHT_CAM].row(2) - projection_matrix[RIGHT_CAM].row(1);

            Eigen::JacobiSVD<Eigen::Matrix4d, Eigen::ComputeThinV> svd(A);
            Eigen::Vector4d hm_initial_guess = svd.matrixV().col(svd.matrixV().cols()-1);
            initial_guess[i] = Eigen::Vector3d(hm_initial_guess.x(), hm_initial_guess.y(), hm_initial_guess.z());
        }
    }));

    // Calculate optimal value using Gauss Newton
    cv::parallel_for_(cv::Range(0, x_1.size()), LambdaBody([&](const cv::Range &range){
        for(int i = range.start; i < range.end; i++){
            Eigen::Vector3d X = initial_guess[i];
            double Fx = 0;
            while(1){
                Eigen::Vector3d x_p1 = projection_matrix[LEFT_CAM] * X;
                Eigen::Vector3d x_p2 = projection_matrix[RIGHT_CAM] * X;
                Eigen::MatrixXd jacobian(2,3);
                double p1 = projection_matrix[LEFT_CAM](0,0) + projection_matrix[LEFT_CAM](1,0) + projection_matrix[LEFT_CAM](2,0);
                double p2 = projection_matrix[LEFT_CAM](0,1) + projection_matrix[LEFT_CAM](1,1) + projection_matrix[LEFT_CAM](2,1);
                double p3 = projection_matrix[LEFT_CAM](0,2) + projection_matrix[LEFT_CAM](1,2) + projection_matrix[LEFT_CAM](2,2);
                double p_p1 = projection_matrix[RIGHT_CAM](0,0) + projection_matrix[RIGHT_CAM](1,0) + projection_matrix[RIGHT_CAM](2,0);
                double p_p2 = projection_matrix[RIGHT_CAM](0,1) + projection_matrix[RIGHT_CAM](1,1) + projection_matrix[RIGHT_CAM](2,1);
                double p_p3 = projection_matrix[RIGHT_CAM](0,2) + projection_matrix[RIGHT_CAM](1,2) + projection_matrix[RIGHT_CAM](2,2);
                jacobian << 2*p1*X.x() - 2*p1*p1 / p3*p3 * X.z()*X.z(),
                            2*p2*X.y() - 2*p2*p2 / p3*p3 * X.z()*X.z(),
                            2*p1*p1 * X.x()*X.x() / p3*p3 * X.z()*X.z() + 2*p2*p2 * X.y()*X.y() / p3*p3 * X.z()*X.z(),
                            2*p_p1*X.x() - 2*p_p1*p_p1 / p_p3*p_p3 * X.z()*X.z(),
                            2*p_p2*X.y() - 2*p_p2*p_p2 / p_p3*p_p3 * X.z()*X.z(),
                            2*p_p1*p_p1 * X.x()*X.x() / p_p3*p_p3 * X.z()*X.z() + 2*p_p2*p_p2 * X.y()*X.y() / p_p3*p_p3 * X.z()*X.z();
                Fx = PointToPointDistance2D(Eigen::Vector2d(x_1[i].x(), x_1[i].y()), Eigen::Vector2d(x_p1.x(), x_p1.y())) + 
                PointToPointDistance2D(Eigen::Vector2d(x_2[i].x(), x_2[i].y()), Eigen::Vector2d(x_p2.x(), x_p2.y()));
                if(Fx < 0.01) {
                    estimated_points[i] = X;
                }
                    break;
                X = X - (jacobian.transpose() * jacobian).inverse() * jacobian.inverse() * Fx;
            }
        }
    }));

    return estimated_points;
}

double FeatureTracker::PointToPointDistance2D(const Eigen::Vector2d& x_1, const Eigen::Vector2d& x_2){
    return std::sqrt(std::pow(x_2.x()-x_1.x(),2) + std::pow(x_2.y()-x_1.y(),2));
}


int main(int argc, char** argv){
    ros::init(argc, argv, "study_slam");
    ROS_INFO("Feature Tracker Started");

    FeatureTracker feature_tracker;
    std::thread sync_thread(&FeatureTracker::SyncData, &feature_tracker);
    ros::spin();
    return 0;
}