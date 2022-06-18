#ifndef FEATURE_TRACKER
#define FEATURE_TRACKER
#include <thread>
#include <mutex>
#include <queue>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include "ParameterLoader.h"

struct ImageAndStamp{
    ImageAndStamp(cv::Mat image, double timestamp) : image_(std::move(image)), timestamp_(std::move(timestamp))
    {};
    cv::Mat image_;
    double timestamp_;
};
class FeatureTracker : public ParameterLoader{
public:
    FeatureTracker();
    ~FeatureTracker();
    void SyncData();

private:
    void LeftImageHandler(const sensor_msgs::Image::ConstPtr& msg);
    void RightImageHandler(const sensor_msgs::Image::ConstPtr& msg);

    void Tracking(const cv::Mat& left_image, const cv::Mat& right_image);
    std::vector<Eigen::Vector3d> Triangulation(const std::vector<cv::KeyPoint> left_features, const std::vector<cv::KeyPoint> right_feature, const Eigen::Matrix4d projection_matrix);

    cv::Ptr<cv::FeatureDetector> orb_feature_detector_;
    cv::Ptr<cv::DescriptorMatcher> orb_feature_matcher_;
    std::queue<ImageAndStamp> left_image_queue_;
    std::queue<ImageAndStamp> right_image_queue_;
    ros::Subscriber sub_left_image_;
    ros::Subscriber sub_right_image_;
    
    std::mutex sync_mutex_;
};

#endif // FEATURE_TRACKER