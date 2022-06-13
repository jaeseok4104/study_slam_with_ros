#ifndef FEATURE_TRACKER
#define FEATURE_TRACKER
#include <opencv2/opencv.hpp>
#include "ParameterLoader.h"

class FeatureTracker : public ParameterLoader{
public:
    FeatureTracker();
    ~FeatureTracker();
private:
    void LeftImageHandler(const sensor_msgs::Image::ConstPtr& msg);
    void RightImageHandler(const sensor_msgs::Image::ConstPtr& msg);

    ros::Subscriber sub_left_image_;
    ros::Subscriber sub_right_image_;
};

#endif // FEATURE_TRACKER