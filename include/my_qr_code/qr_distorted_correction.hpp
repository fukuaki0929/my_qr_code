#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>
#include <sensor_msgs/image_encodings.h>

using PointArray2 = std::vector<std::vector<cv::Point>>;

class QR_distorted_correction{
private:
    cv::Mat image;
    cv::Mat imageCopy;
    cv::Mat monoImage_rev; 
    PointArray2 contours;

    const float epsilon;

    void callback_image(const sensor_msgs::ImageConstPtr& _image_msg);
    cv::Mat makeMonoImage_rev(const sensor_msgs::ImageConstPtr& _image_msg);
    PointArray2 findContours(const cv::Mat& _monoImage);
    void distortedCorrection(const PointArray2& _edgesArray2);

    image_transport::Subscriber image_sub;

public:
    QR_distorted_correction();
    QR_distorted_correction(float _epsilon);
};