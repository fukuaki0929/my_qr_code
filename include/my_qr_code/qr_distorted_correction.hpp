#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <zbar.h>
#include <sensor_msgs/image_encodings.h>

//参考サイト
//http://mtail3x.wpblog.jp/2015/11/06/opencvsharp%E3%81%A7%E5%8F%B0%E5%BD%A2%E8%A3%9C%E6%AD%A3/
//http://nn-hokuson.hatenablog.com/entry/2017/05/31/210808
using PointArray2 = std::vector<std::vector<cv::Point>>;

class QR_distorted_correction{
private:
    cv::Mat image;
    cv::Mat dstImage;
    cv::Mat monoImage_rev; 
    cv::Mat dst;
    PointArray2 contours;

    const zbar::ImageScanner scanner;

    const float epsilon;

    void callback_image(const sensor_msgs::ImageConstPtr& _image_msg);
    cv::Mat makeMonoImage_rev(const sensor_msgs::ImageConstPtr& _image_msg);
    PointArray2 findContours(const cv::Mat& _monoImage);
    void distortedCorrection(const PointArray2& _edgesArray2);
    std::string readQR(const cv::Mat& _dstImage);
    image_transport::Subscriber image_sub;

public:
    QR_distorted_correction();
    QR_distorted_correction(float _epsilon);
};