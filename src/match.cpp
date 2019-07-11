#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include "qr_code/imageimage.h"
//#include <zbar/Exception.h>
//0720動作確認
//透視変換のお試し用のプログラム
using namespace cv;
using namespace std;
using namespace zbar;
 
cv::Mat image;

int main(int argc, char** argv)
{
	ros::init (argc, argv, "match");
	ros::NodeHandle nh("~");
 
	//ros::Subscriber location_sub = nh.advertise<qr_code::imageimage>("location",10,callback);
	//ros::Subscriber barcode_sub_ = nh.advertise<std_msgs::String>("barcode",10,callback);
 
	ros::spin();
 
	return 0;
}
