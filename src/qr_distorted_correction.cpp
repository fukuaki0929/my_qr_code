#include "../include/my_qr_code/qr_distorted_correction.hpp"

void QR_distorted_correction::callback_image(const sensor_msgs::ImageConstPtr& _image_msg){
	try {
		image = cv_bridge::toCvCopy(_image_msg, sensor_msgs::image_encodings::BGR8)->image;
		//cv_bridgeを使って、opencvの形式に変換。BGR8なので、カラー画像
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

    cv::Mat img_copy = image.clone();

    monoImage = makeMonoImage(_image_msg);

    cv::imshow("image",monoImage);
}

cv::Mat QR_distorted_correction::makeMonoImage(const sensor_msgs::ImageConstPtr& _image_msg){
    cv::Mat _greyImage;
    cv::Mat _returnMonoImage;
    _greyImage = cv_bridge::toCvCopy(_image_msg,sensor_msgs::image_encodings::MONO8)->image;
    cv::threshold(_greyImage,_returnMonoImage,0,255,CV_THRESH_BINARY | CV_THRESH_OTSU);

    return _returnMonoImage;
}

PointArray2 QR_distorted_correction::findContours(const cv::Mat& _monoImage){
    PointArray2 _returnContrours;
    cv::findContours(_monoImage,_returnContrours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    return _returnContrours;
}

QR_distorted_correction::QR_distorted_correction(int _epsilon)
    :epsilon(_epsilon)
    {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_sub = it.subscribe("/camera/color/image_raw",10,&QR_distorted_correction::callback_image,this);
}

QR_distorted_correction::QR_distorted_correction()
    :epsilon(10.0f)
    {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    image_sub = it.subscribe("/camera/color/image_raw",10,&QR_distorted_correction::callback_image,this);
}