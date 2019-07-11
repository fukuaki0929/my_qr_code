#include "../include/my_qr_code/qr_distorted_correction.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"qr_distorted_correction");

    QR_distorted_correction qr(1);
    
    ros::spin();
    return 0;
}

void QR_distorted_correction::callback_image(const sensor_msgs::ImageConstPtr& _image_msg){
	try {
		image = cv_bridge::toCvCopy(_image_msg, sensor_msgs::image_encodings::BGR8)->image;
		//cv_bridgeを使って、opencvの形式に変換。BGR8なので、カラー画像
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

    imageCopy = image.clone();

    monoImage = makeMonoImage(_image_msg);
    contours = findContours(monoImage);

    distortedCorrection(contours);

    cv::imshow("image",monoImage);

    cv::waitKey(1);
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

void QR_distorted_correction::distortedCorrection(const PointArray2& _edgesArray2){
    for(const auto& edgesArray : _edgesArray2){
        //直線近似をする
        std::vector<cv::Point> _normalizedEdges;
        cv::approxPolyDP(cv::Mat(edgesArray),_normalizedEdges,epsilon,true); 
    
        //四角形のみを対象とする
        if(_normalizedEdges.size() == 4){
            cv::Rect _rect = cv::boundingRect(_normalizedEdges);
            cv::rectangle(monoImage,_rect,cv::Scalar(255,255,0),2,8,0);
        }
    }
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