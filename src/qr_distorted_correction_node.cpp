#include "../include/my_qr_code/qr_distorted_correction.hpp"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"qr_distorted_correction");

    float epsilon = 10.0f;
    if(argc >0){
        if(atof(argv[0]) > 0)
            epsilon = atof(argv[0]);
    }

    QR_distorted_correction qr(epsilon);
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

    dstImage = image.clone();

    monoImage_rev = makeMonoImage_rev(_image_msg);
    contours = findContours(monoImage_rev);

    distortedCorrection(contours);

}

cv::Mat QR_distorted_correction::makeMonoImage_rev(const sensor_msgs::ImageConstPtr& _image_msg){
    cv::Mat _greyImage;
    cv::Mat _returnMonoImage;
    _greyImage = cv_bridge::toCvCopy(_image_msg,sensor_msgs::image_encodings::MONO8)->image;
    cv::threshold(_greyImage,_returnMonoImage,0,255,CV_THRESH_BINARY | CV_THRESH_OTSU);

    return ~_returnMonoImage;
}

PointArray2 QR_distorted_correction::findContours(const cv::Mat& _monoImage){
    PointArray2 _returnContrours;
    cv::findContours(_monoImage,_returnContrours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    return _returnContrours;
}

std::string QR_distorted_correction::readQR(const cv::Mat& _dstImage){
    return "";
}

void QR_distorted_correction::distortedCorrection(const PointArray2& _edgesArray2){
    
    for(const auto& edgesArray : _edgesArray2){
        //直線近似をする
        std::vector<cv::Point> _normalizedEdges;
        cv::approxPolyDP(cv::Mat(edgesArray),_normalizedEdges,epsilon,true); 
    
        //四角形のみを対象とする
        if(_normalizedEdges.size() != 4)
            continue;
        
        //見つけた輪郭が内接する四角形を作成，正方形に補正
        cv::Rect _rect = cv::boundingRect(_normalizedEdges);
        cv::rectangle(dstImage,_rect,cv::Scalar(255,255,0),2,8,0);
        cv::rectangle(image,_rect,cv::Scalar(255,255,0),2,8,0);
        if(_rect.width > _rect.height){
            _rect.height = _rect.width;
        }
        else{
            _rect.width = _rect.height;
        }

        //補正の準備(補正元の輪郭)
        std::vector<cv::Point2f> srcEdges;    
        for(const auto& edge : _normalizedEdges){
            srcEdges.emplace_back(edge.x,edge.y);
        }

        //補正の準備(補正先の四角)
        std::vector<cv::Point2f> dstEdges;
        dstEdges.emplace_back(_rect.x, _rect.y);
        dstEdges.emplace_back(_rect.x + _rect.width, _rect.y);
        dstEdges.emplace_back(_rect.x+ _rect.width, _rect.y + _rect.height);
        dstEdges.emplace_back(_rect.x , _rect.y + _rect.width);

        //変換行列の作成
        cv::Mat t = cv::getPerspectiveTransform(srcEdges,dstEdges);

        //元画像に補正をかける
        cv::warpPerspective(image,dstImage,t,cv::Size2i(800,600));

        cv::imshow("image_raw",image);
        cv::imshow("image",dstImage);
        cv::waitKey(500);

        //描画テストしただけ
        
        /* line(dstImage,_normalizedEdges.at(0),_normalizedEdges.at(1), cv::Scalar(255, 0, 0), 2, 8, 0);
        line(dstImage,_normalizedEdges.at(1),_normalizedEdges.at(2), cv::Scalar(255, 0, 0), 2, 8, 0);
        line(dstImage,_normalizedEdges.at(2),_normalizedEdges.at(3), cv::Scalar(255, 0, 0), 2, 8, 0);
        line(dstImage,_normalizedEdges.at(3),_normalizedEdges.at(0), cv::Scalar(255, 0, 0), 2, 8, 0); */

    }
}

QR_distorted_correction::QR_distorted_correction(float _epsilon)
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
