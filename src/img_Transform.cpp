/*
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
//#include <zbar/Exception.h>
//0720動作確認
//透視変換のお試し用のプログラム
using namespace cv;
using namespace std;
using namespace zbar;
 
cv::Mat image;

ros::Publisher barcode_pub_;

 
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	try {
		image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
		//cv_bridgeを使って、opencvの形式に変換。BGR8なので、カラー画像
	}
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	
	
	cv::Mat cv_image;
    cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
	cv::Mat img_copy = image.clone();//元画像のコピー
	
	cv::threshold(cv_image, cv_image, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);//2値変換
	
	//画像確認用
	imshow("2way",cv_image);
	
	
	ImageScanner scanner;
	int width  = cv_image.cols;
	int height = cv_image.rows;
	
	
	// wrap image data
    Image qr_image(width, height, "Y800", (unsigned char*)cv_image.data, width * height);
    //unsigned char型　保存できる値は　0-255です
    //恐らくグレースケールなので0-255までの値しか入らないということだと思う。
    //なので、0-255以外の値にならないようにunsigned charをつけているのではないかと考えている
    
    //"Y800"の後の4つ目の変数がqrコードのscanに使っているデータのようだ
	
    // scan the image for barcodes
    int n = scanner.scan(qr_image);
    //読み取れている場合はn=1,読み取れていないとn=0
    //printf("%d",n);
    
    //なので、for文に組み込みたいときはnの0,1でif文を作れば良い？
    
	std::vector<cv::Point> corners;
    // extract results
    for(Image::SymbolIterator symbol = qr_image.symbol_begin();//zbar::Image::SymbolIterator
        symbol != qr_image.symbol_end();
        ++symbol) {
        // do something useful with results
       
        cout << "decoded " << symbol->get_type_name()
             << " symbol \"" << symbol->get_data() << '"' << endl;
             //get_type_nameの方が、どの種類のバーコードか、get_dataがバーコードの内容を指している
            std_msgs::String barcode_string;
			barcode_string.data = symbol->get_data();
			barcode_pub_.publish(barcode_string);
			
			//画像内でのqrコードの位置が分かる
			//0718 動作確認 緑の線でQRコードが囲まれていた
			if (symbol->get_location_size() == 4) {

				//symbolの中に、QRコードの4点が記録されていて、それに合わせて線を引いている
				//cv::line 画像内への直線の描画
				//線の色は赤
                line(img_copy, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(255, 0, 0), 2, 8, 0);
                line(img_copy, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(255, 0, 0), 2, 8, 0);
            }   
            
            //qrコードが複数ある場合に、前のやつと別だと分かるようにする必要がある
    }
    
    imshow("image",img_copy);
    
	cv::waitKey(1);
}
 
int main(int argc, char** argv)
{
	ros::init (argc, argv, "img_subscriber");
	ros::NodeHandle nh("~");
 
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 10, imageCallback);
 
	//code = nh.advertise<std::string>("qr_data", 1);
	 barcode_pub_ = nh.advertise<std_msgs::String>("barcode", 10);
 
	ros::spin();
 
	return 0;
}
*/
#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <iostream>  

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include "std_msgs/String.h"
#include "qr_code/imageimage.h"

#define WIDTH   50
#define HEIGHT  25

void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)   
{   
	
	//lx0 = msg;
	
    int x1, x2, y1, y2;
    int k;
    int i = 320;
    int j = 240;
    int width = WIDTH;
    int height = HEIGHT;
    double sum = 0.0;
    double depth_p;
    double ave;
    cv_bridge::CvImagePtr cv_ptr;
 
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);//cv形式に変換
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
    
    //Matを先に定義
    cv::Mat depth(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
 
	//設定した範囲
	//depthは基のimageと同じ大きさ
    x1 = int(depth.cols / 2) - width;
    x2 = int(depth.cols / 2) + width;
    y1 = int(depth.rows / 2) - height;
    y2 = int(depth.rows / 2) + height;
 
 /*
    for(i = 0; i < cv_ptr->image.rows;i++){
        float* Dimage = cv_ptr->image.ptr<float>(i);//Dimage[i][j]で指定できる？
        float* Iimage = depth.ptr<float>(i);
        char* Ivimage = img.ptr<char>(i);
        for(j = 0 ; j < cv_ptr->image.cols; j++){
            if(Dimage[j] > 0.0){
                Iimage[j] = Dimage[j];
                Ivimage[j] = (char)(255*(Dimage[j]/5.5));
            }else{
            }
 
            if(i > y1 && i < y2){
                if(j > x1 && j < x2){
                    if(Dimage[j] > 0.0){
                        sum += Dimage[j];
                    }
                }
            }
        }
    }
 */
	float* Dimage = cv_ptr->image.ptr<float>(i);
	depth_p = Dimage[j];
	
	//ある一点の距離をとれている
	//恐らく単位は[mm]
	
	//ROS_INFO("location : %f,%f,%f,%f\n%f,%f,%f,%f ",lx0,location_y_0 ,location_x_1 ,location_y_1 ,location_x_2 ,location_y_2 ,location_x_3 ,location_y_3);
    //ROS_INFO("depth : %f [mm]", depth_p);
 
    //cv::imshow("DEPTH image", img);
    cv::waitKey(10);
   
}   

  
int main(int argc, char **argv)  
{  
  
    ros::init(argc, argv, "distance");  
    ros::NodeHandle nh;  
  
    ros::AsyncSpinner spinner(1);  
    spinner.start();  
    
    
    //ros::Subscriber location_sub = nh.subscribe("location", 100, locationCallback);   
	ros::Subscriber sub = nh.subscribe("camera/depth/image_raw", 100, chatterCallback);   
    //ros::Subscriber sub = n.subscribe("camera/depth/image_raw", 100, chatterCallback);   
    ros::Duration(100).sleep();
	return 0;  
}
