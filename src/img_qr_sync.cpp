//0905:距離の計算にこれを用いていただけなので、使わなければよいだけ
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

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
#include "qr_code/distance.h"
#include "qr_code/barcode.h"

#include "std_msgs/Float64.h"

using namespace sensor_msgs;
using namespace message_filters;

#define WIDTH   50
#define HEIGHT  25

ros::Publisher mix_pub;

void callback(const qr_code::imageimage::ConstPtr& lmsg, const sensor_msgs::Image::ConstPtr& dmsg)
{
  // Solve all of perception here...
	
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
    
    double lx[4],ly[4];
    
    try{
        cv_ptr = cv_bridge::toCvCopy(dmsg, sensor_msgs::image_encodings::TYPE_32FC1);//cv形式に変換
        
				lx[0] = lmsg->location_x_0 ;
				ly[0] = lmsg->location_y_0 ;
				lx[1] = lmsg->location_x_1;
				ly[1] = lmsg->location_y_1 ;
				lx[2] = lmsg->location_x_2 ;
				ly[2] = lmsg->location_y_2 ;
				lx[3] = lmsg->location_x_3 ;
				ly[3] = lmsg->location_y_3;
        
        
    }catch (cv_bridge::Exception& ex){
        ROS_ERROR("error");
        exit(-1);
    }
    
    //Matを先に定義
    cv::Mat depth(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
    cv::Mat img(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
 
	//設定した範囲
	//depthは基のimageと同じ大きさ
	
	//画像の真ん中からdepth*2,height*2の範囲
	
	//qrコードの場合は、この4点をqrコードの座標にすれば良い
	//中心座標は4点の重心。そこから一定距離の範囲
	
	//4点のx,yの最大最小を求める
	
	double lx_max =lx[0];
	double lx_min =lx[0];
	double ly_max =ly[0];
	double ly_min =ly[0];
	
	for(int i = 0; i < 4;i++)
	{
		if(lx[i]>lx_max)
		{
			lx_max = lx[i];
		}
		
		if(lx[i]<lx_min)
		{
			lx_min = lx[i];
		}
		
		if(ly[i]>ly_max)
		{
			ly_max = ly[i];
		}
		
		if(ly[i]<ly_min)
		{
			ly_min = ly[i];
		}
	}
	
	//重心と,4点の最大、最小点と平均を取って、QRコードの1回り内側の領域に絞って距離の平均を取る
    x1 = ((lx[0]+lx[1]+lx[2]+lx[3])/4 + lx_min)/2;
    x2 = ((lx[0]+lx[1]+lx[2]+lx[3])/4 + lx_max)/2;
    y1 = ((ly[0]+ly[1]+ly[2]+ly[3])/4 + ly_min)/2;
    y2 = ((ly[0]+ly[1]+ly[2]+ly[3])/4 + ly_max)/2;
   
	// Red，太さ3，4近傍連結
	cv::rectangle(img, cv::Point(x1,y1), cv::Point(x2, y2), cv::Scalar(0,0,200), 3, 4);
 
    for(i = 0; i < cv_ptr->image.rows;i++){//imagena位の全ての行を指定
        float* Dimage = cv_ptr->image.ptr<float>(i);				//Dimage[i][j]で指定できる？
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
 
	
	/*
	float* Dimage = cv_ptr->image.ptr<float>(i);
	depth_p = Dimage[j];
	* */
	
	//ある一点の距離をとれている
	//恐らく単位は[mm]
	
	//ROS_INFO("location : %f,%f,%f,%f\n%f,%f,%f,%f ",lx0,location_y_0 ,location_x_1 ,location_y_1 ,location_x_2 ,location_y_2 ,location_x_3 ,location_y_3);
    //ROS_INFO("depth : %f [mm]", depth_p);
    
    //割ってる数がおかしい？
    //0820:lx_max-lx_minとかでやっていたので、領域が大きくなっていた
    //x2,x1,y2,y1で上記の平均をとっているので、計算の際もこれで割る必要があった
    ave = sum / ((x2 - x1) * (y2 - y1));
    
    
	ROS_INFO("depth : %f [mm]", ave);
	
	qr_code::distance dis;
	
	dis.dis = ave;
	dis.header.stamp = ros::Time::now();
	
	mix_pub.publish(dis);
 
    cv::imshow("DEPTH image", img);
    cv::waitKey(10);
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  
  //ros::AsyncSpinner spinner(1);  
  //spinner.start();  
  
  mix_pub = nh.advertise<qr_code::distance>("distance", 1);

  message_filters::Subscriber<qr_code::imageimage> location_sub(nh, "/img_subscriber/location", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth/image_raw", 1);
  //0806:topic名が/img_subscriber/locationをlocationと間違えていたので動かなかっただけでした。
  /*
  TimeSynchronizer<qr_code::imageimage, sensor_msgs::Image> sync(location_sub, depth_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  */

  typedef sync_policies::ApproximateTime<qr_code::imageimage, sensor_msgs::Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), location_sub, depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

ROS_INFO("SPINNING...");

  ros::spin();

  return 0;
}
