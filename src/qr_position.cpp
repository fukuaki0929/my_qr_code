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
#include <string>
#include <sstream>

#include <geometry_msgs/PoseStamped.h>

#include "std_msgs/Float64.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher marker_pub;
ros::Publisher pose_pub;
int QRn = 3;

visualization_msgs::MarkerArray marker_array;
//visualization_msgs::MarkerArray pos_marker;

  std_msgs::String QR_array[10];



double pos[7];


void callback(const qr_code::distance::ConstPtr& dmsg,const qr_code::barcode::ConstPtr& bmsg)
{
  // Solve all of perception here...
  std_msgs::StringPtr str(new std_msgs::String);
  str->data = bmsg->bar;
  double dis = dmsg->dis/1000;
  
  //ROS_INFO("QR:[%s]/dis:[%f]", str->data.c_str(),dis);
  
  //QRコードを読み取った内容を配列で記録しておく
  
   //マーカを配列型にして、以前読み取ったQRコードと違う場合に新たにrviz上に表示するようにする
  //https://qiita.com/eri_aka/items/3d16bf8cc60a364bd783
  //上記がmarker配列の例
  
   
  marker_array.markers.resize(QRn);
 // pos_marker.markers.resize(QRn);
  
  //記録されていないマーカが順番に記録されるはず

		
	// https://www.codetd.com/article/1093433	
	//連続して文字を表示させるプログラムの例が上記
	
	visualization_msgs::Marker marker;
	//visualization_msgs::Marker pose;
			 
			 //0822:実験による確認の結果、/poseupdateが現在の位置を表しているようだ
			 //これを用いて、/slam_out_poseでmarkerを出力しているらしい
			 //これらのどちらかの値に距離と角度を計算して、足して、markerのデータとして出力すれば良い
			 //おそらくframe_idはmapで良いと思われる

			
	for(int i=0,flag1=0;i<QRn;i++)
	{
	
			if(QR_array[i].data == str->data)
			{
			break;
		}
			
			if(flag1==1)
			{
			continue;
		}
		
			
			if(QR_array[i].data=="")
			{
				 QR_array[i].data = str->data;
				 
				 //launchにすると動かないのは,ROS_INFOとかが動かないせい？
				//sr300_rgbdのlaunchはTFを使っていないので、それを参照すると、
				//markerの位置座標を設定できていないことになる
				//恐らくmapの方を基準にすれば可能になると思われる
				 
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //必要に合わせて、frame名は変える
    /*
    pose.header.frame_id = "/camera_depth_frame";
    pose.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    pose.ns = "basic_shapes";
    
    //複数表示する場合は、marker_idを変える
    pose.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    pose.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD and DELETE
    pose.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    pose.pose.position.x = 1.0;//pos[0] + dis * cos(pos[3]);
    pose.pose.position.y = 1.0;//pos[1] + dis * sin(pos[3]);
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    
    //pos[0]=x,pos[1]=y,pos[3]が角度
    //z方向の高さは、後でrobot_poseから取得する
    
    //std::string str1= i; 

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    pose.scale.x = 1.0;
    pose.scale.y = 1.0;
    pose.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    pose.color.r = 0.0f;
    pose.color.g = 1.0f;
    pose.color.b = 0.0f;
    pose.color.a = 1.0;//色の透明度を表す。0だと完全に透明、1だと全く透けない
	
	//markerを表示させている時間。括弧内を空にすると、無限時間になる。
	pose.lifetime = ros::Duration();
	
	pos_marker.markers.push_back(pose);
	
*/
    
    /////////////////////////////////////////////////////////////////////////////
    //barcodeの内容をrviz上に表示
    
     // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    //必要に合わせて、frame名は変える
    
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    
    //複数表示する場合は、marker_idを変える
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    //markerをtextで出したい場合はこうなる
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = pos[0] + dis * cos(pos[3]);
    marker.pose.position.y = pos[1] + dis * sin(pos[3]);
    marker.pose.position.z = 5.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    //pos[0]=x,pos[1]=y,pos[3]が角度
    //z方向の高さは、後でrobot_poseから取得する
    
    //std::string str1= i; 

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;//色の透明度を表す。0だと完全に透明、1だと全く透けない
	
	//markerを表示させている時間。括弧内を空にすると、無限時間になる。
	marker.lifetime = ros::Duration();
    
    	//textに情報を足す
    std::ostringstream ss;
    ss << dis;
    
    
	
    marker.text = str->data + "," + ss.str();
    marker_array.markers.push_back(marker);
    
    
    
    
    
      
    ROS_INFO("%d,%s",i,str->data.c_str());
      
   // ROS_INFO("pushback");
    flag1 =1;
  
}

}
    
    
 
     //push_backを使えば、複数のマーカを出せる

    // Publish the marker
    marker_pub.publish(marker_array);
    //pose_pub.publish(pos_marker);
  
  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_node");

  ros::NodeHandle nh;

  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("bar_Marker", 10);
  pose_pub = nh.advertise<visualization_msgs::MarkerArray>("pose_Marker", 10);

  
  message_filters::Subscriber<qr_code::distance> dis_sub(nh, "/distance", 1);
  message_filters::Subscriber<qr_code::barcode> barcode_sub(nh, "/img_subscriber/barcode", 1);
  
  typedef sync_policies::ApproximateTime<qr_code::distance, qr_code::barcode> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), dis_sub, barcode_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));
  
 // ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose", 1,pcallback);
  

ROS_INFO("SPINNING...");

  ros::spin();

  return 0;
}
