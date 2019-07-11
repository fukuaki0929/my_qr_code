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

#include "tf/transform_listener.h"

using namespace sensor_msgs;
using namespace message_filters;
ros::Publisher marker_pub;
ros::Publisher text_pub;

visualization_msgs::MarkerArray marker_array;
std_msgs::String QR_array[10];
int QRn = 10;

double dis=0.50;
geometry_msgs::PoseStamped target_pose;
//tf::TransformListener tflistener;

/*
void dCallback(const qr_code::distance::ConstPtr& dmsg)
{
	dis = dmsg->dis/1000;
	
	}
	* 
	* */

void transformPoint(const tf::TransformListener& tflistener){

		ros::Time now = ros::Time(0);
        
        geometry_msgs::PoseStamped source_pose;
        
        //カメラと赤外線センサを固定しているリンク
        source_pose.header.frame_id="link4";
        source_pose.pose.orientation.w=1.0;
        source_pose.pose.position.x =0;
        source_pose.pose.position.y =0;
        source_pose.pose.position.z =dis;//実際には赤外線センサによって得られた距離にあたる
        
        /*
         * link4のz軸上に距離センサの先があると考えると,その距離センサの先の座標は(0,0,z)
         * と表すことが出来る。この座標はlink4の座標系におけるものなので、それをscanmatcher_frameの座標系
         * に変換すれば良い.するとTFで下記のように書ける.
         */
        
        tflistener.waitForTransform("map", "link4", now, ros::Duration(1.0));
        tflistener.transformPose("map",now,source_pose,"link4",target_pose);

}

void bCallback(const qr_code::barcode::ConstPtr& bmsg)
{
  // Solve all of perception here...
  std_msgs::StringPtr str(new std_msgs::String);
  str->data = bmsg->bar;
  
	//marker

    visualization_msgs::Marker marker;
    visualization_msgs::Marker tex_mark;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    		
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
				
	//qrコードの内容を表示 
	tex_mark.header.frame_id = "/map";
    tex_mark.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    tex_mark.ns = "basic_shapes";
    
    //複数表示する場合は、marker_idを変える
    tex_mark.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    //markerをtextで出したい場合はこうなる
    tex_mark.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

    // Set the marker action.  Options are ADD and DELETE
    tex_mark.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    tex_mark.pose.position.x = target_pose.pose.position.x;
    tex_mark.pose.position.y = target_pose.pose.position.y;
    tex_mark.pose.position.z = target_pose.pose.position.z + 0.25;
    tex_mark.pose.orientation.x = 0.0;
    tex_mark.pose.orientation.y = 0.0;
    tex_mark.pose.orientation.z = 0.0;
    tex_mark.pose.orientation.w = 1.0;
    

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    tex_mark.scale.x = 0.10;
    tex_mark.scale.y = 0.10;
    tex_mark.scale.z = 0.10;

    // Set the color -- be sure to set alpha to something non-zero!
    tex_mark.color.r = 0.0f;
    tex_mark.color.g = 1.0f;
    tex_mark.color.b = 0.0f;
    tex_mark.color.a = 1.0;//色の透明度を表す。0だと完全に透明、1だと全く透けない
	
	//markerを表示させている時間。括弧内を空にすると、無限時間になる。
	tex_mark.lifetime = ros::Duration();
    
    	//textに情報を足す
    	
    	/*
    std::ostringstream ss;
    ss << dis;
    * */
    
    tex_mark.text = str->data; //+ "," + ss.str();
    ROS_INFO("%d,%s",i,str->data.c_str());
				 		 
		//位置を表すマーカー
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x =  target_pose.pose.position.x;
    marker.pose.position.y =  target_pose.pose.position.y;
    marker.pose.position.z =  target_pose.pose.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.10;
    marker.scale.y = 0.10;
    marker.scale.z = 0.10;
    
    //scaleで,下から棒みたいなのをはやそうとしていたので、逆に見づらかった
    //cylinderならもっと細くしたほうがわかりやすいし,CUBEならもっと小さくして中空にあった方が分かりやすい

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);
    text_pub.publish(tex_mark);
    // ROS_INFO("push");
     flag1 =1;
 }
 
 }
 

}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle nh;
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	text_pub = nh.advertise<visualization_msgs::Marker>("text_marker", 1);

  //ros::Subscriber dis_sub = nh.subscribe("/img_subscriber/qr_code/distance", 1000, dCallback);
  ros::Subscriber bar_sub = nh.subscribe("/img_subscriber/barcode", 1000, bCallback);
  
   tf::TransformListener tflistener;
    
    //callbackを別で作成
    //Durationが呼び出す間隔
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(&transformPoint, boost::ref(tflistener)));
  
	ROS_INFO("SPINNING...");

  ros::spin();

  return 0;
}
