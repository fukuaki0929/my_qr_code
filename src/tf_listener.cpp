#include "ros/ros.h"

#include "math.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

#include <string>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

//実際にプログラムに組み込むときは下記を参照
//https://myenigma.hatenablog.com/entry/20130210/1360491625#1-rostimenow%E3%82%92%E4%BD%BF%E3%81%A3%E3%81%A6LookupTransform%E3%81%97%E3%81%A6%E3%81%AF%E3%81%84%E3%81%91%E3%81%AA%E3%81%84
//quaternionをroll,pitch,yawに変換する関数も掲載されている

int main(int argc, char **argv){
    ros::init(argc, argv, "vis_tf_listen");
    ros::NodeHandle n;
    ros::Publisher text_pub;
    text_pub = n.advertise<visualization_msgs::Marker>("text_marker", 1);

    tf::TransformListener tflistener;

    //ros::Rate loop_rate(0.1); 
    while (ros::ok()){
		
		ros::Time now = ros::Time(0);
        geometry_msgs::PoseStamped target_pose;
        geometry_msgs::PoseStamped source_pose;
        
        source_pose.header.frame_id="link4";
        source_pose.pose.orientation.w=1.0;
        
        tf::Quaternion tf_q;
        
        double x_m=0.0, y_m=0.0,z_m=0.0, roll=0.0,pitch=0.0,yaw=0.0;
        
         tf::StampedTransform transform;
       /*   
        try{
		
           tflistener.waitForTransform("link4", "scanmatcher_frame", now, ros::Duration(1.0));
             //tflistener.lookupTransform("link4", "scanmatcher_frame", now, transform);
             
             //mapではなく、少なくともscanmatcher_frameから見た相対座標にはなるはず
                                
        x_m = transform.getOrigin().x();
        y_m = transform.getOrigin().y();
        z_m = transform.getOrigin().z();
        //getOrigin()で座標を獲得
        tf_q = transform.getRotation();
        //getRotation()でquarternionを取得
            
            //クォータニオン→オイラー角
            //tf::Quaternion quat;//入力値
			double r,p,y;//出力値
			tf::Matrix3x3(tf_q).getRPY(r, p, y);
			
       //ROS_INFO("r:%03f, p:%03f,y:%03f",r,p,y);
        }
        catch(...){
            ROS_INFO("tf error");
        }
        */
        source_pose.pose.position.x =0;
        source_pose.pose.position.y =0;
        source_pose.pose.position.z =0.5;//0.25が実際には赤外線センサによって得られた距離にあたる
        
        /*
         * link4のz軸上に距離センサの先があると考えると,その距離センサの先の座標は(0,0,z)
         * と表すことが出来る。この座標はlink4の座標系におけるものなので、それをscanmatcher_frameの座標系
         * に変換すれば良い.するとTFで下記のように書ける.
         */
        
        /*
        tflistener.waitForTransform("scanmatcher_frame", "link4", now, ros::Duration(1.0));
        tflistener.transformPose("scanmatcher_frame",now,source_pose,"link4",target_pose);
        */
        
        tflistener.waitForTransform("map", "link4", now, ros::Duration(1.0));
        tflistener.transformPose("map",now,source_pose,"link4",target_pose);
        
        visualization_msgs::Marker tex_mark;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
		 
	tex_mark.header.frame_id = "/map";
    tex_mark.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    tex_mark.ns = "basic_shapes";
    
    //複数表示する場合は、marker_idを変える
    tex_mark.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    //markerをtextで出したい場合はこうなる
    tex_mark.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD and DELETE
    tex_mark.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    tex_mark.pose.position.x = target_pose.pose.position.x;
    tex_mark.pose.position.y = target_pose.pose.position.y;
    tex_mark.pose.position.z = target_pose.pose.position.z;
    tex_mark.pose.orientation.x = 0.0;
    tex_mark.pose.orientation.y = 0.0;
    tex_mark.pose.orientation.z = 0.0;
    tex_mark.pose.orientation.w = 1.0;
    
    //pos[0]=x,pos[1]=y,pos[3]が角度
    //z方向の高さは、後でrobot_poseから取得する
    
    //std::string str1= i; 

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    tex_mark.scale.x = 0.25;
    tex_mark.scale.y = 0.25;
    tex_mark.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    tex_mark.color.r = 0.0f;
    tex_mark.color.g = 1.0f;
    tex_mark.color.b = 0.0f;
    tex_mark.color.a = 1.0;//色の透明度を表す。0だと完全に透明、1だと全く透けない
	
	//markerを表示させている時間。括弧内を空にすると、無限時間になる。
	tex_mark.lifetime = ros::Duration();
	
	ROS_INFO("SPINNING...");
	
	 text_pub.publish(tex_mark);
        
        //loop_rate.sleep();
        ros::spinOnce();
        
    } 
    return 0;
}
