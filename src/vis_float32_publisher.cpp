#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
using namespace std;
int main( int argc, char** argv )
{
    ros::init(argc, argv, "showline");
    ros::NodeHandle n;
    ros::Publisher markerArrayPub = n.advertise<visualization_msgs::MarkerArray>("MarkerArray", 10);

    visualization_msgs::MarkerArray markerArray;
    ros::Rate r(1);
    int k=0;
    while(k<100)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id="/odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id =k;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

        marker.scale.z = 0.1;
        marker.color.b = 255;
        marker.color.g = 255;
        marker.color.r = 255;
        marker.color.a = 1;

        geometry_msgs::Pose pose;
        pose.position.x =  (float)(k)/10;
        pose.position.y =  0;
        pose.position.z =0;
        ostringstream str;
        str<<k;
        marker.text=str.str();
        marker.pose=pose;

        cout<<"k="<<k<<endl;
        markerArray.markers.push_back(marker);
        cout<<"markerArray.markers.size()"<<markerArray.markers.size()<<endl;
        markerArrayPub.publish(markerArray);
        r.sleep();
        k++;
    }
    return 0;
}
