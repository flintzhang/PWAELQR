#include<iostream>
#include<fstream>
#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"


double costmap_resolution;
int costmap_width;
int costmap_height;
double originpoint[3];
bool new_map_available_flag;
int costmap_size;

void callbackcostmap(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	costmap_width = (int)(msg->info.width);
	costmap_height = (int)(msg->info.height);
	ROS_INFO("%d, %d", costmap_width, costmap_height);
	costmap_size = costmap_width * costmap_height;
	costmap_resolution = (double)(msg->info.resolution);
	originpoint[0] = msg->info.origin.position.x;
	originpoint[1] = msg->info.origin.position.y;
	originpoint[2] = msg->info.origin.position.z;
	new_map_available_flag = true;
}


struct unode{
	double ul;
	double ur;
};

struct xnode{
	double x_x;
	double x_y;
	double x_theta;
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "filelqrnode");
	ros::NodeHandle n;
	ros::Subscriber costmap_sub = n.subscribe("/move_base/local_costmap/costmap", 1 , callbackcostmap);
	ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 9999 );
	ros::Publisher geo_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 50);
	new_map_available_flag = false;
	ros::Rate r(10); 
	r.sleep();

	
	while (new_map_available_flag == false)
	{
		if (ros::ok())
			ros::spinOnce();
		else
			return 0;
	}

	double temp;
	std::fstream ufile("/home/cloud/catkin_ws/src/segpanda_pwaelqr/elqr_simulation/data/controlu_ELQR.txt", std::ios_base::in);
	if (!(ufile >> temp))
		ROS_INFO("U file not opened");
	std::fstream xfile("/home/cloud/catkin_ws/src/segpanda_pwaelqr/elqr_simulation/data/statex_ELQR.txt", std::ios_base::in);
	if (!(xfile >> temp))
		ROS_INFO("X file not opened");
	std::list<unode> ulist;
	std::list<xnode> xlist;

	ufile >> temp;
	while(ufile >> temp)
	{
		ROS_INFO("%f", temp);
		unode nu;
		nu.ul = temp;
		ufile >> nu.ur;
		ROS_INFO("%f %f", nu.ul, nu.ur);
		ulist.push_back(nu);
	}

	int size = ulist.size();
	
	visualization_msgs::MarkerArray markers;
	markers.markers.resize(5+size+size);
	int count = 0;

	xfile >> temp;
	xfile >> temp;
	while(xfile >> temp)
	{
		xnode nx;
		nx.x_x = temp;
		xfile >> nx.x_y;
		ROS_INFO("%f %f", nx.x_x, nx.x_y);
		xfile >> nx.x_theta;
		markers.markers[count].header.frame_id = "map";
		markers.markers[count].header.stamp = ros::Time();
		markers.markers[count].ns = "my_namespace";
		markers.markers[count].id = count;
		markers.markers[count].type = visualization_msgs::Marker::SPHERE;
		markers.markers[count].action = visualization_msgs::Marker::ADD;
		markers.markers[count].pose.position.x = (nx.x_x + costmap_width*costmap_resolution/2)  + originpoint[0];
		markers.markers[count].pose.position.y = (nx.x_y + costmap_height*costmap_resolution/2)  + originpoint[1];
		markers.markers[count].pose.position.z = 0;
		markers.markers[count].pose.orientation.x = 0.0;
		markers.markers[count].pose.orientation.y = 0.0;
		markers.markers[count].pose.orientation.z = 0.0;
		markers.markers[count].pose.orientation.w = 1.0;
		markers.markers[count].scale.x = costmap_resolution;
		markers.markers[count].scale.y = costmap_resolution;
		markers.markers[count].scale.z = costmap_resolution;
		markers.markers[count].color.a = 1.0f;
		markers.markers[count].color.r = 0.0f;
		markers.markers[count].color.g = 1.0f;
		markers.markers[count].color.b = 0.0f;
		markers.markers[count].lifetime = ros::Duration();
		xlist.push_back(nx);
		count++;
	}

	markers.markers[count].header.frame_id = "map";
	markers.markers[count].header.stamp = ros::Time();
	markers.markers[count].ns = "my_namespace";
	markers.markers[count].id = count;
	markers.markers[count].type = visualization_msgs::Marker::SPHERE;
	markers.markers[count].action = visualization_msgs::Marker::ADD;
	markers.markers[count].pose.position.x = (1.75 + costmap_width*costmap_resolution/2) + originpoint[0];
	markers.markers[count].pose.position.y = (0 + costmap_height*costmap_resolution/2) + originpoint[1];
	markers.markers[count].pose.position.z = 0;
	markers.markers[count].pose.orientation.x = 0.0;
	markers.markers[count].pose.orientation.y = 0.0;
	markers.markers[count].pose.orientation.z = 0.0;
	markers.markers[count].pose.orientation.w = 1.0;
	markers.markers[count].scale.x = 0.3;
	markers.markers[count].scale.y = 0.3;
	markers.markers[count].scale.z = 0.3;
	markers.markers[count].color.a = 1.0;
	markers.markers[count].color.r = 1.0f;
	markers.markers[count].color.g = 0.0f;
	markers.markers[count].color.b = 0.0f;
	markers.markers[count].lifetime = ros::Duration();
	count++;

	markers.markers[count].header.frame_id = "map";
	markers.markers[count].header.stamp = ros::Time();
	markers.markers[count].ns = "my_namespace";
	markers.markers[count].id = count;
	markers.markers[count].type = visualization_msgs::Marker::SPHERE;
	markers.markers[count].action = visualization_msgs::Marker::ADD;
	markers.markers[count].pose.position.x = (1 + costmap_width*costmap_resolution/2) + originpoint[0];
	markers.markers[count].pose.position.y = (2 + costmap_height*costmap_resolution/2) + originpoint[1];
	markers.markers[count].pose.position.z = 0;
	markers.markers[count].pose.orientation.x = 0.0;
	markers.markers[count].pose.orientation.y = 0.0;
	markers.markers[count].pose.orientation.z = 0.0;
	markers.markers[count].pose.orientation.w = 1.0;
	markers.markers[count].scale.x = 0.3;
	markers.markers[count].scale.y = 0.3;
	markers.markers[count].scale.z = 0.3;
	markers.markers[count].color.a = 1.0;
	markers.markers[count].color.r = 1.0f;
	markers.markers[count].color.g = 0.0f;
	markers.markers[count].color.b = 0.0f;
	markers.markers[count].lifetime = ros::Duration();
	count++;

	markers.markers[count].header.frame_id = "map";
	markers.markers[count].header.stamp = ros::Time();
	markers.markers[count].ns = "my_namespace";
	markers.markers[count].id = count;
	markers.markers[count].type = visualization_msgs::Marker::SPHERE;
	markers.markers[count].action = visualization_msgs::Marker::ADD;
	markers.markers[count].pose.position.x = (0 + costmap_width*costmap_resolution/2) + originpoint[0];
	markers.markers[count].pose.position.y = (1.25 + costmap_height*costmap_resolution/2) + originpoint[1];
	markers.markers[count].pose.position.z = 0;
	markers.markers[count].pose.orientation.x = 0.0;
	markers.markers[count].pose.orientation.y = 0.0;
	markers.markers[count].pose.orientation.z = 0.0;
	markers.markers[count].pose.orientation.w = 1.0;
	markers.markers[count].scale.x = 0.3;
	markers.markers[count].scale.y = 0.3;
	markers.markers[count].scale.z = 0.3;
	markers.markers[count].color.a = 1.0;
	markers.markers[count].color.r = 1.0f;
	markers.markers[count].color.g = 0.0f;
	markers.markers[count].color.b = 0.0f;
	markers.markers[count].lifetime = ros::Duration();
	count++;

	markers.markers[count].header.frame_id = "map";
	markers.markers[count].header.stamp = ros::Time();
	markers.markers[count].ns = "my_namespace";
	markers.markers[count].id = count;
	markers.markers[count].type = visualization_msgs::Marker::SPHERE;
	markers.markers[count].action = visualization_msgs::Marker::ADD;
	markers.markers[count].pose.position.x = (1 + costmap_width*costmap_resolution/2) + originpoint[0];
	markers.markers[count].pose.position.y = (1 + costmap_height*costmap_resolution/2) + originpoint[1];
	markers.markers[count].pose.position.z = 0;
	markers.markers[count].pose.orientation.x = 0.0;
	markers.markers[count].pose.orientation.y = 0.0;
	markers.markers[count].pose.orientation.z = 0.0;
	markers.markers[count].pose.orientation.w = 1.0;
	markers.markers[count].scale.x = 0.3;
	markers.markers[count].scale.y = 0.3;
	markers.markers[count].scale.z = 0.3;
	markers.markers[count].color.a = 1.0;
	markers.markers[count].color.r = 1.0f;
	markers.markers[count].color.g = 0.0f;
	markers.markers[count].color.b = 0.0f;
	markers.markers[count].lifetime = ros::Duration();
	count++;


	vis_pub.publish(markers);
	geometry_msgs::Twist control_msg;
	
	while(!ulist.empty()){
	control_msg.linear.x = ((ulist.front().ul + ulist.front().ur)/2);
	control_msg.angular.z = ((ulist.front().ur -ulist.front().ul)/0.35*1);
	geo_pub.publish(control_msg);
	markers.markers[count].header.frame_id = "map";
	markers.markers[count].header.stamp = ros::Time();
	markers.markers[count].ns = "my_namespace";
	markers.markers[count].id = count;
	markers.markers[count].type = visualization_msgs::Marker::SPHERE;
	markers.markers[count].action = visualization_msgs::Marker::ADD;
	markers.markers[count].pose.position.x = (costmap_width/2)*costmap_resolution  + originpoint[0];
	markers.markers[count].pose.position.y = (costmap_height/2)*costmap_resolution  + originpoint[1];
	markers.markers[count].pose.position.z = 0;
	markers.markers[count].pose.orientation.x = 0.0;
	markers.markers[count].pose.orientation.y = 0.0;
	markers.markers[count].pose.orientation.z = 0.0;
	markers.markers[count].pose.orientation.w = 1.0;
	markers.markers[count].scale.x = costmap_resolution;
	markers.markers[count].scale.y = costmap_resolution;
	markers.markers[count].scale.z = costmap_resolution;
	markers.markers[count].color.a = 1.0f;
	markers.markers[count].color.r = 0.0f;
	markers.markers[count].color.g = 0.0f;
	markers.markers[count].color.b = 1.0f;
	markers.markers[count].lifetime = ros::Duration();
	count++;
	vis_pub.publish(markers);
	ulist.pop_front();
	r.sleep();
	ros::spinOnce();
	}	
	control_msg.linear.x = 0.0;
	control_msg.angular.z = 0.0;
	geo_pub.publish(control_msg);
	new_map_available_flag = false;
	r.sleep();
	return 0;
}


		
		
