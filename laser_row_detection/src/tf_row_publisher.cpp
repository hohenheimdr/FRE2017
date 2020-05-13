#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <ros/subscriber.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Bool.h"
#include <std_msgs/Float64MultiArray.h>
#include "nav_msgs/Odometry.h"
#include <math.h>
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>


// short description:
//reads in the two poses of left and right row, and gives out a tf_ transform for the 
//point follower out. In the ideal case, it is in a defined distance to the robot base...
//date 2.6.16
//author: David Reiser

//input: offset_x defines the distance of the goal
//offset_y descripe an additional offset to the y value of the goal


class TF_PUBLISHER
{
	private:
	geometry_msgs::TransformStamped row_goal;
	
public:																					//Bestimmung der Variablen

	tf::TransformBroadcaster br;
	tf::TransformListener li;
	double offset_x,offset_y,robot_width;
	std::string tf_name;

	geometry_msgs::PoseStamped left_row, right_row,result_pose, zero_pose, transformed_row;
	
	ros::Publisher headland_pub;
	
	TF_PUBLISHER()
	{
		zero_pose.pose.position.x=0;
		zero_pose.pose.position.y=0;
		zero_pose.pose.position.z=0;
		zero_pose.pose.orientation.x=0;
		zero_pose.pose.orientation.y=0;
		zero_pose.pose.orientation.z=0;
		zero_pose.pose.orientation.w=0;
	}
	
	~ TF_PUBLISHER()
	{}
	
	void Position_left (const geometry_msgs::PoseStamped::ConstPtr& msg)				//Einlesen und Umbenennen der Variablen aus Ransac Filter Linke Seite
	{
		left_row=*msg;
	}
	
	void Position_right (const geometry_msgs::PoseStamped::ConstPtr& msg)				//Einlesen und Umbenennen der Variablen aus Ransac Filter Rechte Seite
	{
		right_row=*msg;
	}
	
	double fabs(double input)															//Bildung des Betrages vom Input 
	{
		return sqrt(input*input);
	}
	
	void Compare(const ros::TimerEvent& e)												
	{
		std_msgs::Bool headland;
		//check that at least one row was detected!
	    if(right_row.pose.position.y!=0 || left_row.pose.position.y!=0)
	    {	
			//do some error handling if one side detected no line..
			if(right_row.pose.position.y==0) 
				right_row.pose.position.y=-(robot_width/2.0);
			if(left_row.pose.position.y==0) 
				left_row.pose.position.y=(robot_width/2.0);
				
		result_pose.header.stamp=ros::Time::now();
		result_pose.header.frame_id=left_row.header.frame_id;
		result_pose.pose.position.x=offset_x; //(left_row.pose.position.x+right_row.pose.position.x)/2+ offset_x;
		result_pose.pose.position.y=(left_row.pose.position.y+right_row.pose.position.y)/2+offset_y;
		result_pose.pose.position.z=0;
		result_pose.pose.orientation.x=0; //(left_row.pose.orientation.x+right_row.pose.orientation.x)/2;
		result_pose.pose.orientation.y=0; //(left_row.pose.orientation.y+right_row.pose.orientation.y)/2;
		result_pose.pose.orientation.z=0; //(left_row.pose.orientation.z+right_row.pose.orientation.z)/2;
		result_pose.pose.orientation.w=1; //(left_row.pose.orientation.w+right_row.pose.orientation.w)/2;
		//PublishTransform(result_pose);
		TransformRightPoseToRowPoint();
		TransformResultPose();
		PublishTransform(result_pose);
		headland.data=false;
		headland_pub.publish(headland);
		}else
		{
			ROS_INFO("error with line detection" );
			zero_pose.header.stamp=ros::Time::now();
			zero_pose.header.frame_id=left_row.header.frame_id;
			PublishTransform(zero_pose);
			//publish that headland was detected!
			headland.data=true;
			headland_pub.publish(headland);		
			PublishTransform(zero_pose);
			sleep(1);
			
		}
	}
	
	void TransformRightPoseToRowPoint()
	{
		//transform_row to row_point coordinate system
		//this program tries first to transform the ball to the odom frame...
			try{
						tf::StampedTransform transform;
						//need to transform the ball from the robot camera position to the fixed position
						//ROS_INFO("adding a relative transform to goal!");
						li.waitForTransform(right_row.header.frame_id,tf_name,ros::Time(0),ros::Duration(3.0));		
						//li.lookupTransform(right_row.header.frame_id,tf_name,ros::Time(0), transform);	
						li.transformPose(tf_name,ros::Time(0),right_row,right_row.header.frame_id,transformed_row);
					}catch(...)
					{
						ROS_INFO("error trying relative goal transform");
					}
	}
	
	void TransformResultPose()
	{
		//Transform the pose of row point the the needed distance...
		//result_pose=transformed_row;
		std::cout<<"distance_right row" <<transformed_row.pose.position.y<<std::endl;
	}
	
	void PublishTransform(geometry_msgs::PoseStamped in)
	{
			row_goal.header.stamp=ros::Time::now();
			row_goal.header.frame_id=in.header.frame_id;
			row_goal.child_frame_id =tf_name;
			row_goal.transform.translation.x= in.pose.position.x;
			row_goal.transform.translation.y= in.pose.position.y;
			row_goal.transform.translation.z= in.pose.position.z;
			row_goal.transform.rotation.x = in.pose.orientation.x;
			row_goal.transform.rotation.y = in.pose.orientation.y;
			row_goal.transform.rotation.z = in.pose.orientation.z;
			row_goal.transform.rotation.w = in.pose.orientation.w;
			br.sendTransform(row_goal);	
			
	}
	
	
};

int main(int argc, char** argv)
{

ros::init(argc, argv, "ransac");

ros::NodeHandle a("~");

 TF_PUBLISHER b;
 double frequency;
 std::string pose_left,pose_right,correction, headland,twist;
	a.param<std::string>("pose_left",pose_left, "/ransac_left_pose");
	a.param<std::string>("pose_right",pose_right, "/ransac_right_pose");
	a.param<std::string>("headland_out",headland,"/headland");
	a.param<std::string>("tf_name",b.tf_name,"/row_tf");
	a.param<double>("frequency",frequency,5);
	a.param<double>("offset_x",b.offset_x,1.0);
	//parameter for setting the goal point when one row was not detected (start parameter for the phoenix)
	a.param<double>("robot_width",b.robot_width,0.5);
	
	ros::Subscriber sub_left_r = a.subscribe(pose_left,1, &TF_PUBLISHER::Position_left, &b); 		
	ros::Subscriber sub_right_r = a.subscribe(pose_right,1, &TF_PUBLISHER::Position_right, &b); 	

	b.headland_pub=a.advertise<std_msgs::Bool>(headland.c_str(), 1);				  	 		//Publizierung der Headlanderkennung im Topic
	
	ros::Timer t = a.createTimer(ros::Duration(1.0/frequency),&TF_PUBLISHER::Compare,&b);
	
	ros::spin();
	
}
