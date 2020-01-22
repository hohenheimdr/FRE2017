#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/Bool.h>
#include "sensor_msgs/PointCloud2.h"
#include <msgs/IntStamped.h>

#include <std_msgs/Int64.h>


#include <pcl_ros/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


// dr 12.6.17
//reads in a pointcloud2 msgs (ideally created from the ball detector) together with a goal frame_id
// the node transforms the pointcloud in the defined frame, and check the minimal point distance 
//to the frame. When the Threshold is reached, a bool msgs gets activated.
// this msgs could then be sent to the relais to activate the nozzles


class Sender_Node
{
		
private:

public:
	// public variables
	double goal_distance,waittime;
	std::string goal_frame_id;
	ros::Publisher mode_pub;
	

	Sender_Node()
	{
	}
	
	~Sender_Node()
	{
	}
	
	void readCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{
		//need some error handling?! check later!
		 ROS_INFO("got new goals");
		 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		 sensor_msgs::PointCloud2 transformed_cloud=*msg; 
		 try{
			tf::TransformListener tf_listener;
			tf_listener.waitForTransform(goal_frame_id,(*msg).header.frame_id,(*msg).header.stamp,ros::Duration(0.1));
			pcl_ros::transformPointCloud(goal_frame_id,*msg,transformed_cloud,tf_listener);
			}catch(tf::ExtrapolationException e)
			{
				ROS_INFO("error doing point cloud tranform");
			}
	
		 pcl::fromROSMsg(transformed_cloud,*cloud);	
		
		 cloud=filter_nan_points(cloud);

		if (get_min_distance(cloud)<goal_distance)
			{
				//publish true value when point close enough to tf goal
				ROS_INFO("min value");
				std_msgs::Bool out;
				out.data=true;
				mode_pub.publish(out);
			}	
	}
	
		
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter_nan_points(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
	{
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*input,*input,indices);
		
		return input;	
	}
	
	
	double distance(pcl::PointXYZ in)
	{
	//std::cout<<"("<<in.x<<"/"<<in.y<<"/"<<in.z<<")"<<std::endl;
	double result=double(sqrt(in.x*in.x+in.y*in.y+in.z*in.z));
	return result;
	}
	
  double get_min_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
	double dist_min=10;
	//get smallest distance for front
	for(int i=0;i<cloud->points.size();i++)
			{
				double act_dist=distance(cloud->points[i]);
				if(act_dist<dist_min){
					dist_min=act_dist;
					}
			}
	return dist_min;		
	}

};
//-------------------------------------------------------------------------


int main (int argc, char** argv)
{
	
	std::string input_signal_str, output_signal_str,cloud_str;
	
	ros::init(argc,argv,"relais_controller");
	ros::NodeHandle n("~");

	
	n.param<std::string>("output_signal", output_signal_str, "/relais2"); 
	n.param<std::string>("cloud_in", cloud_str, "/golf_balls"); 


	Sender_Node s;
	//subscribers
	n.param<double>("goal_distance", s.goal_distance, 0.25); 
	n.param<double>("waittime", s.waittime, 0.25); 
	n.param<std::string>("goal_frame_id	", s.goal_frame_id, "/nozzle_1"); 
	
	ros::Subscriber ball_cloud = n.subscribe(cloud_str,1, &Sender_Node::readCloud, &s);
	
	s.mode_pub = n.advertise<std_msgs::Bool>(output_signal_str.c_str(),10);


	ros::spin();
		
}
