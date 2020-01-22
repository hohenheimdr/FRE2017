#include "ros/ros.h"

#include "message_filters/subscriber.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <msgs/IntStamped.h>

#include <std_msgs/Int64.h>

//reads the signals from joystick topics and or another subscriber
//and send the an activate signal to the usb_relais to ensure that they are activated 
//for a specific time rate
//input: std_msg::bool (subscriber)
//			double time
//output: std_msg::bool (publisher)

class Sender_Node
{
		
private:

public:
	// public variables
	double waittime;
	ros::Publisher mode_pub;
	ros::Time last_msg;
	

	Sender_Node()
	{
		last_msg=ros::Time::now();
	}
	
	~Sender_Node()
	{
	}
	
	//button subscribers
	void Input(const std_msgs::Bool::ConstPtr& msg)
	{
		ROS_INFO("got new topic");
		ros::Duration time_since_msg=ros::Time::now()-last_msg;
		last_msg=ros::Time::now();
		
		
		//activate the publisher for the defined time slot
		if (msg->data )//&& time_since_msg.toSec()>waittime)
		{
			std_msgs::Bool d=*msg;
			mode_pub.publish(d);
			sleep(waittime);
			d.data=false;
			mode_pub.publish(d);
		}
		//publish the value
			
	}
	

};
//-------------------------------------------------------------------------


int main (int argc, char** argv)
{
	
	std::string input_signal_str, output_signal_str;
	
	ros::init(argc,argv,"relais_controller");
	ros::NodeHandle n("~");

	n.param<std::string>("input_signal", input_signal_str, "/ball_detected"); 
	n.param<std::string>("output_signal", output_signal_str, "/relais2"); 

	Sender_Node s;
	//subscribers
	ros::Subscriber a=n.subscribe(input_signal_str,1,&Sender_Node::Input,&s);	
	n.param<double>("waittime", s.waittime, 0.0); 
	
	s.mode_pub = n.advertise<std_msgs::Bool>(output_signal_str.c_str(),1);

	ros::spin();
		
}
