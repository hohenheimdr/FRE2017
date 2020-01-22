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
//and send the signal to the relais driver from drcontrol
//be aware that drcontrol.py must have root previleges!
//sudo chown root drcontrol.py
//sudo chmod 4755 drcontrol.py

//estimated path for drcontrol.py is ~/drcontrol/drcontrol/drcontrol.py

std::string driver_path="python ~/drcontrol/drcontrol/drcontrol.py";

class Sender_Node
{
		
private:

public:
	// public variables
	

	Sender_Node()
	{
		system(driver_path.c_str());
	}
	
	~Sender_Node()
	{
	}
	
	//button subscribers
	void Row_button(const std_msgs::Bool::ConstPtr& msg)
	{
		ROS_INFO("run button 1");
			
		if (msg->data){
			system("python ~/drcontrol/drcontrol/drcontrol.py -d A50545WE -c on -r 2 -v");
		}else
		{
			system("python ~/drcontrol/drcontrol/drcontrol.py -d A50545WE -c off -r 2 -v");
		}
	}
	
	void Stop_Button(const std_msgs::Bool::ConstPtr& msg)
	{
		ROS_INFO("run button 2");

		if (msg->data){
			system("python ~/drcontrol/drcontrol/drcontrol.py -d A50545WE -c on -r 6 -v");
		}else
		{
			system("python ~/drcontrol/drcontrol/drcontrol.py -d A50545WE -c off -r 6 -v");
		}
	}
	
	void Action_Button_1(const std_msgs::Bool::ConstPtr& msg)
	{
		ROS_INFO("run button 3");
			if (msg->data){
			system("python ~/drcontrol/drcontrol/drcontrol.py -d A50545WE -c on -r 1 -v");
		}else
		{
			system("python ~/drcontrol/drcontrol/drcontrol.py -d A50545WE -c off -r 1 -v");
		}
	}
	
	void Action_Button_2(const std_msgs::Bool::ConstPtr& msg)
	{
		ROS_INFO("run button 4");
		if (msg->data){
			system("python ~/drcontrol/drcontrol/drcontrol.py -d A50545WE -c on -r 5 -v");
		}else
		{
			system("python ~/drcontrol/drcontrol/drcontrol.py -d A50545WE -c off -r 5 -v");
		}
	}
	
	
	double abs_d(double t)
	{
		return sqrt(t*t);
	}

};
//-------------------------------------------------------------------------


int main (int argc, char** argv)
{
	
	std::string row_button_str,stop_button_str,action_button_1_str,action_button_2_str,odom_reset_str,distance_str;
	
	ros::init(argc,argv,"relais_controller");
	ros::NodeHandle n("~");

	n.param<std::string>("relais_1", row_button_str, "/relais1"); 
	n.param<std::string>("relais_2", stop_button_str, "/relais2"); 
	n.param<std::string>("relais_3", action_button_1_str, "/relais3"); 
	n.param<std::string>("relais_4", action_button_2_str, "/relais4"); 
	
 
	Sender_Node s;
	//subscribers
	ros::Subscriber a=n.subscribe(row_button_str,10,&Sender_Node::Row_button,&s);	
	ros::Subscriber b=n.subscribe(stop_button_str,10,&Sender_Node::Stop_Button,&s);
	ros::Subscriber x=n.subscribe(action_button_1_str,10,&Sender_Node::Action_Button_1,&s);
	ros::Subscriber y=n.subscribe(action_button_2_str,10,&Sender_Node::Action_Button_2,&s);
	
	ros::spin();
		
}
