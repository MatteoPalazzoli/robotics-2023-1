#include "ros/ros.h"
#include <time.h>
#include <math.h>
#include <string>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "first_project/Odom.h"
#include "first_project/reset_odom.h"

#include <tf/transform_broadcaster.h>

#define LENGTH 2.8

//global because of the service call outside the object call
double stateX, stateY, stateTh;
double speedX, speedY, angular;
int seq = 0;

class pub_sub
{
	geometry_msgs::Quaternion message;
	ros::Time oldtime, time;
	ros::Duration dT;
	double speed, steer, deltaT, omega;

	private:
		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Publisher pub, pub2; 
		ros::Timer timer1;
		tf::TransformBroadcaster br;
		
	public:
  	pub_sub(){
  		
  		//wait until the bag starts (to set the time to simulation time)
  		do{
  			time = ros::Time::now();
  		}while(time.toSec() == 0);
  		
  		/* initialization of the state with parameters */
  		std::string param_name;
  		if (n.searchParam("starting_x", param_name)) {
      	n.getParam(param_name, stateX);
    	} else {
      	ROS_WARN("Parameter 'starting_x' not defined");
    	}
    	if (n.searchParam("starting_y", param_name)) {
      	n.getParam(param_name, stateY);
    	} else {
      	ROS_WARN("Parameter 'starting_y' not defined");
    	}
    	if (n.searchParam("starting_th", param_name)) {
      	n.getParam(param_name, stateTh);
    	} else {
      	ROS_WARN("Parameter 'starting_th' not defined");
    	}
  		
  		/* subscriber and publisher creation */
  		sub = n.subscribe("/speed_steer", 1, &pub_sub::callback, this);
			pub = n.advertise<first_project::Odom>("/custom_odometry", 1);
			pub2 = n.advertise<nav_msgs::Odometry>("/odometry", 1);
			timer1 = n.createTimer(ros::Duration(0.2), &pub_sub::myPublish, this);
			ROS_INFO("Listening on /speed_steer topic...");
    	
		}
	
	void callback(const geometry_msgs::Quaternion::ConstPtr& msg){
		tf::Transform transform;
		double stateXNew, stateYNew;
		
		//get data
		speed = msg->x;
		steer = msg->y;
		
		//update deltaT
		oldtime = time;
		time = ros::Time::now();
		dT = time - oldtime;
		deltaT = dT.toSec();
		
		//update coordinates
		stateXNew = stateX + speed * cos(steer) * cos(stateTh) * deltaT;
		stateYNew = stateY + speed * cos(steer) * sin(stateTh) * deltaT;
		//assuming that the speed is referring to the front wheels
		omega = speed * sin(steer) / LENGTH;
		stateTh = stateTh + omega * deltaT;
		
		//tf
		transform.setOrigin( tf::Vector3(stateXNew, stateYNew, 0) );
		tf::Quaternion q;
		q.setRPY(0, 0, stateTh);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "vehicle_centre"));
		
		//update globals for std Odometry 
		speedX = (stateXNew - stateX) / deltaT;
		speedY = (stateYNew - stateY) / deltaT;
		angular = omega;
		
		//update state
		stateX = stateXNew;
		stateY = stateYNew;
	}

	void myPublish(const ros::TimerEvent&)
	{
		//covariance matrix init
		boost::array<double, 36> covariance = { 
		{ 0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0,
			0,0,0,0,0,0} };
		
		//generate the custom message
		first_project::Odom msg;
		msg.x = stateX;
		msg.y = stateY;
		msg.th = stateTh;
		msg.timestamp = std::to_string(time.toSec());
		
		//publish the custom message
		pub.publish(msg);
		
		//generate Odometry message
		geometry_msgs::Point p;
		geometry_msgs::Quaternion q;
		geometry_msgs::Pose P;
		geometry_msgs::Twist twist;
		geometry_msgs::Vector3 lin, ang;
		geometry_msgs::PoseWithCovariance pwc;
		geometry_msgs::TwistWithCovariance twc;
		std_msgs::Header h;
		nav_msgs::Odometry o;
		
		//POSE
		p.x = stateX;
		p.y = stateY;
		p.z = 0;
		q.x = 0;
		q.y = 0;
		q.z = 1;
		q.w = stateTh;
		P.position = p;
		P.orientation = q;
		pwc.pose = P;
		pwc.covariance = covariance;
		
		//TWIST
		lin.x = speedX;
		lin.y = speedY;
		lin.z = 0;
		ang.x = 0;
		ang.y = 0;
		ang.z = angular;
		twist.linear = lin;
		twist.angular = ang;
		twc.twist = twist;
		twc.covariance = covariance;
		
		//HEADER
		seq++;
		h.stamp = time;
		h.seq = seq;
		h.frame_id = "odom";
		
		//Main msg
		o.header = h;
		o.child_frame_id = "vehicle_centre";
		o.pose = pwc;
		o.twist = twc;
		
		//publish
		pub2.publish(o);
	}
};
	
bool global_reset(first_project::reset_odom::Request &req, first_project::reset_odom::Response &res)
	{
  	stateX = 0;
  	stateY = 0;
  	stateTh = 0;
  	ROS_INFO("Odometry set to zero via service.");
  	res.resetted = true;
  	return true;
	}

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "odom_node");
 	pub_sub my_pub_sub;
 	
 	/* service setting */
 	ros::NodeHandle n1;
  ros::ServiceServer service = n1.advertiseService("reset_odom", global_reset);
  ROS_INFO("'reset_odom' service server started.");
  
 	ros::spin();
 	return 0;
}
