#define LPF_CONSTANT 0.7
#define FORWARD_VEL_MAX 0.3
#define SIDEMOVE_VEL_MAX 0.3
#define DEPTH_VEL_MAX 0.3
#define PITCH_VEL_MAX 0.2
#define ROLL_VEL_MAX 0.2
#define YAW_VEL_MAX 0.6


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include<srmauv_msgs/depth.h>
#include<srmauv_msgs/thruster.h>
#include <srmauv_msgs/pid_info.h>
#include<underwater_sensor_msgs/Pressure.h>



double fmap(int input, double in_min, double in_max, double out_min, double out_max);
float interpolateDepth(float adcVal);
void getPressure(const underwater_sensor_msgs::Pressure &msg);
void getThruster(const srmauv_msgs::thruster &msg);
void getPID(const srmauv_msgs::pid_info &msg);
void calculate_pose();


srmauv_msgs::depth depthVal;
srmauv_msgs::pid_info pid;
srmauv_msgs::thruster thruster_speed;
nav_msgs::Odometry odom;

ros::Subscriber thrusterSub;
ros::Subscriber pressureSub;
ros::Subscriber pid_infoSub;

ros::Publisher positionPub;
ros::Publisher pressurePub;

double lowpass_depth;


int main(int argc,char** argv){
	ros::init(argc,argv,"simulator");

	ros::NodeHandle nh;

	thrusterSub=nh.subscribe("/thruster_speed",1000,getThruster);
  pressureSub=nh.subscribe("/sedna/pressure",1000,getPressure);
	pid_infoSub=nh.subscribe("/pid_info",1000,getPID);

	pressurePub=nh.advertise<srmauv_msgs::depth>("/pressure_data",100);
	positionPub=nh.advertise<nav_msgs::Odometry>("/dataNavigator",100);

	ROS_INFO("Initializing simulator...");

	while(ros::ok()){
		pressurePub.publish(depthVal);
		positionPub.publish(odom);
		ros::spinOnce();


	}

	return 0;
}

float interpolateDepth(float adcVal){
	//do depth interpolation here
	return adcVal;
}
void getPressure(const underwater_sensor_msgs::Pressure &msg){

	// the message that is coming is the raw ADC value from the pressure sensor.. lets try to add some filtering to it somewhere else probably

	double depth=(double)interpolateDepth((float)msg.pressure*100);

	//ROS_INFO("%d",msg.depth);

	double new_pressure=depth;
	lowpass_depth=lowpass_depth+ LPF_CONSTANT*(float)(new_pressure-lowpass_depth);


	depthVal.depth=lowpass_depth;
  depthVal.pressure=msg.pressure;

}



void getThruster(const srmauv_msgs::thruster &msg){

	thruster_speed=msg;
	calculate_pose();
}

double fmap(int input, double in_min, double in_max, double out_min, double out_max){

	return (input- in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void calculate_pose(){

	//ROS_INFO("Velx: %ld", (double)odom.twist.twist.linear.x);
//	ROS_INFO("Velx_raw: %i", (int)thruster_speed.speed1);

	odom.pose.pose.position.x=0.0;
	odom.pose.pose.position.y=0.0;
	odom.pose.pose.position.z=0.0;
	odom.pose.pose.orientation.x=0.0;
	odom.pose.pose.orientation.y=0.0;
	odom.pose.pose.orientation.z=0.0;
	odom.pose.pose.orientation.w=1;

	odom.twist.twist.linear.x=0;
	odom.twist.twist.linear.y=(double)fmap(thruster_speed.speed1,-255,255,-FORWARD_VEL_MAX,FORWARD_VEL_MAX);
	odom.twist.twist.linear.z=(double)fmap(thruster_speed.speed3,-400,400,-DEPTH_VEL_MAX,DEPTH_VEL_MAX);


	odom.twist.twist.angular.x=0;//(double)fmap(-pid.pitch.total,-350,350,-PITCH_VEL_MAX,PITCH_VEL_MAX); //pitch : check actuator constraints in controller/cfg
	odom.twist.twist.angular.y=0;//(double)fmap(pid.roll.total,-350,350,-ROLL_VEL_MAX,ROLL_VEL_MAX);//roll : check actuator constraints in controller/cfg
	odom.twist.twist.angular.z=(double)fmap(pid.heading.total,-350,350,-YAW_VEL_MAX,YAW_VEL_MAX);//yaw

	for (int i=0; i<36; i++) {
odom.twist.covariance[i]=0;
odom.pose.covariance[i]=0;
	};

}



void getPID(const srmauv_msgs::pid_info &msg){
	pid=msg;

}
