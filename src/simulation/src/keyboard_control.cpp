/*
  keyboard_control.cpp
  For using WSADE to control the Simulation robot
  Date created: November 2013
  Author: Thien
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

class KeyboardControl
{
public:
  KeyboardControl();
  void keyLoop();

private:
  ros::NodeHandle nh;
  double x, y, z, roll, pitch, yaw;
  ros::Publisher position_pub;

};

KeyboardControl::KeyboardControl():
  x(0.0), y(0.0), z(0.0),
  roll(0.0), pitch(0.0), yaw(0.0)
{
  position_pub = nh.advertise<nav_msgs::Odometry>("/dataNavigator", 10);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_control");
  KeyboardControl keyboard_control;

  signal(SIGINT, quit);

  keyboard_control.keyLoop();

  return(0);
}


void KeyboardControl::keyLoop()
{
  char c;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and WSADQE to move the vehicle.");


  for(;;) {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    x = y = z = roll = pitch = yaw = 0;
    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c) {
      case KEYCODE_LEFT:
        ROS_DEBUG("LEFT");
        yaw = 0.2;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_DEBUG("RIGHT");
        yaw = -0.2;
        dirty = true;
        break;
      case KEYCODE_UP:
	ROS_DEBUG("UP");
	y = 0.2;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("DOWN");
	y = -0.2;
        dirty = true;
        break;
      case KEYCODE_W:
	pitch = -0.2;
	dirty = true;
	break;
      case KEYCODE_S:
	pitch = 0.2;
	dirty = true;
	break;
      case KEYCODE_A:
	roll = -0.2;
	dirty = true;
	break;
      case KEYCODE_D:
	roll = 0.2;
	dirty = true;
	break;
      case KEYCODE_Q:
	z = -0.2;
	dirty = true;
	break;
      case KEYCODE_E:
	z = 0.2;
	dirty = true;
	break;

    }

    if(dirty ==true) {
      nav_msgs::Odometry odom;
      /*
      odom.pose.pose.position.x=0.0;
      odom.pose.pose.position.y=0.0;
      odom.pose.pose.position.z=0.0;
      odom.pose.pose.orientation.x=0.0;
      odom.pose.pose.orientation.y=0.0;
      odom.pose.pose.orientation.z=0.0;
      odom.pose.pose.orientation.w=1;
*/
      odom.twist.twist.linear.x=x;
      odom.twist.twist.linear.y=y;
      odom.twist.twist.linear.z=z;
      odom.twist.twist.angular.x=pitch;
      odom.twist.twist.angular.y=roll;
      odom.twist.twist.angular.z=yaw;
      for (int i=0; i<36; i++) {
	odom.twist.covariance[i]=0;
	odom.pose.covariance[i]=0;
      }
      position_pub.publish(odom);

      position_pub.publish(odom);
      dirty=false;
    }
  }


  return;
}
