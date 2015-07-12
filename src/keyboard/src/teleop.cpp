// teleop.heading_setpoint in runMission for bucket can be set to EAST  !


#define NORTH -180
#define DEPTH_SAUVC -90
#define DEPTH_BUCKET -80
#define DEPTH_BUOY -80

#include <srmauv_msgs/teleop_sedna.h>
#include <ros/ros.h>
#include<keyboard/Key.h>
#include <dynamic_reconfigure/server.h>
#include <srmauv_msgs/depth.h>
#include<sensor_msgs/Imu.h>
#include <std_msgs/Int16.h>
#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose2D.h>
#include <srmauv_msgs/goal.h>
#include <srmauv_msgs/line.h>
#include <srmauv_msgs/buoy.h>
#include <srmauv_msgs/bucket.h>
#include <sensor_msgs/Imu.h>
#include <srmauv_msgs/missions.h>
#include <srmauv_msgs/gate.h>

srmauv_msgs::teleop_sedna teleop;
srmauv_msgs::depth depth;
srmauv_msgs::line line;
srmauv_msgs::buoy buoy;
srmauv_msgs::bucket bucket;
srmauv_msgs::gate gate;
srmauv_msgs::missions missions;
keyboard::Key key;
std_msgs::Bool inLineBool;
std_msgs::Bool inBuoyBool;
std_msgs::Bool inBucketBool;
std_msgs::Bool dropBall;

int pressure;
double yaw,pitch,roll;

ros::Subscriber keyDown_sub;
ros::Subscriber keyUp_sub;
ros::Subscriber pressureSub;
ros::Subscriber teleopSetter;
ros::Subscriber headingSub;
ros::Subscriber imuSub;
ros::Subscriber lineSub;
ros::Subscriber buoySub;
ros::Subscriber bucketSub;
ros::Subscriber gateSub;

ros::Publisher teleopPub;
ros::Publisher inLinePub;
ros::Publisher inBuoyPub;
ros::Publisher inBucketPub;
ros::Publisher dropperPub;
ros::Publisher missionsPub;


int north = NORTH;
int south= NORTH+179;
int east = NORTH-90;
int west = NORTH+ 90 ;

int limit(int value, int lower, int upper);
void keyUp(const keyboard::KeyConstPtr& key);
void keyDown(const keyboard::KeyConstPtr& key);
void getPressure(const srmauv_msgs::depth &msg);

void getHeading(const geometry_msgs::Pose2D::ConstPtr& msg);
void setTeleop(const srmauv_msgs::goal::ConstPtr& msg);
void setCurrent();
void runMission();
void getLine(const srmauv_msgs::line::ConstPtr&msg);
void getBuoy(const srmauv_msgs::buoy::ConstPtr&msg);
void getGate(const srmauv_msgs::gate::ConstPtr&msg);
void getBucket(const srmauv_msgs::bucket::ConstPtr&msg);
void getImu(const sensor_msgs::Imu::ConstPtr& msg);

void stop_missions();
void stop_front_missions();
void stop_bottom_missions();

bool in_depth=false;
bool in_yaw=false;
bool in_roll=false;
bool in_pitch=false;
bool inBucket=false;

bool inSidemove=false;
bool inForward=false;

bool shift=false;
bool pid=false;
//ros::NodeHandle *nh;


int main(int argc,char** argv){
  ros::init(argc,argv,"teleop_sedna");
  ros::NodeHandle nh;


  teleop.enable=true;
  teleop.tune=false;
  teleop.depth_enable=false;
  teleop.pid_enable=false;
  teleop.depth_setpoint=DEPTH_SAUVC;
  teleop.heading_setpoint=north;
  dropBall.data=false;
  stop_missions();

  keyDown_sub=nh.subscribe("/keyboard/keydown",1000,keyDown);
  keyUp_sub=nh.subscribe("/keyboard/keyup",1000,keyUp);
  pressureSub=nh.subscribe("/pressure_data",1000,getPressure);
  imuSub=nh.subscribe("/imu/data",1000,getImu);
  headingSub=nh.subscribe("/imu/HeadingTrue_degree",1000,getHeading);
  teleopSetter=nh.subscribe("/teleop_set",1000,setTeleop);
  lineSub=nh.subscribe("/line_follower",1000,getLine);
  buoySub=nh.subscribe("/buoy",1000,getBuoy);
  bucketSub=nh.subscribe("/bucket",1000,getBucket);
  imuSub=nh.subscribe("/imu/data",1000,getImu);
  gateSub=nh.subscribe("/gate",1000,getGate);

  missionsPub=nh.advertise<srmauv_msgs::missions>("/missions",100);
  teleopPub=nh.advertise<srmauv_msgs::teleop_sedna>("/teleop_sedna",1000);
  dropperPub=nh.advertise<std_msgs::Bool>("/dropper",100);
  ROS_INFO("Teleop dispatcher initialized..");


  // teleop_sub=nh.subscribe("")

  while(ros::ok()){



    //  ROS_INFO("Teleop: %d\tDepth: %d\tHeading: %f\tForward: %d\tReverse: %d\tStrafe: %d",
    //            teleop.enable,teleop.depth_setpoint,teleop.heading_setpoint,teleop.forward_speed,teleop.reverse_speed,teleop.sidemove_speed);

    runMission();
    teleopPub.publish(teleop);
    missionsPub.publish(missions);
    ROS_INFO("Spinning..");





    ros::spinOnce();
    //  ros::spin();

  }

  return 0;
}




void stop_missions(){
  stop_front_missions();
  stop_bottom_missions();
}

void stop_front_missions(){
  missions.buoy_red=false;
  missions.buoy_green=false;
  missions.gate=false;
  missions.style=false;
  missions.torpedo_approach=false;
  missions.torpedo_lid=false;
  missions.torpedo_primary=false;
  missions.torpedo_secondary=false;
  teleop.forward_input=0;
  teleop.sidemove_input=0;

}

void stop_bottom_missions(){
  missions.guide=false;
  missions.dropper_approach=false;
  missions.dropper_lid=false;
  missions.dropper_primary=false;
  missions.dropper_secondary=false;
  teleop.forward_input=0;
  teleop.sidemove_input=0;
}

void runMission(){

  //dropperPub.publish(dropBall);
  if(missions.guide && line.possible){

    teleop.heading_setpoint=yaw + line.heading;
    teleop.sidemove_input=line.distance;

  }


     if (missions.buoy_red && buoy.possible_red){

    teleop.sidemove_input=buoy.x_offset_red;
    teleop.depth_setpoint=DEPTH_BUOY;
  }

  if (missions.buoy_green && buoy.possible_green){

    teleop.sidemove_input=buoy.x_offset_green;
    teleop.depth_setpoint=DEPTH_BUOY;
  }

  if(inBucket && bucket.possible ){

    teleop.forward_input=bucket.y_offset;
    teleop.sidemove_input=bucket.x_offset;

}

  if(missions.gate && gate.possible){
    teleop.sidemove_input=gate.x_offset;
  }


}

void getPressure(const srmauv_msgs::depth &msg){
  depth.depth=msg.depth;

}



void getHeading(const geometry_msgs::Pose2D::ConstPtr& msg){
  //  yaw=msg->theta;
}

void getImu(const sensor_msgs::Imu::ConstPtr& msg){
  double roll,pitch,heading;
  tf::Quaternion q;

  tf::quaternionMsgToTF(msg->orientation,q);
  // tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  // tf::Matrix3x3(q).getEulerYPR(yaw,pitch,roll);
  tf::Matrix3x3(q).getEulerZYX(heading,pitch,roll);
  //  tf::Matrix3x3(q).
  yaw=heading/M_PI*180;

  //int x=5;
}

void getLine(const srmauv_msgs::line::ConstPtr&msg){
  line=*msg;
}

void getBuoy(const srmauv_msgs::buoy::ConstPtr&msg){
  buoy=*msg;
}

void getGate(const srmauv_msgs::gate::ConstPtr&msg){
  gate=*msg;
}


void getBucket(const srmauv_msgs::bucket::ConstPtr&msg){
  bucket.possible=msg->possible;
  bucket.x_offset=msg->x_offset;
  bucket.y_offset=msg->y_offset;
}



void setCurrent(){
  teleop.depth_setpoint=depth.depth;
  teleop.heading_setpoint=yaw;
  teleop.forward_speed=0;
  teleop.sidemove_speed=0;
  teleop.reverse_speed=0;
}

void setTeleop(const srmauv_msgs::goal::ConstPtr &msg){
  if(msg->goDepth==true)
  teleop.depth_setpoint=msg->depth;
  if(msg->goHeading)
  teleop.heading_setpoint=msg->heading;
  if(msg->goPitch)
  teleop.pitch_setpoint=msg->pitch;
  if(msg->goRoll)
  teleop.roll_setpoint=msg->roll;

  //      teleop.depth_setpoint=limit(teleop.depth_setpoint,0,800);
  teleop.heading_setpoint=limit(teleop.heading_setpoint,-179,179);
  teleop.pitch_setpoint=limit(teleop.pitch_setpoint,-60,60);
  teleop.roll_setpoint=limit(teleop.roll_setpoint,-179,179);

  ROS_INFO("Received new goal !.. Depth[%d] Heading[%d] Pitch[%d] Roll[%d]",teleop.depth_setpoint,teleop.heading_setpoint,teleop.pitch_setpoint,
  teleop.roll_setpoint);

}

void keyDown(const keyboard::KeyConstPtr & key){
  // ROS_INFO("%d pressed",key->code);
  /*
  ROS_INFO("\n\nTeleop: [%s]\t\tDepth Enable[%s]\nHeading Setpoint [%d]\tDepth Setpoint [%d]\nPitch Setpoint [%d]\tRoll Setpoint [%d]\n"
  "Forward Velocity [%d]\tSidemove Velocity [%d]\nIn LineFollower [%s]\tDropper [%s]",
  teleop.enable?"True":"False",teleop.depth_enable?"True":"False",teleop.heading_setpoint,teleop.depth_setpoint,teleop.pitch_setpoint,
  teleop.roll_setpoint,teleop.forward_speed,teleop.sidemove_speed,inLine?"True":"False",teleop.dropper?"True":"False");
  */

  switch(key->code){
    case 116: {   //t
      teleop.enable=!teleop.enable;
      break;
    }

    case 100: {
      teleop.depth_enable=!teleop.depth_enable;
      break;
    }

    case 96 : {// ~ tune
      teleop.tune=!teleop.tune;
      break;  }
    }

    if(teleop.enable){
      switch(key->code){
        case 117: {   //u
          //  teleop.depth_setpoint=depth.depth;
          //    teleop.heading_setpoint=yaw;
          //   ROS_INFO("Setpoints Updated ! -> Depth: %d\tHeading :%d ",teleop.depth_setpoint,(int)teleop.heading_setpoint );
          break;
        }
        case 49:{
          teleop.heading_setpoint=north;
          break;
        }
        case 50:{
          teleop.heading_setpoint=east;
          break;
        }
        case 51:{
          teleop.heading_setpoint=west;
          break;
        }
        case 52:{
          teleop.heading_setpoint=south;
          break;
        }

        case 120 : { //x : drop ball !
          dropBall.data=true;
          dropperPub.publish(dropBall);
          dropBall.data=false;
          break;
        }




        case 45 : {  // -
          shift?teleop.depth_setpoint-=10 : teleop.depth_setpoint--;
          break;
        }
        case 61: {  //=
          shift?teleop.depth_setpoint+=10 : teleop.depth_setpoint++;
          break;

        }
        case 115 : { //s : enable/disable sidemove pid
          inSidemove=!inSidemove;
          break;
        }
    
        case 91: { // {
          shift?teleop.heading_setpoint-=10 : teleop.heading_setpoint--;
          break;
        }
        case 93: { // }
        shift?teleop.heading_setpoint+=10 : teleop.heading_setpoint++;
        break;
      }




      case 304: {  //shift
        shift=true;
        break;

      }

      case 48: { // 0
        shift?teleop.pitch_setpoint+=10 : teleop.pitch_setpoint++;
        break;
      }
      case 57: { // 9
        shift?teleop.pitch_setpoint-=10 : teleop.pitch_setpoint--;
        break;
      }
      case 56: { // 8
        shift?teleop.roll_setpoint+=10 : teleop.roll_setpoint++;
        break;
      }
      case 55: { // 7
        shift?teleop.roll_setpoint-=10 : teleop.roll_setpoint--;
        break;
      }

      case 273: { // forward
        teleop.forward_speed=210;
        break;
      }

      case 274: { //reverse
        teleop.forward_speed=-210;
        break;
      }
      case 276 :{ //left
        teleop.sidemove_speed=-220;
        break;
      }
      case 275 :{ //right
        teleop.sidemove_speed=220;
        break;
      }
      case 112:{ //p : enable disable controllers except depth
        teleop.pid_enable=!teleop.pid_enable;
        break;
      }

      case 108: {  // l : enable/disable guide marker
        missions.guide=!missions.guide;

        //inSidemove=!inSidemove;
        if(missions.guide){
          stop_missions();
          missions.guide=true;
        }
        else{
        stop_missions();
        }

        break;
      }

      case 98:{ // b : enable/disable dropper approach
        missions.dropper_approach=!missions.dropper_approach;

        if(missions.dropper_approach){
        stop_missions();
        missions.dropper_approach=true;
        }
        else{
        stop_missions();
        }

        break;
      }
      case 114 : { //r enable disable red_buoy
        missions.buoy_red=!missions.buoy_red;
        //inSidemove=!inSidemove;


        if(missions.buoy_red){
          stop_missions();
          missions.buoy_red=true;
        }
        else{
        stop_missions();
        }

        break;
      }

      case 103 : { //g enable disable green_buoy
        missions.buoy_green=!missions.buoy_green;
        //inSidemove=!inSidemove;


        if(missions.buoy_green){
          stop_missions();
          missions.buoy_green=true;
        }
        else{
        stop_missions();
        }

        break;
      }
      case 113 : { //q enable disable qualification/gate
        missions.gate=!missions.gate;
        //inSidemove=!inSidemove;


        if(missions.gate){
          stop_missions();
          missions.gate=true;
        }
        else{
        stop_missions();
        }

        break;
      }


    }


    // teleop.depth_setpoint=limit(teleop.depth_setpoint,0,800);
    // teleop.heading_setpoint=limit(teleop.heading_setpoint,-179,179);
    // teleop.pitch_setpoint=limit(teleop.pitch_setpoint,-60,60);
    //teleop.roll_setpoint=limit(teleop.roll_setpoint,-179,179);

  }

}


int limit(int value, int lower, int upper){
  if (value <=lower)
  value=lower;
  else if(value>=upper)
  value =upper;
  return value;
}

void keyUp(const keyboard::KeyConstPtr& key){
  //ROS_INFO("%d released",key->code);

  if(key->code==key->KEY_UP){
    if(teleop.enable){
      teleop.forward_speed=0;
    }
  }
  else if(key->code==304){
    shift=false;
  }
  else if(key->code==key->KEY_DOWN){
    if(teleop.enable){
      teleop.forward_speed=0;
    }
  }
  else if(key->code==key->KEY_LEFT){
    if(teleop.enable){
      teleop.sidemove_speed=0;
    }
  }
  else if(key->code==key->KEY_RIGHT){
    if(teleop.enable){
      teleop.sidemove_speed=0;
    }
  }
  else if(key->code==key->KEY_PAGEUP){
    teleop.dropper=false;
  }
  else if(key->code==key->KEY_q){
    teleop.dropper=false;
  }
}
