// teleop.heading_setpoint in runMission for bucket can be set to EAST  !


#define NORTH -150
#define DEPTH_SAUVC 20
#define DEPTH_BUCKET 20
#define DEPTH_FLARE 20
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
#include <srmauv_msgs/flare.h>
#include <srmauv_msgs/bucket.h>


srmauv_msgs::teleop_sedna teleop;
srmauv_msgs::depth depth;
srmauv_msgs::line line;
srmauv_msgs::flare flare;
srmauv_msgs::bucket bucket;
keyboard::Key key;
std_msgs::Bool inLineBool;
std_msgs::Bool inFlareBool;
std_msgs::Bool inBucketBool;
std_msgs::Bool dropBall;

int pressure;
double yaw,pitch,roll;

ros::Subscriber keyDown_sub;
ros::Subscriber keyUp_sub;
ros::Subscriber pressureSub;
ros::Subscriber imuSub;
ros::Subscriber teleopSetter;
ros::Subscriber headingSub;
ros::Subscriber lineSub;
ros::Subscriber flareSub;
ros::Subscriber bucketSub;
ros::Publisher teleopPub;
ros::Publisher inLinePub;
ros::Publisher inFlarePub;
ros::Publisher inBucketPub;
ros::Publisher dropperPub;


int north = NORTH;
int south= NORTH+179;
int east = NORTH-90;
int west = NORTH+ 90 ;

int limit(int value, int lower, int upper);
void keyUp(const keyboard::KeyConstPtr& key);
void keyDown(const keyboard::KeyConstPtr& key);
void getPressure(const srmauv_msgs::depth &msg);
void getOrientation(const sensor_msgs::Imu::ConstPtr &msg);
void getHeading(const geometry_msgs::Pose2D::ConstPtr& msg);
void setTeleop(const srmauv_msgs::goal::ConstPtr& msg);
void setCurrent();
void runMission();
void getLine(const srmauv_msgs::line::ConstPtr&msg);
void getFlare(const srmauv_msgs::flare::ConstPtr&msg);
void getBucket(const srmauv_msgs::bucket::ConstPtr&msg);

bool in_depth=false;
bool in_yaw=false;
bool in_roll=false;
bool in_pitch=false;
bool inLine=false;
bool inFlare=false;
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
  inLineBool.data=false;
  inFlareBool.data=false;
  inBucketBool.data=false;
  teleop.depth_setpoint=DEPTH_SAUVC;
  teleop.heading_setpoint=north;
  dropBall.data=false;
	
  keyDown_sub=nh.subscribe("/keyboard/keydown",1000,keyDown);
  keyUp_sub=nh.subscribe("/keyboard/keyup",1000,keyUp);
  pressureSub=nh.subscribe("/pressure_data",1000,getPressure);
  imuSub=nh.subscribe("/imu/data",1000,getOrientation);
  headingSub=nh.subscribe("/imu/HeadingTrue_degree",1000,getHeading);
  teleopSetter=nh.subscribe("/teleop_set",1000,setTeleop);
  lineSub=nh.subscribe("/line_follower",1000,getLine);
  flareSub=nh.subscribe("/flare",1000,getFlare);
  bucketSub=nh.subscribe("/bucket",1000,getBucket);
	
  inLinePub=nh.advertise<std_msgs::Bool>("/inLine",100);
  inFlarePub=nh.advertise<std_msgs::Bool>("/inFlare",100);
  inBucketPub=nh.advertise<std_msgs::Bool>("/inBucket",100);
  dropperPub=nh.advertise<std_msgs::Bool>("/dropper",100);
  teleopPub=nh.advertise<srmauv_msgs::teleop_sedna>("/teleop_sedna",1000);

ROS_INFO("Teleop dispatcher initialized..");


 // teleop_sub=nh.subscribe("")

  while(ros::ok()){



  //  ROS_INFO("Teleop: %d\tDepth: %d\tHeading: %f\tForward: %d\tReverse: %d\tStrafe: %d",
 //            teleop.enable,teleop.depth_setpoint,teleop.heading_setpoint,teleop.forward_speed,teleop.reverse_speed,teleop.sidemove_speed);

    runMission();
    teleopPub.publish(teleop);
    ROS_INFO("Spinning..");





    ros::spinOnce();
  //  ros::spin();

  }

	return 0;
}


void runMission(){
inLinePub.publish(inLineBool);
inFlarePub.publish(inFlareBool);
inBucketPub.publish(inBucketBool);


//***************************************************
if(!inSidemove)
	teleop.sidemove_input=0;   //critical

//***************************************************



//dropperPub.publish(dropBall);
  if(inLine && line.possible){
   // ******************* subject to change +/- : 
    teleop.heading_setpoint=yaw + line.heading;
    teleop.sidemove_input=line.distance;
	if(!inSidemove)
		teleop.sidemove_input=0;
  }
  


 if (inFlare && flare.possible){
	//	teleop.heading_setpoint=yaw+flare.heading_offset;
	if(inSidemove)
		teleop.sidemove_input=flare.y_offset;
	else
		teleop.sidemove_input=0;
	teleop.depth_setpoint=DEPTH_FLARE;
}

 if(inBucket && bucket.possible ){
	if(inForward)
     teleop.forward_input=bucket.y_offset;
	if(inSidemove)
     teleop.sidemove_input=bucket.x_offset;
     if(!inSidemove)
                     teleop.sidemove_input=0;
     if(!inForward)
                     teleop.forward_input=0;
 }
 else{

       // teleop.forward_input=0;
       // teleop.sidemove_input=0;
 }

}

void getPressure(const srmauv_msgs::depth &msg){
  depth.depth=msg.depth;

}


void getOrientation(const sensor_msgs::Imu::ConstPtr &msg){

}

void getHeading(const geometry_msgs::Pose2D::ConstPtr& msg){
  yaw=msg->theta;
}

void getLine(const srmauv_msgs::line::ConstPtr&msg){
  line.possible=msg->possible;
  line.heading=msg->heading;
  line.distance=msg->distance;
}

void getFlare(const srmauv_msgs::flare::ConstPtr&msg){
	flare.possible=msg->possible;
	flare.heading_offset=msg->heading_offset;
	flare.y_offset=msg->y_offset;
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
      case 113: { //Q: dropper
        teleop.dropper=true;
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

      case 108: {  // l : enable/disable lineFollower
        inLine=!inLine;
	
	//inSidemove=!inSidemove;
	if(inLine){
		inSidemove=true;
		inBucket=false;
		inFlare=false;
	}
	else{
		inSidemove=false;
		teleop.sidemove_input=0;
	}
	inLineBool.data=inLine;
	inBucketBool.data=inBucket;
	inFlareBool.data=inFlare;


        break;
      }

      case 98:{ // b : enable/disable bucket detector
        inBucket=!inBucket;
        
	if(inBucket){
		inLine=false;
		inFlare=false;
		inSidemove=true;
		inForward=true;
	}
	else{
		inForward=false;
		inSidemove=false;
		teleop.forward_input=0;
		teleop.sidemove_input=0;
	}
        //inSidemove=!inSidemove;
        //inForward=!inForward;
	inLineBool.data=inLine;
        inBucketBool.data=inBucket;
        inFlareBool.data=inFlare;

        break;
      }
	case 102 : { //f enable disable flare
		inFlare=!inFlare;
		//inSidemove=!inSidemove;
		
		
		if(inFlare){
			inLine=false;
			inBucket=false;
			inSidemove=true;
			}
		else{
			inSidemove=false;
			teleop.sidemove_input=0;
		}		
	inLineBool.data=inLine;
        inBucketBool.data=inBucket;
        inFlareBool.data=inFlare;
		
	
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
     teleop.torpedo=false;
   }
   else if(key->code==key->KEY_q){
     teleop.dropper=false;
   }


}
