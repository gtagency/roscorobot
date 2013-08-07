#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <corobot_srvs/MoveArm.h>
#include <corobot_msgs/MotorCommand.h>
#include <corobot_msgs/takepic.h>
#include <corobot_msgs/PanTilt.h>
#include <corobot_msgs/velocityValue.h>

#include <math.h>

using namespace corobot_msgs;
using namespace corobot_srvs;

ros::ServiceClient moveArm_client;
ros::ServiceClient moveWrist_client; 
ros::ServiceClient moveGripper_client; 
ros::ServiceClient resetArm_client; 

ros::Publisher driveControl_pub,takepic_pub,pan_tilt_control;

int speed_left, speed_right, speed_value;
bool turningLeft, turningRight;
int pan_value,tilt_value;
double orx;
double ory,orz;
int gripper_state; //0 = open, 1 = closed
int save_image_state = 0;

void velocityCallback(const velocityValue::ConstPtr& msg)
{
	speed_value = msg->velocity;
}
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
//********************************************
//Motor control

	speed_left = joy->axes[1] * 100;
	speed_right = joy->axes[1] * 100;
	speed_left += -joy->axes[0] * 100;
	speed_right += joy->axes[0] * 100;

	if(speed_right >100)
		speed_right = 100;
	if(speed_right < -100)
		speed_right = -100;
	if(speed_left >100)
		speed_left = 100;
	if(speed_left < -100)
		speed_left = -100;

    corobot_msgs::MotorCommand msg;
    msg.leftSpeed = speed_left;
    msg.rightSpeed = speed_right;
    msg.secondsDuration = 3;
    msg.acceleration = 50;
    driveControl_pub.publish(msg);

  if(joy->buttons[8]) // STOP
  {
        corobot_msgs::MotorCommand msg;
        msg.leftSpeed = 0;
        msg.rightSpeed = 0;
        msg.secondsDuration = 0;
        msg.acceleration = 50;
        driveControl_pub.publish(msg);
  }
//*********************************************************
//Take picture
 if(joy->buttons[11]) // right Stick click Take Picture
  {
    if(save_image_state == 0)
    {
        takepic msg;
        msg.camera_index = 1;
        msg.take = true;
        takepic_pub.publish(msg);
	save_image_state = 1;
    }
  }
  else
	save_image_state = 0;

//********************************************************
// Pan Tilt Control	
  if(joy->axes[2]>0.5) // Pan control
  {
	if(pan_value > -70)
	{
    	pan_value -= 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }

  if(joy->axes[2]<-0.5) // Pan control
  {
	if(pan_value < 70)
	{
    	pan_value += 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }

  if(joy->axes[3]<-0.5) // Tilt control
  {
	if(tilt_value > -30)
	{
    	tilt_value -= 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }

  if(joy->axes[3]>0.5) // Tilt control
  {
	if(tilt_value < 30)
	{
    	tilt_value += 5;
		PanTilt msg;
		msg.pan = pan_value;
		msg.tilt = tilt_value;
		msg.reset = 0;
		pan_tilt_control.publish(msg);
	}
  }

 if(joy->buttons[9]) // PTZ reset
  {
    tilt_value = 0;
    pan_value = 0;
    PanTilt msg;
    msg.pan = pan_value;
    msg.tilt = tilt_value;
    msg.reset = 0;
    pan_tilt_control.publish(msg);
  }

//*****************************************************

//*****************************************************
//Arm control
if(joy->axes[4]>0.5) // Shoulder control
  {
	if(ory > 0.7)
	{
    	ory -= M_PI/8;

		corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveArm_client.call(srv1);
	}
  }

  if(joy->axes[4]<-0.5) // Shoulder control
  {
    
	if( ory < 2)
	{
		ory += M_PI/8;

		corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveArm_client.call(srv1);
	}
  }

  if(joy->axes[5]<-0.5) // Elbow control
  {
    
    if( orz > 0.8)
    {
		orz -= M_PI/8;
		corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveArm_client.call(srv1);
	}
  }

  if(joy->axes[5]>0.5) // Elbow control
  {
    
    if( orz < 2.5)
    {
		orz += M_PI/8;

	   corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveArm_client.call(srv1);
	}
  }

 if(joy->buttons[7]) // arm reset
  {
    ory = orz = M_PI/2;

   corobot_srvs::MoveArm srv1;

    srv1.request.gripper = 0;
    srv1.request.wristOrientation = orx;
    srv1.request.shoulderOrientation = ory;
    srv1.request.elbowOrientation = orz;

    resetArm_client.call(srv1);
  }
//******************************************
//wrist control
 if(joy->buttons[4]) // Wrist Left
  {
    
    if(orx < 4.5)
    {
	  orx += 0.5;

	   corobot_srvs::MoveArm srv1;

		srv1.request.gripper = 0;
		srv1.request.wristOrientation = orx;
		srv1.request.shoulderOrientation = ory;
		srv1.request.elbowOrientation = orz;

		moveWrist_client.call(srv1);
	}
  }

 if(joy->buttons[5]) // Wrist Right
  {
    if(orx > 0.1)
    {
	  orx -= 0.5;

   corobot_srvs::MoveArm srv1;

    srv1.request.gripper = 0;
    srv1.request.wristOrientation = orx;
    srv1.request.shoulderOrientation = ory;
    srv1.request.elbowOrientation = orz;

    moveWrist_client.call(srv1);
	}
  }
//****************************************************
//gripper control
  if(joy->buttons[6])
  {
   corobot_srvs::MoveArm srv1;

    if(gripper_state == 0)
	gripper_state = 1;
    else if(gripper_state == 1)
	gripper_state = 0;
    srv1.request.gripper = gripper_state;
    srv1.request.wristOrientation = orx;
    srv1.request.shoulderOrientation = ory;
    srv1.request.elbowOrientation = orz;

    moveGripper_client.call(srv1);
  }

//*****************************************************
  //if(joy->axes[1]>0)
  //{twist.linear.x = speed;twist.angular.z = 1;} // Go Foward

  //if(joy->axes[1]<0)
  //{twist.linear.x = -1*speed;twist.angular.z = -1;} // Go Backward

  //if((joy->buttons[0]==1)&&(joy->buttons[1]==0)&&(joy->buttons[2]==0)&&(joy->buttons[3]==0)&&(joy->buttons[4]==0))
  //{twist.linear.x = 0;twist.angular.z = 0;}//stop

  //if((joy->buttons[0]==0)&&(joy->buttons[1]==0)&&(joy->buttons[2]==0)&&(joy->buttons[3]==1)&&(joy->buttons[4]==0)&&(joy->axes[1]==0))
  //{twist.linear.x = 0;twist.angular.z = 1;}//turn left (constant speed)

  //if((joy->buttons[0]==0)&&(joy->buttons[1]==0)&&(joy->buttons[2]==0)&&(joy->buttons[3]==0)&&(joy->buttons[4]==1)&&(joy->axes[1]==0))
  //{twist.linear.x = 0;twist.angular.z = -1;}//turn right
 
  //if((joy->buttons[0]==0)&&(joy->buttons[1]==1)&&(joy->buttons[2]==0)&&(joy->buttons[3]==0)&&(joy->buttons[4]==0)&&(joy->axes[1]==0))
  //{speed = speed - 0.05;} //decrease speed

  //if((joy->buttons[0]==0)&&(joy->buttons[1]==0)&&(joy->buttons[2]==1)&&(joy->buttons[3]==0)&&(joy->buttons[4]==0)&&(joy->axes[1]==0))
  //{speed = speed + 0.05;} //decrease speed

}


int main(int argc, char** argv)
{
  
  pan_value = 0;
  tilt_value = 0;
  orx = 2.3;//initial value for wrist
  ory = M_PI/2;
  orz = M_PI/2;
  gripper_state = 0;
  speed_left = 0;
  speed_right = 0;
  speed_value = 75;
  turningLeft = false;
  turningRight = false;

  ros::init(argc, argv, "corobot_joystick");
  
  ros::NodeHandle n;
 
  /*ros::Subscriber sub = n.subscribe<joy::Joy>("joy", 1000, joyCallback);*/

  moveArm_client = n.serviceClient<MoveArm>("move_arm");
  moveWrist_client = n.serviceClient<MoveArm>("move_wrist");
  moveGripper_client = n.serviceClient<MoveArm>("move_gripper");
  resetArm_client = n.serviceClient<MoveArm>("reset_arm"); 

  driveControl_pub = n.advertise<corobot_msgs::MotorCommand>("PhidgetMotor", 100);
  takepic_pub = n.advertise<takepic>("takepicture",100);
  pan_tilt_control = n.advertise<PanTilt>("pantilt",10);

  ros::Subscriber sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);
  ros::Subscriber velocity = n.subscribe<velocityValue>("velocityValue", 1000, velocityCallback);

  ros::spin();
}
