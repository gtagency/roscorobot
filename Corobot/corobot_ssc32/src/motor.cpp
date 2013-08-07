#include "ros/ros.h"
#include "ssc32.hpp"
#include "corobot_srvs/SetSpeed.h"
#include "corobot_srvs/MoveArm.h"
#include "corobot_msgs/ssc32_info.h"
#include "corobot_msgs/MotorCommand.h"
#include "corobot_msgs/ServoPosition.h"
#include <math.h>
#include <errno.h>
#include <sys/stat.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>


using namespace Servo;

SSC32 m_ssc32;

std::string SSC32_PORT;
ros::Timer timer;
int ssc32Error = 0; // use for diagnostics purpose

bool first_time_command[20] = {false};


bool SetSpeed(corobot_srvs::SetSpeed::Request  &req, corobot_srvs::SetSpeed::Response &res )
{
	bool ret;
	ret = m_ssc32.SendMessage(1,req.left_speed,req.speed);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;
	ret = m_ssc32.SendMessage(0,req.right_speed,req.speed);	
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;
	return true;
}


void timerCallback(const ros::TimerEvent&)
{
	bool ret;	

	m_ssc32.SendMessage(1,0,0);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;

	m_ssc32.SendMessage(0,0,0);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;
}

void SetSpeedTopic(const corobot_msgs::MotorCommand::ConstPtr &msg)
{
	ros::NodeHandle n;
	bool ret;

	m_ssc32.SendMessage(1,(msg->leftSpeed*-5)+1500,(100-msg->acceleration)*10);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;

	m_ssc32.SendMessage(0,(msg->rightSpeed*-5)+1500,(100-msg->acceleration)*10);
	if (ret)
		ssc32Error = 0;
	else
		ssc32Error = 2;

	timer = n.createTimer(ros::Duration(msg->secondsDuration), timerCallback, true);
}

bool MoveArm(corobot_srvs::MoveArm::Request  &req, corobot_srvs::MoveArm::Response &res )
{
  if(req.shoulderOrientation != 0)
  {
	int amount =  (int) (req.shoulderOrientation / M_PI * 1500) + 1500 - 1500 / 2;
	m_ssc32.SendMessage(4,amount,2000);
	return true;
  }

  if(req.elbowOrientation != 0)
  {
	int amount =  (int) ((req.elbowOrientation) / M_PI * 1500) + 1500 - 1500 / 2;
	m_ssc32.SendMessage(5,amount,2000);
	return true;
  }
  return false;

}

bool MoveWrist(corobot_srvs::MoveArm::Request  &req, corobot_srvs::MoveArm::Response &res )
{	
	int amount =  (int) (req.wristOrientation / M_PI * 2000) + 1000 - 2000 / 2;
	m_ssc32.SendMessage(6,amount,2000);
	return true;
}

bool ResetArm(corobot_srvs::MoveArm::Request  &req, corobot_srvs::MoveArm::Response &res )
{
	int amount =  (int) (req.shoulderOrientation / M_PI * 1500) + 1500 - 1500 / 2;
	m_ssc32.SendMessage(4,amount,2000);

	amount =  (int) (req.elbowOrientation / M_PI * 1500) + 1500 - 1500 / 2;
	m_ssc32.SendMessage(5,amount,2000);
	return true;
}

bool MoveGripper(corobot_srvs::MoveArm::Request  &req, corobot_srvs::MoveArm::Response &res )
{
		int amount;		
		if (req.gripper == req.GRIPPER_OPEN)
			amount = 1000;
		else if (req.gripper == req.GRIPPER_CLOSED)
			amount = 2000;
		m_ssc32.SendMessage(7,amount,2000);
	
		return true;
}

/**
 * @brief Callback for the topic /phidgetServo_setPosition
	  The position of the concerned servo motor is set each time a message is received on this topic.
 * @param corobot_msgs::ServoPosition Message
 */ 
void setPositionCallback(const corobot_msgs::ServoPosition &msg)
{
	int amount =  (int) (msg.position / 180 * 1500) + 750;  //ideally, msg.position is between 0 and 150 degrees

	// The first command should not contain any speed, so we are not sending any
	bool ok;
	if(first_time_command[msg.index] == false)
	{
		ok = m_ssc32.SendMessage(msg.index,amount,-1);
		first_time_command[msg.index] = true;
	}
	else
	{
		ok = m_ssc32.SendMessage(msg.index,amount,2000);
	}
	
	if (!ok)
	{
		ROS_ERROR("Could not set the servo motor number %d to the position %f",msg.index, msg.position);
		ssc32Error = 2;	
	}
}

void ssc32_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
/**
 * Function that will report the status of the hardware to the diagnostic topic
 */
{
	if (!ssc32Error)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "initialized");
	else if (ssc32Error == 1)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot be initialized");
		stat.addf("Recommendation", "Please make sure the port path is the correct one.");
	}
	else if (ssc32Error == 2)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot set speed");
		stat.addf("Recommendation", "Please make sure the port path is the correct one. Also make sure the servo motors / motors are well connected to the ssc32 board");
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ssc32_node");
	ros::NodeHandle n("~");

	//Declare services
	ros::ServiceServer speed = n.advertiseService("ssc32_set_speed", SetSpeed);
	ros::ServiceServer arm = n.advertiseService("/move_arm", MoveArm);
	ros::ServiceServer wrist = n.advertiseService("/move_wrist", MoveWrist);
	ros::ServiceServer gripper = n.advertiseService("/move_gripper", MoveGripper);
	ros::ServiceServer arm_reset = n.advertiseService("/reset_arm", ResetArm);
	ros::Publisher ssc32_info_pub = n.advertise<corobot_msgs::ssc32_info>("ssc32_info", 1000);

	//create an updater that will send information on the diagnostics topics
	diagnostic_updater::Updater updater;
	updater.setHardwareIDf("SSC32");
	updater.add("SSC32", ssc32_diagnostic); //function that will be executed with updater.update()

	//Subscribe to topics
	ros::Subscriber velocity=n.subscribe<corobot_msgs::MotorCommand>("/ssc32_velocity",1000, SetSpeedTopic);
	ros::Subscriber position_sub = n.subscribe("/setPositionServo",100, &setPositionCallback);

	std::string ssc32_port_;

   	n.param<std::string>("ssc32_port", ssc32_port_, "/dev/ttyS1");

    	if(!n.getParam("ssc32_port", ssc32_port_))
    		ROS_INFO("can not get parameter ssc32_port!");

	SSC32_PORT = ssc32_port_;

	// Let's give permission to do anything on the ssc32 port, or else we won't be able to use it.
	/*char mode[] = "0777";
    	int i;
    	i = strtol(mode, 0, 8);
    	if (chmod (SSC32_PORT.c_str(),i) < 0)
    	{
        	ROS_ERROR("%s: error in chmod(%s, %s) - %d (%s)\n", argv[0], SSC32_PORT.c_str(), mode, errno, strerror(errno));
    	}
	*/
	//Let's now initialize the ssc32 and connect to it
	m_ssc32.initSerial();

	if(m_ssc32.startSerial(SSC32_PORT))
	{
	  ROS_INFO("Success!!! Connect to serial port");
	  corobot_msgs::ssc32_info info;
	  info.connected = 1;
	  ssc32_info_pub.publish(info);
	  ssc32Error = 0;
	}
	else
	  ssc32Error = 1; 

	//main ros function
	while (ros::ok())
        {
                ros::spinOnce();
		updater.update();
	}

        m_ssc32.stopSerial();
        
	return 0;
}
