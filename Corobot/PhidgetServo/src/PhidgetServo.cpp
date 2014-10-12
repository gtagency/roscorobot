#include "ros/ros.h"
#include <stdio.h>
#include <libphidgets/phidget21.h>

#include "corobot_srvs/GetEngaged.h"
#include "corobot_srvs/SetEngaged.h"
#include "corobot_srvs/GetPosition.h"
#include "corobot_srvs/SetPosition.h"
#include "corobot_srvs/GetMotorCount.h"
#include "corobot_srvs/SetParam.h"
#include "corobot_msgs/ServoPosition.h"
#include "corobot_srvs/GetSerialNumber.h"
#include "corobot_srvs/GetType.h"
#include "corobot_srvs/SetType.h"
#include "corobot_msgs/ServoType.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>


//Declare a servo handle
CPhidgetAdvancedServoHandle servo = 0;
ros::Publisher position_pub;
int servoError = 0; // used for diagnostics purpose

/**
 * @brief Get the status of a servo motor connected, to know if it is engaged ( powered on) or not
 * @param corobot_srvs::GetEngaged Service
 */ 
bool GetEngaged(corobot_srvs::GetEngaged::Request  &req,corobot_srvs::GetEngaged::Response &res )
{
	int state;
	int err = CPhidgetAdvancedServo_getEngaged(servo, req.index, &state);

	res.state = state;
	return err;
}

/**
 * @brief Set the status of a servo motor, engaged( powered on) or not
 * @param corobot_srvs::SetEngaged Service
 */ 
bool SetEngaged(corobot_srvs::SetEngaged::Request  &req,corobot_srvs::SetEngaged::Response &res )
{
	return CPhidgetAdvancedServo_setEngaged(servo,req.index, req.state);
}

/**
 * @brief Get the position of a servo motor
 * @param corobot_srvs::GetPosition Service
 */ 
bool GetPosition(corobot_srvs::GetPosition::Request  &req,corobot_srvs::GetPosition::Response &res )
{
	double position;
	int err = CPhidgetAdvancedServo_getPosition(servo, req.index, &position);
	res.position = position;
	
	return err;
}

/**
 * @brief Set the position of a servo motor
 * @param corobot_srvs::SetPosition Service
 */ 
bool SetPosition(corobot_srvs::SetPosition::Request  &req,corobot_srvs::SetPosition::Response &res )
{
	return CPhidgetAdvancedServo_setPosition(servo, req.index, req.position);
}

/**
 * @brief Get the maximum number of servo motors that can be connected to this Phidget Servo controller
 * @param Pcorobot_srvs::GetMotorCount Service
 */ 
bool GetMotorCount(corobot_srvs::GetMotorCount::Request  &req,corobot_srvs::GetMotorCount::Response &res )
{
	int count;
	int err = CPhidgetAdvancedServo_getMotorCount(servo, &count);

	res.numberMotors = count;
	return err;
}

/**
 * @brief Get the maximum position possible of a servo motor
 * @param corobot_srvs::GetPosition Service
 */ 
bool GetPositionMax(corobot_srvs::GetPosition::Request  &req,corobot_srvs::GetPosition::Response &res )
{
	double position;
	int err = CPhidgetAdvancedServo_getPositionMax(servo, req.index, &position);

	res.position = position;
	return err;
}

/**
 * @brief Get the minumum position possible of a servo motor
 * @param corobot_srvs::GetPosition Service
 */ 
bool GetPositionMin(corobot_srvs::GetPosition::Request  &req,corobot_srvs::GetPosition::Response &res )
{
	double position;
	int err = CPhidgetAdvancedServo_getPositionMax(servo, req.index, &position);
	
	res.position = position;
	return err;	
}

/**
 * @brief Get the parameters of a servo motor
 * @param corobot_srvs::SetParam Service
 */ 
bool SetServoParameters(corobot_srvs::SetParam::Request  &req,corobot_srvs::SetParam::Response &res )
{
	return CPhidgetAdvancedServo_setServoParameters(servo, req.index, req.min_us, req.max_us, req.degrees, req.velocity_max);
}

/**
 * @brief Each time a servo motor changes position, this function is called and a message is publish on the /phidgetServo_getPosition topic
 */ 
int PositionChangeHandler(CPhidgetAdvancedServoHandle phid, void *userPtr, int index, double position)
{
	if(position_pub)
	{
		corobot_msgs::ServoPosition position_msg;
		position_msg.index = index;
		position_msg.position = position;
		position_pub.publish(position_msg);
	}
	return 0;
}

/**
 * @brief Callback for the topic /phidgetServo_setPosition
	  The position of the concerned servo motor is set each time a message is received on this topic.
 * @param corobot_msgs::ServoPosition Message
 */ 
void setPositionCallback(const corobot_msgs::ServoPosition &msg)
{
	ROS_INFO("phidget_servo, servo: %d, angle: %f", msg.index, msg.position);
	int err = CPhidgetAdvancedServo_setPosition(servo, msg.index, msg.position);
	if (err != 0)
	{
		ROS_ERROR("Could not set the servo motor number %d to the position %f",msg.index, msg.position);
		servoError = 3;
	}
}

/**
 * @brief Phidget error Callback
 */ 
int ErrorHandler(CPhidgetHandle SERV, void *userptr, int ErrorCode, const char *Description)
{
	ROS_ERROR("Error Phidget Servo motor %d : %s",ErrorCode, Description);
	return 0;
}

/**
 * @brief Get the serial number of the current phidget servo controller. Useful if there are two phidget servo controllers connected
 * @param corobot_srvs::GetSerialNumber Service
 */ 
bool GetSerialNumber(corobot_srvs::GetSerialNumber::Request  &req,corobot_srvs::GetSerialNumber::Response &res)
{
	int serial;
	int err = CPhidget_getSerialNumber((CPhidgetHandle)servo, &serial);
	
	res.serialNumber = serial;
	return err;	
}

/**
 * @brief Get the type of servo motor pluged in
 * @param corobot_srvs::GetType Service
 */ 
bool GetServoType(corobot_srvs::GetType::Request  &req,corobot_srvs::GetType::Response &res )
{
	int type;
	int err = CPhidgetAdvancedServo_getServoType(servo, req.index, (CPhidget_ServoType *)type);

	res.type = type;
	return err;
}

/**
 * @brief Set the servo motor to one of the defined types 
 * @param corobot_srvs::SetType Service
 */ 
bool SetServoType(corobot_srvs::SetType::Request  &req,corobot_srvs::SetType::Response &res )
{
	return CPhidgetAdvancedServo_setServoType(servo, req.index, (CPhidget_ServoType)req.type);
}

/**
 * @brief Callback for the topic /phidgetServo_setType
	  The type of the concerned servo motor is set each time a message is received on this topic.
 * @param corobot_msgs::ServoType Message
 */ 
void setTypeCallback(const corobot_msgs::ServoType &msg)
{	
	CPhidgetAdvancedServo_setServoType(servo, msg.index, (CPhidget_ServoType)msg.type);
}

void servo_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
/**
 * Function that will report the status of the hardware to the diagnostic topic
 */
{
	if (!servoError)  
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "intialized");
	else if(servoError == 1)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot be initialized");
		stat.addf("Recommendation", "Please verify that the robot has a Phidget Servo Controller board. If present, please unplug and replug the Phidget Board USB cable from the Motherboard. Also, Please make sure that the phidget library is correctly installed.");
	}

	else if(servoError == 2)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot be attached");
		stat.addf("Recommendation", "Please verify that the robot has a Phidget Servo Controller board. If present, please unplug and replug the Phidget Board USB cable from the Motherboard.");
	}
	else if(servoError == 3)
	{
		stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "cannot set position");
		stat.addf("Recommendation", "Please verify that the servos are well connected to the Phidget Servo Controller Board.");
	}

}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "phidget_servomotors");
        ros::NodeHandle n;
	ros::NodeHandle n_private("~");
	int serialNumber = -1;
	double minAccel, maxVel;
	int err;

	//create an updater that will send information on the diagnostics topics
	diagnostic_updater::Updater updater;
	updater.setHardwareIDf("Phidget");
	updater.add("Servo Controller", servo_diagnostic); //function that will be executed with updater.update()
	ros::Rate loop_rate(20);  //20 Hz


	//create the servo object
	err = CPhidgetAdvancedServo_create(&servo);
	if (err != 0)
	{
		ROS_ERROR("error create the Phidget servo controller device");
		servoError = 1;
	}

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnError_Handler((CPhidgetHandle)servo, ErrorHandler, NULL);

	//Registers a callback that will run when the motor position is changed.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetAdvancedServo_set_OnPositionChange_Handler(servo,PositionChangeHandler,NULL);


	n_private.param("serialNumber", serialNumber, -1);

	//open the servo for device connections
	err = CPhidget_open((CPhidgetHandle)servo, serialNumber);
	if (err != 0)
	{
		ROS_ERROR("error opening the Phidget servo controller device");
		servoError = 1;
	}
	if((err = CPhidget_waitForAttachment((CPhidgetHandle)servo, 10000)))
	{
		ROS_ERROR("Problem waiting for attachment of Phidget servo controller");
		servoError = 2;
	}

	//Engage every servo to make sure they are powered on, setup the acceleration value and setup the default position value. 
	for(int i=0;i<8;i++)
	{
		err = CPhidgetAdvancedServo_getAccelerationMin(servo, i, &minAccel);
		if (err != 0)
			ROS_ERROR("error getting the acceleration min of the servo motor number %d",i);
		err = CPhidgetAdvancedServo_setAcceleration(servo, i, minAccel*2);
		if (err != 0)
			ROS_ERROR("error setting the acceleration of the servo motor number %d",i);
		err = CPhidgetAdvancedServo_getVelocityMax(servo, 0, &maxVel);
		if (err != 0)
			ROS_ERROR("error getting the max velocity of the servo motor number %d",i);
		err = CPhidgetAdvancedServo_setVelocityLimit(servo, 0, maxVel/2);
		if (err != 0)
			ROS_ERROR("error setting the lvelocity limit of the servo motor number %d",i);
		err = CPhidgetAdvancedServo_setEngaged(servo,i,1);
		if (err != 0)
			ROS_ERROR("error engaging the servo motor number %d",i);
	}	
	
	//setup the default type of servo motors
	err = CPhidgetAdvancedServo_setServoType(servo, 0, PHIDGET_SERVO_HITEC_HS645MG);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 0");
	err = CPhidgetAdvancedServo_setServoType(servo, 1, PHIDGET_SERVO_HITEC_HS645MG);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 1");
	err = CPhidgetAdvancedServo_setServoType(servo, 2, PHIDGET_SERVO_HITEC_HS422);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 2");
	err = CPhidgetAdvancedServo_setServoType(servo, 3, PHIDGET_SERVO_HITEC_HS422);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 3");
	err = CPhidgetAdvancedServo_setServoType(servo, 4, PHIDGET_SERVO_HITEC_HS645MG);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 4");
	err = CPhidgetAdvancedServo_setServoType(servo, 5, PHIDGET_SERVO_HITEC_HS645MG);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 5");
	err = CPhidgetAdvancedServo_setServoType(servo, 6, PHIDGET_SERVO_HITEC_HS422);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 6");
	err = CPhidgetAdvancedServo_setServoType(servo, 7, PHIDGET_SERVO_HITEC_HS422);
	if (err != 0)
		ROS_ERROR("error setting up the type of the servo motor number: 7");

	//Declare every services



	ros::ServiceServer getEngaged = n.advertiseService("phidgetServo_getEngaged", GetEngaged);
	ros::ServiceServer setEngaged = n.advertiseService("phidgetServo_setEngaged", SetEngaged);
	ros::ServiceServer getPos = n.advertiseService("phidgetServo_getPosition", GetPosition);
	ros::ServiceServer setPos = n.advertiseService("phidgetServo_setPosition", SetPosition);
	ros::ServiceServer getPosMax = n.advertiseService("phidgetServo_getPositionMax", GetPositionMax);
	ros::ServiceServer getposMin = n.advertiseService("phidgetServo_getPositionMin", GetPositionMin);
	ros::ServiceServer getmotorCount = n.advertiseService("phidgetServo_getMotorCount", GetMotorCount);
	ros::ServiceServer setparam = n.advertiseService("phidgetServo_setServoParameters", SetServoParameters);
	ros::ServiceServer serialNumberService = n.advertiseService("phidgetServo_getSerialNumber", GetSerialNumber);
	ros::ServiceServer getType = n.advertiseService("phidgetServo_getServoType", GetServoType);
	ros::ServiceServer setType = n.advertiseService("phidgetServo_setServoType", SetServoType);

	//Declare every topics
	position_pub = n.advertise<corobot_msgs::ServoPosition>("phidgetServo_getPosition", 100);

	//Subscribe to every necessary topics
        ros::Subscriber position_sub = n.subscribe("phidgetServo_setPosition",100, &setPositionCallback);
        ros::Subscriber type_sub = n.subscribe("phidgetServo_setType",100, &setTypeCallback);
	
	while (ros::ok())
            {
		updater.update();
                ros::spinOnce();
                loop_rate.sleep();
            }


	//Go back to the default position and disengage every servo motor before closing the servo handle
	for(int i=0;i<8;i++)
	{
		err = CPhidgetAdvancedServo_setPosition (servo, i, 0.00);
		err += CPhidgetAdvancedServo_setEngaged(servo,i,0);
		if (err != 0)
			ROS_ERROR("error closing the servo motor number : %d", i);
	}

	CPhidget_close((CPhidgetHandle)servo);
	CPhidget_delete((CPhidgetHandle)servo);

}
