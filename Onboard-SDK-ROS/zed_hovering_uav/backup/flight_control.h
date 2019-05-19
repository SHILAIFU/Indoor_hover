/** @file demo_flight_control.h
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DEMO_FLIGHT_CONTROL_H
#define DEMO_FLIGHT_CONTROL_H

// ROS includes
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>

// PID
#include <pid/pid.h>

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

/*!
 * @brief a bare bone state machine to track the stage of the mission
 */

typedef struct
{
	float P=0;
	float I=0;
	float D=0;
	float E=0;
	float LE=0;
	float LD=0;
	float DE=0;
	float SE=0;
	float OUTPUT=0;
	float OUTPUTI=0;
	float MAX=0;
	float Int_Sat=0;
}PID_CirStructure;

PID_CirStructure Pitch_PID_V;
PID_CirStructure Roll_PID_V;
PID_CirStructure Vertial_PID_P;
PID_CirStructure Pitch_PID_P;
PID_CirStructure Roll_PID_P;

pid Pitch_PID_V;
pid Roll_PID_V;

pid Vertial_PID_P;

pid Pitch_PID_P;
pid Roll_PID_P;

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg);

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg);


bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

bool M100monitoredTakeoff();


#endif // DEMO_FLIGHT_CONTROL_H
