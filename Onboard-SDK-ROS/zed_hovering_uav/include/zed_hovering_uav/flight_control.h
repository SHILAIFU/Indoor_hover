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
#include <geometry_msgs/PoseStamped.h> 
#include <dynamic_reconfigure/server.h>

// DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/QueryDroneVersion.h>
#include <dji_sdk/SetLocalPosRef.h>
#include "dji_sdk/dji_sdk.h"

// PID
#include <pid/pid.h>

//EIGEN
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>

#include "zed_hovering_uav/PIDConfig.h"
#include "math.h"

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

/*!
 * @brief a bare bone state machine to track the stage of the mission
 */


PID Pitch_PID_V;
PID Roll_PID_V;

PID Vertial_PID_P;

PID Pitch_PID_P;
PID Roll_PID_P;

/**
 **********************************************
 *              |X+
 *              |
 *         -----|-----Y-
 *              |
 *              |
 *              |
 *         UP:Z+   
 *****/

double zedX=0;
double zedY=0;
double zedZ=0;
double  zedX_V=0, zedY_V=0;
bool PIDCal_falg = 0;


Eigen::Quaterniond zed_q;
double yawInRad=0;
double zed_yawInRad=0;

float desire_PosX = 0;//Opposite X postion
float desire_PosY = 0;//Opposite Y postion
float desire_PosH = 0.0;//high
float desire_AngW = 0;//Yaw

int delay_time = 30;

unsigned short mission_flag = 0;

float Roll_OUTPUT = 0;
float Pitch_OUTPUT = 0;
float Vertial_OUTPUT = 0;
Eigen::Vector3d desire(desire_PosX,desire_PosY,desire_PosH);
Eigen::Vector3d zed(0,0,0);

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
ros::ServiceClient query_version_service;

ros::Publisher ctrlVxVyYawPub;

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;


void rc_data_callback(const sensor_msgs::Joy& msg);
void zed_data_callback(const geometry_msgs::PoseStamped& msg);
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

bool takeoff_land(int task);

bool obtain_control();

bool is_M100();

bool M100monitoredTakeoff();

#endif // DEMO_FLIGHT_CONTROL_H
