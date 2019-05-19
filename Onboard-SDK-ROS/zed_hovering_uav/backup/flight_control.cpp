/** @file demo_flight_control.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  demo sample of how to use flight control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "zed_hovering_uav/flight_control.h"
#include "dji_sdk/dji_sdk.h"
#include "zed_hovering_uav/PIDConfig.h"

#include <geometry_msgs/PoseStamped.h> 
#include <dynamic_reconfigure/server.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

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


void PID_SetMax(PID_CirStructure* NewStruct,float a,float b)
{
	NewStruct->MAX=a;
	NewStruct->Int_Sat=b;
}

void PID_Init(PID_CirStructure*Newstruct,float KP,float KI,float KD)
{
	Newstruct->P=KP;
	Newstruct->I=KI;
	Newstruct->D=KD;
}

void PID_Empty(PID_CirStructure *Newstruct)//Initialize the PID parameter;
{
	
	Newstruct->E=0;
	Newstruct->SE=0;
	Newstruct->LE=0;
	Newstruct->DE=0;
	Newstruct->LD=0;
	Newstruct->OUTPUT=0;
}

void PID_Cal(PID_CirStructure *Newstruct,float mydesire,float curren_data)
{
	Newstruct->E = mydesire - curren_data;
	Newstruct->SE+=Newstruct->E;
	Newstruct->DE = Newstruct->E-Newstruct->LE;
	
	
 	if(fabs(Newstruct->SE)>Newstruct->Int_Sat)
 	{
 	  if(Newstruct->SE>0)
 	  Newstruct->SE=Newstruct->Int_Sat;
 	  else 
 	  Newstruct->SE=-Newstruct->Int_Sat;
 	}
	Newstruct->OUTPUT=Newstruct->P*Newstruct->E+Newstruct->I*Newstruct->SE+Newstruct->D*Newstruct->DE;
	Newstruct->LE = Newstruct->E;
	if(fabs(Newstruct->OUTPUT)>Newstruct->MAX)
	{
	  if(Newstruct->OUTPUT>0)
	  Newstruct->OUTPUT=Newstruct->MAX;
	  else
	  Newstruct->OUTPUT=-Newstruct->MAX;
	}
}

void pid_callback(dynamic_pid::PIDConfig &config, uint32_t level) {


      Pitch_PID_P.P     = config.Position_Param_P;  Roll_PID_P.P = config.Position_Param_P; 
      Pitch_PID_P.I     = config.Position_Param_I;  Roll_PID_P.I = config.Position_Param_I;  
      Pitch_PID_P.D     = config.Position_Param_D;  Roll_PID_P.D = config.Position_Param_D;   
      Pitch_PID_V.P     = config.Velocity_Param_P;  Roll_PID_V.P = config.Velocity_Param_P;
      Pitch_PID_V.I     = config.Velocity_Param_I;  Roll_PID_V.I = config.Velocity_Param_I; 
      Pitch_PID_V.D     = config.Velocity_Param_D;  Roll_PID_V.D = config.Velocity_Param_D;
      Vertial_PID_P.P   = config.Vertial_Param_P;
      Vertial_PID_P.I   = config.Vertial_Param_I;
      Vertial_PID_P.D   = config.Vertial_Param_D;
      Pitch_PID_P.MAX   = config.Position_OUT_Max;  Roll_PID_P.MAX = config.Position_OUT_Max;
      Pitch_PID_P.Int_Sat    = config.Position_SE_Max;   Roll_PID_P.Int_Sat = config.Position_SE_Max;
      Pitch_PID_V.MAX   = config.Velocity_OUT_Max;  Roll_PID_V.MAX = config.Velocity_OUT_Max;
      Pitch_PID_V.Int_Sat    = config.Velocity_SE_Max;   Roll_PID_V.Int_Sat = config.Velocity_SE_Max;
      Vertial_PID_P.MAX = config.Vertial_OUT_Max;
      Vertial_PID_P.Int_Sat  = config.Vertial_SE_Max;

      ROS_INFO("\r\nPositionPID:%f,%f,%f \r\nVelocityPID:%f,%f,%f \r\nVertialPID:%f,%f,%f \r\nPositionMAX:%f,%f \r\nVelocityMAX:%f,%f \r\nVertialMAX:%f,%f", 
            Pitch_PID_P.P, Pitch_PID_P.I, Pitch_PID_P.D,
	    Pitch_PID_V.P, Pitch_PID_V.I, Pitch_PID_V.D,
	    Vertial_PID_P.P, Vertial_PID_P.I, Vertial_PID_P.D,
	    Pitch_PID_P.MAX, Pitch_PID_P.Int_Sat,
	    Pitch_PID_V.MAX, Pitch_PID_V.Int_Sat,
	    Vertial_PID_P.MAX, Vertial_PID_P.Int_Sat
	     
	    );
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo_flight_control_node");
  ros::NodeHandle nh;

  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 10, &attitude_callback);
  ros::Subscriber rc_data = nh.subscribe("dji_sdk/rc", 10, &rc_data_callback);
  ros::Subscriber zedSub = nh.subscribe("/zed/pose", 10, &zed_data_callback);

  // Publish the control signal
  ctrlVxVyYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10);


  // Basic services
  sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  drone_task_service         = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  query_version_service      = nh.serviceClient<dji_sdk::QueryDroneVersion>("dji_sdk/query_drone_version");


  //dynamic_reconfigure
  dynamic_reconfigure::Server<dynamic_pid::PIDConfig> server;
  dynamic_reconfigure::Server<dynamic_pid::PIDConfig>::CallbackType f;

  f = boost::bind(&pid_callback, _1, _2);
  server.setCallback(f);

  
  bool obtain_control_result = obtain_control();
  bool takeoff_result;
  if(!obtain_control_result)
  {
    ROS_INFO("obtain control fail");
    ros::spin();
  }

  if(is_M100())
  {
    ROS_INFO("M100 taking off!");
    takeoff_result = M100monitoredTakeoff();
  }
  else
  {
    ROS_INFO("It is not M100 drone");
    ROS_INFO("Exit!!");
    ros::spin();
  }
  
 
  ros::spin();
  return 0;
}

bool takeoff_land(int task)
{
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  drone_task_service.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool is_M100()
{
  dji_sdk::QueryDroneVersion query;
  query_version_service.call(query);

  if(query.response.version == DJISDK::DroneFirmwareVersion::M100_31)
  {
    return true;
  }

  return false;
}


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

double gun_data=0;
double yaw_data=0;
double roll_data=0;
double pitch_data=0;
double zedX=0;
double zedY=0;
double zedZ=0;
double yawInRad=0;

float desire_PosX = 0;//Opposite X postion
float desire_PosY = 0;//Opposite Y postion
float desire_PosH = 0;//high
float desire_AngW = 0;//Yaw
int   islaunch = 0;

Eigen::Vector3d desire(desire_PosX,desire_PosY,desire_PosH);
Eigen::Vector3d zed(0,0,0);
geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
  yawInRad=toEulerAngle(msg->quaternion).z;
}

void zed_data_callback(const geometry_msgs::PoseStamped& msg)
{
  Eigen::Quaterniond q;

  
  zed << msg.pose.position.x,msg.pose.position.y,msg.pose.position.z;
  q.x() = msg.pose.orientation.x;
  q.y() = msg.pose.orientation.y;
  q.z() = msg.pose.orientation.z;
  q.w() = msg.pose.orientation.w;
  

  Eigen::Vector3d zed_temp(0,0,0);
  zed_temp = (q.inverse())*zed;
  zedX=zed_temp(0);
  zedY=zed_temp(1);
  zedZ=zed_temp(2);

  Eigen::Vector3d desire_temp(0,0,0);
  desire_temp = (q.inverse())*desire;
  desire_PosX=desire_temp(0);
  desire_PosY=desire_temp(1);
  desire_PosH=desire_temp(2);
    

}

void rc_data_callback(const sensor_msgs::Joy& msg)
{
  
  gun_data = msg.axes[3];
  yaw_data = msg.axes[2];
  pitch_data = msg.axes[1];
  roll_data = msg.axes[0];
  islaunch = msg.axes[5];

  float xCmd=0, yCmd=0, zCmd=0, wCmd=0;
  static float zedX_last=0, zedY_last=0;
  sensor_msgs::Joy controlPosYaw;

  static ros::Time NextT = ros::Time::now();
  ros::Duration dt = ros::Time::now() - NextT;  

  if((fabs(gun_data)>=0.01) || (fabs(yaw_data)>=0.01) || (fabs(pitch_data)>=0.01) || (fabs(roll_data)>=0.01))
  {
    desire = zed;
    desire_PosY = zedY;
    desire_PosX = zedX;
    
    PID_Empty(&Roll_PID_P);
    PID_Empty(&Roll_PID_V);
    PID_Empty(&Pitch_PID_P);
    PID_Empty(&Pitch_PID_V);
    PID_Empty(&Vertial_PID_P);
    
  }
  else
  {
    PID_Cal(&Roll_PID_P, -(desire_PosY), -zedY);//Position PID
    PID_Cal(&Roll_PID_V, Roll_PID_P.OUTPUT, (-zedY-zedY_last)*50);//Velocity PID dt *100 & /0.01 former run speed faster unit:m/s
    zedY_last = -zedY;

    PID_Cal(&Pitch_PID_P, desire_PosX, zedX);//Position PID
    PID_Cal(&Pitch_PID_V, Pitch_PID_P.OUTPUT, (zedX-zedX_last)*50);//Velocity PID dt *100 & /0.01 former run speed faster unit:m/s
    zedX_last = zedX;

    //   PID_Cal(&Vertial_PID_P, desire_PosH, zedZ);
  }

  NextT = ros::Time::now();
//   if(0 == islaunch)
//   {
// 
//   }
//   else if(islaunch <= -9000)
//   {
//     xCmd = roll_data*0.45;
//     yCmd = pitch_data*0.45;
//     wCmd = -yaw_data*0;
//     zCmd = gun_data*5;
//   }
//   else
//   {
//     xCmd = Roll_PID_V.OUTPUT + roll_data*0.45;
//     yCmd = Pitch_PID_V.OUTPUT + pitch_data*0.45;
//     wCmd = -yaw_data*0;
//     zCmd = Vertial_PID_P.OUTPUT + gun_data*5;
//   }

  xCmd = Roll_PID_V.OUTPUT + roll_data*0.45;
  yCmd = Pitch_PID_V.OUTPUT + pitch_data*0.45;
  wCmd = -yaw_data*0;
  zCmd = Vertial_PID_P.OUTPUT + gun_data*5;
  
  controlPosYaw.axes.push_back(xCmd);
  controlPosYaw.axes.push_back(yCmd);
  controlPosYaw.axes.push_back(zCmd);
  controlPosYaw.axes.push_back(wCmd);

  ctrlVxVyYawPub.publish(controlPosYaw);
  
  
  static ros::Time start_time = ros::Time::now();
  ros::Duration elapsed_time = ros::Time::now() - start_time;

  if(elapsed_time > ros::Duration(0.1))
  {
    start_time = ros::Time::now();

    ROS_INFO_STREAM("**********Position_PID**************");
    ROS_INFO_STREAM("Roll_PID_V.SE:"<<Roll_PID_V.SE);
    ROS_INFO_STREAM("Roll_PID_P.OUTPUT:"<<Roll_PID_P.OUTPUT);
    ROS_INFO_STREAM("Roll_PID_V.OUTPUT:"<<Roll_PID_V.OUTPUT);

    
    ROS_INFO_STREAM("**********Position_Data**************");
    ROS_INFO_STREAM("position.x:"<<zedX);
    ROS_INFO_STREAM("position.y:"<<zedY);
    ROS_INFO_STREAM("position.z:"<<zedZ);
    ROS_INFO_STREAM("yawInRad:"<<yawInRad);
    
    ROS_INFO_STREAM("**********Pos_Data**************");
    ROS_INFO_STREAM("zed_PosX:"<<zed(0)<<"\tdesire_PosX:"<<desire_PosX);
    ROS_INFO_STREAM("zed_PosY:"<<zed(1)<<"\tdesire_PosY:"<<desire_PosY);
    ROS_INFO_STREAM("zed_PosH:"<<zed(2)<<"\tdesire_PosH:"<<desire_PosH);

    ROS_INFO_STREAM("**********desire_Pos_Change**************");
    ROS_INFO_STREAM("desire_PosChangeX:"<<desire(0));
    ROS_INFO_STREAM("desire_PosChangeY:"<<desire(1));
    ROS_INFO_STREAM("desire_PosChangeH:"<<desire(2));

    ROS_INFO_STREAM("**********PID_OUTPUT**************");
    ROS_INFO_STREAM("xCmd:"<<xCmd);
    ROS_INFO_STREAM("yCmd:"<<yCmd);
    ROS_INFO_STREAM("zCmd:"<<zCmd);
    ROS_INFO_STREAM("wCmd:"<<wCmd);ROS_INFO_STREAM("dt:"<<dt);
  }

  
}


void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

float current_zed_altitude=0;
bool M100monitoredTakeoff()
{
//   ros::Time start_time = ros::Time::now();
// 
//   float home_altitude = current_zed_altitude;
//   if(!takeoff_land(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF))
//   {
//     return false;
//   }
// 
//   ros::Duration(0.01).sleep();
//   ros::spinOnce();

//   // Step 1: If M100 is not in the air after 10 seconds, fail.
//   while (ros::Time::now() - start_time < ros::Duration(10))
//   {
//     ros::Duration(0.01).sleep();
//     ros::spinOnce();
//   }
// 
//   if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR ||
//       current_zed_altitude - home_altitude < 1.0)
//   {
//     ROS_ERROR("Takeoff failed.");
//     return false;
//   }
//   else
//   {
//     start_time = ros::Time::now();
//     ROS_INFO("Successful takeoff!");
//     ros::spinOnce();
//   }

  return true;
}
