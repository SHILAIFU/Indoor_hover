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


void pid_callback(dynamic_pid::PIDConfig &config, uint32_t level)
{

	Pitch_PID_P.Set_PID(config.Position_Param_P,config.Position_Param_I,config.Position_Param_D);
	Roll_PID_P.Set_PID(config.Position_Param_P,config.Position_Param_I,config.Position_Param_D);
	Pitch_PID_V.Set_PID(config.Velocity_Param_P,config.Velocity_Param_I,config.Velocity_Param_D);
	Roll_PID_V.Set_PID(config.Velocity_Param_P,config.Velocity_Param_I,config.Velocity_Param_D);

	Vertial_PID_P.Set_PID(config.Vertial_Param_P,config.Vertial_Param_I,config.Vertial_Param_D);

	Pitch_PID_P.PID_SetMax(config.Position_OUT_Max,config.Position_SE_Max);
	Roll_PID_P.PID_SetMax(config.Position_OUT_Max,config.Position_SE_Max);
	
	Pitch_PID_V.PID_SetMax(config.Velocity_OUT_Max,config.Velocity_SE_Max);
	Roll_PID_V.PID_SetMax(config.Velocity_OUT_Max,config.Velocity_SE_Max);

	Vertial_PID_P.PID_SetMax(config.Vertial_OUT_Max,config.Vertial_SE_Max);
	
	ROS_INFO("***********Pitch Position***********");
	Pitch_PID_P.Print();
	ROS_INFO("***********Roll Position***********");
	Roll_PID_P.Print();
	
	ROS_INFO("***********Pitch Velocity***********");
	Pitch_PID_V.Print();
	ROS_INFO("***********Pitch Velocity***********");
	Roll_PID_V.Print();
	
	ROS_INFO("***********Vertial***********");
	Vertial_PID_P.Print();

}


double dt=0;
int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_flight_control_node");
	ros::NodeHandle nh;

	// Subscribe to messages from dji_sdk_node
	ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 1000, &attitude_callback);
	ros::Subscriber rc_data = nh.subscribe("dji_sdk/rc", 1000, &rc_data_callback);
	ros::Subscriber zedSub = nh.subscribe("/zed/pose", 1000, &zed_data_callback);

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
	
	ros::AsyncSpinner spinner(3); // Use 4 threads
	spinner.start();
	
	ros::Rate loop_rate(100);
	
	while(ros::ok())
	{
		
		if(PIDCal_falg)
		{
			Roll_PID_P.PID_Cal(-(desire_PosY), -zedY);//Position PID
			Roll_PID_V.PID_Cal(Roll_PID_P.OUTPUT, -zedY_V);//Velocity PID dt *100 & /0.01 former run speed faster unit:m/s

			Pitch_PID_P.PID_Cal(desire_PosX, zedX);//Position PID
			Pitch_PID_V.PID_Cal(Pitch_PID_P.OUTPUT, zedX_V);//Velocity PID dt *100 & /0.01 former run speed faster unit:m/s
			PIDCal_falg = 0;
			
			Vertial_PID_P.PID_Cal(desire_PosH, zedZ);
			
// 			ROS_INFO_STREAM("**********Position_PID**************");
// 			ROS_INFO_STREAM("Pitch_PID_V.OUTPUT:"<<Pitch_PID_V.OUTPUT);
// 			ROS_INFO_STREAM("Roll_PID_V.OUTPUT:"<<Roll_PID_V.OUTPUT);
		}

		
		loop_rate.sleep();
	}
	
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
	static double zedX_last=zedX;
	static double zedY_last=zedY;
	static double zedZ_last=zedZ;
	static ros::Time _t = ros::Time::now();
	
// 
// 
// 	zed << msg.pose.position.x,msg.pose.position.y,msg.pose.position.z;
	
	zed_q.x() = msg.pose.orientation.x;
	zed_q.y() = msg.pose.orientation.y;
	zed_q.z() = msg.pose.orientation.z;
	zed_q.w() = msg.pose.orientation.w;
// 
	zed_yawInRad = toEulerAngle(msg.pose.orientation).z;

// 	Eigen::Vector3d zed_temp(0,0,0);
// 	zed_temp = (q.inverse())*zed;
// 	zedX=zed_temp(0);
// 	zedY=zed_temp(1);
// 	zedZ=zed_temp(2);
// 
// 	Eigen::Vector3d desire_temp(0,0,0);
// 	desire_temp = (q.inverse())*desire;
// 	desire_PosX=desire_temp(0);
// 	desire_PosY=desire_temp(1);
// 	desire_PosH=desire_temp(2);
//   
	//
	zedX_last = zedX;
	zedY_last = zedY;
	zedZ_last = zedZ;
	//
	zedX=msg.pose.position.x;
	zedY=msg.pose.position.y;
	zedZ=msg.pose.position.z;
	//
	double dt = ros::Time::now().toSec() - _t.toSec();
	_t = ros::Time::now();
	//
	zedX_V = (zedX - zedX_last)*30.0;
	zedY_V = (zedY - zedY_last)*30.0;
	PIDCal_falg = 1;

}

void rc_data_callback(const sensor_msgs::Joy& msg)
{
	
	double gun_data = msg.axes[3];
	double yaw_data = msg.axes[2];
	double pitch_data = msg.axes[1];
	double roll_data = msg.axes[0];
	double islaunch = msg.axes[5];
	static int cnt = 0;
	
	if((fabs(pitch_data)>=0.01) || (fabs(roll_data)>=0.01))
	{
		
		desire_PosY = zedY;
		desire_PosX = zedX;

		Roll_PID_P.PID_Empty();
		Roll_PID_V.PID_Empty();
		Pitch_PID_P.PID_Empty();
		Pitch_PID_V.PID_Empty();
		cnt = 0;

	}

	if((fabs(gun_data)>=0.01))
	{
		desire_PosH = zedZ;
		Vertial_PID_P.PID_Empty();
	}
	
	//number one
	Roll_OUTPUT =  Roll_PID_V.OUTPUT * cos(zed_yawInRad) + Pitch_PID_V.OUTPUT * sin(zed_yawInRad);
	Pitch_OUTPUT = Pitch_PID_V.OUTPUT * cos(zed_yawInRad) - Roll_PID_V.OUTPUT * sin(zed_yawInRad);
	if(fabs(Vertial_PID_P.OUTPUT) >= 0.001)
	{
		Vertial_OUTPUT = Vertial_PID_P.OUTPUT;
	}
	else
		Vertial_OUTPUT = 0;
	
	if(cnt == delay_time)
	{
		desire_PosY = zedY;
		desire_PosX = zedX;
		
		cnt++;
	}
	else if(cnt < delay_time)
	{
		cnt++;
	}
	
	
	static bool missionIsChange = 0;
	if(0 == islaunch)
	{
		Roll_OUTPUT = 0;
		Pitch_OUTPUT = 0;
		Vertial_OUTPUT = 0;
	}
	else if(islaunch <= -9000 && missionIsChange)
	{
		mission_flag++;
		missionIsChange = 0;
		switch(mission_flag)
		{
			case 1:
				desire_PosX += 1.0;
			break;
			case 2:
				desire_PosY += 1.0;
			break;
			case 3:
				desire_PosX -= 1.0;
			break;
			case 4:
				desire_PosY -= 1.0;
			break;
			default:
				desire_PosH -= 1.0;
			break;
			
		}
		
		
	}
	else if(islaunch > -5000)
	{
		missionIsChange = 1;
	}
	
	
	float xCmd=0, yCmd=0, zCmd=0, wCmd=0;
	xCmd = Roll_OUTPUT + roll_data*0.45;
	yCmd = Pitch_OUTPUT + pitch_data*0.45;
	wCmd = -yaw_data*1.1;
	zCmd = Vertial_OUTPUT + gun_data*3.0;

	sensor_msgs::Joy controlPosYaw;
	controlPosYaw.axes.push_back(xCmd);
	controlPosYaw.axes.push_back(yCmd);
	controlPosYaw.axes.push_back(zCmd);
	controlPosYaw.axes.push_back(wCmd);

	ctrlVxVyYawPub.publish(controlPosYaw);


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
