/** @file pid.cpp
 *  @version 1.0
 *  @date May, 2019
 *
 *  @brief
 *  pid class
 *
 *  @copyright 2019 RC. All rights reserved.
 *
 */

#include "pid/pid.h"

PID::PID(float p, float i, float d,float max,float int_sat)
{
	P=p;
	I=i;
	D=d;
	E=0;
	LE=0;
	LD=0;
	DE=0;
	SE=0;
	OUTPUT=0;
	MAX=max;
	Int_Sat=int_sat;
}
void PID::PID_SetMax(float a, float b)
{
	MAX = a;
	Int_Sat = b;
}
void PID::Set_PID(float p,float i,float d)
{
	P=p;
	I=i;
	D=d;
}

void PID::PID_Empty()
{
	E=0;
	LE=0;
	LD=0;
	DE=0;
	SE=0;
	OUTPUT=0;
}
void PID::PID_Cal(float mydesire, float curren_data)
{
	E = mydesire - curren_data;
	SE+=E;
	DE = E-LE;
	
	if(fabs(SE)>Int_Sat)
		SE>0?SE=Int_Sat:SE=-Int_Sat;

	OUTPUT=P*E+I*SE+D*DE;
	LE = E;
	
	if(fabs(OUTPUT)>MAX)
	  OUTPUT>0?OUTPUT=MAX:OUTPUT=-MAX;
}
PID::~PID()
{
	P=0;
	I=0;
	D=0;
	E=0;
	LE=0;
	LD=0;
	DE=0;
	SE=0;
	OUTPUT=0;
	MAX=0;
	Int_Sat=0;
}
PID::PID()
{
	P=0;
	I=0;
	D=0;
	E=0;
	LE=0;
	LD=0;
	DE=0;
	SE=0;
	OUTPUT=0;
	MAX=0;
	Int_Sat=0;
}

void PID::Print()
{
      ROS_INFO("PID:%f,%f,%f,  MAX:%f,  Int_Sat:%f", P, I, D,MAX,Int_Sat);
}