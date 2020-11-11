           
// Automatically generated realtime application source file for STEP platforms
//
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP).
//
// Copyright (C) 2013-2015 Neuromeka <http://www.neuromeka.com>

//-system-/////////////////////////////////////////////////////////////////
#ifndef __XENO__
#define __XENO__
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>		// string function definitions
#include <fcntl.h>		// File control definitions
#include <errno.h>		// Error number definitions
#include <termios.h>	// POSIX terminal control definitions
#include <time.h>		// time calls
#include <sys/ioctl.h>
#include <math.h>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>		//The rdtk real-time printing library
/****************************************************************************/

#include "SystemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.cpp"
//#include "PDOConfig.cpp"
#include "EcatDataSocket.h"
#include "EcatControlSocket.h"

#include "ServoAxis.h"

////////////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include "../include/indy7/Algorithms.cpp"
#include "../include/indy7/Kinematics.cpp"
#include "../include/indy7/Indy_class.h"
#include "../include/indy7/Indytraj.cpp"
CIndy  carm;
AIndy  Afun;
Kfun kfun;
Tfun  tfun;
#include "ftsensor/ftsensorMsg.h"
//////////////////////////////////////////////////////////////////////////////////

#define NUM_AXIS	(6 + 1 + 1)		//Modify this number to indicate the actual number of motor on the network

#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define WAKEUP_TIME		(5)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000

#define DEMO_MODE_TORQUE		1
#define DEMO_MODE_POSITION		2
#define DEMO_MODE_GUI		3

////////// LOGGING BUFFER ///////////////
#define MAX_BUFF_SIZE 1000

static int sampling_time = 5;	// Data is sampled every 5 cycles.
volatile int sampling_tick = 0;

struct LOGGING_PACK
{
	double Time;
	INT32 	ActualPos[NUM_AXIS];
	INT32 	ActualVel[NUM_AXIS];
};

unsigned int frontIdx = 0, rearIdx = 0;
LOGGING_PACK _loggingBuff[MAX_BUFF_SIZE];
/////////////////////////////////////////

// Cycle time in nanosecond
unsigned int cycle_ns = 1000000; /* 1 ms */

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;


// NRMKDataSocket for plotting axes data in Data Scope
EcatDataSocket datasocket;

// EtherCAT System interface object
SystemInterface_EtherCAT_Neuromeka_NRMK_IO_Module _systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module;

// NRMK socket for online commands from NRMK EtherLab Configuration Tool
NRMKHelper::EcatControlSocket<NUM_AXIS> guicontrolsocket;

// Demo Mode
int demo_mode = DEMO_MODE_GUI;
// When all slaves or drives reach OP mode,
// system_ready becomes 1.
int system_ready = 0;

// Global time (beginning from zero)
double gt=0;

/// TO DO: This is user-code.
double sine_amp=50000, f=0.2, period;

int InitFlag[NUM_AXIS] = {0,0,0,0,0,0,0,0};

// EtherCAT Data (in pulse)
INT32 	ZeroPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT16	StatusWord[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32 	ActualPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32 	ActualVel[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16 	ActualTor[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	DataIn[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	ModeOfOperationDisplay[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	StatusCode[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DI5V[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DI1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DI2[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT16	AI1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT16	AI2[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	FTRawFx[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	FTRawFy[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	FTRawFz[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	FTRawTx[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	FTRawTy[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	FTRawTz[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	FTOverloadStatus[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	FTErrorFlag[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxCnt[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD0[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD2[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD3[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD4[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD5[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD6[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD7[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD8[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485RxD9[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	IStatus[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	IButton[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//INT16	FTRawFx[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//INT16	FTRawFy[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//INT16	FTRawFz[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//INT16	FTRawTx[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//INT16	FTRawTy[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//INT16	FTRawTz[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//UINT8	FTOverloadStatus[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//UINT8	FTErrorFlag[NUM_AXIS] = {0,0,0,0,0,0,0,0};

INT32 	TargetPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32 	TargetVel[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16 	TargetTor[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32 	DataOut[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8 	ModeOfOperation[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT16	Controlword[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	ControlCode[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DO5V[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	TO[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DO[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT16	AO1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT16	AO2[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	FTConfigParam[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485ConfigParam[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485CMD[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxCnt[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD0[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD2[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD3[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD4[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD5[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD6[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD7[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD8[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	RS485TxD9[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	ILed[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	IGripper[NUM_AXIS] = {0,0,0,0,0,0,0,0};
//UINT32	FTConfigParam[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	LEDMode[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	LEDG[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	LEDR[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	LEDB[NUM_AXIS] = {0,0,0,0,0,0,0,0};

///// SDO Access /////////



//////////////////////////

// Interface to physical axes
NRMKHelper::ServoAxis Axis[NUM_AXIS];

/****************************************************************************/

// Xenomai RT tasks
RT_TASK Neuromeka_NRMK_IO_Module_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK gui_task;

// For RT thread management
static int run = 1;
unsigned long fault_count=0;
long ethercat_time=0, worst_time=0;
#define min_time	0
#define max_time	100000
#define hist_step	(100)
unsigned int histdata[hist_step+1];
unsigned int interval_size=350;

// Signal handler for CTRL+C
void signal_handler(int signum);

/////////////////////////////////////////////////////////////////
FILE *gtData, *gtData2, *fp_record, *fp_replay, *sfData, *link3Data;

const int qInitCnt[NUM_AXIS] = {0, -5648533,  -5674694,  18817133,  -9746129, -13246305, -12434733};
const int qDirection[NUM_AXIS] = {0, -1,-1,1,-1,-1,-1, 0};
int qAbsCnt[NUM_AXIS] = {0,0,0,0,0,0,0,0};
double qAbsRad[NUM_AXIS] = {0,0,0,0,0,0,0,0};
double qVelRad[NUM_AXIS] = {0,0,0,0,0,0,0,0};
double tempTor[NUM_AXIS] = {0,0,0,0,0,0,0,0};
double tempTorA[NUM_AXIS] = {0,0,0,0,0,0,0,0};
double q0Rad[6] = {0,0,0,0,0,0};
double dq0Rad[6] = {0,0,0,0,0,0};

const int gearRatio[NUM_AXIS] = {0,121,121,121,101,101,101,0};
const int currentRatio[NUM_AXIS] = {0,48,48,96,96,96,96,0};
const double torqueRatio[NUM_AXIS] = {0,0.10211,0.10211,0.100486,0.067155,0.067155,0.067155,0};

double  PulseperRev = 65536;

VectorXd dqd_pre(6);
Vector3d dxd_pre;
Vector3d dthd_pre;
Vector3d xc_pre,fext_pre, thc_pre;
VectorXd temp_fil(6);
float NormK, NormX,NormF,NormX_pre,NormF_pre,NormXf=0,NormFf=0;

int ctrl=0;
int Mflag;
Vector3d Mdx;
Vector3d Mdth;
int VIAMode=0;

int cmd_mode=0;
#define CMD_MODE_NO_CNTL		0
#define CMD_MODE_TORQUE		1
#define CMD_MODE_POSITION		2
#define CMD_MODE_VELOCITY		3

char filename_record[100],filename_replay[100];
float SwitchS[5];
float SwitchPre[5];
int PointTraj[30];   //Point or Trajectory, 0=none, 1=Path point, 2=Trajectory 
int SEQ_switch=0,pointnum=0,SEQ_play=0,stop_flag=0,PV_flag=0; 
int switch_on=0,SEQ_record=0,SEQ_point=0,SEQ_traj=0, force_on=0;
int pause_cnt=0;
double MaxFDesired=0;

VectorXd Prox_Bias(4);

void msgCallbackSwitch(const ftsensor::ftsensorMsg::ConstPtr& msg)
{
	if(SwitchS[3]<msg->C4) Prox_Bias<<Prox_Bias(0) + carm.ProxSensor(0),Prox_Bias(1) + carm.ProxSensor(1),Prox_Bias(2) + carm.ProxSensor(2), Prox_Bias(3) + carm.ProxSensor(3);
	SwitchS[0]=msg->C1;
	SwitchS[1]=msg->C2;
	SwitchS[2]=msg->C3;
	SwitchS[3]=msg->C4;
	SwitchS[4]=msg->C5;
	if(SwitchS[0]>30) SwitchS[0]=30;
	if(SwitchS[4]==0){
		if(SwitchS[3]==1)	ctrl=8;
		//if(SwitchS[3]==2) 	ctrl=9;
		switch_on=0;

	}
	if(SwitchS[4]==1){
		if(switch_on==0){
			ctrl=0;
			SEQ_switch=0;
		}
		switch_on=1;
	}
	if(SwitchS[4]==2){
		if(switch_on==1){
			ctrl=75;

			SEQ_switch=0;
		}
		switch_on=2;
	}
}

void msgCallbackdataftsensor(const ftsensor::ftsensorMsg::ConstPtr& msg)
{
	carm.FTSensor(0) = msg ->Fx;
	carm.FTSensor(1) = msg ->Fy;
	carm.FTSensor(2) = msg ->Fz;
	carm.FTSensor(3) = msg ->Mx;
	carm.FTSensor(4) = msg ->My;
	carm.FTSensor(5) = msg ->Mz;

	/*for(int i=0;i<6;i++){
		carm.dist(1,i)=1;
		if(fabs(carm.FTSensor(i))>10)
			carm.FTSensor(i)=10;
	}
	if(carm.FTSensor(0)>0)
		carm.dist(1,0)=-0.13*carm.FTSensor(0)/8+0.73/4;
	else if(carm.FTSensor(0)<0)
		carm.dist(1,2)=-0.13*fabs(carm.FTSensor(0))/8+0.73/4;
	if(carm.FTSensor(1)>0)
		carm.dist(1,1)=-0.13*carm.FTSensor(1)/8+0.73/4;
	if(carm.FTSensor(1)<0)
		carm.dist(1,3)=-0.13*fabs(carm.FTSensor(1))/8+0.73/4;
	if(carm.FTSensor(2)>0)
		carm.dist(1,4)=-0.13*carm.FTSensor(2)/8+0.73/4;
	if(carm.FTSensor(2)<0)
		carm.dist(1,5)=-0.13*fabs(carm.FTSensor(2))/8+0.73/4;*/


}

void msgCallbackprox(const ftsensor::ftsensorMsg::ConstPtr& msg)
{
	carm.ProxSensor(0) = msg ->C1 - Prox_Bias(0);
	carm.ProxSensor(1) = msg ->C2 - Prox_Bias(1);
	carm.ProxSensor(2) = msg ->C3 - Prox_Bias(2);
	carm.ProxSensor(3) = msg ->C4 - Prox_Bias(3);
	carm.ProxSensor(4) = msg ->C5;
}

void msgCallbackdatamaster(const ftsensor::ftsensorMsg::ConstPtr& msg)
{
	Mflag = (int)msg ->G1; 
	carm.Masterdx << msg ->Fx, msg ->Fy, msg ->Fz;
	carm.Masterdth << msg ->Mx, msg ->My, msg ->Mz;
}

int initAxes()
{
	/*
	const int gearRatio[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const int pulsePerRevolution[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const double ratedTau[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const int dirQ[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const int dirTau[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const int zeroPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	*/

	for (int i = 0; i < NUM_AXIS; i++)
	{
		Axis[i].setGearRatio(1);
		Axis[i].setPulsePerRevolution(1);
		Axis[i].setRatedTau(1);

		Axis[i].setDirQ(1);
		Axis[i].setDirTau(1);

		Axis[i].setConversionConstants();

		Axis[i].setTrajPeriod(period);
		
		Axis[i].setTarVelInCnt(0);
		Axis[i].setTarTorInCnt(0);
		
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.setServoOn(i);
	}
	
	return 1;
}

void saveLogData()
{
	if (datasocket.hasConnection() && sampling_tick-- == 0)
	{
		sampling_tick = sampling_time - 1; // 'minus one' is necessary for intended operation

		if (rearIdx < MAX_BUFF_SIZE)
		{
			_loggingBuff[rearIdx].Time = gt;
			for (int i=0; i<NUM_AXIS; i++)
			{
				_loggingBuff[rearIdx].ActualPos[i] = ActualPos[i];
				_loggingBuff[rearIdx].ActualVel[i] = ActualVel[i];
			}
			rearIdx++;
		}
	}
}

/****************************************************************************/
	
VectorXd startp(6);
int gtcnt=0, gtcnt1=0, flag1=0, flag2=0;
double Tdqd=0;
int HYflag=0;

VectorXd filter_pre(6);
VectorXd tfilter_pre(6);
VectorXd dqcfilter_pre(6);



int compute()
{
	for (int i=0; i<NUM_AXIS; ++i)
	{
		Axis[i].setCurrentPosInCnt(ActualPos[i]);
		Axis[i].setCurrentVelInCnt(ActualVel[i]);
		Axis[i].setCurrentTorInCnt(ActualTor[i]);
		Axis[i].setDataIn(DataIn[i]);
		
		Axis[i].setCurrentTime(gt);
				
		qAbsCnt[i] = qDirection[i] * (ActualPos[i] - qInitCnt[i]);
		qAbsRad[i] = (double)qAbsCnt[i]/ gearRatio[i] / PulseperRev * PI2;
		qVelRad[i] = (double)ActualVel[i]*qDirection[i]/ gearRatio[i] / PulseperRev * PI2;
		tempTorA[i] = (double)ActualTor[i]*qDirection[i]/currentRatio[i]*gearRatio[i]*torqueRatio[i];
	}

	for (int i=0; i<6; ++i){
		carm.Tor_A(i) = tempTorA[i+1];
		carm.qc(i) = qAbsRad[i+1];
		carm.dqc(i) = qVelRad[i+1];
		carm.dqc_cal(i) = (carm.qc(i) - q0Rad[i])/0.001;
		q0Rad[i] = carm.qc(i);
		carm.ddqc(i) = (carm.dqc(i) - dq0Rad[i])/0.001;
		dq0Rad[i] = carm.dqc(i);

		carm.ddqcf(i) = 0.95*filter_pre(i) + 0.05*carm.ddqc(i);
		filter_pre(i) = carm.ddqcf(i);
		carm.Tor_Af(i) = 0.95*tfilter_pre(i) + 0.05*carm.Tor_A(i);
		tfilter_pre(i) = carm.Tor_Af(i);
		carm.dqcf(i) = 0.95*dqcfilter_pre(i) + 0.05*carm.dqc(i);
		dqcfilter_pre(i) = carm.dqcf(i);
	}	

	startp << 0,0,0,0,0,0;
	carm.Tau << 0,0,0,0,0,0;
	carm.Tor << 0,0,0,0,0,0;
	carm.Err_sum << 0, 0, 0, 0, 0, 0;

	kfun.ForwardK_T(&carm);
	kfun.Gravity_comp(&carm);
	kfun.Rotation2EulerAngle(&carm);
	kfun.Self_collide(&carm);
		
	carm.dxc << (carm.xc - xc_pre)/0.001;
	xc_pre = carm.xc; 

	if(ctrl==0){ //0 Pause
		cmd_mode=CMD_MODE_NO_CNTL;
		carm.qt = carm.qc;
		carm.qd = carm.qc;
		carm.xt = carm.xc;
		carm.tht = carm.thc;
		carm.xd = carm.xc;
		carm.Tau = carm.Tor_grav;
		//carm.Tau << 0,0,0,0,0,0;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++)
				carm.Rstd(i, j) = carm.Tc(i, j);
		}
		carm.dqd << 0,0,0,0,0,0;
		if(flag1==1) flag1=0, flag2=1;
		if(flag1==2) flag1=0, flag2=0;
		DO[0] = 0x10;
	}
	else if(ctrl==1){ //1
		///DI1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
		///DI2[NUM_AXIS] = {0,0,0,0,0,0,0,0};
		//DO[0] = 0x50;

		///DO[NUM_AXIS] = {0,0,0,0,0,0,0,0};

				int ret=0;
				int ret2=0;
				carm.Traj_seq[0]=0;
				kfun.ForwardK_link2(&carm);

				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						carm.Rc2(i,j)=carm.Tc2(i,j);
					}
				}
				carm.Rc2_i = carm.Rc2.transpose();

				carm.Traj_Xt2 << carm.xt2;
				carm.Traj_Xc2 << carm.xc2;
				carm.Traj_Vc2 << carm.J2*carm.dqc2;
				ret2 = tfun.Change_xt_link2(&carm,0.1,0.1);
				ret = tfun.TaskTraj_rspb_velocity_link2(&carm);
				carm.dqd2 = carm.J2_i*carm.Traj_Vd2;


		cmd_mode=CMD_MODE_NO_CNTL;
		carm.Tau = carm.Tor_grav;
	}
	else if(ctrl==2){ //2
		///DI1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
		///DI2[NUM_AXIS] = {0,0,0,0,0,0,0,0};
		DO[0] = 0x90;

		cmd_mode=CMD_MODE_NO_CNTL;
		carm.Tau = carm.Tor_grav;

		///DO[NUM_AXIS] = {0,0,0,0,0,0,0,0};	
	}
	else if(ctrl==3){ //3
		double Md,Kd;
		Vector3d Fd;
		Matrix3d Cd,I,Co, Ko;
		I=Matrix3d::Identity();
		Cd = 30*I;  
		Co = 15*I;  
		Md=10;  
		//Cd = 50*I;  
		//Co = 30*I;  
		//Md=15;
		//Cd = 100*I;  
		//Co = 50*I;  
		//Md=5;
		Kd=300;


		Matrix3d Re, Ret, Re2std;
		Vector3d Fend, Tend;
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				Re(j,i)=carm.Tc(j,i);
			}
		}
		Ret=Re.transpose();

		Fend << carm.FTSensor(0), carm.FTSensor(1), carm.FTSensor(2);
		Tend << carm.FTSensor(3), carm.FTSensor(4), carm.FTSensor(5);

		Vector3d l, cal_td;
		l << 0.02, 0.02, 0.04;//Handle information, length from center of sensor
		cal_td << Tend(0)/l(0), Tend(1)/l(1), Tend(2)/l(2); //Calculated Torque
		carm.fext = Re*Fend;
		carm.text = Re*cal_td;

		kfun.Jacobian(&carm);
 		carm.J_t = carm.J.transpose();

		MatrixXd JJt;
		JJt = carm.J*carm.J_t;
		carm.J_i = carm.J_t*JJt.inverse();

		//kfun.DynErr_comp(&carm);
		kfun.Dynamics(&carm);

		kfun.MappingMatrix(&carm);
		Vector3d tempth;
		tempth = carm.tht - carm.thc;
		for(int j=0;j<3;j++){
			if(tempth(j)>PI)  	tempth(j)=tempth(j) - PI2;
			if(tempth(j)<-PI)  	tempth(j)=tempth(j) + PI2;
		}

		carm.fd << carm.fext  - Cd*carm.Jp*carm.dqc + 1000*(carm.xt - carm.xc);
		carm.td << carm.text  - Co*carm.Jw*carm.dqc + 30*carm.Tmap_it*(tempth);
		carm.Fd << carm.fd, carm.td;
		carm.Fext << carm.fext, carm.text;
		

		carm.Tau = carm.J_t*carm.Fd + carm.Tor_grav;// + carm.Tor_DEC;
		kfun.Friction_comp(&carm);
		carm.Tau = carm.Tau + carm.Tor_fric;

		//MatrixXd M_x(6,6);
		//M_x = carm.J_i.transpose()*carm.M_q*carm.J_i;
/*
		MatrixXd Mxi;
		Mxi=carm.J*carm.M_q.inverse()*carm.J_t;

		MatrixXd M_d(6,6);
		M_d=Mxi;
		for(int i=0;i<3;i++){
			for(int j=0;j<6;j++){
				M_d(j,i)=0;
			}
			M_d(i,i)=0.1;
		}

		kfun.dotJacobian(&carm);
		//carm.temp1 = carm.J_t*carm.Fd;

		if(gtcnt==0){
			carm.temp1 = carm.J_t*carm.Fd;
			gtcnt=1;
		}
		else if(gtcnt==1){
			carm.ddXd <<carm.fd/10,0,0,0;
			carm.Fd << 0,0,0, carm.td;
			carm.temp2 = carm.J_t*carm.Fd + carm.M_q*carm.J_i*(carm.ddXd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc;
			gtcnt=2;
		}
		else if(gtcnt==2){
			carm.temp3 = carm.M_q*carm.J_i*(M_d*carm.Fd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc - carm.J_t*carm.Fext;
			gtcnt=0;
		}


		kfun.dotJacobian(&carm);
		//carm.ddXd <<carm.fd/10,0,0,0;
		//carm.Fd << 0,0,0, carm.td;
		//carm.Tau = carm.J_t*carm.Fd + carm.M_q*carm.J_i*(carm.ddXd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc + carm.Tor_grav - carm.J_t*carm.Fext;//  + carm.Tor_DEC;
		carm.Tau = carm.M_q*carm.J_i*(M_d*carm.Fd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc + carm.Tor_grav - carm.J_t*carm.Fext;// + carm.Tor_DEC;
		kfun.Friction_comp(&carm);
		carm.Tau = carm.Tau + carm.Tor_fric;
*/
		//carm.ddXd <<carm.fd/10,carm.td/3;
		//carm.ddXd <<carm.fd,0,0,0;
		//carm.ddXd = M_d*carm.Fd;
		//carm.Tau = carm.Tor_grav + carm.Tor_fric + carm.Tor_DEC;
		//carm.Tau = carm.J_t*carm.Fd + carm.Tor_grav;// + carm.Tor_DEC;
		//carm.Tau = carm.M_q*carm.J_i*(M_d*carm.Fd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc + carm.Tor_grav;// - carm.J_t*carm.Fext;// + carm.Tor_DEC;
		//kfun.Friction_comp(&carm);
		//carm.Tau = carm.Tau + carm.Tor_fric;
		//carm.temp1 = carm.J_t*carm.Fd;

		//carm.temp2 = carm.M_q*carm.J_i*(M_d*carm.Fd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc;
		//carm.Fd << 0,0,0, carm.td;
		//carm.temp1 = carm.J_t*carm.Fd + carm.M_q*carm.J_i*(M_d*carm.ddXd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc;
		//carm.temp1 <<carm.fd/10,0,0,0 ;
		//carm.temp2 = M_d*carm.ddXd;
		//std::cout<<M_d<<std::endl;

		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.temp1(0),carm.temp1(1),carm.temp1(2),carm.temp1(3),carm.temp1(4),carm.temp1(5),carm.temp2(0),carm.temp2(1),carm.temp2(2),carm.temp2(3),carm.temp2(4),carm.temp2(5),carm.temp3(0),carm.temp3(1),carm.temp3(2),carm.temp3(3),carm.temp3(4),carm.temp3(5));
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.temp1(0),carm.temp1(1),carm.temp1(2),carm.temp1(3),carm.temp1(4),carm.temp1(5), M_d(0,0), M_d(0,1), M_d(0,2), M_d(0,3), M_d(0,4), M_d(0,5), M_d(1,0), M_d(1,1), M_d(1,2), M_d(1,3), M_d(1,4), M_d(1,5), M_d(2,0), M_d(2,1), M_d(2,2), M_d(2,3), M_d(2,4), M_d(2,5),carm.ddXd(0),carm.ddXd(1),carm.ddXd(2),carm.ddXd(3),carm.ddXd(4),carm.ddXd(5));

		//fprintf(gtData, "%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",gtcnt,carm.qc(0),carm.qc(1),carm.qc(2),carm.qc(3),carm.qc(4),carm.qc(5),carm.dqc(0),carm.dqc(1),carm.dqc(2),carm.dqc(3),carm.dqc(4),carm.dqc(5),carm.Tor_A(0),carm.Tor_A(1),carm.Tor_A(2),carm.Tor_A(3),carm.Tor_A(4),carm.Tor_A(5),carm.ddqd(0),carm.ddqd(1),carm.ddqd(2),carm.ddqd(3),carm.ddqd(4),carm.ddqd(5),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));

	}

	else if(ctrl==4){ //4
		cmd_mode=CMD_MODE_VELOCITY;
		Afun.ArmAdmittance(&carm, &kfun);
		
		carm.temp1 << (carm.fext - fext_pre), (carm.xc - xc_pre);
		temp_fil << 0.95*temp_fil + 0.05*carm.temp1;
		fext_pre = carm.fext;
		xc_pre = carm.xc; 
		for(int i=0;i<3;i++){
			if(temp_fil(i+3)<0.00001&&temp_fil(i+3)>-0.00001) carm.temp2(i) = 0;
			else carm.temp2(i) = abs(temp_fil(i)/temp_fil(i+3));
		}
		fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.fext(0),carm.fext(1),carm.fext(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2));
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.temp1(0),carm.temp1(1),carm.temp1(2),carm.temp1(3),carm.temp1(4),carm.temp1(5),carm.temp2(0),carm.temp2(1),carm.temp2(2),carm.temp2(3),carm.temp2(4),carm.temp2(5),carm.fext(0),carm.fext(1),carm.fext(2),carm.xc(0),carm.xc(1),carm.xc(2));
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",Mdx(0),Mdx(1),Mdx(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),Mdth(0),Mdth(1),Mdth(2),carm.tht(0),carm.tht(1),carm.tht(2),carm.thc(0),carm.thc(1),carm.thc(2));
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",Mdx(0),Mdx(1),Mdx(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
	}

	else if(ctrl==5){ //5
		cmd_mode=CMD_MODE_TORQUE;
		Afun.ArmImpedance(&carm, &kfun);
		
		carm.temp1 << (carm.fext - fext_pre), (carm.xc - xc_pre);
		temp_fil << 0.95*temp_fil + 0.05*carm.temp1;
		fext_pre = carm.fext;
		xc_pre = carm.xc; 
		for(int i=0;i<3;i++){
			if(temp_fil(i+3)<0.00001&&temp_fil(i+3)>-0.00001) carm.temp2(i) = 0;
			else carm.temp2(i) = abs(temp_fil(i)/temp_fil(i+3));
		}

		fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.temp1(0),carm.temp1(1),carm.temp1(2),carm.temp1(3),carm.temp1(4),carm.temp1(5),carm.temp2(0),carm.temp2(1),carm.temp2(2),carm.temp2(3),carm.temp2(4),carm.temp2(5),carm.fext(0),carm.fext(1),carm.fext(2),carm.xc(0),carm.xc(1),carm.xc(2));
///////////////////////////////////////////////////////////
		//fprintf(gtData, "%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",gtcnt,carm.qc(0),carm.qc(1),carm.qc(2),carm.qc(3),carm.qc(4),carm.qc(5),carm.dqc(0),carm.dqc(1),carm.dqc(2),carm.dqc(3),carm.dqc(4),carm.dqc(5),carm.Tor_A(0),carm.Tor_A(1),carm.Tor_A(2),carm.Tor_A(3),carm.Tor_A(4),carm.Tor_A(5),carm.ddqd(0),carm.ddqd(1),carm.ddqd(2),carm.ddqd(3),carm.ddqd(4),carm.ddqd(5),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",Mdx(0),Mdx(1),Mdx(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
	}

	else if(ctrl==6){ //6
		double Md,Kd;
		Vector3d Fd;
		Matrix3d Cd,I,Co, Ko;
		I=Matrix3d::Identity();
		//Cd = 30*I;  
		//Co = 15*I;  
		//Md=10;  
		Cd = 200*I;  
		Co = 30*I;  
		Md=5;
		//Cd = 20*I;  
		//Co = 10*I;  
		//Md=5;
		Kd=300;




		Matrix3d Re, Ret, Re2std;
		Vector3d Fend, Tend;
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				Re(i,j)=carm.Tc(i,j);
			}
		}
		Ret=Re.transpose();

		Fend << carm.FTSensor(0), carm.FTSensor(1), carm.FTSensor(2);
		Tend << carm.FTSensor(3), carm.FTSensor(4), carm.FTSensor(5);

		Vector3d l, cal_td;
		l << 0.02, 0.02, 0.15;//Handle information, length from center of sensor
		cal_td << Tend(0)/l(0), Tend(1)/l(1), Tend(2)/l(2); //Calculated Torque
		carm.fext = Re*Fend;
		carm.text = Re*cal_td;

		kfun.Jacobian(&carm);
 		carm.J_t = carm.J.transpose();
 		carm.Jp_t = carm.Jp.transpose();

		MatrixXd JJt;
		JJt = carm.J*carm.J_t;
		carm.J_i = carm.J_t*JJt.inverse();


		kfun.MappingMatrix(&carm);

		carm.dthc << carm.Tmap_it*(carm.thc - thc_pre)/0.001;
		thc_pre = carm.thc; 

		carm.xt = carm.xt + carm.Masterdx/1000;
		carm.tht = carm.tht + carm.Masterdth/1000;

		Vector3d tempth;
		tempth = carm.tht - carm.thc;
		for(int j=0;j<3;j++){
			if(tempth(j)>PI)  	tempth(j)=tempth(j) - PI2;
			if(tempth(j)<-PI)  	tempth(j)=tempth(j) + PI2;
		}

		carm.fd << carm.fext  - Cd*carm.Jp*carm.dqc + 1000*(carm.xt - carm.xc);
		carm.td << carm.text  - Co*carm.Jw*carm.dqc + 100*carm.Tmap_it*(tempth);
		carm.Fd << carm.fd, carm.td;
		carm.Fext<< carm.fext, carm.text;
		int UAIn = 20;
		gtcnt++;
		if(gtcnt<17){
			//kfun.DynErr_comp(&carm);
			kfun.Dynamics(&carm);
			kfun.dotJacobian(&carm);
			carm.ddXd <<carm.fd/Md,0,0,0;
			carm.Fd << 0,0,0, carm.td;
			carm.Tau = carm.J_t*carm.Fd + carm.M_q*carm.J_i*(carm.ddXd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc + carm.Tor_grav - carm.J_t*carm.Fext;//  + carm.Tor_DEC;
			kfun.Friction_comp(&carm);
			carm.Tau = carm.Tau + carm.Tor_fric;
			//dxd_pre<<carm.Masterdx;
			//dthd_pre<<carm.Masterdth;
			dxd_pre<<carm.dxc;
			dthd_pre<<carm.dthc;
			cmd_mode=CMD_MODE_NO_CNTL;
		}
		else{
			carm.dxd=(carm.fd)/Md * 0.001 + dxd_pre;
			carm.dthd=(carm.td)/3 * 0.001 + dthd_pre;
			dxd_pre=carm.dxd;
			dthd_pre=carm.dthd;

			VectorXd dxdg(6);
			dxdg << carm.dxd, carm.dthd;
			carm.dqd = carm.J_i*dxdg;
	
			cmd_mode=CMD_MODE_VELOCITY;
		}
		if(gtcnt>UAIn-1) gtcnt=0;






///////////////////////////////////////////////////////////
		//fprintf(gtData, "%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",gtcnt,carm.qc(0),carm.qc(1),carm.qc(2),carm.qc(3),carm.qc(4),carm.qc(5),carm.dqc(0),carm.dqc(1),carm.dqc(2),carm.dqc(3),carm.dqc(4),carm.dqc(5),carm.Tor_A(0),carm.Tor_A(1),carm.Tor_A(2),carm.Tor_A(3),carm.Tor_A(4),carm.Tor_A(5),carm.ddqd(0),carm.ddqd(1),carm.ddqd(2),carm.ddqd(3),carm.ddqd(4),carm.ddqd(5),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",Mdx(0),Mdx(1),Mdx(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",Mdx(0),Mdx(1),Mdx(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
	}
	else if(ctrl==7){ //7
		Matrix3d Re, Ret, Re2std;
		Vector3d Fend, Tend;
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				Re(i,j)=carm.Tc(i,j);
			}
		}
		Ret=Re.transpose();

		Fend << carm.FTSensor(0), carm.FTSensor(1), carm.FTSensor(2);
		Tend << carm.FTSensor(3), carm.FTSensor(4), carm.FTSensor(5);
		Vector3d l, cal_td;
		l << 0.02, 0.02, 0.04;//Handle information, length from center of sensor
		cal_td << Tend(0)/l(0), Tend(1)/l(1), Tend(2)/l(2); //Calculated Torque
		carm.fext = Re*Fend;
		carm.text = Re*cal_td;

		kfun.Jacobian(&carm);
 		carm.J_t = carm.J.transpose();
 		carm.Jp_t = carm.Jp.transpose();

		MatrixXd JJt;
		JJt = carm.J*carm.J_t;
		carm.J_i = carm.J_t*JJt.inverse();


		kfun.MappingMatrix(&carm);

		carm.xt = carm.xt + Mdx/1000;
		carm.tht = carm.tht + Mdth/1000;

		Vector3d tempth;
		tempth = carm.tht - carm.thc;
		for(int j=0;j<3;j++){
			if(tempth(j)>PI)  	tempth(j)=tempth(j) - PI2;
			if(tempth(j)<-PI)  	tempth(j)=tempth(j) + PI2;
		}

		float absF;
		Vector3d absFC;
		absFC= Fend;// + 1000*(carm.xt - carm.xc);
		absF = sqrt(absFC(0)*absFC(0) + absFC(1)*absFC(1) + absFC(2)*absFC(2));
/*
		carm.temp1 << (carm.fext - fext_pre), (carm.xc - xc_pre);
		temp_fil << 0.95*temp_fil + 0.05*carm.temp1;
		fext_pre = carm.fext;
		xc_pre = carm.xc; 
		for(int i=0;i<3;i++){
			if(temp_fil(i+3)<0.00001&&temp_fil(i+3)>-0.00001) carm.temp2(i) = 0;
			else carm.temp2(i) = abs(temp_fil(i)/temp_fil(i+3));
		}
		carm.temp1 << (carm.fext - fext_pre), (carm.xc - xc_pre);
		temp_fil << 0.95*temp_fil + 0.05*carm.temp1;
		fext_pre = carm.fext;
		xc_pre = carm.xc; 
*/
		carm.temp1 << (carm.fext - fext_pre), (carm.xc - xc_pre);

		NormF = carm.temp1(0)*carm.temp1(0) +  carm.temp1(1)*carm.temp1(1) +  carm.temp1(2)*carm.temp1(2);
		NormX = carm.temp1(3)*carm.temp1(3) +  carm.temp1(4)*carm.temp1(4) +  carm.temp1(5)*carm.temp1(5);

		//NormF = carm.fext.transpose()*carm.fext - fext_pre.transpose()*fext_pre;
		//NormX = carm.xc.transpose()*carm.xc - xc_pre.transpose()*xc_pre;
		fext_pre = carm.fext;
		xc_pre = carm.xc; 
		NormFf = 0.95*NormFf + 0.05*sqrt(NormF);
		NormXf = 0.95*NormXf + 0.05*sqrt(NormX);


		if(NormXf<0.00001&&NormXf>-0.00001) NormK = 0;
		else NormK = abs(NormFf/NormXf);

		if(absF>1&&NormK>50000) VIAMode=1;
		if(absF<5) VIAMode=0;


		carm.temp2(0)=NormK;
		carm.temp2(1)=VIAMode;
		carm.temp2(2)=absF;
		fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.temp1(0),carm.temp1(1),carm.temp1(2),carm.temp1(3),carm.temp1(4),carm.temp1(5),carm.temp2(0),carm.temp2(1),carm.temp2(2),carm.temp2(3),carm.temp2(4),carm.temp2(5),carm.fext(0),carm.fext(1),carm.fext(2),carm.xc(0),carm.xc(1),carm.xc(2));
		
		if(VIAMode==1){
			double Mo;
			Vector3d Kd,Cd,Md;
			Matrix3d CdI,Co,I,KdI,invMdI;
			I=Matrix3d::Identity();
			Cd << 1000, 1000, 300;
			Co=100*I; 
			CdI = I;  
			for(int i=0;i<3;i++) 	CdI(i,i) = Cd(i);

			Kd << 1000, 1000, 1000;
			KdI = I;
			for(int i=0;i<3;i++) 	KdI(i,i) = Kd(i);

			Md << 10, 10, 10;
			invMdI = I;
			for(int i=0;i<3;i++) 	invMdI(i,i) = 1/Md(i);
			Mo=5;
/*

			Vector3d Direction;
			Direction=carm.fext;

			Matrix3d Rp;
			float alpha=atan2(Direction(1),Direction(0));
			Matrix3d Ralpha = MatrixXd::Zero(3,3);
			Ralpha(0,0)=cos(alpha);
			Ralpha(0,1)=-sin(alpha);
			Ralpha(1,0)=sin(alpha);
			Ralpha(1,1)=cos(alpha);
			Ralpha(2,2)=1;
			float beta=-atan2(Direction(2),sqrt(Direction(0)*Direction(0)+Direction(1)*Direction(1)));
			Matrix3d Rbeta = MatrixXd::Zero(3,3);
			Rbeta(0,0)=cos(beta);
			Rbeta(0,2)=sin(beta);
			Rbeta(2,0)=-sin(beta);
			Rbeta(2,2)=cos(beta);
			Rbeta(1,1)=1;
			Rp = Ralpha*Rbeta;


			CdI=Rp*Cph_x*Rp.transpose();
			KdI=Rp*invMph_x*Rp.transpose();
*/

			carm.fd << carm.fext  - CdI*carm.Jp*carm.dqc + KdI*(carm.xt - carm.xc);
			carm.td << carm.text  - Co*carm.Jw*carm.dqc + 100*carm.Tmap_it*(tempth);
			carm.Fd << carm.fd, carm.td;
			carm.Fext<< carm.fext, carm.text;

			//kfun.DynErr_comp(&carm);
			kfun.Dynamics(&carm);
			kfun.dotJacobian(&carm);
			carm.ddXd <<invMdI*carm.fd,0,0,0;
			carm.Fd << 0,0,0, carm.td;
			carm.Tau = carm.J_t*carm.Fd + carm.M_q*carm.J_i*(carm.ddXd - carm.dJ*carm.dqc) + carm.C_q*carm.dqc + carm.Tor_grav - carm.J_t*carm.Fext;//  + carm.Tor_DEC;
			kfun.Friction_comp(&carm);
			carm.Tau = carm.Tau + carm.Tor_fric;
			cmd_mode=CMD_MODE_NO_CNTL;
		}
		else{
			double Md,Mo;
			Vector3d Kd;
			Matrix3d Cd,Co,I,KdI;
			I=Matrix3d::Identity();
			Cd = 50*I;  
			Co = 30*I;  
			Md=5; 
			Mo=3;

			carm.fd << carm.fext  - Cd*carm.Jp*carm.dqc + 500*(carm.xt - carm.xc);
			carm.td << carm.text  - Co*carm.Jw*carm.dqc + 50*carm.Tmap_it*(tempth);
			carm.Fd << carm.fd, carm.td;
			carm.Fext<< carm.fext, carm.text;

			carm.dxd=(carm.fd)/Md * 0.001 + dxd_pre;
			carm.dthd=(carm.td)/Mo * 0.001 + dthd_pre;
			dxd_pre=carm.dxd;
			dthd_pre=carm.dthd;

			VectorXd dxdg(6);
			dxdg << carm.dxd, carm.dthd;
			carm.dqd = carm.J_i*dxdg;
	
			cmd_mode=CMD_MODE_VELOCITY;
		}






///////////////////////////////////////////////////////////
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",Mdx(0),Mdx(1),Mdx(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
		//fprintf(gtData, "%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",gtcnt,carm.qc(0),carm.qc(1),carm.qc(2),carm.qc(3),carm.qc(4),carm.qc(5),carm.dqc(0),carm.dqc(1),carm.dqc(2),carm.dqc(3),carm.dqc(4),carm.dqc(5),carm.Tor_A(0),carm.Tor_A(1),carm.Tor_A(2),carm.Tor_A(3),carm.Tor_A(4),carm.Tor_A(5),carm.ddqd(0),carm.ddqd(1),carm.ddqd(2),carm.ddqd(3),carm.ddqd(4),carm.ddqd(5),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
		//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",Mdx(0),Mdx(1),Mdx(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),carm.fext(0),carm.fext(1),carm.fext(2),carm.text(0),carm.text(1),carm.text(2));
	}

	else if(ctrl==51){ //a

		cmd_mode=CMD_MODE_POSITION;


		//carm.qt = carm.qc;
		//carm.qt << 0, -15*PI/180, -75*PI/180, -90*PI/180, 0,0;
		carm.qt << 0, -5*PI/180, -85*PI/180, 0, -90*PI/180, -90*PI/180;

		int ret=0;
		ret=tfun.Traj_rspb(&carm);
		if(ret>0) ctrl = 0;

	}

	else if(ctrl==21){ //c

		cmd_mode=CMD_MODE_POSITION;

		carm.qt << 0,	-0.262183799948159,	-1.61816642159002,	0,	-1.26124243205162,	0;
		//carm.qt << 0,	-0.729957252622557,	-0.861815780697708,	0,	-1.54981962026953,	0;
		carm.xt = carm.xc;
		carm.tht = carm.thc;
		int ret=0;
		ret=tfun.Traj_rspb(&carm);
		if(ret>0) ctrl = 0;
  
	}
	else if(ctrl==21){ //c

		cmd_mode=CMD_MODE_POSITION;

		carm.qt << 0,	-0.262183799948159,	-1.61816642159002,	0,	-1.26124243205162,	0;
		//carm.qt << 0,	-0.729957252622557,	-0.861815780697708,	0,	-1.54981962026953,	0;
		carm.xt = carm.xc;
		carm.tht = carm.thc;
		int ret=0;
		ret=tfun.Traj_rspb(&carm);
		if(ret>0) ctrl = 0;

	}
	else if(ctrl==22){ //v
		gtcnt++;

		/*float length;
		length=0.05;
		if(gtcnt==1000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==6000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==11000) 	carm.xt(2) = carm.xt(2)+length;
		if(gtcnt==16000) 	carm.xt(2) = carm.xt(2)+length;
		if(gtcnt>21000) {
			ctrl=0;
			gtcnt=0;
		}*/

   
		float length;
		length=0.005;
		if(gtcnt==1000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==1500) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==2000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==2500) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==3000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==3500) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==4000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==4500) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==5000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==5500) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==6000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==6500) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt>7000) {
			ctrl=0;
			gtcnt=0;
		}
		cmd_mode=CMD_MODE_VELOCITY;
		Afun.ArmAdmittance(&carm, &kfun);
		//fprintf(gtData2, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.fext(0),carm.fext(1),carm.fext(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2));
	}
	else if(ctrl==23){ //b
		gtcnt++;
		/*float length;
		length=0.05;
		if(gtcnt==1000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==6000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==11000) 	carm.xt(2) = carm.xt(2)+length;
		if(gtcnt==16000) 	carm.xt(2) = carm.xt(2)+length;
		if(gtcnt>21000) {
			ctrl=0;
			gtcnt=0;
		}*/
		float length;
		length=0.025;     
		if(gtcnt==1000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==6000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==11000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==16000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt==21000) 	carm.xt(2) = carm.xt(2)-length;
		if(gtcnt>26000) {
			ctrl=0;
			gtcnt=0;
		}



		cmd_mode=CMD_MODE_TORQUE;
		Afun.ArmImpedance(&carm, &kfun);

		//cmd_mode=CMD_MODE_VELOCITY;
		//Afun.ArmAdmittance(&carm, &kfun);
		fprintf(gtData2, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.fext(0),carm.fext(1),carm.fext(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2));

	}
	else if(ctrl==8){ //8
		cmd_mode=CMD_MODE_VELOCITY;

		VectorXd tempS(4);
		for(int i=0;i<4;i++){                //force
			if(carm.ProxSensor(i)<-4000)	 	tempS(i) = 50;
			//else if(carm.ProxSensor(i)<-500)	tempS(i) =  -carm.ProxSensor(i)/70 + 30/7;
			else if(carm.ProxSensor(i)<-500)	tempS(i) =  50;
			else if(carm.ProxSensor(i)>-100)	tempS(i) = 0;
			else tempS(i) = -carm.ProxSensor(i)/8 + 100/8;
		}

		Matrix3d tempR;
		//tempR << 0.923879, -0.382683, 0, 0.382683, 0.923879,0, 0, 0, 1;
		//tempR << 0.382683, -0.923879, 0, 0.923879, 0.382683, 0, 0, 0, 1;
		//tempR << 0.7071, 0.7071, 0,  -0.7071,  0.7071, 0, 0, 0, 1;
		//tempR << 0.866, 0.5, 0, -0.5, 0.866,0, 0, 0, 1;
		Vector3d tempfd;

		//tempfd << tempS(0) - tempS(2), tempS(1) - tempS(3), 0;
		tempfd << tempS(1) - tempS(3),0, 0;
		//tempfd << tempS(0) - tempS(2), tempS(1), 0;
		carm.fd =tempfd;// tempR * tempfd;
		carm.td << 0,0,0; 

		Afun.ArmAdmittanceProxMove(&carm, &kfun);
		//Afun.ArmAdmittanceProx(&carm, &kfun);
		//carm.dqd(3)=0;	
	}
	else if(ctrl==9){ //9
		cmd_mode=CMD_MODE_VELOCITY;

		VectorXd tempS(4);
		for(int i=0;i<4;i++){  
			if(carm.ProxSensor(i)<-10000)	tempS(i) = 2;
			else if(carm.ProxSensor(i)<-5000)	tempS(i) = 1;
			else tempS(i) = 0;
		}


		Matrix3d tempR;
		//tempR << 0.7071, -0.7071, 0,  0.7071,  0.7071, 0, 0, 0, 1;
		//tempR << 0.923879, 0.382683, 0, -0.382683, 0.923879,0, 0, 0, 1;
		tempR << 0.923879, -0.382683, 0, 0.382683, 0.923879,0, 0, 0, 1;
		Vector3d tempfd;

		tempfd << tempS(0) - tempS(2), tempS(1) - tempS(3), 0;
		//tempfd << tempS(0) - tempS(2), tempS(1), 0;
		carm.fd = tempR * tempfd;
		carm.td << 0,0,0; 
		Afun.ArmAdmittanceTact(&carm, &kfun);

	}
	else if (ctrl == 70) {  ///Joint Basic Direct with collision avoidance with Gripper
		float Switch[5];
		for (int i = 0; i < 5; i++)
			Switch[i] = SwitchS[i];
		if (Switch[0] == 0 && Switch[1] == 0 && Switch[2] == 0 && Switch[3] == 0) {
			pointnum = 0;
			for (int i = 0; i < 5; i++) {
				Switch[i] = 0;
				SwitchS[i] = 0;
				SwitchPre[i] = 0;
			}
			for (int i = 0; i < 20; i++)
				PointTraj[i] = 0;
		}
		if (SEQ_switch == 0) { //initialize
			std::cout << "Position Control Deirect Teaching" << std::endl;
			for (int i = 0; i < 5; i++) {
				Switch[i] = 0;
				SwitchS[i] = 0;
				SwitchPre[i] = 0;
			}
			for (int i = 0; i < 20; i++)
				PointTraj[i] = 0;

			SEQ_switch = 1;
			pointnum = 0;


			sprintf(filename_record, "/home/user/catkin_ws/src/indy7/PathPointRecord1.txt");
			sprintf(filename_replay, "/home/user/catkin_ws/src/indy7/PathPointRecord1.txt");
			fp_record = fopen(filename_record, "wt");
			fclose(fp_record);

			cmd_mode=100;
		}
		else if (SEQ_switch == 1) { //while moving, save points and trajectory
			cmd_mode=100;
			carm.qt = carm.qc;
			if (Switch[0] > SwitchPre[0]) {
				SwitchPre[0] = Switch[0];

				fp_record = fopen(filename_record, "at");
				fprintf(fp_record, "%d\n", 1);
				fprintf(fp_record, "%f %f %f %f %f %f\n", carm.qc[0], carm.qc[1], carm.qc[2], carm.qc[3], carm.qc[4], carm.qc[5]);
				fclose(fp_record);
			}
			if (Switch[2] > SwitchPre[2]) {
				SwitchPre[2] = Switch[2];

				fp_record = fopen(filename_record, "at");
				fprintf(fp_record, "%d\n", 2);
				fprintf(fp_record, "%f %f %f %f %f %f\n", carm.qc[0], carm.qc[1], carm.qc[2], carm.qc[3], carm.qc[4], carm.qc[5]);
				fclose(fp_record);
			}
			/*if (Switch[3] > SwitchPre[3]) {
				SwitchPre[3] = Switch[3];
				if ((int)SwitchPre[3] % 2 == 1) {
					DO[0] = 0x55;
					fp_record = fopen(filename_record, "at");
					fprintf(fp_record, "%d\n", 11);
					fclose(fp_record);
					std::cout << "Gripper Close" << std::endl;
				}
				else if ((int)SwitchPre[3] % 2 == 0) {
					DO[0] = 0xaa;
					fp_record = fopen(filename_record, "at");
					fprintf(fp_record, "%d\n", 12);
					fclose(fp_record);
					std::cout << "Gripper Open" << std::endl;
				}
			}*/
			if (Switch[1] > 0){
				SwitchS[1] = 0;
				if(fp_replay = fopen(filename_replay, "rt"))
					SEQ_switch = 2;
			}
		}
		else if (SEQ_switch == 2) { //arrange the play sequence
			pause_cnt++;
			if (pause_cnt > 100) {
				pause_cnt = 0;
				int SEQ_record;
				int reti = fscanf(fp_replay, "%d\n", &SEQ_record);
				carm.temp2<<123,123,123,123,123,123;
				if (reti != EOF) {
					if (SEQ_record == 1) {
						SEQ_switch = 3;
					}
					else if (SEQ_record == 2) {
						SEQ_switch = 4;
					}
					else if (SEQ_record == 11) {
						SEQ_switch = 11;
					}
					else if (SEQ_record == 12) {
						SEQ_switch = 12;
					}
				}
				else {
					fclose(fp_replay );
					SEQ_play = 0;
					SEQ_switch = 1;
				}
			}
			stop_flag = 0;
			cmd_mode=100;
		}
		else if (SEQ_switch == 3) { //Path to point planning
			if (SEQ_point == 0) {
			carm.temp2<<111,111,111,111,111,111;
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;
			}
			if (SEQ_point == 1) {
			carm.temp2<<111,111,111,111,111,222;
				cmd_mode=CMD_MODE_POSITION;
				int ret=0;
				//cmd_mode=100;

				//if(carm.ProxSensor(0)<-5000||carm.ProxSensor(1)<-5000||carm.ProxSensor(2)<-5000||carm.ProxSensor(3)<-5000)	HYflag = 1;
				if(carm.ProxSensor(0)<-200||carm.ProxSensor(1)<-200||carm.ProxSensor(2)<-200||carm.ProxSensor(3)<-200) HYflag = 1;

				if (HYflag == 0)
					ret=tfun.Traj_rspb(&carm);

				else if (HYflag == 1)
					ret = tfun.ArmJointSlowStop(carm.Traj_dQ, carm.qc, carm.qd);

				//else if (HYflag == 2) % Need upgrade 
					//ret = tfun.ArmJointFastBack(carm.Traj_dQ, carm.qd);

				if (ret > 0) {
					if (HYflag == 0) SEQ_point = 2;
					if (HYflag == 1) HYflag = 0;
					if (HYflag == 2) HYflag = 0;
				}

			}
			if (SEQ_point == 2) {
			carm.temp2<<111,111,111,111,111,333;
				pause_cnt++;
				if (pause_cnt > 100) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
				//cmd_mode=100;
				cmd_mode=CMD_MODE_POSITION;
			}
		}
		else if (SEQ_switch == 4) { //Path to point planning
			carm.temp2<<222,222,222,222,222,222;
			if (SEQ_point == 0) {
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;

				/*MatrixXd tempT(4,4);
				kfun.iForwardK_T(carm.qt, tempT,0);
				Matrix3d tempR;
				Vector3d tempth;
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						tempR(i,j)=tempT(i,j);
					}
				}
				kfun.iRotation2EulerAngle(tempR, tempth);

				carm.xt << tempT(0,3),tempT(1,3),tempT(2,3);
				carm.tht = tempth;
				carm.Traj_Xt << carm.xt, carm.tht;
				carm.Traj_Xc << carm.xc, carm.thc;*/
			}
			if (SEQ_point == 1) {

				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;

				if(carm.ProxSensor(0)<-3000||carm.ProxSensor(1)<-3000||carm.ProxSensor(2)<-3000||carm.ProxSensor(3)<-3000)	HYflag = 1;
				//if(carm.ProxSensor(0)<-300||carm.ProxSensor(1)<-300||carm.ProxSensor(2)<-300||carm.ProxSensor(3)<-300) HYflag = 1;

				if (HYflag == 0)
					ret=tfun.Traj_rspb(&carm);
					//ret=tfun.TaskTraj_rspb(&carm);

				else if (HYflag == 1){
					carm.qd = carm.qc;
					ret = tfun.TrajInitial();
					//ret = tfun.ArmJointSlowStop(carm.dqd, carm.qc, carm.qd);
					//ret = tfun.ArmJointSlowStop(carm.dqc, carm.qc, carm.qd);
				}
				//else if (HYflag == 2) % Need upgrade 
					//ret = tfun.ArmJointFastBack(carm.Traj_dQ, carm.qd);


				VectorXd tempS(4);
				for(int i=0;i<4;i++){                //force
					if(carm.ProxSensor(i)<-2000)	 	tempS(i) = 50 - 10;
					else if(carm.ProxSensor(i)<-500)	tempS(i) =  -carm.ProxSensor(i)/60 + 250/6 - 10;
					else if(carm.ProxSensor(i)>-100)	tempS(i) = 0;
					else tempS(i) = -carm.ProxSensor(i)/10 - 10;
				}

				Matrix3d tempR;
				tempR << 0.923879, 0.382683, 0, -0.382683, 0.923879,0, 0, 0, 1;
				Vector3d tempfd;

				tempfd << tempS(0) - tempS(2), tempS(1) - tempS(3), 0;
				//tempfd << tempS(0) - tempS(2), tempS(1), 0;
				carm.fd = tempR * tempfd*1.2;
				carm.td << 0,0,0; 
				if (HYflag == 1) carm.fd << 0,0,0; 

				//Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				Afun.ArmAdmittanceJointTraj(&carm, &kfun);

				if (ret > 0) {
					if (HYflag == 0) SEQ_point = 2;
					if (HYflag == 1) HYflag = 0;
					if (HYflag == 2) HYflag = 0;
				}

			}
			if (SEQ_point == 2) {
				//std::cout <<"@@"<< carm.qc.transpose() << std::endl;
				pause_cnt++;
				if (pause_cnt > 2000) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
				//cmd_mode=100;
				cmd_mode=CMD_MODE_VELOCITY;
				Afun.ArmAdmittanceJointTraj(&carm, &kfun);
				//Afun.ArmAdmittanceTaskTraj(&carm, &kfun);

			}
		}
		else if (SEQ_switch == 11) { //Gripper Close
			carm.temp2<<888,888,888,888,888,888;
			if (SEQ_point == 0) {
				SEQ_point = 1;
				DO[0] = 0x55;
			}
			if (SEQ_point == 1) {
				pause_cnt++;
				cmd_mode=100;
				if (pause_cnt > 1000) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
			}
		}
		else if (SEQ_switch == 12) { //Gripper Open
			carm.temp2<<999,999,999,999,999,999;
			if (SEQ_point == 0) {
				SEQ_point = 1;
				DO[0] = 0xaa;
			}
			if (SEQ_point == 1) {
				pause_cnt++;
				cmd_mode=100;
				if (pause_cnt > 1000) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
			}
		}
	}
	else if (ctrl == 75) {  ///Gitae journal test
		float Switch[5];
		for (int i = 0; i < 5; i++)
			Switch[i] = SwitchS[i];
		if (Switch[0] == 0 && Switch[1] == 0 && Switch[2] == 0 && Switch[3] == 0) {
			pointnum = 0;
			for (int i = 0; i < 5; i++) {
				Switch[i] = 0;
				SwitchS[i] = 0;
				SwitchPre[i] = 0;
			}
			for (int i = 0; i < 20; i++)
				PointTraj[i] = 0;
		}
		if (SEQ_switch == 0) { //initialize
			std::cout << "Position Control Deirect Teaching" << std::endl;
			for (int i = 0; i < 5; i++) {
				Switch[i] = 0;
				SwitchS[i] = 0;
				SwitchPre[i] = 0;
			}
			for (int i = 0; i < 20; i++)
				PointTraj[i] = 0;

			SEQ_switch = 1;
			pointnum = 0;


			sprintf(filename_record, "/home/user/catkin_ws/src/indy7/PPR_T1.txt");
			sprintf(filename_replay, "/home/user/catkin_ws/src/indy7/PPR_T5.txt");
			fp_record = fopen(filename_record, "wt");
			fclose(fp_record);

			cmd_mode=100;
		}
		else if (SEQ_switch == 1) { //while moving, save points and trajectory
			cmd_mode=100;
			carm.qt = carm.qc;
			if (Switch[0] > SwitchPre[0]) {
				SwitchPre[0] = Switch[0];

				fp_record = fopen(filename_record, "at");
				fprintf(fp_record, "%d\n", 1);
				fprintf(fp_record, "%f %f %f %f %f %f\n", carm.qc[0], carm.qc[1], carm.qc[2], carm.qc[3], carm.qc[4], carm.qc[5]);
				fclose(fp_record);
			}
			if (Switch[2] > SwitchPre[2]) {
				SwitchPre[2] = Switch[2];

				fp_record = fopen(filename_record, "at");
				fprintf(fp_record, "%d\n", 2);
				fprintf(fp_record, "%f %f %f %f %f %f\n", carm.qc[0], carm.qc[1], carm.qc[2], carm.qc[3], carm.qc[4], carm.qc[5]);
				fclose(fp_record);
			}
			if (Switch[1] > 0){
				SwitchS[1] = 0;
				if(fp_replay = fopen(filename_replay, "rt"))
					SEQ_switch = 2;
			}
		}
		else if (SEQ_switch == 2) { //arrange the play sequence
			//pause_cnt++;
			//if (pause_cnt > 5) {
			//	pause_cnt = 0;
				int SEQ_record;
				int reti = fscanf(fp_replay, "%d\n", &SEQ_record);
				if (reti != EOF) {
					if (SEQ_record == 1) {
						SEQ_switch = 3;
						stop_flag = 0;
						cmd_mode=100;
					}
					else if (SEQ_record == 2) {
						SEQ_switch = 4;
					}
					else if (SEQ_record == 3) {
						SEQ_switch = 5;
					}
				}
				else {
					fclose(fp_replay);
					SEQ_play = 0;
					SEQ_switch = 1;
					stop_flag = 0;
					cmd_mode=100;
				}
			//}
		}
		else if (SEQ_switch == 3) { //Path to point planning
			if (SEQ_point == 0) {
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;
				carm.temp1=carm.qt;
			}
			if (SEQ_point == 1) {
				cmd_mode=CMD_MODE_POSITION;
				int ret=0;
				ret=tfun.Traj_rspb(&carm);
				if(ret>0) SEQ_point = 2;

/*				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				ret=tfun.Traj_rspb(&carm);
				Afun.ArmAdmittanceJointTraj(&carm, &kfun);
				if(ret>0) SEQ_point = 2;
*/
			}                     

			if (SEQ_point == 2) {
				pause_cnt++;
				if (pause_cnt > 500) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
				cmd_mode=CMD_MODE_POSITION;
				//cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceJointTraj(&carm, &kfun);
			}
		}
		else if (SEQ_switch == 4) { //Path to point planning
			if (SEQ_point == 0) {
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;
				carm.dxd_guide<<0,0,0;
				carm.dthd_guide<<0,0,0;
				carm.xd_guide=carm.xc;
				carm.qd = carm.qc;
				carm.xd = carm.xc;

				MatrixXd tempT(4,4);
				kfun.iForwardK_T(carm.qt, tempT,0);
				Matrix3d tempR;
				Vector3d tempth;
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						tempR(i,j)=tempT(i,j);
					}
				}
				kfun.iRotation2EulerAngle(tempR, tempth);

				carm.xt << tempT(0,3),tempT(1,3),tempT(2,3);
				carm.tht = tempth;
				carm.Traj_Xt << carm.xt, carm.tht;
				carm.Traj_Xc << carm.xc, carm.thc;
				//cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				cmd_mode=CMD_MODE_TORQUE;   
				//Afun.ArmImpedanceTaskTraj(&carm, &kfun);
				Afun.ArmUIATaskTraj(&carm, &kfun);
				//cmd_mode=100;
			}
			if (SEQ_point == 1) {
				int ret=0;
				ret=tfun.TaskTraj_rspb(&carm);
				//cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				cmd_mode=CMD_MODE_TORQUE;   
				//Afun.ArmImpedanceTaskTraj(&carm, &kfun);
				Afun.ArmUIATaskTraj(&carm, &kfun);
				//cmd_mode=100;

				if(ret>0) SEQ_point = 2;
			}
			if (SEQ_point == 2) {
				carm.Traj_Xd=carm.Traj_Xt;
				carm.Traj_Vd<< 0, 0, 0, 0, 0, 0;
				//std::cout <<"@@"<< carm.qc.transpose() << std::endl;
				pause_cnt++;
				if (pause_cnt > 3000) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
				//cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				cmd_mode=CMD_MODE_TORQUE;   
				//Afun.ArmImpedanceTaskTraj(&carm, &kfun);
				Afun.ArmUIATaskTraj(&carm, &kfun);
				//cmd_mode=100;

			}
		}
		else if (SEQ_switch == 5) { //Path to point planning
			if (SEQ_point == 0) {
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;
				carm.dxd_guide<<0,0,0;
				carm.dthd_guide<<0,0,0;
				carm.xd_guide=carm.xc;
				carm.qd = carm.qc;
				carm.xd = carm.xc;

				MatrixXd tempT(4,4);
				kfun.iForwardK_T(carm.qt, tempT,0);
				Matrix3d tempR;
				Vector3d tempth;
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						tempR(i,j)=tempT(i,j);
					}
				}
				kfun.iRotation2EulerAngle(tempR, tempth);

				carm.xt << tempT(0,3),tempT(1,3),tempT(2,3);
				carm.tht = tempth;
				carm.Traj_Xt_pre << carm.Traj_Xt;
				carm.Traj_Xt << carm.xt, carm.tht;
				carm.Traj_Xc << carm.xc, carm.thc;
				//cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				cmd_mode=CMD_MODE_TORQUE;   
				//Afun.ArmImpedanceTaskTraj(&carm, &kfun);
				Afun.ArmUIATaskTraj(&carm, &kfun);
				//cmd_mode=100;
			}
			if (SEQ_point == 1) {
				int ret=0;
				ret=tfun.TaskTraj_rspb_gitae(&carm);
				//cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				cmd_mode=CMD_MODE_TORQUE;   
				//Afun.ArmImpedanceTaskTraj(&carm, &kfun);
				Afun.ArmUIATaskTraj(&carm, &kfun);
				//cmd_mode=100;

				if(ret>0) SEQ_point = 2;
			}
			if (SEQ_point == 2) {  
				carm.Traj_Xd=carm.Traj_Xt;
				carm.Traj_Vd<< 0, 0, 0, 0, 0, 0;
				//std::cout <<"@@"<< carm.qc.transpose() << std::endl;
				pause_cnt++;    
				if (pause_cnt > 3000) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
				//cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				cmd_mode=CMD_MODE_TORQUE;   
				//Afun.ArmImpedanceTaskTraj(&carm, &kfun);
				Afun.ArmUIATaskTraj(&carm, &kfun);
				//cmd_mode=100;
			}
		}
fprintf(gtData2, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.fext(0),carm.fext(1),carm.fext(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),carm.FErrTrack(0),carm.FErrTrack(1),carm.FErrTrack(2),carm.temp5(0),carm.temp5(1),carm.temp5(2),carm.temp5(3),carm.temp5(4),carm.temp5(5));
	}

 
	else if (ctrl == 76) {  ///Basic Direct For Force in Cartesian task
		float Switch[5];
		for (int i = 0; i < 5; i++)
			Switch[i] = SwitchS[i];
		if (Switch[0] == 0 && Switch[1] == 0 && Switch[2] == 0 && Switch[3] == 0) {
			pointnum = 0;
			for (int i = 0; i < 5; i++) {
				Switch[i] = 0;
				SwitchS[i] = 0;
				SwitchPre[i] = 0;
			}
			for (int i = 0; i < 20; i++)
				PointTraj[i] = 0;

		}
		if (SEQ_switch == 0) { //initialize
			std::cout << "Position Control Deirect Teaching" << std::endl;
			for (int i = 0; i < 5; i++) {
				Switch[i] = 0;
				SwitchS[i] = 0;
				SwitchPre[i] = 0;
			}
			for (int i = 0; i < 20; i++)
				PointTraj[i] = 0;

			SEQ_switch = 1;
			pointnum = 0;



			sprintf(filename_record, "/home/user/catkin_ws/src/indy7/PPR_T1.txt");
			sprintf(filename_replay, "/home/user/catkin_ws/src/indy7/PPR_T3.txt");
			fp_record = fopen(filename_record, "wt");
			fclose(fp_record);

			cmd_mode=100;
		}
		else if (SEQ_switch == 1) { //while moving, save points and trajectory
			cmd_mode=100;
			carm.qt = carm.qc;
			if (Switch[0] > SwitchPre[0]) {
				SwitchPre[0] = Switch[0];

				fp_record = fopen(filename_record, "at");
				fprintf(fp_record, "%d\n", 1);
				fprintf(fp_record, "%f %f %f %f %f %f\n", carm.qc[0], carm.qc[1], carm.qc[2], carm.qc[3], carm.qc[4], carm.qc[5]);
				fclose(fp_record);
			}
			if (Switch[2] > SwitchPre[2]) {
				SwitchPre[2] = Switch[2];

				fp_record = fopen(filename_record, "at");
				fprintf(fp_record, "%d\n", 2);
				fprintf(fp_record, "%f %f %f %f %f %f\n", carm.qc[0], carm.qc[1], carm.qc[2], carm.qc[3], carm.qc[4], carm.qc[5]);
				fclose(fp_record);
			}
			if (Switch[1] > 0){
				SwitchS[1] = 0;
				carm.FCtrlDir << 0, 0, -1;
				carm.FErrTrack << 0, 0,0 ;
				MaxFDesired = 10;

				if(fp_replay = fopen(filename_replay, "rt"))
					SEQ_switch = 2;
			}
			Afun.ArmForceGuide(&carm, &kfun);
			cmd_mode=CMD_MODE_VELOCITY;
		}
		else if (SEQ_switch == 2) { //arrange the play sequence
			pause_cnt++;
			if (pause_cnt > 100) {
				pause_cnt = 0;
				int SEQ_record;
				int reti = fscanf(fp_replay, "%d\n", &SEQ_record);
				if (reti != EOF) {
					if (SEQ_record == 1) {
						SEQ_switch = 3;
					}
					else if (SEQ_record == 2) {
						SEQ_switch = 4;
					}
					else if (SEQ_record == 21) {
						SEQ_switch = 21;
					}
					else if (SEQ_record == 22) {
						SEQ_switch = 22;
					}
				}
				else {
					fclose(fp_replay );
					SEQ_play = 0;
					carm.dxd << 0, 0, 0;
					carm.dthd << 0, 0, 0;
					SEQ_switch = 1;
				}
			}
			stop_flag = 0;
			if(force_on==1){
				carm.xt = carm.xc;
				carm.tht = carm.thc;
				carm.Traj_Vd << 0,0,0,0,0,0;
				//cmd_mode=CMD_MODE_TORQUE;
				//Afun.ArmImpedanceForcedTraj(&carm, &kfun);
				cmd_mode=carm.cmd_mode;
				//Afun.ArmUAIForcedTraj_journal(&carm, &kfun);
				Afun.ArmUAIForcedTraj(&carm, &kfun);
			}
			else	cmd_mode=100;	
		}
		else if (SEQ_switch == 3) { //Path to point planning
			if (SEQ_point == 0) {
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;
			}
			if (SEQ_point == 1) {

				cmd_mode=CMD_MODE_POSITION;
				int ret=0;
				ret=tfun.Traj_rspb(&carm);
				if(ret>0) SEQ_point = 2;

				/*cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				ret=tfun.Traj_rspb(&carm);
				Afun.ArmAdmittanceJointTraj(&carm, &kfun);
				if(ret>0) SEQ_point = 2;*/

			}
			if (SEQ_point == 2) {
				pause_cnt++;
				if (pause_cnt > 500) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
				cmd_mode=CMD_MODE_POSITION;
				//cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceJointTraj(&carm, &kfun);

			}
		}
		else if (SEQ_switch == 4) { //Path to point planning
			if (SEQ_point == 0) {
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;

				MatrixXd tempT(4,4);
				kfun.iForwardK_T(carm.qt, tempT,0);
				Matrix3d tempR;
				Vector3d tempth;
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						tempR(i,j)=tempT(i,j);
					}
				}
				kfun.iRotation2EulerAngle(tempR, tempth);
				carm.xt = carm.xc;
				carm.tht = carm.thc;
				carm.Traj_Xt << tempT(0,3),tempT(1,3),tempT(2,3), tempth;
				carm.Traj_Xc << carm.xc, carm.thc;
			}
			if (SEQ_point == 1) {
				int ret=0;
				ret=tfun.TaskTraj_rspb(&carm);
				//cmd_mode=CMD_MODE_TORQUE;
				//Afun.ArmImpedanceForcedTraj(&carm, &kfun);
				cmd_mode=carm.cmd_mode;
				//Afun.ArmUAIForcedTraj_journal(&carm, &kfun);
				Afun.ArmUAIForcedTraj(&carm, &kfun);
				if(ret>0) SEQ_point = 2;
			}
			if (SEQ_point == 2) {
				pause_cnt++;
				if (pause_cnt > 500) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}

				//cmd_mode=CMD_MODE_TORQUE;
				//Afun.ArmImpedanceForcedTraj(&carm, &kfun);
				cmd_mode=carm.cmd_mode;
				//Afun.ArmUAIForcedTraj_journal(&carm, &kfun);
				Afun.ArmUAIForcedTraj(&carm, &kfun);
				//cmd_mode=100;
			}
		}
		else if (SEQ_switch == 21) { //Force
			if (SEQ_point == 0) {
				carm.FDesired = 10;

				carm.Traj_Vd << 0,0,0,0,0,0;
				//cmd_mode=CMD_MODE_TORQUE;
				//Afun.ArmImpedanceForcedTraj(&carm, &kfun);
				cmd_mode=carm.cmd_mode;
				//Afun.ArmUAIForcedTraj_journal(&carm, &kfun);
				Afun.ArmUAIForcedTraj(&carm, &kfun);
				//cmd_mode=100;
				//carm.temp2<<carm.fd,carm.FErrTrack ;

				double InProduct;
				InProduct = carm.fext(0)*carm.FCtrlDir(0) + carm.fext(1)*carm.FCtrlDir(1) + carm.fext(2)*carm.FCtrlDir(2);
				if(InProduct < -5) SEQ_point = 1;
			}
			if (SEQ_point == 1) {

				if(carm.FDesired < MaxFDesired){
					carm.FDesired = carm.FDesired  + 0.02;
					pause_cnt = 0;

				}
				else pause_cnt++;

				carm.Traj_Vd << 0,0,0,0,0,0;
				//cmd_mode=CMD_MODE_TORQUE;
				//Afun.ArmImpedanceForcedTraj(&carm, &kfun);
				cmd_mode=carm.cmd_mode;
				//Afun.ArmUAIForcedTraj_journal(&carm, &kfun);
				Afun.ArmUAIForcedTraj(&carm, &kfun);

				if (pause_cnt > 5000){
					SEQ_point = 0;
					pause_cnt = 0;
					force_on=1;
					SEQ_switch = 2;
				}
			}
		}
		else if (SEQ_switch == 22) { //Release
			if (SEQ_point == 0) {

				if(carm.FDesired > 0){
					carm.FDesired = carm.FDesired - 0.1;
					pause_cnt = 0;
				}
				else SEQ_point = 1;

				carm.Traj_Vd << 0,0,0,0,0,0;
				//cmd_mode=CMD_MODE_TORQUE;
				//Afun.ArmImpedanceForcedTraj(&carm, &kfun);
				cmd_mode=carm.cmd_mode;
				//Afun.ArmUAIForcedTraj_journal(&carm, &kfun);
				Afun.ArmUAIForcedTraj(&carm, &kfun);
				//cmd_mode=100;
				carm.temp2<<carm.fd,carm.FErrTrack ;

			}
			if (SEQ_point == 1) {
				pause_cnt++;
				if (pause_cnt > 1000){
					SEQ_point = 0;
					pause_cnt = 0;
					force_on=0;
					SEQ_switch = 2;
				}
				carm.Traj_Vd << 0,0,0,0,0,0;
				//cmd_mode=CMD_MODE_TORQUE;
				//Afun.ArmImpedanceForcedTraj(&carm, &kfun);
				cmd_mode=carm.cmd_mode;
				//Afun.ArmUAIForcedTraj_journal(&carm, &kfun);
				Afun.ArmUAIForcedTraj(&carm, &kfun);



			}
		}
fprintf(gtData2, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.fext(0),carm.fext(1),carm.fext(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.xc(0),carm.xc(1),carm.xc(2),carm.FErrTrack(0),carm.FErrTrack(1),carm.FErrTrack(2));
	}



	else if (ctrl == 91) {  ///SJM
		float Switch[5];
		for (int i = 0; i < 5; i++)
			Switch[i] = SwitchS[i];
		if (Switch[0] == 0 && Switch[1] == 0 && Switch[2] == 0 && Switch[3] == 0) {
			pointnum = 0;
			for (int i = 0; i < 5; i++) {
				Switch[i] = 0;
				SwitchS[i] = 0;
				SwitchPre[i] = 0;
			}
			for (int i = 0; i < 20; i++)
				PointTraj[i] = 0;
		}
		if (SEQ_switch == 0) { //initialize
			std::cout << "Position Control Deirect Teaching" << std::endl;
			for (int i = 0; i < 5; i++) {
				Switch[i] = 0;
				SwitchS[i] = 0;
				SwitchPre[i] = 0;
			}
			for (int i = 0; i < 20; i++)
				PointTraj[i] = 0;

			SEQ_switch = 1;
			pointnum = 0;


			sprintf(filename_record, "/home/user/catkin_ws/src/indy7/PPR_T12.txt");
			sprintf(filename_replay, "/home/user/catkin_ws/src/indy7/PPR_T12.txt");
			fp_record = fopen(filename_record, "wt");
			fclose(fp_record);

			cmd_mode=100;
		}
		else if (SEQ_switch == 1) { //while moving, save points and trajectory
			cmd_mode=100;
			carm.qt = carm.qc;
			if (Switch[0] > SwitchPre[0]) {
				SwitchPre[0] = Switch[0];

				fp_record = fopen(filename_record, "at");
				fprintf(fp_record, "%d\n", 1);
				fprintf(fp_record, "%f %f %f %f %f %f\n", carm.qc[0], carm.qc[1], carm.qc[2], carm.qc[3], carm.qc[4], carm.qc[5]);
				fclose(fp_record);
			}
			if (Switch[2] > SwitchPre[2]) {
				SwitchPre[2] = Switch[2];

				fp_record = fopen(filename_record, "at");
				fprintf(fp_record, "%d\n", 2);
				fprintf(fp_record, "%f %f %f %f %f %f\n", carm.qc[0], carm.qc[1], carm.qc[2], carm.qc[3], carm.qc[4], carm.qc[5]);
				fclose(fp_record);
			}
			if (Switch[1] > 0){
				SwitchS[1] = 0;
				if(fp_replay = fopen(filename_replay, "rt"))
					SEQ_switch = 2;
			}
		}
		else if (SEQ_switch == 2) { //arrange the play sequence
			pause_cnt++;
			if (pause_cnt > 100) {
				pause_cnt = 0;
				int SEQ_record;
				int reti = fscanf(fp_replay, "%d\n", &SEQ_record);
				if (reti != EOF) {
					if (SEQ_record == 1) {
						SEQ_switch = 3;
					}
					else if (SEQ_record == 2) {
						SEQ_switch = 4;
					}
				}
				else {
					fclose(fp_replay);
					SEQ_play = 0;
					SEQ_switch = 1;
				}
			}
			stop_flag = 0;
			cmd_mode=100;
		}
		else if (SEQ_switch == 3) { //Path to point planning
			if (SEQ_point == 0) {
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;
				carm.temp1=carm.qt;
			}
			if (SEQ_point == 1) {
				/*cmd_mode=CMD_MODE_POSITION;
				int ret=0;
				ret=tfun.Traj_rspb(&carm);
				if(ret>0) SEQ_point = 2;*/

				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				//ret=tfun.Traj_rspb_rt(&carm, 0.5,0.5);
				//ret=tfun.Traj_rspb(&carm);
				ret=tfun.Traj_rspb_velocity(&carm);
				//Afun.ArmAdmittanceJointTraj(&carm, &kfun);
				if(ret>0) SEQ_point = 2;

			}
			if (SEQ_point == 2) {
				pause_cnt++;
				if (pause_cnt > 500) {
					SEQ_point = 0;
					pause_cnt = 0;
					carm.Traj_seq[0]=0;
					SEQ_switch = 2;
				}
				//cmd_mode=CMD_MODE_POSITION;
				cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceJointTraj(&carm, &kfun);
			}
			if (fabs(carm.FTSensor(0)) > 5 || fabs(carm.FTSensor(1)) > 5 || fabs(carm.FTSensor(2)) > 5) {
				SEQ_point = 0;
				pause_cnt = 0;
				SEQ_switch = 9;
				carm.Traj_seq[0]=0;
				carm.dqc0=carm.dqc;
				cmd_mode=CMD_MODE_VELOCITY;
			}
		}
		else if (SEQ_switch == 4) { //Path to point planning
			if (SEQ_point == 0) {
				float d1, d2, d3, d4, d5, d6;
				int reti = fscanf(fp_replay, "%f %f %f %f %f %f\n", &d1, &d2, &d3, &d4, &d5, &d6);
				carm.qt << d1, d2, d3, d4, d5, d6;
				SEQ_point = 1;
				carm.dxd<<0,0,0;
    				carm.dthd<<0,0,0;

				MatrixXd tempT(4,4);
				kfun.iForwardK_T(carm.qt, tempT,0);
				Vector3d tempth;
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						carm.Rstd(i,j)=tempT(i,j);
					}
				}
				kfun.iRotation2EulerAngle(carm.Rstd, tempth);
				carm.Rstd_i=carm.Rstd.inverse();

				carm.xt << tempT(0,3),tempT(1,3),tempT(2,3);
				carm.tht = tempth;
				carm.Traj_Xt << carm.xt, carm.tht;
				carm.Traj_Xc << carm.xc, carm.thc;
			}
			if (SEQ_point == 1) {
				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				ret=tfun.TaskTraj_rspb(&carm);
				Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				if(ret>0) SEQ_point = 2;
			}
			if (SEQ_point == 2) {
				//std::cout <<"@@"<< carm.qc.transpose() << std::endl;
				pause_cnt++;
				if (pause_cnt > 500) {
					SEQ_point = 0;
					pause_cnt = 0;
					SEQ_switch = 2;
				}
				cmd_mode=CMD_MODE_VELOCITY;
				Afun.ArmAdmittanceTaskTraj(&carm, &kfun);
				//cmd_mode=100;
			}
		}
		else if (SEQ_switch == 5) { //Fogale
			if (SEQ_point == 0) {
				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				int ret2=0;
				kfun.Jacobian(&carm);
 				carm.J_t = carm.J.transpose();
				kfun.ForwardK_T123456(&carm);
				ret = tfun.fogale_direction_calculation(&carm);
				kfun.Jacobian123456(&carm);
				carm.J6_t=carm.J6.transpose();
				ret2 = tfun.fogale_velocity_calculation(&carm);
				carm.Traj_seq[0]=0;
 				//carm.Q_t = carm.Q.transpose();
				//carm.QQt = carm.Q*carm.Q_t;
				carm.Q_i = carm.Q.inverse();
				carm.dqd=-carm.Q_i*carm.r;
				if(fabs(carm.FTSensor(0)) < 2 && fabs(carm.FTSensor(1)) < 2) SEQ_point = 1;
			}
			if (SEQ_point == 1) {
				SEQ_point = 0;
				pause_cnt = 0;
				SEQ_switch = 7;
				cmd_mode=CMD_MODE_VELOCITY;
			}
		}
		else if (SEQ_switch == 6) { //Avoidance
			if (SEQ_point == 0) {
				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				int ret2=0;
				carm.Traj_seq[0]=0;
				kfun.Jacobian(&carm);
 				carm.J_t = carm.J.transpose();
 				carm.Jp_t = carm.Jp.transpose();

				MatrixXd JJt;
				JJt = carm.J*carm.J_t;
				carm.J_i = carm.J_t*JJt.inverse();

				MatrixXd tempT(4,4);
				kfun.iForwardK_T(carm.qt, tempT,0);
				Vector3d tempth;
				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						carm.Rstd(i,j)=tempT(i,j);
					}
				}
				kfun.iRotation2EulerAngle(carm.Rstd, tempth);
				carm.Rstd_i=carm.Rstd.transpose();

				carm.xt << tempT(0,3),tempT(1,3),tempT(2,3);
				carm.tht = tempth;
				carm.Traj_Xt << carm.xt, carm.tht;
				carm.Traj_Xc << carm.xc, carm.thc;
				carm.Traj_Vc << carm.J*carm.dqc;
				ret2 = tfun.Change_xt(&carm,0.1,0.1);
				ret = tfun.TaskTraj_rspb_velocity(&carm);
				carm.dqd = carm.J_i*carm.Traj_Vd;
				double dqd_max = 0.5;
				if(carm.dett<0.02){
					for(int i=0;i<6;i++){
						if(carm.dqd(i)>dqd_max)
							carm.dqd(i)=dqd_max;
					}
					//carm.dqd=carm.dqd*0.1/dqd_max;
				}

				if(fabs(carm.FTSensor(0)) < 2 && fabs(carm.FTSensor(1)) < 2 && fabs(carm.FTSensor(2)) < 2) SEQ_point = 1;
			}
			if (SEQ_point == 1) {
				SEQ_point = 0;
				pause_cnt = 0;
				SEQ_switch = 7;
				carm.Traj_seq[0]=0;
				cmd_mode=CMD_MODE_VELOCITY;
			}
		}
		else if (SEQ_switch == 8) { //Link3 Avoidance Test
			if (SEQ_point == 0) {
				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				int ret2=0;
				carm.Traj_seq[0]=0;
				kfun.ForwardK_link3(&carm);
 				carm.J3_t = carm.J3.transpose();

				Matrix3d JJt;
				JJt = carm.J3*carm.J3_t;
				carm.J3_i = carm.J3_t*JJt.inverse();

				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						carm.Rc3(i,j)=carm.Tc3(i,j);
					}
				}
				carm.Rc3_i = carm.Rc3.transpose();

				carm.Traj_Xt3 << carm.xt3;
				carm.Traj_Xc3 << carm.xc3;
				carm.Traj_Vc3 << carm.J3*carm.dqc3;
				ret2 = tfun.Change_xt_link3(&carm,0.1,0.1);
				ret = tfun.TaskTraj_rspb_velocity_link3(&carm);
				carm.dqd3 = carm.J3_i*carm.Traj_Vd3;
				carm.dqd << carm.dqd3[0],carm.dqd3[1],carm.dqd3[2],carm.dqc0[3],carm.dqc0[4],carm.dqc0[5];
				if(fabs(carm.FTSensor(0)) < 2 && fabs(carm.FTSensor(1)) < 2 && fabs(carm.FTSensor(2)) < 2) SEQ_point = 1;
			}
			if (SEQ_point == 1) {
				SEQ_point = 0;
				pause_cnt = 0;
				SEQ_switch = 7;
				carm.Traj_seq[0]=0;
				cmd_mode=CMD_MODE_VELOCITY;
			}
		}
		else if (SEQ_switch == 9) { //Link2 Avoidance Test
			if (SEQ_point == 0) {
				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				int ret2=0;
				carm.Traj_seq[0]=0;
				kfun.ForwardK_link2(&carm);

				for(int i=0;i<3;i++){
					for(int j=0;j<3;j++){
						carm.Rc2(i,j)=carm.Tc2(i,j);
					}
				}
				carm.Rc2_i = carm.Rc2.transpose();

				carm.Traj_Xt2 << carm.xt2;
				carm.Traj_Xc2 << carm.xc2;
				carm.Traj_Vc2 << carm.J2*carm.dqc2;
				ret2 = tfun.Change_xt_link2(&carm,0.1,0.1);
				ret = tfun.TaskTraj_rspb_velocity_link2(&carm);
				carm.dqd2 = carm.J2_i*carm.Traj_Vd2;
				carm.dqd << carm.dqd2[0],carm.dqd2[1],carm.dqc0[2],carm.dqc0[3],carm.dqc0[4],carm.dqc0[5];
				if(fabs(carm.FTSensor(0)) < 2 && fabs(carm.FTSensor(1)) < 2 && fabs(carm.FTSensor(2)) < 2) SEQ_point = 1;
			}
			if (SEQ_point == 1) {
				SEQ_point = 0;
				pause_cnt = 0;
				SEQ_switch = 7;
				carm.Traj_seq[0]=0;
				cmd_mode=CMD_MODE_VELOCITY;
			}
		}
		else if (SEQ_switch == 7) { //Path to point planning
			if (SEQ_point == 0) {
				cmd_mode=CMD_MODE_VELOCITY;
				int ret=0;
				ret=tfun.Traj_rspb_velocity(&carm);
				//ret=tfun.Traj_rspb_rt(&carm, 0.5,0.5);
				//Afun.ArmAdmittanceJointTraj(&carm, &kfun);
				if(ret>0) SEQ_point = 1;

			}
			if (SEQ_point == 1) {
				pause_cnt++;
				if (pause_cnt > 500) {
					SEQ_point = 0;
					pause_cnt = 0;
					carm.Traj_seq[0]=0;
					SEQ_switch = 2;
				}
				cmd_mode=CMD_MODE_VELOCITY;
				//Afun.ArmAdmittanceJointTraj(&carm, &kfun);
			}
			if (fabs(carm.FTSensor(0)) > 5 || fabs(carm.FTSensor(1)) > 5 || fabs(carm.FTSensor(2)) > 5) {
				SEQ_point = 0;
				pause_cnt = 0;
				SEQ_switch = 9;
				carm.Traj_seq[0]=0;
				carm.dqc0=carm.dqc;
				cmd_mode=CMD_MODE_VELOCITY;
			}
		}


		fprintf(sfData,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",carm.qc(0),carm.qc(1),carm.qc(2),carm.qc(3),carm.qc(4),carm.qc(5),carm.qd(0),carm.qd(1),carm.qd(2),carm.qd(3),carm.qd(4),carm.qd(5),carm.dqd(0),carm.dqd(1),carm.dqd(2),carm.dqd(3),carm.dqd(4),carm.dqd(5),carm.dqc(0),carm.dqc(1),carm.dqc(2),carm.dqc(3),carm.dqc(4),carm.dqc(5),carm.dqc0(0),carm.dqc0(1),carm.dqc0(2),carm.dqc0(3),carm.dqc0(4),carm.dqc0(5),carm.dist(5,0),carm.dist(5,1),carm.dist(5,2),carm.dist(5,3),carm.xc(0),carm.xc(1),carm.xc(2),carm.Traj_xt(0),carm.Traj_xt(1),carm.Traj_xt(2),carm.Traj_Vc(0),carm.Traj_Vc(1),carm.Traj_Vc(2),carm.dett);
		fprintf(link3Data,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",carm.dqd3(0),carm.dqd3(1),carm.dqd3(2),carm.Traj_Xt3(0),carm.Traj_Xt3(1),carm.Traj_Xt3(2),carm.Traj_Xc3(0),carm.Traj_Xc3(1),carm.Traj_Xc3(2),carm.Traj_Vc3(0),carm.Traj_Vc3(1),carm.Traj_Vc3(2),carm.Traj_Vd3(0),carm.Traj_Vd3(1),carm.Traj_Vd3(2),carm.Traj_xt(0),carm.Traj_xt(1),carm.Traj_xt(2),carm.Traj_xc(0),carm.Traj_xc(1),carm.Traj_xc(2),carm.Traj_vc(0),carm.Traj_vc(1),carm.Traj_vc(2));
		//fprintf(sfData,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",carm.qc(0),carm.qc(1),carm.qc(2),carm.qc(3),carm.qc(4),carm.qc(5),carm.qd(0),carm.qd(1),carm.qd(2),carm.qd(3),carm.qd(4),carm.qd(5),carm.dqd(0),carm.dqd(1),carm.dqd(2),carm.dqd(3),carm.dqd(4),carm.dqd(5),carm.dqc(0),carm.dqc(1),carm.dqc(2),carm.dqc(3),carm.dqc(4),carm.dqc(5),carm.dqc0(0),carm.dqc0(1),carm.dqc0(2),carm.dqc0(3),carm.dqc0(4),carm.dqc0(5),carm.dist(5,0),carm.dist(5,1),carm.dist(5,2),carm.dist(5,3));
		//fprintf(sfData,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",carm.Q(0,0),carm.Q(0,1),carm.Q(0,2),carm.Q(0,3),carm.Q(0,4),carm.Q(0,5),carm.Q(1,0),carm.Q(1,1),carm.Q(1,2),carm.Q(1,3),carm.Q(1,4),carm.Q(1,5),carm.Q(2,0),carm.Q(2,1),carm.Q(2,2),carm.Q(2,3),carm.Q(2,4),carm.Q(2,5),carm.Q(3,0),carm.Q(3,1),carm.Q(3,2),carm.Q(3,3),carm.Q(3,4),carm.Q(3,5),carm.Q(4,0),carm.Q(4,1),carm.Q(4,2),carm.Q(4,3),carm.Q(4,4),carm.Q(4,5),carm.Q(5,0),carm.Q(5,1),carm.Q(5,2),carm.Q(5,3),carm.Q(5,4),carm.Q(5,5));
		//fprintf(sfData,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",carm.Q_sum(0,0),carm.Q_sum(0,1),carm.Q_sum(0,2),carm.Q_sum(0,3),carm.Q_sum(0,4),carm.Q_sum(0,5),carm.Q_sum(1,0),carm.Q_sum(1,1),carm.Q_sum(1,2),carm.Q_sum(1,3),carm.Q_sum(1,4),carm.Q_sum(1,5),carm.Q_sum(2,0),carm.Q_sum(2,1),carm.Q_sum(2,2),carm.Q_sum(2,3),carm.Q_sum(2,4),carm.Q_sum(2,5),carm.Q_sum(3,0),carm.Q_sum(3,1),carm.Q_sum(3,2),carm.Q_sum(3,3),carm.Q_sum(3,4),carm.Q_sum(3,5),carm.Q_sum(4,0),carm.Q_sum(4,1),carm.Q_sum(4,2),carm.Q_sum(4,3),carm.Q_sum(4,4),carm.Q_sum(4,5),carm.Q_sum(5,0),carm.Q_sum(5,1),carm.Q_sum(5,2),carm.Q_sum(5,3),carm.Q_sum(5,4),carm.Q_sum(5,5));

	}


	stop_flag=0;

	if(cmd_mode==CMD_MODE_POSITION&&stop_flag==0){

		double errsum=0;
		double maxerr=0;
		for(int i=0;i<6;i++){
			double err=carm.qd(i)-carm.qc(i);
			if(maxerr<fabs(err)){
				maxerr=fabs(err);
			}
		}

		double maxerrmin;
		maxerrmin=0.5;

		if(maxerr>maxerrmin){ 
			carm.Tau<<0,0,0,0,0,0;
			cmd_mode=CMD_MODE_TORQUE;
		}
	}

///////////Command Mode //////////////////////////////////////////
//--------Torque=1, Position=2;, Velocity=3, No Control=4 -------------------------//
	if(cmd_mode==CMD_MODE_TORQUE){ //Input: carm.Tau  without compensations

		carm.Tau = carm.Tau + carm.Tor_grav;
		kfun.Friction_comp_Stribeck(&carm);        
		//kfun.Friction_comp(&carm);
		//kfun.Friction_comp_Lugre(&carm);
		carm.Tau = carm.Tau + carm.Tor_fric;
	}
	else if(cmd_mode==CMD_MODE_POSITION){//Input: carm.qd
		VectorXd Kp(6), Kd(6);
		MatrixXd I6, KpI, KdI;
		I6=MatrixXd::Identity(6,6);
		Kp << 10000, 10000, 10000, 10000, 10000, 10000;
		Kd << 400, 1200, 600, 300, 300, 300;
		KpI = I6;
		KdI = I6;
		for(int i=0;i<6;i++) 	KpI(i,i) = Kp(i);
		for(int i=0;i<6;i++) 	KdI(i,i) = Kd(i);

		carm.Tau =  KpI*(carm.qd - carm.qc) - KdI*carm.dqc + carm.Tor_grav;
		kfun.Friction_comp(&carm);
		carm.Tau = carm.Tau + carm.Tor_fric;
	}
	else if(cmd_mode==CMD_MODE_VELOCITY){ //Input: carm.dqd

		VectorXd Kp(6), Ki(6);
		MatrixXd I6, KpI, KiI;
		I6=MatrixXd::Identity(6,6);
		Kp << 1000, 1300, 400, 300, 250, 400;
		KpI = I6;
		for(int i=0;i<6;i++) 	KpI(i,i) = Kp(i);

		//Ki << 1,1,1,1,1,1;
		Ki << 0,0,0,0,0,0;
		KiI = I6;
		for(int i=0;i<6;i++) 	KiI(i,i) = Ki(i);  

		carm.Err_sum = carm.Err_sum + carm.dqd - carm.dqc;
		carm.Tau =  KpI*(carm.dqd - carm.dqc) + KiI*carm.Err_sum + carm.Tor_grav;
		kfun.Friction_comp(&carm);
		//kfun.Friction_comp_Lugre(&carm);
		carm.Tau = carm.Tau + carm.Tor_fric;
            
	}
	else if(cmd_mode==CMD_MODE_NO_CNTL){//Input: carm.Tau with compensations

	}
	else{
		carm.Tau<<0,0,0,0,0,0;
		carm.Tau = carm.Tau + carm.Tor_grav;

	}
	/////////// Joint limitation and Torque Command/////////////////////////////////
	for (int i=0; i<6; ++i){
		if(carm.dqc(i)> 2.0){		
			std::cout<<i<<"Vel Limit!!"<<std::endl;
			ctrl=0;
		}
		else if(carm.dqc(i)< -2.0 ){	
			std::cout<<i<<"Vel Limit!!"<<std::endl;
			ctrl=0;
		}
	}

	for (int i=1; i<NUM_AXIS-1; ++i){
		if (i>0&&i<7 ) tempTor[i] = carm.Tau[i-1];

		if(qAbsRad[i]*180/PI > 160 || qAbsRad[i]*180/PI< -160)
		{		
			std::cout<<"Range Limit!!"<<std::endl;
			ctrl=0;
		}

		TargetTor[i] = qDirection[i]*tempTor[i]*currentRatio[i]/gearRatio[i]/torqueRatio[i];
		
		if(TargetTor[i]>1000 ){		
			TargetTor[i] = 1000;
			std::cout<<i<<"Jonit Limit!!"<<std::endl;
			//ctrl=0;
		}
		else if(TargetTor[i] < -1000 ){
			TargetTor[i] = -1000;
			std::cout<<i<<"Limit!!"<<std::endl;
			//ctrl=0;
		}
	}
//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.temp2(0),carm.temp2(1),carm.temp2(2),carm.temp2(3),carm.temp2(4),carm.temp2(5),carm.temp3(0),carm.temp3(1),carm.temp3(2),carm.temp3(3),carm.temp3(4),carm.temp3(5),carm.Tau(0),carm.Tau(1),carm.Tau(2),carm.Tau(3),carm.Tau(4),carm.Tau(5),carm.Tor_A(0),carm.Tor_A(1),carm.Tor_A(2),carm.Tor_A(3),carm.Tor_A(4),carm.Tor_A(5),carm.temp1(0));
//fprintf(gtData, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",carm.temp1(0),carm.temp4(0),carm.temp4(1),carm.temp4(2),carm.temp5(0),carm.temp5(1),carm.temp5(2),carm.temp2(0),carm.temp2(1),carm.temp2(2),carm.temp2(3),carm.temp2(4),carm.temp2(5),carm.temp3(0),carm.temp3(1),carm.temp3(2),carm.temp3(3),carm.temp3(4),carm.temp3(5));

//fprintf(gtData, "%d, %d,%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",SEQ_switch, SEQ_point,cmd_mode,carm.tht(0),carm.tht(1),carm.tht(2),carm.xt(0),carm.xt(1),carm.xt(2),carm.thc(0),carm.thc(1),carm.thc(2),carm.xc(0),carm.xc(1),carm.xc(2));
//fprintf(gtData, "%d, %d,%d, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n",SEQ_switch, SEQ_point,cmd_mode,carm.dqc[0],carm.dqc[1],carm.dqc[2],carm.dqc[3],carm.dqc[4],carm.dqc[5],carm.Traj_dQ[0],carm.Traj_dQ[1],carm.Traj_dQ[2],carm.Traj_dQ[3],carm.Traj_dQ[4],carm.Traj_dQ[5]);

//fprintf(gtData, "%d, %d,%d,%10i,%10i,%10i,%10i,%10i,%10i,%10i,%10i,%10i,%10i,%10i,%10i\n",SEQ_switch, SEQ_point,cmd_mode,TargetTor[1],TargetTor[2],TargetTor[3],TargetTor[4],TargetTor[5],TargetTor[6],ActualTor[1],ActualTor[2],ActualTor[3],ActualTor[4],ActualTor[5],ActualTor[6]);
//fprintf(gtData, "%d, %d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",SEQ_switch, SEQ_point,cmd_mode,carm.qd[0],carm.qd[1],carm.qd[2],carm.qd[3],carm.qd[4],carm.qd[5],carm.qc[0],carm.qc[1],carm.qc[2],carm.qc[3],carm.qc[4],carm.qc[5]);
	return 0;
}

// Neuromeka_NRMK_IO_Module_task	
void Neuromeka_NRMK_IO_Module_run(void *arg)
{
	unsigned int runcount=0;
	RTIME now, previous;
	
	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.syncEcatMaster();
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */
	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);



	ros::NodeHandle nh;  
	ros::Publisher pub1 = nh.advertise<ftsensor::ftsensorMsg>("SlaveForce", 1);
	///ros::Subscriber sub_forcesensor = nh.subscribe("FTSensor2", 100, msgCallbackdataftsensor);
	//ros::Subscriber sub_mastercmd = nh.subscribe("MasterCmd", 1, msgCallbackdatamaster);

	filter_pre<< 0,0,0,0,0,0;
	tfilter_pre<< 0,0,0,0,0,0;
	dqcfilter_pre<< 0,0,0,0,0,0;
	
	while (run)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle

		runcount++;

		if (!run)
		{
			break;
		}

		previous = rt_timer_read();

		/// TO DO: read data from sensors in EtherCAT system interface
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.processTxDomain();
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60410, StatusWord);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60640, ActualPos);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x606c0, ActualVel);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60770, ActualTor);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60610, ModeOfOperationDisplay);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61001, StatusCode);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61002, DI5V);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61003, DI1);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61004, DI2);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61005, AI1);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61006, AI2);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61007, FTRawFx);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61008, FTRawFy);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x61009, FTRawFz);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610010, FTRawTx);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610011, FTRawTy);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610012, FTRawTz);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610013, FTOverloadStatus);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610014, FTErrorFlag);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610015, RS485RxCnt);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610016, RS485RxD0);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610017, RS485RxD1);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610018, RS485RxD2);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610019, RS485RxD3);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610020, RS485RxD4);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610021, RS485RxD5);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610022, RS485RxD6);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610023, RS485RxD7);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610024, RS485RxD8);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x610025, RS485RxD9);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60001, IStatus);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60002, IButton);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60003, FTRawFx);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60004, FTRawFy);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60005, FTRawFz);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60006, FTRawTx);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60007, FTRawTy);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60008, FTRawTz);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x60009, FTOverloadStatus);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.readBuffer(0x600010, FTErrorFlag);		
		
		/// TO DO: Main computation routine...
		compute();

		ftsensor::ftsensorMsg msg;
		msg.Fx=carm.fext(0); 
		msg.Fy=carm.fext(1);
		msg.Fz=carm.fext(2);
		msg.Mx=carm.text(0);
		msg.My=carm.text(1);
		msg.Mz=carm.text(2);
		pub1.publish(msg);

		/// TO DO: write data to actuators in EtherCAT system interface
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.writeBuffer(0x607a0, TargetPos);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.writeBuffer(0x60ff0, TargetVel);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.writeBuffer(0x60710, TargetTor);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.writeBuffer(0x60600, ModeOfOperation);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.writeBuffer(0x71003, TO);
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.writeBuffer(0x71004, DO);
		
		_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.processRxDomain();
			
		ros::spinOnce();

		if (system_ready)
			saveLogData();
			
		// For EtherCAT performance statistics
		now = rt_timer_read();
		ethercat_time = (long) now - previous;

		if (_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.isSystemReady() && (runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)) )
		{
			system_ready=1;	//all drives have been done

			gt+= period;
			
			if (worst_time<ethercat_time) worst_time=ethercat_time;
			if(ethercat_time > max_time)
				++fault_count;
		}
	}
}

// Console cycle
// Note: You have to use rt_printf in Xenomai RT tasks
void print_run(void *arg)
{
	RTIME now, previous=0;
	int i;
	unsigned long itime=0, step;
	long stick=0;
	int count=0;
	unsigned int NumSlaves=0, masterState=0, slaveState=0;
	
	rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);
	
	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100ms = 0.1s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e8);
	while (1)
	{
		if (++count==10)
		{
			++stick;
			count=0;
		}
		if (system_ready)
		{
			now = rt_timer_read();
			step=(unsigned long)(now - previous) / 1000000;
			itime+=step;
			previous=now;
			rt_printf("Time=%d.%d s, ", itime/1000, itime % 1000);
			rt_printf("dt= %li, worst= %li\n", ethercat_time, worst_time);
			
			if (_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.getMasterStatus(NumSlaves, masterState))
				rt_printf("Master: Online - State %i - %i slave(s)\n", masterState, NumSlaves);
			else
				rt_printf("Master: Offline\n");

			if (_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.getRxDomainStatus())
				rt_printf("RxDomain: Online\n");
			else
				rt_printf("RxDomain: Offline\n");

			if (_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.getTxDomainStatus())
				rt_printf("TxDomain: Online\n");
			else
				rt_printf("TxDomain: Offline\n");
				

			if (_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.getAxisEcatStatus(7, slaveState))
				rt_printf("\e[32;1mSlave: Online %i,  \e[0m\n", slaveState);
			else
				rt_printf("\e[32;1mSlave: Offline,  \e[0m\n");

			rt_printf("\e[32;1m ActualPos: %10i,%10i,%10i,%10i,%10i,%10i, \e[0m\n",ctrl,ActualPos[2],ActualPos[3],ActualPos[4],ActualPos[5],ActualPos[6]);
			rt_printf("\e[32;1m DesActRad : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.qd[0]-carm.qc[0],carm.qd[1]-carm.qc[1],carm.qd[2]-carm.qc[2],carm.qd[3]-carm.qc[3],carm.qd[4]-carm.qc[4],carm.qd[5]-carm.qc[5]);
			rt_printf("\e[32;1m DesireRad : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.qd[0],carm.qd[1],carm.qd[2],carm.qd[3],carm.qd[4],carm.qd[5]);
			rt_printf("\e[32;1m ActualRad: %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.qc[0],carm.qc[1],carm.qc[2],carm.qc[3],carm.qc[4],carm.qc[5]);;
			rt_printf("\e[32;1m DesireVel : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.dqd[0],carm.dqd[1],carm.dqd[2],carm.dqd[3],carm.dqd[4],carm.dqd[5]);
			rt_printf("\e[32;1m ActualVel: %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.dqc[0],carm.dqc[1],carm.dqc[2],carm.dqc[3],carm.dqc[4],carm.dqc[5]);
			rt_printf("\e[32;1m ActualDeg: %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.qc[0]*180/PI,carm.qc[1]*180/PI,carm.qc[2]*180/PI,carm.qc[3]*180/PI,carm.qc[4]*180/PI,carm.qc[5]*180/PI);
			rt_printf("\e[32;1m DesireTor: %10i,%10i,%10i,%10i,%10i,%10i, \e[0m\n",TargetTor[1],TargetTor[2],TargetTor[3],TargetTor[4],TargetTor[5],TargetTor[6]);
			rt_printf("\e[32;1m ActualTor: %10i,%10i,%10i,%10i,%10i,%10i, \e[0m\n",ActualTor[1],ActualTor[2],ActualTor[3],ActualTor[4],ActualTor[5],ActualTor[6]);
			rt_printf("\e[32;1m FTSensor : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.FTSensor[0],carm.FTSensor[1],carm.FTSensor[2],carm.FTSensor[3],carm.FTSensor[4],carm.FTSensor[5]);
			rt_printf("\e[32;1m BaseForce: %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.fext[0],carm.fext[1],carm.fext[2],carm.fext[0],carm.fext[1],carm.fext[2]);
			rt_printf("\e[32;1m TaskPosA : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.xc[0],carm.xc[1],carm.xc[2],carm.thc[0],carm.thc[1],carm.thc[2]);
			rt_printf("\e[32;1m TaskPosT : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.xt[0],carm.xt[1],carm.xt[2],carm.tht[0],carm.tht[1],carm.tht[2]);
			//rt_printf("\e[32;1m DesireVel : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.dqd[0],carm.dqd[1],carm.dqd[2],carm.dqd[3],carm.dqd[4],carm.dqd[5]);
			rt_printf("\e[32;1m Prox : %10f,%10f,%10f,%10f \e[0m\n",carm.ProxSensor[0],carm.ProxSensor[1],carm.ProxSensor[2],carm.ProxSensor[3]);
			rt_printf("\e[32;1m Temp1 : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.temp1[0],carm.temp1[1],carm.temp1[2],carm.temp1[3],carm.temp1[4],carm.temp1[5]);
			rt_printf("\e[32;1m Temp2 : %10f,%10f,%10f,%10f,%10f,%10f, \e[0m\n",carm.temp2[0],carm.temp2[1],carm.temp2[2],carm.temp2[3],carm.temp2[4],carm.temp2[5]);
			rt_printf("\n");
		}
		else
		{
			if (count==0){
				rt_printf("%i", stick);
				for(i=0; i<stick; ++i)
					rt_printf(".");
				rt_printf("\n");
			}
		}

		rt_task_wait_period(NULL); //wait for next cycle
	}
}

// GUI Command process cycle
void gui_run(void *arg)
{
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 1 s)
	 */
	rt_task_set_periodic(NULL, TM_NOW, 1e7);	// period = 1 (msec)

	INT8 modeOpDisp[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	UINT16 status[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	INT32 actPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	INT32 actVel[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	INT16 actTor[NUM_AXIS] = {0,0,0,0,0,0,0,0};

	INT8 modeOp[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	float tarval[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	float maxvel[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	float maxacc[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	float maxjerk[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	
	while (1)
	{		
		if (guicontrolsocket.hasConnection())
		{
			for (int i=0; i<NUM_AXIS; i++)
			{
				modeOpDisp[i] = ModeOfOperationDisplay[i];
				status[i] = StatusWord[i];
				actVel[i] = ActualVel[i];
				actTor[i] = ActualTor[i];
				actPos[i] = ActualPos[i];			
			}

			guicontrolsocket.sendMotionData(modeOpDisp, status, actPos, actVel, actTor);

			if (guicontrolsocket.getMotionData(modeOp, tarval, maxvel, maxacc, maxjerk) != 0)
			{
				for (int index=0; index<NUM_AXIS; index++)
				{
					ModeOfOperation[index] = modeOp[index];
					
					switch (modeOp[index])
					{
					case OP_MODE_CYCLIC_SYNC_POSITION:
						Axis[index].setTrajBoundaryCond((double) maxvel[index], (double) maxacc[index]);
						Axis[index].setTarPosInCnt(tarval[index]);
						break;

					case OP_MODE_CYCLIC_SYNC_VELOCITY:
						Axis[index].resetTraj();
						Axis[index].setTarVelInCnt((INT32) tarval[index]);
						break;

					case OP_MODE_CYCLIC_SYNC_TORQUE:
						Axis[index].resetTraj();
						Axis[index].setTarTorInCnt((INT16) tarval[index]);
						break;
						
					default:
						Axis[index].setDataOut((INT32) tarval[index]);
						break;
					}
				}
			}
		}

		rt_task_wait_period(NULL);
	}
}

void plot_run(void *arg)
{
	/*
	 * Arguments: &task (NULL=self),
	 *            start time,
	 *            period (here: 100 ms)
	 */
	//rt_task_set_periodic(NULL, TM_NOW, 1e8);	// period = 100 (msec)

	gtData = fopen("/home/user/catkin_ws/gtdata1.txt","wt");

	while (1)
	{
		/// TO DO: You have to prepare data for NRMKDataSocket
		if (datasocket.hasConnection() && system_ready)
		{
			if (frontIdx < rearIdx)
			{
				datasocket.updateControlData(_loggingBuff[frontIdx].ActualPos, _loggingBuff[frontIdx].ActualVel);
				datasocket.update(_loggingBuff[frontIdx].Time);

				frontIdx++;
			}
			else if (rearIdx == MAX_BUFF_SIZE)
			{
				frontIdx = rearIdx = 0;
			}
		}
		else
		{
			frontIdx = rearIdx = 0;
		}
		usleep(1000);
		//rt_task_wait_period(NULL);
	}
}

/****************************************************************************/
void signal_handler(int signum = 0)
{
	rt_task_delete(&plot_task);
	rt_task_delete(&gui_task);
	rt_task_delete(&Neuromeka_NRMK_IO_Module_task);
	rt_task_delete(&print_task);
    printf("Servo drives Stopped!\n");

    _systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.deinit();
    exit(1);
}

/****************************************************************************/
int main(int argc, char **argv)
{
///////////////////////////////////////////////////////////////////

	Prox_Bias<<0,0,0,0;
	//execl("/home/user/release/Indy7ToolSet","./tpgpio_EMG_POWERCTRL",(char *)NULL);
	ros::init(argc, argv, "Indy7_arm"); 
	ros::NodeHandle nh; 
	ros::Subscriber sub_proxsensor = nh.subscribe("Proxsensor", 1, msgCallbackprox);
	ros::Subscriber sub_switch = nh.subscribe("FTSwitch", 100, msgCallbackSwitch);
	ros::Subscriber sub_forcesensor = nh.subscribe("FTSensor2", 100, msgCallbackdataftsensor);
	ros::Subscriber sub_mastercmd = nh.subscribe("MasterCmd", 1, msgCallbackdatamaster);
	gtData = fopen("/home/user/catkin_ws/gtdata1.txt","wt");
	gtData2 = fopen("/home/user/catkin_ws/gtdata2.txt","wt");
	sfData = fopen("/home/user/catkin_ws/sfdata.txt","wt");
	link3Data = fopen("/home/user/catkin_ws/link3data.txt","wt");

	carm.n_link1 << 1, 0, 0, 0, 1, 0, -1, 0, 0, 0, -1, 0;
	//carm.n_link1 << 1/sqrt(2),1/sqrt(2), 0, -1/sqrt(2),1/sqrt(2), 0, -1/sqrt(2),-1/sqrt(2), 0, 1/sqrt(2),-1/sqrt(2), 0;
	carm.n_link2 << 0, 1/sqrt(2), 1/sqrt(2), 0, -1/sqrt(2), 1/sqrt(2), 0, -1/sqrt(2), -1/sqrt(2), 0, 1/sqrt(2), -1/sqrt(2);
	carm.n_link3 << 1/sqrt(2), 0, 1/sqrt(2), -1/sqrt(2), 0, 1/sqrt(2), -1/sqrt(2), 0, -1/sqrt(2), 1/sqrt(2), 0, -1/sqrt(2);
	carm.n_link4 << 1/sqrt(2), 0, 1/sqrt(2), -1/sqrt(2), 0, 1/sqrt(2), -1/sqrt(2), 0, -1/sqrt(2), 1/sqrt(2), 0, -1/sqrt(2);
	carm.n_link5 << 1/sqrt(2), 0, 1/sqrt(2), -1/sqrt(2), 0, 1/sqrt(2), -1/sqrt(2), 0, -1/sqrt(2), 1/sqrt(2), 0, -1/sqrt(2);
	carm.n_link6 << 1/sqrt(2), 1/sqrt(2), 0, -1/sqrt(2), 1/sqrt(2), 0, -1/sqrt(2), -1/sqrt(2), 0, 1/sqrt(2), -1/sqrt(2), 0;
	carm.n_link1_t=carm.n_link1.transpose();
	carm.n_link2_t=carm.n_link2.transpose();
	carm.n_link3_t=carm.n_link3.transpose();
	carm.n_link4_t=carm.n_link4.transpose();
	carm.n_link5_t=carm.n_link5.transpose();
	carm.n_link6_t=carm.n_link6.transpose();

	carm.p_link1 << 0.1/sqrt(2), 0.1/sqrt(2), -0.2, 1, -0.1/sqrt(2), 0.1/sqrt(2), -0.2, 1, -0.1/sqrt(2), -0.1/sqrt(2), -0.2, 1, 0.1/sqrt(2), -0.1/sqrt(2), -0.2, 1;
	carm.p_link2 << 0.2, 0.08/sqrt(2), 0.08/sqrt(2)+0.190, 1, 0.2, -0.08/sqrt(2), 0.08/sqrt(2)+0.190, 1, 0.2, -0.08/sqrt(2), -0.08/sqrt(2)+0.190, 1, 0.2, 0.08/sqrt(2), -0.08/sqrt(2)+0.190, 1;
	carm.p_link3 << 0.06/sqrt(2), -0.15, 0.06/sqrt(2), 1, -0.06/sqrt(2), -0.15, 0.06/sqrt(2), 1, -0.06/sqrt(2), -0.15, -0.06/sqrt(2), 1, 0.06/sqrt(2), -0.15, -0.06/sqrt(2), 1;
	carm.p_link4 << 0.06/sqrt(2), -0.05, 0.06/sqrt(2), 1, -0.06/sqrt(2), -0.05, 0.06/sqrt(2), 1, -0.06/sqrt(2), -0.05, -0.06/sqrt(2), 1, 0.06/sqrt(2), -0.05, -0.06/sqrt(2), 1;
	carm.p_link5 << 0.045/sqrt(2), 0.1, 0.045/sqrt(2), 1, -0.045/sqrt(2), 0.1, 0.045/sqrt(2), 1, -0.045/sqrt(2), 0.1, -0.045/sqrt(2), 1, 0.045/sqrt(2), 0.1, -0.045/sqrt(2), 1;
	carm.p_link6 << 0.045/sqrt(2), 0.045/sqrt(2), -0.01, 1, -0.045/sqrt(2), 0.045/sqrt(2), -0.01, 1, -0.045/sqrt(2), -0.045/sqrt(2), -0.01, 1, 0.045/sqrt(2), -0.045/sqrt(2), -0.01, 1;


////////////////////////////////////////////////////////////////////

	// Perform auto-init of rt_print buffers if the task doesn't do so
    rt_print_auto_init(1);

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	/* Avoids memory swapping for this program */
	mlockall(MCL_CURRENT|MCL_FUTURE);

	// TO DO: Specify the cycle period (cycle_ns) here, or use default value
	cycle_ns=1000000; // nanosecond
	period=((double) cycle_ns)/((double) NSEC_PER_SEC);	//period in second unit
	
	// Set the demo mode for EtherCAT application
	demo_mode = DEMO_MODE_TORQUE;	
	if (demo_mode == DEMO_MODE_TORQUE)
	{
		// For CST (cyclic synchronous torque) control
		if (_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns) == -1)
		{
			printf("System Initialization Failed\n");
		    return 0;
		}
		for (int i = 0; i < NUM_AXIS; ++i)
			ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_TORQUE;
	}
	else if (demo_mode == DEMO_MODE_POSITION)
	{
		// For CSP (cyclic synchronous position) control
		if (_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.init(OP_MODE_CYCLIC_SYNC_POSITION, cycle_ns) == -1)
		{
			printf("System Initialization Failed\n");
		    return 0;
		}
		for (int i = 0; i < NUM_AXIS; ++i)
			ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_POSITION;
	}
	else // DEMO_MODE_GUI 
	{
		// For CSP (cyclic synchronous position) control
		if (_systemInterface_EtherCAT_Neuromeka_NRMK_IO_Module.init(OP_MODE_CYCLIC_SYNC_POSITION, cycle_ns) == -1)
		{
			printf("System Initialization Failed\n");
		    return 0;
		}
		for (int i = 0; i < NUM_AXIS; ++i)
			ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_POSITION;		
	}
	
	// For trajectory interpolation
	initAxes();

	// TO DO: Create data socket server
	datasocket.setPeriod(period);

	if (datasocket.startServer(SOCK_TCP, NRMK_PORT_DATA))
		printf("Data server started at IP of : %s on Port: %d\n", datasocket.getAddress(), NRMK_PORT_DATA);

	printf("Waiting for Data Scope to connect...\n");
	datasocket.waitForConnection(0);
	
	// TO DO: Create control socket server
	if (guicontrolsocket.startServer(SOCK_TCP, 6868))
		printf("Control server started at IP of : %s on Port: %d\n", guicontrolsocket.getAddress(), 6868);

	printf("Waiting for Control Tool to connect...\n");
	guicontrolsocket.waitForConnection(0);

	// Neuromeka_NRMK_IO_Module_task: create and start
	printf("Now running rt task ...\n");

	rt_task_create(&Neuromeka_NRMK_IO_Module_task, "Neuromeka_NRMK_IO_Module_task", 0, 99, 0);
	rt_task_start(&Neuromeka_NRMK_IO_Module_task, &Neuromeka_NRMK_IO_Module_run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 80, 0);
	rt_task_start(&print_task, &print_run, NULL);
	
	// plotting: data socket comm
	rt_task_create(&plot_task, "plotting", 0, 80, 0);
	rt_task_start(&plot_task, &plot_run, NULL);

	// controlling: control socket
	rt_task_create(&gui_task, "gui_controlling", 0, 85, 0);
	rt_task_start(&gui_task, &gui_run, NULL);
	
	// Must pause here
	//pause();
	char	key_MODE=0;
	while(key_MODE=getchar())
	{
		if (key_MODE == 'a')	ctrl=51;
		else if (key_MODE == 'p')	ctrl=10;
		else if (key_MODE == 'q')	break;
		else if (key_MODE == 'w')	ctrl=12;
		else if (key_MODE == 'e')	ctrl=13;
		else if (key_MODE == 'r')	ctrl=14;
		else if (key_MODE == 't')	ctrl=15;
		else if (key_MODE == 'y')	ctrl=16;
		else if (key_MODE == 'c')	ctrl=21;
		else if (key_MODE == 'v')	ctrl=22;
		else if (key_MODE == 'b')	ctrl=23;
		else if (key_MODE == '1')	ctrl=1;		
		else if (key_MODE == '2')	ctrl=2;		
		else if (key_MODE == '3')	ctrl=3;		
		else if (key_MODE == '4')	ctrl=4;		
		else if (key_MODE == '5')	ctrl=5;		
		else if (key_MODE == '6')	ctrl=6;		
		else if (key_MODE == '7')	ctrl=7;		
		else if (key_MODE == '8')	ctrl=8;		
		else if (key_MODE == '9')	ctrl=9;		
		else if (key_MODE == '0')	ctrl=0;
		else{
		}
		
	}

	// Finalize
	signal_handler();

    return 0;
}



