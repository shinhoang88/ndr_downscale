// **********************sub-system description*********************************
/*

	input : cal_position
	output : Actual position , Actual velocities, Actual force , torque

	Subscriber: node name:
	downscale_cal_position -> calculated position from main controller / unit: [rad]

	Publishers: node name
	downscale_actp -> actual joint angle - unit : [rad]
	downscale_actv -> actual joint velocity - unit : [rad/s]
	downscale_tfsensor -> force value x,y,z,momentX,momentY,momentZ

*/
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
#define ms(x) (x*1000000)
/****************************************************************************/

#include "SystemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.h"
#include "EcatDataSocket.h"
#include "EcatControlSocket.h"

#include "ServoAxis.h"
//ROS
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32MultiArray.h"
#include "ndr_downscale/one.h"
#include "ndr_downscale/two.h"
#include "ndr_downscale/three.h"
#include "ndr_downscale/seven.h"
#include "ndr_downscale/six.h"
#include "ndr_downscale/twentyone.h"
#include "../include/Eigen/Dense"
#include "../include/Eigen/Core"
#include <boost/bind.hpp>
//time measure
#include <chrono>

#define NUM_AXIS	(1 + 7)		//Modify this number to indicate the actual number of motor on the network

#ifndef PI
#define PI	(3.14159265359)
#define PI2	(6.28318530718)
#endif

#define WAKEUP_TIME		(5)	// wake up timeout before really run, in second
#define NSEC_PER_SEC 			1000000000

#define DEMO_MODE_TORQUE		1
#define DEMO_MODE_POSITION		2
#define DEMO_MODE_GUI			3

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
SystemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD;

// NRMK socket for online commands from NRMK EtherLab Configuration Tool
NRMKHelper::EcatControlSocket<NUM_AXIS> guicontrolsocket; // 8-> NUM_AXIS

// Demo Mode (DEMO_MODE_TORQUE or DEMO_MODE_POSITION)
int demo_mode = DEMO_MODE_POSITION;
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
UINT16	StatusWord[NUM_AXIS] =  {0,0,0,0,0,0,0,0};
INT32 	ActualPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32 	ActualVel[NUM_AXIS] =  {0,0,0,0,0,0,0,0};
INT16 	ActualTor[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	DataIn[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	ModeOfOperationDisplay[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF2[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF3[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF4[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF5[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF6[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF7[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF8[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF9[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF10[NUM_AXIS] ={0,0,0,0,0,0,0,0};
UINT8	DF11[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF12[NUM_AXIS] ={0,0,0,0,0,0,0,0};
UINT8	DF13[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF14[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF15[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF16[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawFx[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawFy[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawFz[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawTx[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawTy[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawTz[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	OverloadStatus[NUM_AXIS] =  {0,0,0,0,0,0,0,0};
UINT8	ErrorFlag[NUM_AXIS] = {0,0,0,0,0,0,0,0};

INT16	TorqueDemandValue[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	AnalogInput1[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	DigitalInputs[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32	ManuspecMachineHWPositionExternalcommand[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32	FollowingErrorActualValue[NUM_AXIS] = {0,0,0,0,0,0,0,0};

INT32 	TargetPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32 	TargetVel[NUM_AXIS] ={0,0,0,0,0,0,0,0};
INT16 	TargetTor[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32 	DataOut[NUM_AXIS] ={0,0,0,0,0,0,0,0};
UINT8 	ModeOfOperation[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	ConfigParam1[NUM_AXIS] = {0,0,0,0,0,0,0,0}; //torque sensor -> 11 sensing start
UINT32	ConfigParam2[NUM_AXIS] = {0,0,0,0,0,0,0,0}; //torque sensor
UINT16	ControlWord[NUM_AXIS] ={0,0,0,0,0,0,0,0};
UINT32	ProfileVelocity[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	DigitalOutpus[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	TorqueOffset[NUM_AXIS] = {0,0,0,0,0,0,0,0};

///// SDO Access /////////

double ts;

//////////////////////////

// Interface to physical axes
NRMKHelper::ServoAxis Axis[NUM_AXIS];

/****************************************************************************/

// Xenomai RT tasks
RT_TASK Servotronix_motion_control_ltd__CDHD_task;
RT_TASK print_task;
RT_TASK plot_task;
RT_TASK gui_task;
RT_TASK intial_task;

// For RT thread management
static int run = 1;
unsigned long fault_count=0;
long ethercat_time=0, worst_time=0;
#define min_time	0
#define max_time	100000
#define hist_step	(100)
unsigned int histdata[hist_step+1];
unsigned int interval_size=350;

//ROS
ros::Publisher torque_pub;
std_msgs::Float64MultiArray msgtorque;
ros::Publisher Targettorque_pub;
std_msgs::Float64 msgTargettorque;
ros::Publisher absolute_position_pub;
std_msgs::Int32 msg_absolute_position;

//Target
double Target_torque = 0;
double Target_torque_raw = 0;

// reset value
double calculated_reset = 0;
double ts_reset = 0;

// Sine trajectory
bool sine_start = false, switch_2pi=true, goto_center=false, negative=false, positive=false;
int initial_position;
int current_magnitude=1000;
int input_magnitude=1000;
float frequency=0.2;
double target_position_sine;

// enc to deg
double enctodeg[NUM_AXIS]={0,0,0,0,0,0,0};
double prev_enctodeg[NUM_AXIS]={0,0,0,0,0,0,0};
double disp[NUM_AXIS]={0,0,0,0,0,0,0}; // angle displacement
//ang_vel
double vel_enctodeg[NUM_AXIS]={0,0,0,0,0,0,0};

//Torque converting
double Mt_Amp[7]={2.744,8.033,4.101,4.101,2.744,2.63,1.131};// MOtor rated torque from motor driver
double Mt_cosnt[7]={0.3656, 0.4401, 0.4626, 0.4626, 0.658, 0.3658, 0.2325}; // motor torque constant
double G_ratio[7] = {161,160,160,160,160,160,120}; // gear ratio RV
//double G_ratio[7] = {160,160,160,160,160,160,120}; // gear ratio 1ST HARMONIC
int T_limit[7]={0,0,0,0,0,0,0};
double time_taken=0;

//********************FRTICTION COMPENSATE FUCTION VARIABLES *******************

double z[7] = {0,0,0,0,0,0,0};
double prev_z[7] = {0,0,0,0,0,0,0};
double z_dot[7] = {0,0,0,0,0,0,0};
double prev_z_dot[7] = {0,0,0,0,0,0,0};
double fun_s[7] = {0,0,0,0,0,0,0};
double Friction_LG[7] = {0,0,0,0,0,0,0};

double al_0[7] = {0,0,0,0,0,0,0};
double al_1[7] = {0,0,0,0,0,0,0};
double av_s[7] = {0,0,0,0,0,0,0};
double sig_0[7] = {0,0,0,0,0,0,0};
double sig_1[7] = {0,0,0,0,0,0,0};
double sig_2[7] = {0,0,0,0,0,0,0};
double c_time = 0.001;

//**********************GRAVITY COMPENSATE FUCTION VARIABLES *******************
//revised 190818
double L2= 0.547;
double L4= 0.484;
double L6= 0.273;

double lg3= 0.24043;
double lg5= 0.21711;
double lg7= 0.15735;
double grav = 9.8;
double LL2=-L2+lg3;
double LL4=-L4+lg5;
double LL6=-L6+lg7;
//follwong modellingfile
// updated 190926
double m1 = 23;
double m2 = 11.66;//5.86;//13.66;
double m3 = 9.71;
double m4 = 16;//7.08;
double m5 = 6.5; //5;
double m6 = 3.5;//4.112;
double m7 = 0.808;


//******************************************************************************
//refer to below contrents,.
/*
//motor driver continuos current
Icon[0]=2.744;	//motor 1 driver continuous current (A)
Icon[1]=8.033;	//motor 2 driver continuous current (A)
Icon[2]=4.101;	//motor 3 driver continuous current (A)
Icon[3]=4.101;	//motor 4 driver continuous current (A)
Icon[4]=2.744;	//motor 5 driver continuous current (A)
Icon[5]=2.63;	//motor 6 driver continuous current (A)
Icon[6]=1.131;	//motor 7 driver continuous current (A)


//motor torque constants
Kt[0]=0.3656;	//motor 1 torque constant (Nm/A)
Kt[1]=0.4401;	//motor 2 torque constant (Nm/A)
Kt[2]=0.4626;	//motor 3 torque constant (Nm/A)
Kt[3]=0.4626;	//motor 4 torque constant (Nm/A)
Kt[4]=0.3658;	//motor 5 torque constant (Nm/A)
Kt[5]=0.3658;	//motor 6 torque constant (Nm/A)
Kt[6]=0.2325;	//motor 7 torque constant (Nm/A)
*/

//********************************OTHER VARIABLES ******************************
//force sensing
double FTsen[6]={0,0,0,0,0,0};
const int df = 50; // for sensor
const int dm = 1000;
const double dt=0.001; // time
int test=0;
int input=0;
// data saving pointer
FILE *pData;
double ts_data;
double previous_position=100000; // arbitrary number

// Joint position recorded from encoders
double joint1p, joint2p, joint3p, joint4p, joint5p, joint6p, joint7p;
// Joint velocity in rad/s computed by encoders enctodeg
double jointvel[7]={0,0,0,0,0,0,0};
// Calculated torque from the main Algorithm program (exclude: Gravity comp)
double cal_torque[7]={0,0,0,0,0,0,0};
// Calculated joint angle (in deg) from the main Algorithm program
double cal_position[7]={0,0,0,0,0,0,0};
// Applying torque considering the torque limits
double f_torque[7]={0,0,0,0,0,0,0};
// Gravity compensate torque
double G_matrix[7]={0,0,0,0,0,0,0};
// Friction compensate torque
double Firction_tq[7]={0,0,0,0,0,0,0};
// Joint limits in deg
double jointlimit[7]={180,110,180,130,180,119.1,180};

// chrono
std::chrono::high_resolution_clock::time_point chrono_start;


// Signal handler for CTRL+C
void signal_handler(int signum);

/*
void msgCallbackt(const ndr_downscale::seven::ConstPtr &msg)
{

	cal_torque[0] = msg->a;
	cal_torque[1] = msg->b;
	cal_torque[2] = msg->c;
	cal_torque[3] = msg->d;
	cal_torque[4] = msg->e;
	cal_torque[5] = msg->f;
	cal_torque[6] = msg->g;

}
*/
void msgCallbackp(const ndr_downscale::seven::ConstPtr &msg)
{

	cal_position[0] = msg->a;
	cal_position[1] = msg->b;
	cal_position[2] = msg->c;
	cal_position[3] = msg->d;
	cal_position[4] = msg->e;
	cal_position[5] = msg->f;
	cal_position[6] = msg->g;

}

/*
//************************ GRAVITY COMPENSTATE TORQUE **************************
void Gr_Matrix(double th1, double th2, double th3, double th4, double th5, double th6, double th7){

    G_matrix[0]=0;

    G_matrix[1]=grav*(-(sin(th2)*(L2*m2 + LL2*m2 + L2*m3 + L2*m4 + L2*m5 + L2*m6 + L2*m7 + cos(th4)*(LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(th6)) -
                    (LL6*m6 + L6*(m6 + m7))*cos(th5)*sin(th4)*sin(th6))) + cos(th2)*
                ((LL6*m6 + L6*(m6 + m7))*sin(th3)*sin(th5)*sin(th6) - cos(th3)*
                    ((LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(th6))*sin(th4) + (LL6*m6 + L6*(m6 + m7))*cos(th4)*cos(th5)*sin(th6))));

    G_matrix[2]=grav*(sin(th2)*((LL6*m6 + L6*(m6 + m7))*cos(th3)*sin(th5)*sin(th6) +
                    sin(th3)*((LL4*( m4) + L4*( m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(th6))*sin(th4) + (LL6*m6 + L6*(m6 + m7))*cos(th4)*cos(th5)*sin(th6))));

    G_matrix[3]=grav*(-(cos(th2)*((LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(th6))*sin(th4) + (LL6*m6 + L6*(m6 + m7))*cos(th4)*cos(th5)*sin(th6))) -
                cos(th3)*sin(th2)*(cos(th4)*(LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(th6)) - (LL6*m6 + L6*(m6 + m7))*cos(th5)*sin(th4)*sin(th6)));

    G_matrix[4]=grav*((LL6*(m5+m6) + L6*(m5+m6 + m7))*(cos(th5)*sin(th2)*sin(th3) + (cos(th3)*cos(th4)*sin(th2) + cos(th2)*sin(th4))*sin(th5))*sin(th6));

    G_matrix[5]=grav*((-(LL6*m6) - L6*(m6 + m7))*(-(cos(th6)*sin(th2)*sin(th3)*sin(th5)) + cos(th2)*(cos(th5)*cos(th6)*sin(th4) + cos(th4)*sin(th6)) +
                    cos(th3)*sin(th2)*(cos(th4)*cos(th5)*cos(th6) - sin(th4)*sin(th6))));

    G_matrix[6]=0;
}

//*************************FRICTION COMPENSTATE*********************************
double friction_LuGre(int axis, double ang_v) // axis , ang_vel
{

  switch (axis) {
  	case 0: {
  		//1j
      al_0[0] = 30.5165;
      al_1[0] = -15.9017;
      av_s[0] = 0.183;
      sig_2[0] = 38.2933;

      sig_0[0] = 8090;//1009;
      sig_1[0] =200;//13.3075;
  	}

  	case 1: {
  		//2j seems good but not sure...

      al_0[1] = 47.4133*0.8;
      al_1[1] = -0.9973;
      av_s[1] = 0.0027;
      sig_2[1] = 226.4123;

      sig_0[1] = 1890700;//*0.1;//1009;
      sig_1[1] = 302040*0.02;
  	}

  	case 2: {
  		//3j done!
  		//6factor done!
      al_0[2] =45.6405;
      al_1[2] = -3.1568;
      av_s[2] =-0.0566;
      sig_2[2] = 110.8247;

      sig_0[2] = 1174059;// high -> initail moving hard
      sig_1[2] = 2336.62;// high -> torque saturation occured + motor sound weired at reversal velocity
  	}

  	case 3: {
  		//4j done!
      al_0[3] = 40.7462; //44.7462;
      al_1[3] = -8.5697;
      av_s[3] = 0.0897;
      sig_2[3] = 86.2281;

      sig_0[3] = 1047400*0.7;
      sig_1[3] = 1626.3;//334800*0.008;//*0.003;//*0.0106;
  	}

  	case 4: {
  		//5j done!
      al_0[4] = 40;
      al_1[4] = 0.9945;
      av_s[4] = -0.00000013723;
      sig_2[4] =100;

      sig_0[4] =500000;
      sig_1[4] =1500;
  	}

  	case 5:	{
  		//6j done!
      al_0[5] = 13.0009;
      al_1[5] = -4.7344;
      av_s[5] = -0.0198;
      sig_2[5] = 47.0259;

      sig_0[5] = 106490;
      sig_1[5] =520.9762;
  	}

  	case 6: {
  		//7j done!
      al_0[6] = 4.2048;
      al_1[6] = 0.3953;
      av_s[6] = 0.0208;
      sig_2[6] = 9.1313;//26.5761;

      sig_0[6] = 136320*0.8;//104668;//646680;//
      sig_1[6] =2170.3*0.07;//550;//1800;//
  	}
  }

  fun_s[axis] = al_0[axis]+ al_1[axis]*exp(-pow((ang_v/av_s[axis]),2));

  z[axis] = (c_time*(prev_z_dot[axis])) + prev_z[axis];

  z_dot[axis] = (ang_v - ((sig_0[axis] * fabs(ang_v))*z[axis]) / fun_s[axis])*exp(-fabs(ang_v)/0.1);

  Friction_LG[axis] = sig_1[axis]*z_dot[axis] + sig_0[axis] *z[axis] + sig_2[axis]*ang_v ;

  prev_z[axis]= z[axis]; //save previous z value
  prev_z_dot[axis] = z_dot[axis];  //save previous z_dot value

}
*/
//*************************DATA SAVING FUNCTION*********************************
void DataSave(unsigned int count)
{

	pData = fopen("/home/user/catkin_ws/tien_realtorquedata.txt","a+");

	fprintf(pData,"%u ", count);
	fprintf(pData,"%f ", (double)count/1000); // realtime (sec)

	fprintf(pData,"%f ", (double)ethercat_time/ 1000000000); //dt
	/*
	fprintf(pData,"%f ", cal_torque[0]) ; // sin torque
	fprintf(pData,"%f ",  G_matrix[0] ); //  gravity torque
	fprintf(pData,"%f ", f_torque[0]); //  applied torque
	fprintf(pData,"%f ", jointvel[0]); //  angular velocity
	fprintf(pData,"%f ", joint1p); //  current angle
	fprintf(pData,"%f ", disp[1]); // angle displacment*/
	fprintf(pData,"%f ", (double)ActualTor[0]); // actual torque
	fprintf(pData,"%f ", (double)ActualTor[1]); // actual torque
	fprintf(pData,"%f ", (double)ActualTor[2]); // actual torque
	fprintf(pData,"%f ", (double)ActualTor[3]); // actual torque
	fprintf(pData,"%f ", (double)ActualTor[4]); // actual torque
	fprintf(pData,"%f ", (double)ActualTor[5]); // actual torque
	fprintf(pData,"%f ", (double)ActualTor[6]); // actual torque

	fprintf(pData,"\n");

	fclose(pData);
}

//***********************MOTOR INIT AND BREAK OFF*******************************
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

    // Motor break off
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOn(i);
	}

  	// Use this commands for stopping the motor (break on)
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(0);
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(1);
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(2);
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(3);
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(4);
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(5);
    //_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(6);
	return 1;
}

//***********************MOTOR INIT AND BREAK ON********************************
int offAxes()
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

		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(i);

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
			for (int i=0; i<NUM_AXIS; i++) // 8 -> NUM_AXIS

			{
				_loggingBuff[rearIdx].ActualPos[i] = ActualPos[i];
				_loggingBuff[rearIdx].ActualVel[i] = ActualVel[i];
			}
			rearIdx++;
		}
	}
}

//********AUTO COMPENSATE GRAVITY and MEASURE F/T SENSOR'S DATA*****************
int compute()
{
	/*
		if (demo_mode == DEMO_MODE_TORQUE)
		{
			// CST
			for (int i=0; i<NUM_AXIS; ++i)
			{
				if (system_ready)
					TargetTor[i]=0;
				else
				{
					_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD_EtherCAT_Drive__CoE_.setServoOn(i);
					TargetTor[i]=0;
				}
			}
		}
		// will not use below contents
		else if (demo_mode == DEMO_MODE_POSITION)
		{
			// CSP
			for (int i=0; i<NUM_AXIS; ++i)
			{
				if (system_ready)
				{
					TargetPos[i]=(int) (sine_amp*(sin(PI2*f*gt))) + ZeroPos[i];
				}
				else if ((InitFlag[i]==0) && (ActualPos[i]!=0))
				{
					_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD_EtherCAT_Drive__CoE_.setServoOn(i);
					TargetPos[i] = ZeroPos[i] = ActualPos[i];
					InitFlag[i] = 1;
				}
			}
		}
		else
		{
			// For Command Control	-> ?????? getting feed back???
			for (int i=0; i<NUM_AXIS; ++i) // numaxis -> torque sensor included
			{
				Axis[i].setCurrentPosInCnt(ActualPos[i]);
				Axis[i].setCurrentVelInCnt(ActualVel[i]);
				Axis[i].setCurrentTorInCnt(ActualTor[i]);
				Axis[i].setDataIn(DataIn[i]);

				Axis[i].setCurrentTime(gt);


				/*
				// Init Trajectory
				if ((InitFlag[i] == 0) && (ModeOfOperation[i] == OP_MODE_CYCLIC_SYNC_POSITION))
				{
					if ((ActualPos[i] != 0) || (StatusWord[i] != 0))
					{
						TargetPos[i] = ActualPos[i];
						Axis[i].setTarPosInCnt(ActualPos[i]);
						InitFlag[i] = 1;
					}
				}
				else
					_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD_EtherCAT_Drive__CoE_.setServoOn(i);

				TargetPos[i] = Axis[i].getDesPosInCnt();
				TargetVel[i] = Axis[i].getDesVelInCnt();
				TargetTor[i] = Axis[i].getDesTorInCnt();
				DataOut[i] = Axis[i].getDataOut();
			} */
	//}*/

	// For Command Control	-> ?????? getting feed back???
	for (int i=0; i<NUM_AXIS; ++i) // numaxis -> torque sensor included
	{
		Axis[i].setCurrentPosInCnt(ActualPos[i]);
		Axis[i].setCurrentVelInCnt(ActualVel[i]);
		Axis[i].setCurrentTorInCnt(ActualTor[i]);
		Axis[i].setDataIn(DataIn[i]);

		Axis[i].setCurrentTime(gt);

		if(i==6) //Only 7 joint gear ratio : 120
		{
			enctodeg[i] = ActualPos[i]/2796202.6666; // link output deg /(pow(2,23)*120)*360
			// vel_enctodeg[i] = ActualVel[i]/2796202.6666; // link output deg/s
		}

		else if(i==0) //Only 7 joint gear ratio : 161
		{
			enctodeg[i] = ActualPos[i]/3751571.9111; // link output deg /(pow(2,23)*161)*360
			// vel_enctodeg[i] = ActualVel[i]/3751571.9111; // link output deg/s
		}

		else{
			enctodeg[i] = ActualPos[i]/3728270.2222; // link output deg /(pow(2,23)*160)*360
			vel_enctodeg[i] = ActualVel[i]/3728270.2222; // link output deg/s
			//vel_enctodeg[i] = (enctodeg[i]-prev_enctodeg[i])/dt; // link output deg/s
		}
		disp[i] =(enctodeg[i]-prev_enctodeg[i]);
		vel_enctodeg[i] =(enctodeg[i]-prev_enctodeg[i])/dt;
		prev_enctodeg[i]=enctodeg[i];

		//vel_enctodeg[i]=ActualVel[i]/(pow(2,23)*160); // link output deg/s
	}

  //force sensing
  FTsen[0] =(double)RawFx[7]/df; // force x
  FTsen[1] = (double)RawFy[7]/df; // force y
  FTsen[2] = (double)RawFz[7]/df; // force z
  FTsen[3] =(double)RawTx[7]/dm; // moment x
  FTsen[4] = (double)RawTy[7]/dm; // moment y
  FTsen[5] = (double)RawTz[7]/dm; // moment z

	/*
	//compensation tq
  Gr_Matrix(joint1p,joint2p,joint3p,joint4p,joint5p,joint6p,joint7p);

	//Torque limits condsidering allowable starting peak torque(Nm)
	// safe torque : 60% of allowabel torque
	T_limit[0] = 229*0.6;	//J1
	T_limit[1] = 841*0.6;	//J2
	T_limit[2] = 484*0.6;	//J3
	T_limit[3] = 484*0.6;	//J4
	T_limit[4] = 229*0.6;	//J5
	T_limit[5] = 229*0.6;	//J6
	T_limit[6] = 70*0.6;	//J7

	// Applying torque considering the torque limits
	for (int i=0; i<7; i++)
	{
		f_torque[i]=cal_torque[i]+G_matrix[i];// + Firction_tq[i];

		if (f_torque[i]>T_limit[i])
		{
			//f_torque[i] = T_limit[i];
			f_torque[i] =G_matrix[i];
		}

		else
		{
			f_torque[i]=f_torque[i];
		}

	}
	*/

 	// cal_position: joint angle position in deg received by ROS subcriber Node
	// Check Joint Limits

  for (int i=0; i<7; i++)
  {
	/*
	if (enctodeg[i] > jointlimit[i])
	{
		TargetPos[i] = jointlimit[i];
	}
	else if (enctodeg[i] < -jointlimit[i])
	{
		TargetPos[i] = -jointlimit[i];
	}
    else
    {
      // torque converting : (computed torque*1000) / (motor contstant*motor contnous current)
      TargetPos[i] = (int) cal_position[i];
    }
	*/
	TargetPos[i] = (int) cal_position[i];
  }


	return 0;
}

// Servotronix_motion_control_ltd__CDHD_task
void Servotronix_CDHD__run(void *arg)
{
	int s_mode=0;
	unsigned int runcount=0;
	RTIME now, previous;

	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.syncEcatMaster();


	/* Arguments: &task (NULL=self),
	 *            start time,
	 *            period
	 */

   // ROS Nodes
  ros::NodeHandle nh;
  ros::Publisher pub_jointp = nh.advertise<ndr_downscale::seven>("downscale_actp", 100); //  sending feedback data to main controller
  ros::Publisher pub_jointv = nh.advertise<ndr_downscale::seven>("downscale_actv", 100);
  ros::Publisher pub_torquesensorCAN2ETH = nh.advertise<ndr_downscale::six>("downscale_tfsensor", 100);

	rt_task_set_periodic(NULL, TM_NOW, cycle_ns);

	while (run)
	{
		rt_task_wait_period(NULL); 	//wait for next cycle

		runcount++;

		// calculating time chceking
		clock_t t;
    t = clock();
    double time = double(t) / CLOCKS_PER_SEC; //time

		if (!run)
		{
			break;
		}

		previous = rt_timer_read();

    // Convert joint angle from encoder from deg to RAD
		joint1p = (enctodeg[0])*PI/180; // rad
		joint2p = (enctodeg[1])*PI/180; // rad
		joint3p = (enctodeg[2])*PI/180; // rad
		joint4p = (enctodeg[3])*PI/180; // rad
		joint5p = (enctodeg[4])*PI/180; // rad
		joint6p = (enctodeg[5])*PI/180; // rad
		joint7p = (enctodeg[6])*PI/180; // rad


    // Publish these joint angles info to ROS Node
		ndr_downscale::seven msgp;
		msgp.a = joint1p;
		msgp.b = joint2p;
		msgp.c = joint3p;
		msgp.d = joint4p;
		msgp.e = joint5p;
		msgp.f = joint6p;
		msgp.g = joint7p;

		// Publish joint angle - unit [rad]
		pub_jointp.publish(msgp);

    // Convert joint velocity from encoder from deg/s to RAD/S
		jointvel[0] = (vel_enctodeg[0])*PI/180; //rad/s
		jointvel[1] = (vel_enctodeg[1])*PI/180; //rad/s
		jointvel[2] = (vel_enctodeg[2])*PI/180;//rad/s
		jointvel[3] = (vel_enctodeg[3])*PI/180;//rad/s
		jointvel[4] = (vel_enctodeg[4])*PI/180;//rad/s
		jointvel[5] = (vel_enctodeg[5])*PI/180;//rad/s
		jointvel[6] = (vel_enctodeg[6])*PI/180;//rad/s

    // Publish these joint velocities info to ROS Node
		ndr_downscale::seven msgpa;
		msgpa.a = jointvel[0];
		msgpa.b = jointvel[1];
		msgpa.c = jointvel[2];
		msgpa.d = jointvel[3];
		msgpa.e = jointvel[4];
		msgpa.f = jointvel[5];
		msgpa.g = jointvel[6];

    // Velocity simple filter
		for (int i=0 ; i<7 ;i++)
		{
			if(fabs(jointvel[i])>=0.6) //|| measure[i]>0.5
      {
				if(jointvel[i]>=0.6) // +
				{
					jointvel[i]=0.6;
				}

				else  // -
				{
					jointvel[i]=-0.6;
				}

			}

      else
      {
          {jointvel[i]=jointvel[i];}
      }
		 }

		// Publish joint velocity - unit [rad/s]
		pub_jointv.publish(msgpa);

    // Force/Torque sensor ROS publishing data
		ndr_downscale::six msg;
		msg.a = FTsen[0]; //FORCE X
		msg.b = FTsen[1]; //FORCE Y
		msg.c = FTsen[2]; //FORCE Z
		msg.d = FTsen[3]; //moment
		msg.e = FTsen[4];
		msg.f = FTsen[5];

		pub_torquesensorCAN2ETH.publish(msg);


		/// TO DO: read data from sensors in EtherCAT system interface
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processTxDomain();
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60410, StatusWord);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60640, ActualPos);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x606c0, ActualVel);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60770, ActualTor);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60610, ModeOfOperationDisplay);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60001, DF1);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60002, DF2);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60003, DF3);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60004, DF4);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60005, DF5);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60006, DF6);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60007, DF7);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60008, DF8);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60009, DF9);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600010, DF10);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600011, DF11);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600012, DF12);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600013, DF13);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600014, DF14);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600015, DF15);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600016, DF16);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600017, RawFx);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600018, RawFy);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600019, RawFz);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600020, RawTx);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600021, RawTy);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600022, RawTz);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600023, OverloadStatus);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x600024, ErrorFlag);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60740, TorqueDemandValue);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x20f20, AnalogInput1);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60fd0, DigitalInputs);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x20b60, ManuspecMachineHWPositionExternalcommand);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.readBuffer(0x60f40, FollowingErrorActualValue);


		/// TO DO: Main computation routine...
		compute(); // main function

		//DataSave(runcount);


		/// TO DO: write data to actuators in EtherCAT system interface
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x607a0, TargetPos);
		//_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60ff0, TargetVel);
		//_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60710, TargetTor);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60600, ModeOfOperation);

		if(runcount<20000 && runcount>10000)
		{

			ConfigParam1[7]=273;
			_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70001, ConfigParam1);
			_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70002, ConfigParam2);
			//_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processRxDomain();
			s_mode=1;
		}

		else if (runcount>25000 && s_mode==1)
		{
			ConfigParam1[7]=11;
			_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70001, ConfigParam1);
			_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70002, ConfigParam2);
			//_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processRxDomain();
			s_mode =2;
		}
		else
		{

		}

		//ConfigParam1[0]=11;
		 //ConfigParam1[7]=273;
		//_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70001, ConfigParam1);
		//_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70002, ConfigParam2);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60400, ControlWord);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60810, ProfileVelocity);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60fe1, DigitalOutpus);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60b20, TorqueOffset);

		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processRxDomain();
			ros::spinOnce();
		if (system_ready)
			saveLogData();

		// For EtherCAT performance statistics
		now = rt_timer_read();
		ethercat_time = (long) now - previous;

		if (( (runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)) ) )//(runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns))-=>for one joint //(_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.isSystemReady() && (runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)) )
		{
			system_ready=1;	//all drives have been done

			gt+= period;

			if (worst_time<ethercat_time) worst_time=ethercat_time;
			if(ethercat_time > max_time)
				++fault_count;
		}

		t = clock() - t;
    	time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds

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
	rt_task_set_periodic(NULL, TM_NOW, (int)ms(100));

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


			if (_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getMasterStatus(NumSlaves, masterState))
				rt_printf("Master: Online - State %i - %i slave(s)\n", masterState, NumSlaves);
			else
				rt_printf("Master: Offline\n");

			if (_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getRxDomainStatus())
				rt_printf("RxDomain: Online\n");
			else
				rt_printf("RxDomain: Offline\n");

			if (_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getTxDomainStatus())
				rt_printf("TxDomain: Online\n");
			else
				rt_printf("TxDomain: Offline\n");
			/*
			for(i=0; i<NUM_AXIS; ++i){
				if (_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(i, slaveState))
					rt_printf("\e[32;1mSlave: Online %i,  \e[0m\n", slaveState);
				else
					rt_printf("\e[32;1mSlave: Offline,  \e[0m\n");

				rt_printf("\e[32;1m\t StatusWord: 0x%x,  \e[0m\n",		StatusWord[i]);
				rt_printf("\e[32;1m\t ModeOfOpDisp: 0x%x,  \e[0m\n",	ModeOfOperationDisplay[i]);
				rt_printf("\e[32;1m\t ************************************************************  \e[0m\n");
				rt_printf("\e[32;1m\t joint: %i,  \e[0m\n", 	 	i);
				rt_printf("\e[32;1m\t ActualPos: %i,  \e[0m\n", 	 	ActualPos[i]);
				rt_printf("\e[32;1m\t ActualVel: %i,  \e[0m\n", 	 	ActualVel[i]);
				rt_printf("\e[32;1m\t ActualTor: %i,  \e[0m\n", 	 	ActualTor[i]);
				rt_printf("\e[32;1m\t Commandtorque: %i,  \e[0m\n", 	 	TargetTor[i]);

				rt_printf("\e[32;1m\t DF1: %i,  \e[0m\n", 	 	DF1[7]);
				rt_printf("\e[32;1m\t DF2: %i,  \e[0m\n", 	 	DF2[7]);
				rt_printf("\e[32;1m\t DF3: %i,  \e[0m\n", 	 	DF3[7]);
				rt_printf("\e[32;1m\t DF4: %i,  \e[0m\n", 	 	DF4[7]);
				rt_printf("\e[32;1m\t DF5: %i,  \e[0m\n", 	 	DF5[7]);
				rt_printf("\e[32;1m\t DF6: %i,  \e[0m\n", 	 	DF6[7]);
				rt_printf("\e[32;1m\t DF7: %i,  \e[0m\n", 	 	DF7[7]);
				rt_printf("\e[32;1m\t DF8: %i,  \e[0m\n", 	 	DF8[7]);
				rt_printf("\e[32;1m\t DF9: %i,  \e[0m\n", 	 	DF9[7]);
				rt_printf("\e[32;1m\t DF10: %i,  \e[0m\n", 	 	DF10[7]);
				rt_printf("\e[32;1m\t DF11: %i,  \e[0m\n", 	 	DF11[7]);
				rt_printf("\e[32;1m\t DF12: %i,  \e[0m\n", 	 	DF12[7]);
				rt_printf("\e[32;1m\t DF13: %i,  \e[0m\n", 	 	DF13[7]);
				rt_printf("\e[32;1m\t DF14: %i,  \e[0m\n", 	 	DF14[7]);
				rt_printf("\e[32;1m\t DF15: %i,  \e[0m\n", 	 	DF15[7]);
				rt_printf("\e[32;1m\t DF16: %i,  \e[0m\n", 	 	DF16[7]);
				rt_printf("\e[32;1m\t RawFx: %i,  \e[0m\n", 	 	RawFx[i]);
				rt_printf("\e[32;1m\t RawFy: %i,  \e[0m\n", 	 	RawFy[i]);
				rt_printf("\e[32;1m\t RawFz: %i,  \e[0m\n", 	 	RawFz[i]);
				rt_printf("\e[32;1m\t RawTx: %i,  \e[0m\n", 	 	RawTx[i]);
				rt_printf("\e[32;1m\t RawTy: %i,  \e[0m\n", 	 	RawTy[i]);
				rt_printf("\e[32;1m\t RawTz: %i,  \e[0m\n", 	 	RawTz[i]);
				rt_printf("\e[32;1m\t OverloadStatus: %i,  \e[0m\n", 	 	OverloadStatus[i]);
				rt_printf("\e[32;1m\t ErrorFlag: %i,  \e[0m\n", 	 	ErrorFlag[i]);

				//rt_printf("\e[32;1m\t TorqueDemandValue: %i,  \e[0m\n", 	 	TorqueDemandValue[i]);
				//rt_printf("\e[32;1m\t AnalogInput1: %i,  \e[0m\n", 	 	AnalogInput1[i]);
				//rt_printf("\e[32;1m\t DigitalInputs: %i,  \e[0m\n", 	 	DigitalInputs[i]);
				//rt_printf("\e[32;1m\t ManuspecMachineHWPositionExternalcommand: %i,  \e[0m\n", 	 	ManuspecMachineHWPositionExternalcommand[i]);
				//rt_printf("\e[32;1m\t FollowingErrorActualValue: %i,  \e[0m\n", 	 	FollowingErrorActualValue[i]);

			} 	 */

				if (_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(0, slaveState)&&
				_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(1, slaveState)&&
				_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(2, slaveState)&&
				_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(3, slaveState)&&
				_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(4, slaveState)&&
				_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(5, slaveState)&&
				_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(6, slaveState)&&
				_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.getAxisEcatStatus(7, slaveState))
					{rt_printf("\e[32;1mSlave: Online %i  \e[0m\n", slaveState);}
				else
				{
					rt_printf("\e[32;1mSlave: Offline,  \e[0m\n");
				}


				//rt_printf("\e[32;1m\t StatusWord: 0x%x,  \e[0m\n",		StatusWord[i]);
				//rt_printf("\e[32;1m\t ModeOfOpDisp: 0x%x,  \e[0m\n",	ModeOfOperationDisplay[i]);
				rt_printf("\e[32;1m\t ************************************************************  \e[0m\n");
				rt_printf("\e[32;1m\t    joint[1]]		  joint[2]	       joint[3]	          joint[4]	        joint[5]	      joint[6]	        joint[7]  \e[0m\n");
				rt_printf("\e[32;1m\t StatusWor[1]: %i, StatusWor[2]: %i, StatusWor[3]: %i, StatusWor[4]: %i, StatusWor[5]: %i, StatusWor[6]: %i, StatusWor[7]: %i \e[0m\n", StatusWord[0], StatusWord[1], StatusWord[2], StatusWord[3], StatusWord[4], StatusWord[5], StatusWord[6]);
				rt_printf("\e[32;1m\t ContrsWor[1]: %i, ContrsWor[2]: %i, ContrsWor[3]: %i, ContrsWor[4]: %i, ContrsWor[5]: %i, ContrsWor[6]: %i, ContrsWor[7]: %i \e[0m\n", ControlWord[0], ControlWord[1], ControlWord[2], ControlWord[3], ControlWord[4], ControlWord[5], ControlWord[6]);
				rt_printf("\e[32;1m\t ActPosdeg[1]: %.3f, ActPosdeg[2]: %.3f  ActPosdeg[3]: %.3f, ActPosdeg[4]: %.3f, ActPosdeg[5]: %.3f, ActPosdeg[6]: %.3f, ActPosdeg[7]: %.3f \e[0m\n", enctodeg[0], enctodeg[1], enctodeg[2], enctodeg[3], enctodeg[4], enctodeg[5], enctodeg[6]);
				rt_printf("\e[32;1m\t ActVeldeg[1]: %.3f, ActVeldeg[2]: %.3f, ActVeldeg[3]: %.3f, ActVeldeg[4]: %.3f, ActVeldeg[5]: %.3f, ActVeldeg[6]: %.3f, ActVeldeg[7]: %.3f \e[0m\n", vel_enctodeg[0], vel_enctodeg[1], vel_enctodeg[2], vel_enctodeg[3], vel_enctodeg[4], vel_enctodeg[5], vel_enctodeg[6]);
				//rt_printf("\e[32;1m\t ActualTor[1]: %i, ActualTor[2]: %i, ActualTor[3]: %i, ActualTor[4]: %i, ActualTor[5]: %i, ActualTor[6]: %i, ActualTor[7]: %i \e[0m\n", ActualTor[0], ActualTor[1], ActualTor[2], ActualTor[3], ActualTor[4], ActualTor[5], ActualTor[6]);
				//rt_printf("\e[32;1m\t ComdTorpr[1]: %d, ComdTorpr[2]: %d, ComdTorpr[3]: %d, ComdTorpr[4]: %d, ComdTorpr[5]: %d, ComdTorpr[6]: %d, ComdTorpr[7]: %d \e[0m\n", TargetTor[0], TargetTor[1], TargetTor[2], TargetTor[3], TargetTor[4], TargetTor[5], TargetTor[6]);
				//rt_printf("\e[32;1m\t ComdTorNm[1]: %.3f, ComdTorNm[2]: %.3f, ComdTorNm[3]: %.3f, ComdTorNm[4]: %.3f, ComdTorNm[5]: %.3f, ComdTorNm[6]: %.3f, ComdTorNm[7]: %.3f \e[0m\n", f_torque[0], f_torque[1], f_torque[2], f_torque[3], f_torque[4], f_torque[5], f_torque[6]);
				//rt_printf("\e[32;1m\t FricTorNm[1]: %.3f, FricTorNm[2]: %.3f, FricTorNm[3]: %.3f, FricTorNm[4]: %.3f, FricTorNm[5]: %.3f, FricTorNm[6]: %.3f, FricTorNm[7]: %.3f \e[0m\n", Firction_tq[0], Firction_tq[1], Firction_tq[2], Firction_tq[3], Firction_tq[4], Firction_tq[5], Firction_tq[6]);

				//rt_printf("\e[32;1m\t ReciTorNm[1]: %.3f, ReciTorNm[2]: %.3f, ReciTorNm[3]: %.3f, ReciTorNm[4]: %.3f, ReciTorNm[5]: %.3f, ReciTorNm[6]: %.3f, ReciTorNm[7]: %.3f \e[0m\n", cal_torque[0], cal_torque[1], cal_torque[2], cal_torque[3], cal_torque[4], cal_torque[5], cal_torque[6]);
				rt_printf("\e[32;1m\t ReciPosRad[1]: %.3f, ReciPosRad[2]: %.3f, ReciPosRad[3]: %.3f, ReciPosRad[4]: %.3f, ReciPosRad[5]: %.3f, ReciPosRad[6]: %.3f, ReciPosRad[7]: %.3f \e[0m\n", cal_position[0], cal_position[1], cal_position[2], cal_position[3], cal_position[4], cal_position[5], cal_position[6]);

				rt_printf("\e[32;1m\t Actdisplc[1]: %.3f, Actdisplc[2]: %.3f  Actdisplc[3]: %.3f, Actdisplc[4]: %.3f, Actdisplc[5]: %.3f, Actdisplc[6]: %.3f, Actdisplc[7]: %.3f \e[0m\n", disp[0], disp[1], disp[2], disp[3], disp[4], disp[5], disp[6]);
				//rt_printf("\e[32;1m\t FctualVel[1]: %i, FctualVel[2]: %i FctualVel[3]: %i, FctualVel[4]: %i, FctualVel[5]: %i, FctualVel[6]: %i, FctualVel[7]: %i \e[0m\n", ActualVel[0], ActualVel[1], ActualVel[2], ActualVel[3], ActualVel[4], ActualVel[5], ActualVel[6]);


				rt_printf("\e[32;1m\t RawFx: %.3f,  \e[0m\n", 	 	FTsen[0]);
				rt_printf("\e[32;1m\t RawFy: %.3f,  \e[0m\n", 	 	FTsen[1]);
				rt_printf("\e[32;1m\t RawFz: %.3f,  \e[0m\n", 	 	FTsen[2]);
				rt_printf("\e[32;1m\t RawTx: %.3f,  \e[0m\n", 	 	FTsen[3]);
				rt_printf("\e[32;1m\t RawTy: %.3f,  \e[0m\n", 	 	FTsen[4]);
				rt_printf("\e[32;1m\t RawTz: %.3f,  \e[0m\n", 	 	FTsen[5]);
				rt_printf("\e[32;1m\t errorFlags: %i,  \e[0m\n", 	 	ErrorFlag[7]);
				rt_printf("\e[32;1m\t time taken: %.5f,  \e[0m\n", 	 	time_taken);
				//rt_printf("\e[32;1m\t Digiinput[1]: %i, Digiinput[2]: %i, Digiinput[3]: %i, Digiinput[4]: %i, Digiinput[5]: %i, Digiinput[6]: %i, Digiinput[7]: %i \e[0m\n", DigitalInputs[0], DigitalInputs[1], DigitalInputs[2], DigitalInputs[3], DigitalInputs[4], DigitalInputs[5], DigitalInputs[6]);
				//rt_printf("\e[32;1m\t Statuswod[1]: %i, Statuswod[2]: %i, Statuswod[3]: %i, Statuswod[4]: %i, Statuswod[5]: %i, Statuswod[6]: %i, Statuswod[7]: %i \e[0m\n", StatusWord[0], StatusWord[1], StatusWord[2], StatusWord[3], StatusWord[4], StatusWord[5], StatusWord[6]);


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

	INT8 modeOpDisp[NUM_AXIS] = {0,0,0,0,0,0,0};
	UINT16 status[NUM_AXIS] = {0,0,0,0,0,0,0};
	INT32 actPos[NUM_AXIS] = {0,0,0,0,0,0,0};
	INT32 actVel[NUM_AXIS] = {0,0,0,0,0,0,0};
	INT16 actTor[NUM_AXIS] = {0,0,0,0,0,0,0};

	INT8 modeOp[NUM_AXIS] = {0,0,0,0,0,0,0};
	float tarval[NUM_AXIS] = {0,0,0,0,0,0,0};
	float maxvel[NUM_AXIS] = {0,0,0,0,0,0,0};
	float maxacc[NUM_AXIS] = {0,0,0,0,0,0,0};
	float maxjerk[NUM_AXIS] = {0,0,0,0,0,0,0};

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
	 *            period (here: 100 ms) //
	 */

	//rt_task_set_periodic(NULL, TM_NOW, 1e8);	// period = 100 (msec)

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

/*
// not working.....
void intial_run(void *arg)
{
 rt_task_set_periodic(NULL, TM_NOW, (int)ms(4));
 while (1)
	{
	rt_task_wait_period(NULL);

	if(input == 1)
	{
	ConfigParam1[0]=12;
	//ControlWord[0]=0; ControlWord[1]=0; ControlWord[2]=0; ControlWord[3]=0;ControlWord[4]=0; ControlWord[5]=0;ControlWord[6]=0;
	DigitalOutpus[0]=1; DigitalOutpus[1]=1; DigitalOutpus[2]=1; DigitalOutpus[3]=1;DigitalOutpus[4]=1; DigitalOutpus[5]=1;DigitalOutpus[6]=1;
	printf("Servo drives Stopped 5second!\n");
	    _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70001, ConfigParam1); //torque sensor setting value
	//	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60400, ControlWord);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60fe1, DigitalOutpus);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processRxDomain();

	//runcount>5*(NSEC_PER_SEC/cycle_ns);
	rt_task_delete(&Servotronix_motion_control_ltd__CDHD_task);
	rt_task_delete(&print_task);
   // usleep(5000000);
	//printf("Servo drives Stopped!\n");
   // _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.deinit();


	}

	}
} */

/****************************************************************************/
void signal_handler(int signum = 0)
{
	//rt_task_delete(&plot_task);
	//rt_task_delete(&gui_task);
		ConfigParam1[7]=12;
	//	ControlWord[0]=6; ControlWord[1]=6; ControlWord[2]=6; ControlWord[3]=6;ControlWord[4]=6; ControlWord[5]=6;ControlWord[6]=6;
	printf("Servo drives Stopped 5second!\n");
	    _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70001, ConfigParam1); //torque sensor setting value
		ControlWord[0]=14; ControlWord[1]=14; ControlWord[2]=14; ControlWord[3]=14; ControlWord[4]=14; ControlWord[5]=14; ControlWord[6]=14;
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x60400, ControlWord);
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processTxDomain();
		_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processRxDomain();

	//offAxes();

	//runcount>5*(NSEC_PER_SEC/cycle_ns);
	rt_task_delete(&Servotronix_motion_control_ltd__CDHD_task);
	rt_task_delete(&print_task);
    //usleep(500000);
    printf("Brake on!\n");
	//DigitalOutpus[0] = 1; DigitalOutpus[1] = 1; DigitalOutpus[2] = 1; DigitalOutpus[3] = 1; DigitalOutpus[4] = 1; DigitalOutpus[5] = 1; DigitalOutpus[6] = 1;

	printf("Servo drives Stopped!\n");
    _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.deinit();
    exit(1);
}

/****************************************************************************/
int main(int argc, char **argv)
{

	ros::init(argc, argv, "sub_downscale");

	ros::NodeHandle nh;

	//ros::Subscriber sub_jointt = nh.subscribe("downscale_cal_torque", 100, msgCallbackt);
	ros::Subscriber sub_jointp = nh.subscribe("downscale_cal_position", 100, msgCallbackp);

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
	demo_mode = DEMO_MODE_POSITION;
	if (demo_mode == DEMO_MODE_TORQUE)
	{
		// For CST (cyclic synchronous torque) control
		if (_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.init(OP_MODE_CYCLIC_SYNC_TORQUE, cycle_ns) == -1)
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
		if (_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.init(OP_MODE_CYCLIC_SYNC_POSITION, cycle_ns) == -1)
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
		if (_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.init(OP_MODE_CYCLIC_SYNC_POSITION, cycle_ns) == -1)
		{
			printf("System Initialization Failed\n");
		    return 0;
		}
		for (int i = 0; i < NUM_AXIS; ++i)
			ModeOfOperation[i] = OP_MODE_CYCLIC_SYNC_POSITION;
	}
	
	// For trajectory interpolation
	initAxes(); //servo on

	


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

	// Servotronix_motion_control_ltd__CDHD_task: create and start
	printf("Now running rt task ...\n");

	// FT SENSOR BIAS START
	//********************************************************************************************************************
	 ConfigParam1[7]=11;
	 _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70001, ConfigParam1);
	 _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70002, ConfigParam2);

	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processRxDomain();
	//********************************************************************************************************************


	// FT SENSOR GETTING VALUE START
	//********************************************************************************************************************
	ConfigParam1[7]=273;
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70001, ConfigParam1);
	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.writeBuffer(0x70002, ConfigParam2);

	_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.processRxDomain();
	//********************************************************************************************************************

	rt_task_create(&Servotronix_motion_control_ltd__CDHD_task, "Servotronix_motion_control_ltd__CDHD_task", 0, 99, 0);
	rt_task_start(&Servotronix_motion_control_ltd__CDHD_task, &Servotronix_CDHD__run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 80, 0);
	rt_task_start(&print_task, &print_run, NULL);

	//rt_task_create(&intial_task, "intial", 0, 90, 0);
	//rt_task_start(&intial_task, &intial_run, NULL);



	// plotting: data socket commd
	//rt_task_create(&plot_task, "plotting", 0, 80, 0);
	//rt_task_start(&plot_task, &plot_run, NULL);

	// controlling: control socket
	//rt_task_create(&gui_task, "gui_controlling", 0, 85, 0);
	//rt_task_start(&gui_task, &gui_run, NULL);

	// Must pause here

	while (1)
	{



		//scanf("%d", &input);
		usleep(1e5);
	}

	// Finalize
	signal_handler();

    return 0;
}
