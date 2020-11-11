/*
Geomagic Touch Device Publisher PROGRAM
Author: Phi Tien Hoang
Description:
	subscribing transformation matrix, position, joint angle, button states from
	the haptic Device
	publishing these info
*/
#include <ros/ros.h>
#include <std_msgs/Int8.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>

#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

#include  "ndr_downscale/trans.h"
#include  "ndr_downscale/buttons.h"
#include "ndr_downscale/three.h"
#include "ndr_downscale/six.h"

#include <pthread.h>
#include <signal.h>
#include <rate.h>



double Input_data[16];
int Initialization_mode;
int buttons[3];
int count=0;
float end_force[3];
float force_x, force_y, force_z;
float hs_force=1;
double hs_force_1=1;
double cur_tau[7];


ros::Subscriber sub_end_force;

typedef struct
{
	HDboolean m_buttonState;		/* Has the device button has been pressed. */
	hduVector3Dd m_devicePosition;	/* Current device coordinates. */
	HDErrorInfo m_error;
} DeviceData;

	static DeviceData gServoDeviceData;
	HDSchedulerHandle hUpdateHandle;
	HDErrorInfo error;
	HHD hHD;


// ***************GEOMAGIC TOUCH DEVICE CALLBACK VARIABLES**********************
HDdouble hd_position[3], hd_joint_angles[3], hd_gimbal_angles[3];
HDint hd_buttons = 0;
HDboolean hd_inkwell;

HDdouble hd_transform_matrix[16];

HDdouble position[3];
HDfloat force[3];
HDfloat limit_force = 1;

void signal_handler(int signum = 0)
{
	printf("Servo drives Stopped!\n");
	exit(1);
}


// ***************GEOMAGIC TOUCH DEVICE CALLBACK FUNCTION***********************
HDCallbackCode HDCALLBACK Haptic_data_acquisition(void *pUserData){

	hdBeginFrame(hdGetCurrentDevice());

	hdGetDoublev(HD_CURRENT_TRANSFORM, &hd_transform_matrix[0]);
	hdGetDoublev(HD_CURRENT_POSITION, hd_position);
	hdGetDoublev(HD_CURRENT_JOINT_ANGLES, hd_joint_angles);
	hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, hd_gimbal_angles);

	int nButtons=0;
	hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
	hdGetBooleanv(HD_CURRENT_INKWELL_SWITCH, &hd_inkwell);

	hdSetFloatv(HD_CURRENT_FORCE, force);
	hdSetFloatv(HD_SOFTWARE_FORCE_IMPULSE_LIMIT, &limit_force);

	/* In order to get the specific button 1 state, we use a bitmask to
	   test for the HD_DEVICE_BUTTON_1 bit. */
	//gServoDeviceData.m_buttonState =
	buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
	buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

	hdEndFrame(hdGetCurrentDevice());

	return HD_CALLBACK_CONTINUE;
}

// *************************CALLBACK FUNCTIONS**********************************

void msgCallbackE(const ndr_downscale::three::ConstPtr& msge)
{

	end_force[0] = msge->a;
	end_force[1] = msge->b;
	end_force[2] = msge->c;

	force_x=1*end_force[0]/50;
	force_y=1*end_force[1]/50;
	force_z=1*end_force[2]/50;

}

void msgCallbackT(const ndr_downscale::six::ConstPtr& msg)
{
	cur_tau[0] = msg->a;
	cur_tau[1] = msg->b;
	cur_tau[2] = msg->c;
	cur_tau[3] = msg->d;
	cur_tau[4] = msg->e;
	cur_tau[5] = msg->f;
}

int main(int argc, char** argv)
{
	DeviceData currentData;

	ros::init(argc, argv, "hd_pub_node"); // Node name initialization
	ros::NodeHandle nh;                   // Node handle declaration for communication with ROS

	hHD = hdInitDevice("omni1");
	if (HD_DEVICE_ERROR(error = hdGetError())){
		ROS_ERROR("Failed to initialize haptic device");
		return -1;
	}

	ROS_INFO("Found %s.",hdGetString(HD_DEVICE_MODEL_TYPE));

	hdSetSchedulerRate(1000);		// 1000Hz

	hdStartScheduler();
	if (HD_DEVICE_ERROR(error = hdGetError())){
		ROS_ERROR("Failed to start the scheduler");
		return -1;
	}

	signal(SIGINT, signal_handler);
	signal(SIGTERM, signal_handler);

	// Schedule the frictionless plane callback, which will then run at
	// servoloop rates and command forces if the user penetrates the plane.




	ros::Publisher hd_trans = nh.advertise<ndr_downscale::trans>("/hd_trans", 1);
	ros::Publisher hd_buttons = nh.advertise<ndr_downscale::buttons>("/hd_buttons", 1);
	ros::Publisher ft_sensor_data = nh.advertise<ndr_downscale::six>("/ft_sensor_data", 1);
	ros::Subscriber sub_end_force = nh.subscribe("end_force", 100, &msgCallbackE);
	ros::Subscriber sub_torquesensorCAN2ETH = nh.subscribe("downscale_tfsensor", 1, &msgCallbackT);
	//sub_end_force = nh.subscribe("end_force", 100, msgCallbackE);

	hdScheduleAsynchronous(Haptic_data_acquisition, &currentData, HD_DEFAULT_SCHEDULER_PRIORITY);
	hdEnable(HD_FORCE_OUTPUT);



	ros::Rate rate(1000); //control frequency


	while (ros::ok()) {
		ros::spinOnce(); //very important**
		Input_data[0] = hd_transform_matrix[0];
		Input_data[1] = hd_transform_matrix[1];
		Input_data[2] = hd_transform_matrix[2];
		Input_data[3] = hd_transform_matrix[3];
		Input_data[4] = hd_transform_matrix[4];
		Input_data[5] = hd_transform_matrix[5];
		Input_data[6] = hd_transform_matrix[6];
		Input_data[7] = hd_transform_matrix[7];
		Input_data[8] = hd_transform_matrix[8];
		Input_data[9] = hd_transform_matrix[9];
		Input_data[10] = hd_transform_matrix[10];
		Input_data[11] = hd_transform_matrix[11];
		Input_data[12] = hd_transform_matrix[12];
		Input_data[13] = hd_transform_matrix[13];
		Input_data[14] = hd_transform_matrix[14];
		Input_data[15] = hd_transform_matrix[15];


if(buttons[0]==1){
	force[0]=-force_y;//-cur_tau[1]/50;//-force_y;
	force[1]=force_z;//-cur_tau[2]/50;//-force_z;
	force[2]=-force_x;//-end_force[0]*5;//-force_x;
}
else{
	force[0]=0;
	force[1]=0;
	force[2]=0;
}

	//force[0]=-force_y;//-cur_tau[1]/50;//-force_y;
	//force[1]=force_z;//-cur_tau[2]/50;//-force_z;
	//force[2]=-force_x;//-end_force[0]*5;//-force_x;
		//std::cout<<std::string(80,'-')<<std::endl;
		//printf("%f \t %f \t %f \t %f \n", hd_transform_matrix[0], hd_transform_matrix[4], hd_transform_matrix[8], hd_transform_matrix[12]);
		//printf("%f \t %f \t %f \t %f \n", hd_transform_matrix[1], hd_transform_matrix[5], hd_transform_matrix[9], hd_transform_matrix[13]);
		//printf("%f \t %f \t %f \t %f \n", hd_transform_matrix[2], hd_transform_matrix[6], hd_transform_matrix[10], hd_transform_matrix[14]);
		//printf("%f \t %f \t %f \t %f \n", hd_transform_matrix[3], hd_transform_matrix[7], hd_transform_matrix[11], hd_transform_matrix[15]);
		//printf("%d \t %d \t %d \n", buttons[0], buttons[1], hd_inkwell);
		//printf("%f \t %f \t %f \n", force[0], force[1], force[2]);
		//std::cout<<std::string(80,'-')<<std::endl;

/*
	printf("**********************master device***************************************\n");
	//rt_printf("angle: %f %f %f %f %f %f %f \n", act_ang[0]  , act_ang[1]  , act_ang[2] , act_ang[3], act_ang[4], act_ang[5], act_ang[6] );
	printf("\e[32;1m\t Current_X: %f,  \e[0m\n", Input_data[12]);
	printf("\e[32;1m\t Current_Y: %f,  \e[0m\n", Input_data[13]);
	printf("\e[32;1m\t Current_Z: %f,  \e[0m\n", Input_data[14]);
	printf("\e[32;1m\t Del_X: %d,  \e[0m\n", buttons[0]);
	printf("\e[32;1m\t Del_Y: %d,  \e[0m\n", buttons[1]);
	printf("\e[32;1m\t Del_Z: %d,  \e[0m\n", hd_inkwell);
*/
//printf("%f \t %f \t %f \n", end_force[0], end_force[1], end_force[2]);
//printf("%f \t %f \t %f \n", force_x, force_y, force_z);
//printf("%f \t %f \t %f \n", -end_force[1], end_force[2], -end_force[0]);
		//printf("%d \n", count);
		//////////////////// trans //////////////////////////////////////
		ndr_downscale::trans msg;

		msg.a = Input_data[0];
		msg.b = Input_data[1];
		msg.c = Input_data[2];
		msg.d = Input_data[3];
		msg.e = Input_data[4];
		msg.f = Input_data[5];
		msg.g = Input_data[6];
		msg.h = Input_data[7];
		msg.i = Input_data[8];
		msg.j = Input_data[9];
		msg.k = Input_data[10];
		msg.l = Input_data[11];
		msg.m = Input_data[12];
		msg.n = Input_data[13];
		msg.o = Input_data[14];
		msg.p = Input_data[15];

		hd_trans.publish(msg);
		/////////////////////////////////////////////////////////////////
		ndr_downscale::six msgt;

		msgt.a = cur_tau[0];
		msgt.b = cur_tau[1];
		msgt.c = cur_tau[2];
		msgt.d = cur_tau[3];
		msgt.e = cur_tau[4];
		msgt.f = cur_tau[5];


		ft_sensor_data.publish(msgt);


		//////////////////// buttons ////////////////////////////////////
		ndr_downscale::buttons msga;

		msga.a = buttons[0];
		msga.b = buttons[1];
		msga.c = hd_inkwell;

		hd_buttons.publish(msga);
		/////////////////////////////////////////////////////////////////

		count ++;
		rate.sleep(); //control frequency

		////////////////////////////////////// force feedback /////////////////////////



		////////////////////////////////////////////////////////////////////////////////

	}
	hdStopScheduler();
}
