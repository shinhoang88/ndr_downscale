/*
    190915
    carteisiona impedance _ refer to dlr robot
    Test version for gazebo simulation
	gravity torque : works ( mass refered world file )

 */
#ifndef __XENO__
#define __XENO__
#endif

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sys/ioctl.h>
#include <math.h>
//xenomai
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <rtdk.h>
#define ms(x) (x*1000000)

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

// master device *********************
#include  "ndr_downscale/trans.h"
#include  "ndr_downscale/buttons.h"
//*********************************
#include "../include/Eigen/Dense"
#include "../include/Eigen/Core"
#include <boost/bind.hpp>
#include <iostream> //??
#include <sstream> //??
#include <getopt.h> // ??
#include <list> //??

RT_TASK demo_task;
RT_TASK printt_task;
RT_TASK intial_task;

using Eigen::MatrixXd;

//************************************GLOBAL DEFINITION**********************************************
#define pi 3.14159265359
#define g_acc 9.81
#define grav 9.81
//using namespace powPMACcontrol_ns;
using Eigen::MatrixXd;
//*************************************ARM CONFIGURATION**********************************************
//Arm offset (m)
#define d1 0.371
#define d3 0.547
#define d5 0.484
#define d7 0.273



// Arm offset (m) for dyn eqns_issac
#define L1 0.371
#define L3 0.547
#define L5 0.484
#define L7 0.273

//revised 190818
#define L2 0.547
#define L4 0.484
#define L6 0.273



//Center of mass for each joint, from local coordinate (m)
#define lg1 0.247
#define lg2 0.32328
#define lg3 0.24043
#define lg4 0.52448
#define lg5 0.21711
#define lg6 0.46621
#define lg7 0.15735
#define lge 0.02301

//------------------data save -------------------------------------------

FILE *pData;



// terms for dyn eqns
double th1;
double th2;
double th3;
double th4;
double th5;
double th6;
double th7;

double dth1;
double dth2;
double dth3;
double dth4;
double dth5;
double dth6;
double dth7;

double ddth1;
double ddth2;
double ddth3;
double ddth4;
double ddth5;
double ddth6;
double ddth7;

//gear ratio
double kr1=160;
double kr2=160;
double kr3=160;
double kr4=160;
double kr5=160;
double kr6=160;
double kr7=120;


//motor inertia
double im1=0.24*0.0001;
double im1zz=0.24*0.0001;

double im2=1.447*0.0001;
double im2zz=1.447*0.0001;

double im3=0.437*0.0001;
double im3zz=0.437*0.0001;

double im4=0.437*0.0001;
double im4zz=0.437*0.0001;

double im5=0.24*0.0001;
double im5zz=0.24*0.0001;

double im6=0.24*0.0001;
double im6zz=0.24*0.0001;

double im7=0.0303*0.0001;
double im7zz=0.0303*0.0001;



//Center of mass for each joint, from local coordinate (m)
double LL1=L1-lg1;
double LL3=L3-lg3;
double LL5=L5-lg5;
double LL7=L7-lg7;

double LL2=-L3+lg3;
double LL4=-L5+lg5;
double LL6=-L7+lg7;

//Joint mass (kg)
double m[7];



//follwong modelling world file
double m1 = 23;
double m2 = 15;
double m3 = 24;
double m4 = 6.3;
double m5 = 6.74;
double m6 = 3.8;
double m7 = 0.808; // 3.8 + eef

//inertia for dyn
double ia1xx=0.2558;
double ia1yy=0.1516;
double ia1zz=0.1804;
double ia1xy=0;
double ia1xz=0;
double ia1yz=0;

double ia2xx=0.2235;
double ia2yy=0.1796;
double ia2zz=0.0707;
double ia2xy=0;
double ia2xz=0;
double ia2yz=0.0041;

double ia3xx=0.1651;
double ia3yy=0.0868;
double ia3zz=0.1268;
double ia3xy=0;
double ia3xz=0;
double ia3yz=0;

double ia4xx=0.097;
double ia4yy=0.0768;
double ia4zz=0.03;
double ia4xy=0;
double ia4xz=0;
double ia4yz=-0.0025;

double ia5xx=0.0228;
double ia5yy=0.003;
double ia5zz=0.0151;
double ia5xy=0;
double ia5xz=0;
double ia5yz=0;

double ia6xx=0.0371;
double ia6yy=0.0288;
double ia6zz=0.0141;
double ia6xy=0;
double ia6xz=0;
double ia6yz=0;

double ia7xx=0.0001;
double ia7yy=0.0001;
double ia7zz=0.0001;
double ia7xy=0;
double ia7xz=0;
double ia7yz=0;



double dr_posX;
double dr_posY;
double dr_posZ;

double Friction_I;

double Firction_tq[7]={0,0,0,0,0,0,0};


//**************************************FRTICTION FUCTION VARIABLES ************

double z[7] = {0,0,0,0,0,0,0};
double prev_z[7] = {0,0,0,0,0,0,0};
double z_dot[7] = {0,0,0,0,0,0,0};
double prev_z_dot[7] = {0,0,0,0,0,0,0};

double fun_s[7] = {0,0,0,0,0,0,0};


double Friction_LG[7] = {0,0,0,0,0,0,0};


//**************************************************************************************


/////////////////////////////////// teleoperation ///////////////////////////////////////////////////////////////
double Output_data[16];
int Buttons[2];
int Inkwell = 0;
int button1_click_time = 0;
int button1_si_click_time = 0;
int button2_si_click_time = 0;
int count_num = 0;
int delay_step = 0;

double hd_rotation_matrix[9]={0,0,0,0,0,0,0,0,0};
double hd_current_xyz[3]={0,0,0};
double hd_del_xyz[3]={0,0,0};
double hd_initial_xyz[3]={0,0,0};
double robot_initial_xyz[3]={0,0,0};
//double robot_current_xyz[3];
double robot_prev_xyz[3]={0,0,0};
double robot_del_xyz[3]={0,0,0};
//double th[7];
double haptic_del_xyz[3]={0,0,0};
double haptic_del_xyz_1[3]={0,0,0};
//double cur_vel[7];
//double cur_vel_1[7];
double Euler_r, Euler_b, Euler_a;
double Euler_scale=2;
//double cur_tau[7];
//double Tau[7];
float end_force[3];
double vd[7]; //desired velocity
double Tau_publish[7];
//double time_taken;
//double time_taken= 0;

double act_ang[7] = {0,0,0,0,0,0,0};
double act_ang_vel[7] = {0,0,0,0,0,0,0};

MatrixXd hd_initial_rotation(3, 3); 		//
MatrixXd hd_del_rotation(3, 3); 		//
MatrixXd haptic_del_rotation(3, 3); 		//
MatrixXd haptic_del_rotation_1(3, 3); 		//
MatrixXd hd_current_rotation(3, 3); 		//
MatrixXd robot_initial_rotation(3, 3); 		//
MatrixXd robot_current_rotation(3, 3); 		//


//**************************************prev PMAC DEFINITION***********************************************
int motor[7];

#define selected_PLC	8

double Kt[7];
/*
Kt[4]=0.3658;	//motor 5 torque constant (1/A)
Kt[5]=0.3658;	//motor 6 torque constant (1/A)
*/

double Nhd[7];
/*
Nhd[4]=160;	//Harmonic driver 5 gear ratio
Nhd[5]=160;	//Harmonic driver 5 gear ratio
*/

double Icon[7];
/*
Icon[4]=2.744;	//motor 5 driver continuous current (A)
Icon[5]=2.63;	//motor 6 driver continuous current (A)
*/

double Kp[7], Kd[7];	//P gain for joint i

double T_limit[7];	//Torque limit


double	current_joint_states[7];	//current joint angles _ from  msgCallbackP function
// euler angle
double  current_alpha;
double  current_beta;
double  current_gamma;

double	cur_vel[7];			//current joint vel _ from  msgCallbackv function
double	cur_tau[7];			//current joint torque _ from  msgCallbackT function
double	command_tau[7];			//command torque
double	robot_current_xyz[3];;		//current pos - forward kinematics
double time_taken;

// euler angle for initial position
double init_euler_beta = 0; // beta for calculating euler angle in rotation matrix
double init_euler_alpha = 0; // alpha for calculating euler angle in rotation matrix
double init_euler_gamma = 0; // gamma for calculating euler angle in rotation matrix

double eef_des_euler_beta = 0; // beta for calculating euler angle in rotation matrix
double eef_des_euler_alpha = 0; // alpha for calculating euler angle in rotation matrix
double eef_des_euler_gamma = 0; // gamma for calculating euler angle in rotation matrix

double SI,SI_COM,SI_HIG,SI_LOW,SI_FIX;
double q[7], q_pmac[7], th[7], q_pmac_update[7], dq_pmac_update[7];
const int N = 1000;		//Number of Cartesian steps
double	dt;			//incremental step time
//********************************Eigen Matrices Definition***********************************
//Given
MatrixXd P_INIT(4,4);	//Given init homogeneous transformation matrix
MatrixXd P_END(4,4);	//Given end homogeneous transformation matrix
MatrixXd P_COM(4,4);	//Given end homogeneous transformation matrix
//Temp matrices for Inverse Kinematics computation
MatrixXd W(3,1); 		//Wrist position relating to base
MatrixXd DZ(3,1);		//[0; 0; 1] matrix
MatrixXd L_SW(3,1); 	//Wrist position relating to shoulder
MatrixXd D1(3,1); 		//D1 column matrix
MatrixXd KSW(3,3); 		//Skew symetric matrix of vector between wrist and shoulder
MatrixXd USW(3,1); 		//Unit vector of L_SW
MatrixXd R03dot(3,3); 		//Rotation matrix of the ref joint angles q1dot, q2dot
MatrixXd XS(3,3); 		//Shoulder constant matrix
MatrixXd YS(3,3); 		//
MatrixXd ZS(3,3); 		//
MatrixXd I(3,3); 		//Identity matrix
MatrixXd I7(7,7); 		//Identity matrix 7x7
MatrixXd R34(3,3); 		//Rotation matrix from joint 3 to joint 4
MatrixXd XW(3,3); 		//Wrist constant matrix
MatrixXd YW(3,3); 		//
MatrixXd ZW(3,3); 		//
MatrixXd desired_theta(7,1);	//Desired joint positions
MatrixXd prev_desired_theta(7,1);	//Desired joint positions
MatrixXd desired_dtheta(7,1);	//Desired joint velocities
MatrixXd current_theta(7,1);	//Current joint position
MatrixXd current_dtheta(7,1);	//Current joint velocities
MatrixXd p_current_dtheta(7,1); // previous joint velocities


/*
MatrixXd ddth(7,1);             // joint acc
MatrixXd f_current_theta(7,1);
MatrixXd p_current_theta(7,1);
MatrixXd pa_current_theta(7,1);

MatrixXd f_current_vel(7,1);
MatrixXd a_current_vel(7,1);
MatrixXd a_current_theta(7,1);
MatrixXd p_current_vel(7,1);
MatrixXd pa_current_vel(7,1); */


MatrixXd theta_err1(7,1);	//Joint position error 1
MatrixXd n_theta_err1(7,1);	//Joint position error 1 null space
MatrixXd n_dtheta_err1(7,1);	//Joint position error 1 null space

MatrixXd theta_err2(7,1);	//Joint position error 2
//MatrixXd dtheta_err(7,1);	//Joint velocity error

MatrixXd Jacob(6,7);		//Jacobian matrix 6x7
MatrixXd diff_jacob(6,7);   // differentiation jacobian
MatrixXd Trans_Jacob(7,6);	//Jacobian transpose matrix 7x6
MatrixXd pinv_Jacob(7,6);	//Jacobian pseudo inverse matrix 7x6

MatrixXd Jacob_3(3,7); // jacibian only velocity 3x7
MatrixXd diff_jacob_3(3,7);   // differentiation jacobian 3x7


/*
//Cartesian stiffness matrix
MatrixXd Kx(6,6); 	//cartesian stiffness diagonal matrix
MatrixXd KxVec(6,1); 	//stiffness diagonal vector
MatrixXd Ki(7,7); 	//joint stiffness matrix
MatrixXd Kmin(7,7); 	//minimum joint stiffness matrix under mapping Kc -> Ki
MatrixXd Knull(7,7); 	//Null space joint stiffness under mapping Kc -> Ki
MatrixXd K0(7,7); 	//weighting matrix (symmetric positive definite) under mapping Kc -> Ki
MatrixXd K0Vec(7,1); 	//weighting matrix diagonal vector
*/
MatrixXd KnVec(7,1); 	//stiffness diagonal vector
MatrixXd DnVec(7,1); 	//damp diagonal vector
MatrixXd Kn(7,7);
MatrixXd Dn(7,7);

MatrixXd Prj_matrix(7,7);
//MatrixXd Prj_matrix_3(7,7);

//MatrixXd Dx(6,6); 	//cartesian damping diagonal matrix
//MatrixXd DxVec(6,1); 	//damping diagonal vector
//MatrixXd Di(7,7); 	//joint damping matrix




MatrixXd Tau_stif(7,1);		//Torque computed from stiffness control
MatrixXd Tau_stif_3(7,1);		//Torque computed from stiffness control

//MatrixXd Friction_I(7,1);	//Torque with friction compensator
MatrixXd Gravity_T(7,1);	//Torque with gravity compensator
MatrixXd Tau_I(7,1);		//Convert torque to Ampere command send to PMAC

/*
//computed torque control based on dyn eqns matrix
MatrixXd Up_L_matrix(7,22);	//regression matrix
MatrixXd cst_matrix(22,1);	//dynmaic parameters
MatrixXd Tau_I_cal(7,1); //Torque computed from model based control
*/

//dynamics terms
MatrixXd M_matrix(7,7); //mass matrix
MatrixXd G_matrix(7,1); //gravity matrix

//MatrixXd checkval(7,1);

//    MatrixXd act_position(6,1); //actual position _ cartesian
MatrixXd d_position(6,1); // desired position
//	MatrixXd d_position_3(3,1); // desired position
MatrixXd prev_d_position(6,1); // prev desired position
MatrixXd c_position(6,1); // current(actual) position
// MatrixXd c_position_3(3,1); // current(actual) position
//   MatrixXd pre_position(6,1); // current(actual) position
MatrixXd d_vel(6,1); // desired velocity
MatrixXd c_vel(6,1); // current velocity
MatrixXd d_acc(6,1);  // desired acceleration

MatrixXd pre_dvelocity(6,1);
MatrixXd c_dvelocity(6,1);

MatrixXd err_position(6,1); // error position

MatrixXd err_velocity(6,1); // error velocity

MatrixXd resolved_acc(6,1);  //resolved accerlation
MatrixXd acc_p(3,1);  //resolved accerlation transitional
MatrixXd acc_o(3,1);  //resolved accerlation orientation



MatrixXd err_position_3(3,1); // error position 3x1
MatrixXd prev_err_position_3(3,1); // error position 3x1

MatrixXd err_velocity_3(3,1); // error velocity 3x1
/*
MatrixXd err_acc(6,1); // error acc

MatrixXd pre_ang(7,1);
MatrixXd c_ang(7,1);
MatrixXd pre_ang_vel(7,1);
*/

MatrixXd ang_vel(7,1); // current anglular velocity matrix for y
MatrixXd ang_acc(7,1); // current anglular acceleration matrix for y

MatrixXd dr_ang_vel(7,1); //desired angular velocity

MatrixXd ROT_0d(3,3); // ROTATION frame 0 to selted frame
MatrixXd ROT_0dd(3,3); // ROTATION frame 0 to desired
MatrixXd ROT_0e(3,3); // ROTATION frame 0 to eef frame
MatrixXd ROT_ed(3,3); // ROTATION frame 0 to eef frame

//      MatrixXd dot_ROT_0d(3,3); // dt_ ROTATION
//      MatrixXd pr_ROT_0d(3,3); //previous rotation

//      MatrixXd b_mat(3,1); //b matrix -> 나중에 회전까지 고려하여 수정해야함.
//      MatrixXd dot_b_mat(3,1);
//       MatrixXd pr_b_mat(3,1);

MatrixXd h_0e(6,1); // force frame frome base to eef
MatrixXd h_0e_3(3,1); // force frame frome base to eef

//    MatrixXd h_de(6,1); // force from desired to acutal(current)

MatrixXd h_A(6,1);



/*
MatrixXd val_check(3,1); // check matrix.
MatrixXd val_check_1(3,1); // check matrix
MatrixXd val_check_2(3,1); // check matrix
MatrixXd val_check2(3,3); // check matrix
*/

MatrixXd Md_matrix_6(6,6);
MatrixXd kd_matrix_6(6,6);
MatrixXd kp_matrix_6(6,6);


MatrixXd Md_matrix_3(3,3);
MatrixXd kd_matrix_3(3,3);
MatrixXd kp_matrix_3(3,3);


MatrixXd input_y(7,1);
MatrixXd input_y_3(7,1);

MatrixXd ROT_temp(3,1);
MatrixXd ROT_vel_d(3,1);
MatrixXd ROT_vel_d_2(3,1);
MatrixXd ROT_pos_d(3,1);
MatrixXd ROT_pos_d_2(3,1);

MatrixXd ROT_vel_C(3,1);
MatrixXd ROT_vel_C_2(3,1);
MatrixXd ROT_pos_C(3,1);
MatrixXd ROT_pos_C_2(3,1);

//MatrixXd T_angvel_matrix(3,3); // transformation matrix for angvel to euler anges vel
// MatrixXd T_rot_matrix(3,3); // transformation matrix for ang to euler anges
MatrixXd T_de_matrix(3,3); // transformation matrix for euler to ang
MatrixXd T_e_de_matrix(3,3); // transformation matrix for euler to ang based eef frame
MatrixXd T_de_matrix_dot(3,3);
MatrixXd T_e_de_matrix_dot(3,3);

//	MatrixXd pre_euler(3,1);
//	MatrixXd pre_euler_2(3,1);

MatrixXd T01(4,4);
MatrixXd T12(4,4);
MatrixXd T23(4,4);
MatrixXd T34(4,4);
MatrixXd T45(4,4);
MatrixXd T56(4,4);
MatrixXd T67(4,4);
MatrixXd T7E(4,4);
MatrixXd T0E(4,4);

MatrixXd cartesian_Mass(6,6); // cacrtesian mass matrix
//    MatrixXd cartesian_Mass_3(3,3); // cacrtesian mass matrix

MatrixXd FT_TEMP(3,1); // EE frame force
MatrixXd FT_xyz(3,1); //  force xyz


MatrixXd checkval(7,1);
MatrixXd checkval_2(6,1);

double null[7];
double fric_amp_lim[7];
// global ros
ros::Publisher pub_jointt;

//*****************************************SUB PROGRAMS***********************************************

void delay_ms(int count)
{
	int i,j;
	for (i=0; i<count; i++)
	{
		for (j=0; j<1000; j++)
		{
		}
	}
}

//************************************** data *******************************************************
void DataSave(unsigned int count)
{
    pData = fopen("/home/user/catkin_ws/impedance_analysis_real.txt","a+");

    fprintf(pData,"%i", count);
    //fprintf(pData,"%f", timer);ㄴ



    //current position
    fprintf(pData," %f ", c_position(0,0));
    fprintf(pData," %f ", c_position(1,0));
    fprintf(pData," %f ", c_position(2,0));

    //desired position
    fprintf(pData," %f ", dr_posX); //desired x
    fprintf(pData," %f ", dr_posY);//desired y
    fprintf(pData," %f ", dr_posZ);//desired z

    //rotation error
    fprintf(pData," %f ", err_position_3(0,0)); //desired x
    fprintf(pData," %f ", err_position_3(1,0));//desired y
    fprintf(pData," %f ", err_position_3(2,0));//desired z

     //rotation vel error
    fprintf(pData," %f ", err_velocity_3(0,0)); //desired x
    fprintf(pData," %f ", err_velocity_3(1,0));//desired y
    fprintf(pData," %f ", err_velocity_3(2,0));//desired z

    //null position error
    fprintf(pData," %f ", n_theta_err1(0,0)); //desired x
    fprintf(pData," %f ", n_theta_err1(1,0));//desired y
    fprintf(pData," %f ", n_theta_err1(2,0));//desired z
    fprintf(pData," %f ", n_theta_err1(3,0)); //desired x
    fprintf(pData," %f ", n_theta_err1(4,0));//desired y
    fprintf(pData," %f ", n_theta_err1(5,0));//desired z
    fprintf(pData," %f ", n_theta_err1(6,0));//desired z

    //null VELOCITY error
    fprintf(pData," %f ", n_dtheta_err1(0,0)); //desired x
    fprintf(pData," %f ", n_dtheta_err1(1,0));//desired y
    fprintf(pData," %f ", n_dtheta_err1(2,0));//desired z
    fprintf(pData," %f ", n_dtheta_err1(3,0)); //desired x
    fprintf(pData," %f ", n_dtheta_err1(4,0));//desired y
    fprintf(pData," %f ", n_dtheta_err1(5,0));//desired z
    fprintf(pData," %f ", n_dtheta_err1(6,0));//desired z

    //null torque
    fprintf(pData," %f ", Tau_stif(0,0)); //desired x
    fprintf(pData," %f ", Tau_stif(1,0));//desired y
    fprintf(pData," %f ", Tau_stif(2,0));//desired z
    fprintf(pData," %f ", Tau_stif(3,0)); //desired x
    fprintf(pData," %f ", Tau_stif(4,0));//desired y
    fprintf(pData," %f ", Tau_stif(5,0));//desired z
    fprintf(pData," %f ", Tau_stif(6,0));//desired z

    //gravity torque
    fprintf(pData," %f ", G_matrix(0,0)); //desired x
    fprintf(pData," %f ", G_matrix(1,0));//desired y
    fprintf(pData," %f ", G_matrix(2,0));//desired z
    fprintf(pData," %f ", G_matrix(3,0)); //desired x
    fprintf(pData," %f ", G_matrix(4,0));//desired y
    fprintf(pData," %f ", G_matrix(5,0));//desired z
    fprintf(pData," %f ", G_matrix(6,0));//desired z

    //force sensor
    fprintf(pData, "%f ", h_0e_3(0,0));
    fprintf(pData," %f ", h_0e_3(1,0));
    fprintf(pData," %f ", h_0e_3(2,0));

    //input_y value
    fprintf(pData," %f ", input_y(0,0)); //desired x
    fprintf(pData," %f ", input_y(1,0));//desired y
    fprintf(pData," %f ", input_y(2,0));//desired z
    fprintf(pData," %f ", input_y(3,0)); //desired x
    fprintf(pData," %f ", input_y(4,0));//desired y
    fprintf(pData," %f ", input_y(5,0));//desired z


    //only impedance torque checkval
    fprintf(pData," %f ", checkval(0,0)); //desired x
    fprintf(pData," %f ", checkval(1,0));//desired y
    fprintf(pData," %f ", checkval(2,0));//desired z
    fprintf(pData," %f ", checkval(3,0)); //desired x
    fprintf(pData," %f ", checkval(4,0));//desired y
    fprintf(pData," %f ", checkval(5,0));//desired z
    fprintf(pData," %f ", checkval(6,0));//desired z

		// checkval_2_3_1=diff_jacob_3*ang_vel;
		//         checkval_3_3_1=Md_matrix_3.inverse()*h_0e_3;
		//          checkval_4_3_1=Md_matrix_3.inverse()*(kd_matrix_3*err_velocity_3 + kp_matrix_3*err_position_3);
		/*
		//diff_jacob_3*ang_vel check
		fprintf(pData," %f ", checkval_2_3_1(0,0)); //desired x
		fprintf(pData," %f ", checkval_2_3_1(1,0));//desired y
		fprintf(pData," %f ", checkval_2_3_1(2,0));//desired z

		//Md_matrix_3.inverse()*h_0e_3;
		fprintf(pData," %f ", checkval_3_3_1(0,0)); //desired x
		fprintf(pData," %f ", checkval_3_3_1(1,0));//desired y
		fprintf(pData," %f ", checkval_3_3_1(2,0));//desired z

		//Md_matrix_3.inverse()*h_0e_3; kd kp final value
		fprintf(pData," %f ", checkval_4_3_1(0,0)); //desired x
		fprintf(pData," %f ", checkval_4_3_1(1,0));//desired y
		fprintf(pData," %f ", checkval_4_3_1(2,0));//desired z

		//total torque
		fprintf(pData," %f ", checkval_4(0,0)); //desired x
		fprintf(pData," %f ", checkval_4(1,0));//desired y
		fprintf(pData," %f ", checkval_4(2,0));//desired z
		fprintf(pData," %f ", checkval_4(3,0)); //desired x
		fprintf(pData," %f ", checkval_4(4,0));//desired y
		fprintf(pData," %f ", checkval_4(5,0));//desired z
		fprintf(pData," %f ", checkval_4(6,0));//desired z
		*/


    //current angle
    fprintf(pData," %f ", current_theta(0,0)); //desired x
    fprintf(pData," %f ", current_theta(1,0));//desired y
    fprintf(pData," %f ", current_theta(2,0));//desired z
    fprintf(pData," %f ", current_theta(3,0)); //desired x
    fprintf(pData," %f ", current_theta(4,0));//desired y
    fprintf(pData," %f ", current_theta(5,0));//desired z
    fprintf(pData," %f ", current_theta(6,0));//desired z

    //desired angle

    fprintf(pData," %f ", q[0]); //desired x
    fprintf(pData," %f ", q[1]);//desired y
    fprintf(pData," %f ", q[2]);//desired z
    fprintf(pData," %f ", q[3]); //desired x
    fprintf(pData," %f ", q[4]);//desired y
    fprintf(pData," %f ", q[5]);//desired z
    fprintf(pData," %f ", q[6]);//desired z

	//cartesian impedance
    fprintf(pData," %f ", checkval_2(0,0)); //desired x
    fprintf(pData," %f ", checkval_2(1,0));//desired y
    fprintf(pData," %f ", checkval_2(2,0));//desired z
    fprintf(pData," %f ", checkval_2(3,0)); //desired x
    fprintf(pData," %f ", checkval_2(4,0));//desired y
    fprintf(pData," %f ", checkval_2(5,0));//desired z


    fprintf(pData,"\n");


    fclose(pData);

}


//****************************************sign function***********************************************

int sgn(double x)
{
	if (x > 0) return 1;
	if (x < 0) return -1;
	if (x == 0) return 0;
}

//**********************************Call Back Joint Position******************************************
void msgCallbacdataangle(const ndr_downscale::seven::ConstPtr &msg)
{

	current_joint_states[0] = msg->a;
	current_joint_states[1] = msg->b;
	current_joint_states[2] = msg->c;
	current_joint_states[3] = msg->d;
	current_joint_states[4] = msg->e;
	current_joint_states[5] = msg->f;
	current_joint_states[6] = msg->g;

	/*th[0] = current_joint_states[0];
	th[1] = current_joint_states[1];
	th[2] = current_joint_states[2];
	th[3] = current_joint_states[3];
	th[4] = current_joint_states[4];
	th[5] = current_joint_states[5];
	th[6] = current_joint_states[6]; */
}

//*********************************Call Back Joint Velocities*****************************************
void msgCallbacdataangvel(const ndr_downscale::seven::ConstPtr &msg)
{

	cur_vel[0] = msg->a;
	cur_vel[1] = msg->b;
	cur_vel[2] = msg->c;
	cur_vel[3] = msg->d;
	cur_vel[4] = msg->e;
	cur_vel[5] = msg->f;
	cur_vel[6] = msg->g;
}

//***********************************Call Back Joint Torque*******************************************

// force sensor not yet
void msgCallbackT(const ndr_downscale::six::ConstPtr &msg)
{

	cur_tau[0] = msg->a;
	cur_tau[1] = msg->b;
	cur_tau[2] = msg->c;
	cur_tau[3] = msg->d;
	cur_tau[4] = msg->e;
	cur_tau[5] = msg->f;

}


/////////////////////////////////// haptic device ////////////////////////////////////////////////////////

void hd_callback_trans(const ndr_downscale::trans::ConstPtr &msg) {

	Output_data[0] = msg->a;
	Output_data[1] = msg->b;
	Output_data[2] = msg->c;
	Output_data[3] = msg->d;
	Output_data[4] = msg->e;
	Output_data[5] = msg->f;
	Output_data[6] = msg->g;
	Output_data[7] = msg->h;
	Output_data[8] = msg->i;
	Output_data[9] = msg->j;
	Output_data[10] = msg->k;
	Output_data[11] = msg->l;
	Output_data[12] = msg->m;
	Output_data[13] = msg->n;
	Output_data[14] = msg->o;
	Output_data[15] = msg->p;

	/*
	hd_rotation_matrix[0] = Output_data[0];
	hd_rotation_matrix[1] = Output_data[1];
	hd_rotation_matrix[2] = Output_data[2];
	hd_rotation_matrix[3] = Output_data[4];
	hd_rotation_matrix[4] = Output_data[5];
	hd_rotation_matrix[5] = Output_data[6];
	hd_rotation_matrix[6] = Output_data[8];
	hd_rotation_matrix[7] = Output_data[9];
	hd_rotation_matrix[8] = Output_data[10];
	*/

	hd_rotation_matrix[0] = -Output_data[2];
	hd_rotation_matrix[1] = -Output_data[0];
	hd_rotation_matrix[2] = Output_data[1];
	hd_rotation_matrix[3] = -Output_data[6];
	hd_rotation_matrix[4] = -Output_data[4];
	hd_rotation_matrix[5] = Output_data[5];
	hd_rotation_matrix[6] = -Output_data[10];
	hd_rotation_matrix[7] = -Output_data[8];
	hd_rotation_matrix[8] = Output_data[9];


	//hd_current_xyz[0] = -Output_data[14];
	//hd_current_xyz[1] = -Output_data[12];
	//hd_current_xyz[2] = Output_data[13];

	hd_current_xyz[0] = Output_data[12]; // x
	hd_current_xyz[1] = Output_data[13]; // y
	hd_current_xyz[2] = Output_data[14]; // z

}


void hd_callback_buttons(const ndr_downscale::buttons::ConstPtr &msga) {

	Buttons[0] = msga->a;
	Buttons[1] = msga->b;
	Inkwell = msga->c;


	//std::cout << std::string(80, '-') << std::endl;

}

//********************************sending torque to sub system ***************************************
void jointt_publish(double tau[7])
{
	ndr_downscale::seven msgp;
	msgp.a = tau[0];
	msgp.b = tau[1];
	msgp.c = tau[2];
	msgp.d = tau[3];
	msgp.e = tau[4];
	msgp.f = tau[5];
	msgp.g = tau[6];

	pub_jointt.publish(msgp);

}


//*******************************Inverse Kinematics***************************************************
void InvK7(const Eigen::MatrixXd &data, double si) // input: pos, psi output: angles
{

	double q1dot, q2dot;
	double norm_L_SW;
	int GC2, GC4, GC6;
	//Given homogeneous transformation matrix

	DZ << 	0, 0, 1;
	D1 << 	0, 0, d1;
	I = I.setIdentity(3,3);

	//Compute wrist position relating to base from the given EE position end orientation
	W = data.topRightCorner(3,1)-d7*(data.topLeftCorner(3,3)*DZ);	//P(1:3,4)
	//Compute wrist position relating to shoulder
	L_SW = W - D1;
	norm_L_SW = L_SW.norm();
	//Elbow angle q4 in radian
	q[3] = acos((pow(norm_L_SW,2) - pow(d3,2) - pow(d5,2))/(2*d3*d5));
	if (q[3]>=0) GC4 = 1;
	else GC4 = -1;
	//Compute skew symetric matrix of the vector between wrist and shoulder:
	USW = L_SW/norm_L_SW;
	KSW << 	0, -USW(2), USW(1),
			USW(2), 0, -USW(0),
			-USW(1), USW(0), 0;
	//Set q3=0, compute reference joint angle q1dot, q2dot of the ref plane
	q1dot = atan2(W(1),W(0));
	q2dot = pi/2 - asin((W(2)-d1)/norm_L_SW) - acos((pow(d3,2)+pow(norm_L_SW,2)-pow(d5,2))/(2*d3*norm_L_SW));
	//Rotation matrix of the ref joint angles q1dot, q2dot:
	R03dot << 	cos(q1dot)*cos(q2dot),	-cos(q1dot)*sin(q2dot),	-sin(q1dot),
				cos(q2dot)*sin(q1dot),	-sin(q1dot)*sin(q2dot),	cos(q1dot),
				-sin(q2dot),			-cos(q2dot),			0;
	//Shoulder constant matrices Xs Ys Zs
	XS = KSW*R03dot;
	YS = -(KSW*KSW)*R03dot;
	ZS = (I+KSW*KSW)*R03dot;
	//constant matrices Xw Yw Zw
	R34 << 		cos(q[3]),		0,		sin(q[3]),
				sin(q[3]),		0, 		-cos(q[3]),
				0,				1,		0;
	XW = R34.transpose()*XS.transpose()*data.topLeftCorner(3,3);
	YW = R34.transpose()*YS.transpose()*data.topLeftCorner(3,3);
	ZW = R34.transpose()*ZS.transpose()*data.topLeftCorner(3,3);

	//Theta2
	q[1] = acos(-sin(si)*XS(2,1)-cos(si)*YS(2,1)-ZS(2,1));
	if (q[1]>=0)GC2 = 1;
	else GC2 = -1;
	//Theta1, theta3
	q[0] = atan2(GC2*(-sin(si)*XS(1,1)-cos(si)*YS(1,1)-ZS(1,1)),GC2*(-sin(si)*XS(0,1)-cos(si)*YS(0,1)-ZS(0,1)));
	q[2] = atan2(GC2*(sin(si)*XS(2,2)+cos(si)*YS(2,2)+ZS(2,2)),GC2*(-sin(si)*XS(2,0)-cos(si)*YS(2,0)-ZS(2,0)));

	//Theta6
	q[5] = acos(sin(si)*XW(2,2)+cos(si)*YW(2,2)+ZW(2,2));
	if (q[5]>=0)GC6 = 1;
	else GC6 = -1;
	//Theta5, theta7
	q[4] = atan2(GC6*(sin(si)*XW(1,2)+cos(si)*YW(1,2)+ZW(1,2)),GC6*(sin(si)*XW(0,2)+cos(si)*YW(0,2)+ZW(0,2)));
	q[6] = atan2(GC6*(sin(si)*XW(2,1)+cos(si)*YW(2,1)+ZW(2,1)),GC6*(-sin(si)*XW(2,0)-cos(si)*YW(2,0)-ZW(2,0)));

}

//***********************************Forward Kinematics***********************************************
void forwd7(double ang1, double ang2, double ang3, double ang4, double ang5 ,double ang6 ,double ang7) //// input: joint angles  output: position + rotation  need
{
	/*
		description
		forward kinemaitcs
		input -> angles
		output: position(x,y,z) , eulerangles(alpha,beta,gamma)
	 */



    T01 << 	cos(ang1), 	0, 	-sin(ang1), 	0,
                sin(ang1), 	0, 	cos(ang1), 	0,
                    0, 		-1,	 0			, d1,
				0,		0,	0			,	1;

    T12 << 	cos(ang2), 0, 	sin(ang2), 0,
                sin(ang2), 0,     -cos(ang2),0,
                	0,		 1,		 	0, 0,
				0,		0,		0	,	1;

    T23 << 	cos(ang3), 	0, 	-sin(ang3), 0,
                sin(ang3), 	0, 	cos(ang3),0,
                    0, 		-1, 0,		d3,
				0,		0,		0	,	1;

		T34 << 	cos(ang4), 0, 	sin(ang4), 	0,
                    sin(ang4), 0,     -cos(ang4), 	0,
                    	0,		 1,		 	0,0 ,
						0,		0,		0	,	1;

		T45 << 	cos(ang5), 	0, 	-sin(ang5), 0,
                    sin(ang5), 	0, 	cos(ang5), 0,
                        0, 		-1, 	0,	d5,
						0,		0,		0	,	1;

		T56 << 	cos(ang6), 0, 	sin(ang6), 	0,
                    sin(ang6), 0,     -cos(ang6),0,
                    	0,		 1,		 	0, 0,
						0,		0,		0	,	1;

		T67 << 	cos(ang7), -sin(ang7), 	0, 	0,
                    sin(ang7), cos(ang7), 	0,	0,
                        0, 		0, 		1, 	d7,
						 0,		0,		0	,	1;

    T7E << 	1, 	0, 	0, 0,
            0, 	1, 	0, 	0,
             0, 0, 	1, 0,
		 0,	0,	0,	1;

		T0E=T01*T12*T23*T34*T45*T56*T67*T7E;
	/*
	//Update theta from msgCallbackP function
	th1 = current_theta(0,0);
	th2 = current_theta(1,0);
	th3 = current_theta(2,0);
	th4 = current_theta(3,0);
	th5 = current_theta(4,0);
	th6 = current_theta(5,0);
	th7 = current_theta(6,0); */

	robot_current_xyz[0] = T0E(0,3);
	robot_current_xyz[1] = T0E(1,3);
	robot_current_xyz[2] = T0E(2,3);



}

// CURRENT VERSION
void Mass_Matrix(double th1, double th2, double th3, double th4, double th5, double th6, double th7)
{ //mass matrix calculation
			M_matrix(0,0)=(ia1yy + ia2zz + im1zz*pow(kr1,2) + (ia2xx - ia2zz + ia3zz + 2*L2*LL2*m2 + pow(LL2,2)*m2 + pow(L2,2)*(m2 + m3 + m4 + m5 + m6 + m7))*pow(sin(th2),2) +
							((ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*sin(th3)*
							(sin(th3) - pow(cos(th2),2)*sin(th3) + pow(sin(th2),2)*sin(th3)))/2. + (ia3yy + ia4zz)*L2*(2*cos(th4)*pow(sin(th2),2) + cos(th3)*sin(2*th2)*sin(th4)) +
							((ia5yy + ia6zz)*(3 + pow(cos(th4),2) + (pow(cos(th2),2)*(2 - 2*cos(2*th3) + cos(2*(th3 - th4)) + 6*cos(2*th4) + cos(2*(th3 + th4))))/2. -
								pow(sin(th2),2) - 3*pow(cos(th4),2)*pow(sin(th2),2) - pow(sin(th3),2) + pow(cos(th4),2)*pow(sin(th3),2) - pow(sin(th2),2)*pow(sin(th3),2) +
								pow(cos(th4),2)*pow(sin(th2),2)*pow(sin(th3),2) - 8*cos(th3)*cos(th4)*sin(2*th2)*sin(th4) - pow(sin(th4),2) -
								(-3 + cos(2*th2))*pow(cos(th3),2)*pow(sin(th4),2) + 3*pow(sin(th2),2)*pow(sin(th4),2) - pow(sin(th3),2)*pow(sin(th4),2) -
								pow(sin(th2),2)*pow(sin(th3),2)*pow(sin(th4),2)))/8. +
							(ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*
							(pow(cos(th3),2)*pow(cos(th4),2)*pow(sin(th2),2) + pow(cos(th2),2)*pow(sin(th4),2) + cos(th2)*cos(th3)*sin(th2)*sin(2*th4)) +
							(ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
							(pow(cos(th5),2)*pow(sin(th2),2)*pow(sin(th3),2) + 2*cos(th2)*cos(th5)*sin(th2)*sin(th3)*sin(th4)*sin(th5) +
							pow(cos(th2),2)*pow(sin(th4),2)*pow(sin(th5),2) - (pow(cos(th3),2)*pow(cos(th4),2)*sin(th2)*
								(pow(cos(th5),2)*sin(th2) - sin(th2)*(1 + pow(sin(th5),2))))/2. +
							(cos(th3)*cos(th4)*(8*cos(th2)*sin(th2)*sin(th4)*pow(sin(th5),2) + 4*pow(sin(th2),2)*sin(th3)*sin(2*th5)))/4.) +
							(ia5xx - ia5zz + ia6zz)*(pow(cos(th3),2)*pow(cos(th4),2)*pow(cos(th5),2)*pow(sin(th2),2) + pow(cos(th2),2)*pow(cos(th5),2)*pow(sin(th4),2) -
							2*cos(th2)*cos(th5)*sin(th2)*sin(th3)*sin(th4)*sin(th5) + pow(sin(th2),2)*pow(sin(th3),2)*pow(sin(th5),2) +
							(cos(th3)*(4*cos(th2)*cos(th4)*pow(cos(th5),2)*sin(th2)*sin(th4) - 2*cos(th4)*pow(sin(th2),2)*sin(th3)*sin(2*th5)))/2.) +
							((LL6*m6 + L6*(m6 + m7))*(cos(th6)*(10*L4 + 8*cos(th4)*(L2 + L2*pow(sin(th2),2)) +
									L4*pow(cos(th4),2)*(-3 + 5*pow(sin(th2),2) + cos(2*th3)*(1 + pow(sin(th2),2))) + 2*L4*pow(sin(th3),2) + 2*L4*pow(sin(th4),2) +
									2*L4*pow(sin(th3),2)*pow(sin(th4),2) + L4*pow(cos(th3),2)*
									(-2 + (-3 + cos(2*th4))*pow(sin(th2),2) + 2*pow(cos(th4),2)*(1 + pow(sin(th2),2)) - 2*pow(sin(th4),2)) +
									pow(cos(th2),2)*(-2*L4 - 8*L2*cos(th4) - L4*(5 + cos(2*th3))*pow(cos(th4),2) + L4*(-3 + cos(2*th4))*pow(sin(th3),2) + 6*L4*pow(sin(th4),2) +
									4*L4*pow(cos(th3),2)*pow(sin(th4),2)) + 2*pow(sin(th2),2)*(L4 - 3*L4*pow(sin(th4),2) + L4*pow(sin(th3),2)*(1 + pow(sin(th4),2))) +
									8*cos(th3)*(L2*sin(2*th2)*sin(th4) + L4*sin(2*th2)*sin(2*th4))) +
								(L4*pow(cos(th3),2)*(-3*cos(th5)*sin(2*th4) + cos(2*th2)*cos(th5)*sin(2*th4)) +
									2*(-4*L2*cos(th5)*sin(th4) + L4*cos(th5)*sin(2*th4) + L4*cos(th5)*pow(sin(th3),2)*sin(2*th4) -
									pow(sin(th2),2)*(-(L4*cos(th5)*pow(sin(th3),2)*sin(2*th4)) + cos(th5)*(4*L2*sin(th4) + 3*L4*sin(2*th4))) - 4*L2*sin(2*th2)*sin(th3)*sin(th5) -
									4*L4*cos(th4)*sin(2*th2)*sin(th3)*sin(th5)) + pow(cos(th2),2)*
									(8*L2*cos(th5)*sin(th4) + 5*L4*cos(th5)*sin(2*th4) + 2*L4*pow(cos(th3),2)*cos(th5)*sin(2*th4) + L4*cos(2*th3)*cos(th5)*sin(2*th4) -
									8*L4*cos(th3)*sin(th3)*sin(th4)*sin(th5)) + 8*cos(th3)*
									(cos(th5)*(L2*cos(th4)*sin(2*th2) + L4*pow(cos(th4),2)*sin(2*th2) - L4*sin(2*th2)*pow(sin(th4),2)) + L4*sin(th3)*sin(th4)*sin(th5) +
									L4*pow(sin(th2),2)*sin(th3)*sin(th4)*sin(th5)))*sin(th6)))/8. +
							(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
							(pow(cos(th6),2)*pow(sin(th2),2)*pow(sin(th3),2)*pow(sin(th5),2) + pow(cos(th2),2)*pow(cos(th5)*cos(th6)*sin(th4) + cos(th4)*sin(th6),2) +
							pow(cos(th3),2)*(pow(cos(th4),2)*pow(cos(th5),2)*pow(cos(th6),2)*pow(sin(th2),2) -
								2*cos(th4)*cos(th5)*cos(th6)*pow(sin(th2),2)*sin(th4)*sin(th6) + pow(sin(th2),2)*pow(sin(th4),2)*pow(sin(th6),2)) -
							(cos(th3)*(cos(th4)*(-4*cos(th2)*pow(cos(th5),2)*pow(cos(th6),2)*sin(th2)*sin(th4) + 2*pow(cos(th6),2)*pow(sin(th2),2)*sin(th3)*sin(2*th5)) -
									4*cos(th2)*pow(cos(th4),2)*cos(th5)*cos(th6)*sin(th2)*sin(th6) - 4*cos(th6)*pow(sin(th2),2)*sin(th3)*sin(th4)*sin(th5)*sin(th6) -
									2*cos(th2)*sin(th4)*(-2*cos(th5)*cos(th6)*sin(th2)*sin(th4)*sin(th6) - 2*cos(th4)*sin(th2)*pow(sin(th6),2))))/2. -
							(cos(th2)*(2*pow(cos(th6),2)*sin(th2)*sin(th3)*sin(th4)*sin(2*th5) + 2*cos(th4)*sin(th2)*sin(th3)*sin(th5)*sin(2*th6)))/2.));



		M_matrix(0,1)=((ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*cos(th3)*sin(th2)*sin(th3) - (ia3yy + ia4zz)*L2*cos(th2)*sin(th3)*sin(th4) +
							(ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*
							(-(cos(th3)*pow(cos(th4),2)*sin(th2)*sin(th3)) - cos(th2)*cos(th4)*sin(th3)*sin(th4)) +
							((ia5yy + ia6zz)*(-4*cos(th3)*(-(pow(cos(th4),2)*sin(th2)*sin(th3)) - ((-3 + cos(2*th4))*sin(th2)*sin(th3))/2.) + 4*cos(th2)*sin(th3)*sin(2*th4)))/8. +
							(ia5xx - ia5zz + ia6zz)*(-(cos(th2)*cos(th4)*pow(cos(th5),2)*sin(th3)*sin(th4)) - pow(cos(th3),2)*cos(th4)*cos(th5)*sin(th2)*sin(th5) +
							cos(th4)*cos(th5)*sin(th2)*pow(sin(th3),2)*sin(th5) + (cos(th3)*
								(-2*pow(cos(th4),2)*pow(cos(th5),2)*sin(th2)*sin(th3) + 2*(-(cos(th2)*cos(th5)*sin(th4)*sin(th5)) + sin(th2)*sin(th3)*pow(sin(th5),2))))/2.) +
							(ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
							(-(cos(th4)*cos(th5)*sin(th2)*pow(sin(th3),2)*sin(th5)) - cos(th2)*cos(th4)*sin(th3)*sin(th4)*pow(sin(th5),2) +
							(cos(th3)*(4*pow(cos(th5),2)*sin(th2)*sin(th3) + (pow(cos(th4),2)*sin(th2)*(4*pow(cos(th5),2)*sin(th3) + 2*(-3 + cos(2*th5))*sin(th3)))/2. +
									4*cos(th2)*cos(th5)*sin(th4)*sin(th5)))/4. + (pow(cos(th3),2)*cos(th4)*sin(th2)*sin(2*th5))/2.) +
							((LL6*m6 + L6*(m6 + m7))*(cos(th6)*(4*cos(th3)*sin(th2)*(2*L4*sin(th3) - 2*L4*pow(cos(th4),2)*sin(th3) + 2*L4*sin(th3)*pow(sin(th4),2)) -
									2*cos(th2)*(4*L2*sin(th3)*sin(th4) + 4*L4*sin(th3)*sin(2*th4))) +
								(8*L4*pow(cos(th3),2)*sin(th2)*sin(th4)*sin(th5) - 4*sin(th2)*(-(L4*cos(th5)*sin(2*th3)*sin(2*th4)) + 2*L4*pow(sin(th3),2)*sin(th4)*sin(th5)) +
									8*cos(th2)*(-(L4*pow(cos(th4),2)*cos(th5)*sin(th3)) + L4*cos(th5)*sin(th3)*pow(sin(th4),2) - L2*cos(th3)*sin(th5) -
									cos(th4)*(L2*cos(th5)*sin(th3) + L4*cos(th3)*sin(th5))))*sin(th6)))/8. +
							(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
							(cos(th4)*cos(th5)*pow(cos(th6),2)*sin(th2)*pow(sin(th3),2)*sin(th5) - cos(th6)*sin(th2)*pow(sin(th3),2)*sin(th4)*sin(th5)*sin(th6) +
							pow(cos(th3),2)*(-(cos(th4)*cos(th5)*pow(cos(th6),2)*sin(th2)*sin(th5)) + cos(th6)*sin(th2)*sin(th4)*sin(th5)*sin(th6)) -
							(cos(th2)*(2*cos(th4)*pow(cos(th5),2)*pow(cos(th6),2)*sin(th3)*sin(th4) + 2*pow(cos(th4),2)*cos(th5)*cos(th6)*sin(th3)*sin(th6) -
									2*cos(th5)*cos(th6)*sin(th3)*pow(sin(th4),2)*sin(th6) - 2*cos(th4)*sin(th3)*sin(th4)*pow(sin(th6),2)))/2. -
							(cos(th3)*(2*pow(cos(th4),2)*pow(cos(th5),2)*pow(cos(th6),2)*sin(th2)*sin(th3) + 2*cos(th2)*cos(th5)*pow(cos(th6),2)*sin(th4)*sin(th5) +
									2*sin(th2)*(-(pow(cos(th6),2)*sin(th3)*pow(sin(th5),2)) + sin(th3)*pow(sin(th4),2)*pow(sin(th6),2)) +
									cos(th4)*(2*cos(th2)*cos(th6)*sin(th5)*sin(th6) - 2*cos(th5)*sin(th2)*sin(th3)*sin(th4)*sin(2*th6))))/2.));



		M_matrix(0,2)=((ia3yy + ia4zz)*L2*cos(th3)*sin(th2)*sin(th4) + ((ia5yy + ia6zz)*
						(-8*cos(th3)*cos(th4)*sin(th2)*sin(th4) + 4*cos(th2)*(1 + pow(cos(th4),2) - pow(sin(th4),2))))/8. +
						(ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*(cos(th2)*pow(sin(th4),2) + (cos(th3)*sin(th2)*sin(2*th4))/2.) +
						(ia5xx - ia5zz + ia6zz)*(cos(th3)*cos(th4)*pow(cos(th5),2)*sin(th2)*sin(th4) + cos(th2)*pow(cos(th5),2)*pow(sin(th4),2) -
						cos(th5)*sin(th2)*sin(th3)*sin(th4)*sin(th5)) + (ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
						(cos(th5)*sin(th2)*sin(th3)*sin(th4)*sin(th5) + cos(th3)*cos(th4)*sin(th2)*sin(th4)*pow(sin(th5),2) -
						(cos(th2)*(2*pow(cos(th5),2)*(1 + pow(sin(th4),2)) - 2*(1 + pow(sin(th5),2) - 2*pow(cos(th4),2)*pow(sin(th5),2) +
								pow(sin(th4),2)*(1 + pow(sin(th5),2)))))/8.) + ((LL6*m6 + L6*(m6 + m7))*
						(cos(th6)*(-2*cos(th2)*(-4*L4 + 4*L4*pow(cos(th4),2) - 4*L4*pow(sin(th4),2)) + 4*cos(th3)*sin(th2)*(2*L2*sin(th4) + 2*L4*sin(2*th4))) +
							(8*cos(th3)*(L2*cos(th4) + L4*cos(2*th4))*cos(th5)*sin(th2) + 8*L4*cos(th2)*cos(th5)*sin(2*th4) +
								2*(-4*L2*sin(th2)*sin(th3)*sin(th5) - 4*L4*cos(th4)*sin(th2)*sin(th3)*sin(th5)))*sin(th6)))/8. +
						(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
						(-(cos(th5)*pow(cos(th6),2)*sin(th2)*sin(th3)*sin(th4)*sin(th5)) - cos(th4)*cos(th6)*sin(th2)*sin(th3)*sin(th5)*sin(th6) -
						(cos(th2)*(-2*pow(cos(th5),2)*pow(cos(th6),2)*pow(sin(th4),2) - 4*cos(th4)*cos(th5)*cos(th6)*sin(th4)*sin(th6) -
								2*pow(cos(th4),2)*pow(sin(th6),2)))/2. - (cos(th3)*
							(-2*cos(th4)*pow(cos(th5),2)*pow(cos(th6),2)*sin(th2)*sin(th4) - 2*pow(cos(th4),2)*cos(th5)*cos(th6)*sin(th2)*sin(th6) +
								2*cos(th5)*cos(th6)*sin(th2)*pow(sin(th4),2)*sin(th6) + 2*cos(th4)*sin(th2)*sin(th4)*pow(sin(th6),2)))/2.));



		M_matrix(0,3)=((ia4yy + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*sin(th2)*sin(th3) + (ia3yy + ia4zz)*L2*cos(th4)*sin(th2)*sin(th3) +
						(ia5xx - ia5zz + ia6zz)*(-(cos(th3)*cos(th4)*cos(th5)*sin(th2)*sin(th5)) - cos(th2)*cos(th5)*sin(th4)*sin(th5) + sin(th2)*sin(th3)*pow(sin(th5),2)) +
						(ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
						(cos(th3)*cos(th4)*cos(th5)*sin(th2)*sin(th5) + cos(th2)*cos(th5)*sin(th4)*sin(th5) - (sin(th2)*sin(th3)*(-2 - 2*pow(cos(th5),2) + 2*pow(sin(th5),2)))/4.)\
						+ ((LL6*m6 + L6*(m6 + m7))*(cos(th6)*(16*L4*sin(th2)*sin(th3) + 8*L2*cos(th4)*sin(th2)*sin(th3)) +
							(-8*L2*cos(th5)*sin(th2)*sin(th3)*sin(th4) - 8*L4*cos(th2)*cos(th4)*sin(th5) + 8*L4*cos(th3)*sin(th2)*sin(th4)*sin(th5))*sin(th6)))/8. +
						(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
						(pow(cos(th6),2)*sin(th2)*sin(th3)*pow(sin(th5),2) - (cos(th2)*(2*cos(th5)*pow(cos(th6),2)*sin(th4)*sin(th5) + 2*cos(th4)*cos(th6)*sin(th5)*sin(th6)))/
							2. - (cos(th3)*(2*cos(th4)*cos(th5)*pow(cos(th6),2)*sin(th2)*sin(th5) - 2*cos(th6)*sin(th2)*sin(th4)*sin(th5)*sin(th6)))/2.));



		M_matrix(0,4)=(((ia5yy + ia6zz)*(8*cos(th2)*cos(th4) - 8*cos(th3)*sin(th2)*sin(th4)))/8. +
						((LL6*m6 + L6*(m6 + m7))*(8*cos(th3)*(L2 + L4*cos(th4))*cos(th5)*sin(th2) + 8*L4*cos(th2)*cos(th5)*sin(th4) +
							2*(-4*L4*sin(th2)*sin(th3)*sin(th5) - 4*L2*cos(th4)*sin(th2)*sin(th3)*sin(th5)))*sin(th6))/8. +
						(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
						(-(cos(th6)*sin(th2)*sin(th3)*sin(th5)*sin(th6)) - (cos(th2)*(-2*cos(th5)*cos(th6)*sin(th4)*sin(th6) - 2*cos(th4)*pow(sin(th6),2)))/2. -
						(cos(th3)*(2*sin(th2)*sin(th4)*pow(sin(th6),2) - cos(th4)*cos(th5)*sin(th2)*sin(2*th6)))/2.));



		M_matrix(0,5)=((ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
						(cos(th5)*sin(th2)*sin(th3) + cos(th3)*cos(th4)*sin(th2)*sin(th5) + cos(th2)*sin(th4)*sin(th5)) +
						((LL6*m6 + L6*(m6 + m7))*(cos(th6)*(8*L4*cos(th5)*sin(th2)*sin(th3) + 8*L2*cos(th4)*cos(th5)*sin(th2)*sin(th3) + 8*L4*cos(th2)*sin(th4)*sin(th5) +
								4*cos(th3)*sin(th2)*(2*L2*sin(th5) + 2*L4*cos(th4)*sin(th5))) - 8*L2*sin(th2)*sin(th3)*sin(th4)*sin(th6)))/8.);



		M_matrix(0,6)=0;






		M_matrix(1,0)=((-4*cos(th2)*(2*L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7)) + cos(th4)*(2*ia4xx - 2*ia4zz + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 +
								pow(LL6,2)*m6 + 2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7 - (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*th5)))*sin(th3)*sin(th4) +
						4*(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*pow(cos(th3),2)*cos(th4)*sin(th2)*sin(2*th5) +
						sin(th2)*(sin(2*th3)*(4*ia4yy + 4*ia5zz + ia6yy + ia7yy + 4*pow(L4,2)*m4 + 8*L4*LL4*m4 + 4*pow(LL4,2)*m4 + 4*pow(L4,2)*m5 + 4*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + 4*pow(L4,2)*m7 + L6*m7 +
								pow(L6,2)*m7 + 6*L6*LL6*m6*pow(cos(th5),2) + pow(cos(th4),2)*(-8*L4*LL4*m4 + (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*pow(cos(th5),2) + L6*LL6*m6*cos(2*th5)) +
								ia6yy*pow(sin(th4),2) + ia7yy*pow(sin(th4),2) + L6*m6*pow(sin(th4),2) + pow(L6,2)*m6*pow(sin(th4),2) + LL6*m6*pow(sin(th4),2) + 2*L6*LL6*m6*pow(sin(th4),2) + pow(LL6,2)*m6*pow(sin(th4),2) + L6*m7*pow(sin(th4),2) +
								pow(L6,2)*m7*pow(sin(th4),2) - 6*L6*LL6*m6*pow(sin(th5),2) + ia6yy*pow(sin(th4),2)*pow(sin(th5),2) + ia7yy*pow(sin(th4),2)*pow(sin(th5),2) + L6*m6*pow(sin(th4),2)*pow(sin(th5),2) +
								pow(L6,2)*m6*pow(sin(th4),2)*pow(sin(th5),2) + LL6*m6*pow(sin(th4),2)*pow(sin(th5),2) + 2*L6*LL6*m6*pow(sin(th4),2)*pow(sin(th5),2) + pow(LL6,2)*m6*pow(sin(th4),2)*pow(sin(th5),2) +
								L6*m7*pow(sin(th4),2)*pow(sin(th5),2) + pow(L6,2)*m7*pow(sin(th4),2)*pow(sin(th5),2)) - 4*(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(th4)*pow(sin(th3),2)*sin(2*th5)
							) + cos(th3)*((5*ia6yy + 5*ia7yy + 5*L6*m6 + 5*pow(L6,2)*m6 + 5*LL6*m6 - 2*L6*LL6*m6 + 5*pow(LL6,2)*m6 + 5*L6*m7 + 5*pow(L6,2)*m7 +
								(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*th4))*pow(cos(th5),2)*sin(th2)*sin(th3) -
							pow(cos(th4),2)*(8*ia4xx - 8*ia4zz + 8*ia5zz + 3*ia6yy + 3*ia7yy + 8*pow(L4,2)*m4 + 8*pow(LL4,2)*m4 + 8*pow(L4,2)*m5 + 8*pow(L4,2)*m6 + 3*L6*m6 + 3*pow(L6,2)*m6 + 3*LL6*m6 + 4*L6*LL6*m6 + 3*pow(LL6,2)*m6 + 8*pow(L4,2)*m7 +
								3*L6*m7 + 3*pow(L6,2)*m7 - (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*th5))*sin(th2)*sin(th3) - 6*ia6yy*sin(th2)*sin(th3)*pow(sin(th5),2) -
							6*ia7yy*sin(th2)*sin(th3)*pow(sin(th5),2) - 6*L6*m6*sin(th2)*sin(th3)*pow(sin(th5),2) - 6*pow(L6,2)*m6*sin(th2)*sin(th3)*pow(sin(th5),2) - 6*LL6*m6*sin(th2)*sin(th3)*pow(sin(th5),2) -
							6*pow(LL6,2)*m6*sin(th2)*sin(th3)*pow(sin(th5),2) - 6*L6*m7*sin(th2)*sin(th3)*pow(sin(th5),2) - 6*pow(L6,2)*m7*sin(th2)*sin(th3)*pow(sin(th5),2) + 8*L6*LL6*m6*cos(th2)*sin(th4)*sin(2*th5) +
							4*(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(th2)*sin(th4)*sin(2*th5)))/8.);



		M_matrix(1,1)=((32*ia2yy + 32*ia3zz + 8*ia4xx + 16*ia4yy - 8*ia4zz + 24*ia5zz + 12*ia6yy + 12*ia7yy + 32*im2zz*pow(kr2,2) + 32*pow(L2,2)*m2 + 64*L2*LL2*m2 + 32*pow(LL2,2)*m2 + 32*pow(L2,2)*m3 + 32*pow(L2,2)*m4 + 24*pow(L4,2)*m4 + 48*L4*LL4*m4 +
						24*pow(LL4,2)*m4 + 32*pow(L2,2)*m5 + 24*pow(L4,2)*m5 + 32*pow(L2,2)*m6 + 24*pow(L4,2)*m6 + 12*L6*m6 + 12*pow(L6,2)*m6 + 12*LL6*m6 + 24*L6*LL6*m6 + 12*pow(LL6,2)*m6 + 32*pow(L2,2)*m7 + 24*pow(L4,2)*m7 + 12*L6*m7 +
						12*pow(L6,2)*m7 + 4*(-2*ia4xx + 4*ia4yy + 2*ia4zz + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 +
							2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7)*cos(2*th3) - 2*(2*ia4xx - 2*ia4zz + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 +
							pow(LL6,2)*m6 + 2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7)*cos(2*(th3 - th4)) + 64*L2*L4*m4*cos(th4) + 64*L2*LL4*m4*cos(th4) + 64*L2*L4*m5*cos(th4) + 64*L2*L4*m6*cos(th4) + 64*L2*L4*m7*cos(th4) + 8*ia4xx*cos(2*th4) - 8*ia4zz*cos(2*th4) +
						8*ia5zz*cos(2*th4) + 4*ia6yy*cos(2*th4) + 4*ia7yy*cos(2*th4) + 8*pow(L4,2)*m4*cos(2*th4) + 16*L4*LL4*m4*cos(2*th4) + 8*pow(LL4,2)*m4*cos(2*th4) + 8*pow(L4,2)*m5*cos(2*th4) + 8*pow(L4,2)*m6*cos(2*th4) + 4*L6*m6*cos(2*th4) +
						4*pow(L6,2)*m6*cos(2*th4) + 4*LL6*m6*cos(2*th4) + 8*L6*LL6*m6*cos(2*th4) + 4*pow(LL6,2)*m6*cos(2*th4) + 8*pow(L4,2)*m7*cos(2*th4) + 4*L6*m7*cos(2*th4) + 4*pow(L6,2)*m7*cos(2*th4) - 4*ia4xx*cos(2*(th3 + th4)) + 4*ia4zz*cos(2*(th3 + th4)) -
						4*ia5zz*cos(2*(th3 + th4)) - 2*ia6yy*cos(2*(th3 + th4)) - 2*ia7yy*cos(2*(th3 + th4)) - 4*pow(L4,2)*m4*cos(2*(th3 + th4)) - 8*L4*LL4*m4*cos(2*(th3 + th4)) - 4*pow(LL4,2)*m4*cos(2*(th3 + th4)) - 4*pow(L4,2)*m5*cos(2*(th3 + th4)) -
						4*pow(L4,2)*m6*cos(2*(th3 + th4)) - 2*L6*m6*cos(2*(th3 + th4)) - 2*pow(L6,2)*m6*cos(2*(th3 + th4)) - 2*LL6*m6*cos(2*(th3 + th4)) - 4*L6*LL6*m6*cos(2*(th3 + th4)) - 2*pow(LL6,2)*m6*cos(2*(th3 + th4)) - 4*pow(L4,2)*m7*cos(2*(th3 + th4)) -
						2*L6*m7*cos(2*(th3 + th4)) - 2*pow(L6,2)*m7*cos(2*(th3 + th4)) - 4*ia6yy*cos(2*th3 - th4 - 2*th5) - 4*ia7yy*cos(2*th3 - th4 - 2*th5) - 4*L6*m6*cos(2*th3 - th4 - 2*th5) - 4*pow(L6,2)*m6*cos(2*th3 - th4 - 2*th5) -
						4*LL6*m6*cos(2*th3 - th4 - 2*th5) - 8*L6*LL6*m6*cos(2*th3 - th4 - 2*th5) - 4*pow(LL6,2)*m6*cos(2*th3 - th4 - 2*th5) - 4*L6*m7*cos(2*th3 - th4 - 2*th5) - 4*pow(L6,2)*m7*cos(2*th3 - th4 - 2*th5) - 4*ia6yy*cos(2*th3 + th4 - 2*th5) -
						4*ia7yy*cos(2*th3 + th4 - 2*th5) - 4*L6*m6*cos(2*th3 + th4 - 2*th5) - 4*pow(L6,2)*m6*cos(2*th3 + th4 - 2*th5) - 4*LL6*m6*cos(2*th3 + th4 - 2*th5) - 8*L6*LL6*m6*cos(2*th3 + th4 - 2*th5) - 4*pow(LL6,2)*m6*cos(2*th3 + th4 - 2*th5) -
						4*L6*m7*cos(2*th3 + th4 - 2*th5) - 4*pow(L6,2)*m7*cos(2*th3 + th4 - 2*th5) + 6*ia6yy*cos(2*(th3 - th5)) + 6*ia7yy*cos(2*(th3 - th5)) + 6*L6*m6*cos(2*(th3 - th5)) + 6*pow(L6,2)*m6*cos(2*(th3 - th5)) + 6*LL6*m6*cos(2*(th3 - th5)) +
						12*L6*LL6*m6*cos(2*(th3 - th5)) + 6*pow(LL6,2)*m6*cos(2*(th3 - th5)) + 6*L6*m7*cos(2*(th3 - th5)) + 6*pow(L6,2)*m7*cos(2*(th3 - th5)) + ia6yy*cos(2*(th3 - th4 - th5)) + ia7yy*cos(2*(th3 - th4 - th5)) + L6*m6*cos(2*(th3 - th4 - th5)) +
						pow(L6,2)*m6*cos(2*(th3 - th4 - th5)) + LL6*m6*cos(2*(th3 - th4 - th5)) + 2*L6*LL6*m6*cos(2*(th3 - th4 - th5)) + pow(LL6,2)*m6*cos(2*(th3 - th4 - th5)) + L6*m7*cos(2*(th3 - th4 - th5)) + pow(L6,2)*m7*cos(2*(th3 - th4 - th5)) -
						2*ia6yy*cos(2*(th4 - th5)) - 2*ia7yy*cos(2*(th4 - th5)) - 2*L6*m6*cos(2*(th4 - th5)) - 2*pow(L6,2)*m6*cos(2*(th4 - th5)) - 2*LL6*m6*cos(2*(th4 - th5)) - 4*L6*LL6*m6*cos(2*(th4 - th5)) - 2*pow(LL6,2)*m6*cos(2*(th4 - th5)) -
						2*L6*m7*cos(2*(th4 - th5)) - 2*pow(L6,2)*m7*cos(2*(th4 - th5)) + ia6yy*cos(2*(th3 + th4 - th5)) + ia7yy*cos(2*(th3 + th4 - th5)) + L6*m6*cos(2*(th3 + th4 - th5)) + pow(L6,2)*m6*cos(2*(th3 + th4 - th5)) + LL6*m6*cos(2*(th3 + th4 - th5)) +
						2*L6*LL6*m6*cos(2*(th3 + th4 - th5)) + pow(LL6,2)*m6*cos(2*(th3 + th4 - th5)) + L6*m7*cos(2*(th3 + th4 - th5)) + pow(L6,2)*m7*cos(2*(th3 + th4 - th5)) + 4*ia6yy*cos(2*th5) + 4*ia7yy*cos(2*th5) + 4*L6*m6*cos(2*th5) + 4*pow(L6,2)*m6*cos(2*th5) +
						4*LL6*m6*cos(2*th5) + 8*L6*LL6*m6*cos(2*th5) + 4*pow(LL6,2)*m6*cos(2*th5) + 4*L6*m7*cos(2*th5) + 4*pow(L6,2)*m7*cos(2*th5) + 6*ia6yy*cos(2*(th3 + th5)) + 6*ia7yy*cos(2*(th3 + th5)) + 6*L6*m6*cos(2*(th3 + th5)) +
						6*pow(L6,2)*m6*cos(2*(th3 + th5)) + 6*LL6*m6*cos(2*(th3 + th5)) + 12*L6*LL6*m6*cos(2*(th3 + th5)) + 6*pow(LL6,2)*m6*cos(2*(th3 + th5)) + 6*L6*m7*cos(2*(th3 + th5)) + 6*pow(L6,2)*m7*cos(2*(th3 + th5)) + ia6yy*cos(2*(th3 - th4 + th5)) +
						ia7yy*cos(2*(th3 - th4 + th5)) + L6*m6*cos(2*(th3 - th4 + th5)) + pow(L6,2)*m6*cos(2*(th3 - th4 + th5)) + LL6*m6*cos(2*(th3 - th4 + th5)) + 2*L6*LL6*m6*cos(2*(th3 - th4 + th5)) + pow(LL6,2)*m6*cos(2*(th3 - th4 + th5)) +
						L6*m7*cos(2*(th3 - th4 + th5)) + pow(L6,2)*m7*cos(2*(th3 - th4 + th5)) - 2*ia6yy*cos(2*(th4 + th5)) - 2*ia7yy*cos(2*(th4 + th5)) - 2*L6*m6*cos(2*(th4 + th5)) - 2*pow(L6,2)*m6*cos(2*(th4 + th5)) - 2*LL6*m6*cos(2*(th4 + th5)) -
						4*L6*LL6*m6*cos(2*(th4 + th5)) - 2*pow(LL6,2)*m6*cos(2*(th4 + th5)) - 2*L6*m7*cos(2*(th4 + th5)) - 2*pow(L6,2)*m7*cos(2*(th4 + th5)) + ia6yy*cos(2*(th3 + th4 + th5)) + ia7yy*cos(2*(th3 + th4 + th5)) + L6*m6*cos(2*(th3 + th4 + th5)) +
						pow(L6,2)*m6*cos(2*(th3 + th4 + th5)) + LL6*m6*cos(2*(th3 + th4 + th5)) + 2*L6*LL6*m6*cos(2*(th3 + th4 + th5)) + pow(LL6,2)*m6*cos(2*(th3 + th4 + th5)) + L6*m7*cos(2*(th3 + th4 + th5)) + pow(L6,2)*m7*cos(2*(th3 + th4 + th5)) +
						4*ia6yy*cos(2*th3 - th4 + 2*th5) + 4*ia7yy*cos(2*th3 - th4 + 2*th5) + 4*L6*m6*cos(2*th3 - th4 + 2*th5) + 4*pow(L6,2)*m6*cos(2*th3 - th4 + 2*th5) + 4*LL6*m6*cos(2*th3 - th4 + 2*th5) + 8*L6*LL6*m6*cos(2*th3 - th4 + 2*th5) +
						4*pow(LL6,2)*m6*cos(2*th3 - th4 + 2*th5) + 4*L6*m7*cos(2*th3 - th4 + 2*th5) + 4*pow(L6,2)*m7*cos(2*th3 - th4 + 2*th5) + 4*ia6yy*cos(2*th3 + th4 + 2*th5) + 4*ia7yy*cos(2*th3 + th4 + 2*th5) + 4*L6*m6*cos(2*th3 + th4 + 2*th5) +
						4*pow(L6,2)*m6*cos(2*th3 + th4 + 2*th5) + 4*LL6*m6*cos(2*th3 + th4 + 2*th5) + 8*L6*LL6*m6*cos(2*th3 + th4 + 2*th5) + 4*pow(LL6,2)*m6*cos(2*th3 + th4 + 2*th5) + 4*L6*m7*cos(2*th3 + th4 + 2*th5) + 4*pow(L6,2)*m7*cos(2*th3 + th4 + 2*th5))/32.);



		M_matrix(1,2)=((sin(th4)*((-4*L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7)) - 2*cos(th4)*(2*ia4xx - 2*ia4zz + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 +
							pow(LL6,2)*m6 + 2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7 - (ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*th5)))*sin(th3) +
						2*(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(th3)*sin(2*th5)))/4.);


		M_matrix(1,3)=((cos(th3)*(2*ia4yy + 2*ia5zz + ia6yy + ia7yy + 2*pow(L4,2)*m4 + 4*L4*LL4*m4 + 2*pow(LL4,2)*m4 + 2*pow(L4,2)*m5 + 2*pow(L4,2)*m6 + L6*m6 + pow(L6,2)*m6 +
						LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + 2*pow(L4,2)*m7 + L6*m7 + pow(L6,2)*m7 + 2*L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*cos(th4) +
						(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(2*th5)) -
					(ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*cos(th4)*sin(th3)*sin(2*th5))/2.);


		M_matrix(1,4)=0;


		M_matrix(1,5)=((ia6yy + ia7yy + L6*m6 + pow(L6,2)*m6 + LL6*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + L6*m7 + pow(L6,2)*m7)*(cos(th3)*cos(th5) - cos(th4)*sin(th3)*sin(th5)));


		M_matrix(1,6)=0;





		M_matrix(2,0)=(im3zz*kr3*cos(th2) + L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*cos(th3)*sin(th2)*sin(th4) +
						(ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*sin(th4)*(cos(th3)*cos(th4)*sin(th2) + cos(th2)*sin(th4)) +
						((ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*sin(th5)*
						(2*cos(th5)*sin(th2)*sin(th3)*sin(th4) + (2*cos(th2)*pow(sin(th4),2) + cos(th3)*sin(th2)*sin(2*th4))*sin(th5)))/2. +
						((LL6*m6 + L6*(m6 + m7))*(2*cos(th6)*sin(th4)*(cos(th3)*(L2 + 2*L4*cos(th4))*sin(th2) + 2*L4*cos(th2)*sin(th4)) +
							2*(cos(th3)*(L2*cos(th4) + L4*cos(2*th4))*cos(th5)*sin(th2) + L4*cos(th2)*cos(th5)*sin(2*th4) - (L2 + L4*cos(th4))*sin(th2)*sin(th3)*sin(th5))*sin(th6)))/2. +
						(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*(cos(th5)*cos(th6)*sin(th4) + cos(th4)*sin(th6))*
						(-(cos(th6)*sin(th2)*sin(th3)*sin(th5)) + cos(th2)*(cos(th5)*cos(th6)*sin(th4) + cos(th4)*sin(th6)) +
						cos(th3)*sin(th2)*(cos(th4)*cos(th5)*cos(th6) - sin(th4)*sin(th6))));


		M_matrix(2,1)=(-(L2*(LL4*m4 + L4*(m4 + m5 + m6 + m7))*sin(th3)*sin(th4)) - (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*cos(th4)*
						sin(th3)*sin(th4) + ((ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
						(-8*sin(th3)*sin(2*th4)*pow(sin(th5),2) + 8*cos(th3)*sin(th4)*sin(2*th5)))/16. -
						(ia6xx - ia6zz + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*(cos(th5)*cos(th6)*sin(th4) + cos(th4)*sin(th6))*
						(cos(th4)*cos(th5)*cos(th6)*sin(th3) + cos(th3)*cos(th6)*sin(th5) - sin(th3)*sin(th4)*sin(th6)) -
						(LL6*m6 + L6*(m6 + m7))*((L2 + 2*L4*cos(th4))*cos(th6)*sin(th3)*sin(th4) +
						(L4*pow(cos(th4),2)*cos(th5)*sin(th3) - L4*cos(th5)*sin(th3)*pow(sin(th4),2) + L2*cos(th3)*sin(th5) +
							cos(th4)*(L2*cos(th5)*sin(th3) + L4*cos(th3)*sin(th5)))*sin(th6)));



		M_matrix(2,2)=(im3zz*pow(kr3,2) + (ia4xx - ia4zz + ia5zz + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*(m4 + m5 + m6 + m7))*pow(sin(th4),2) +
						(ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*pow(sin(th4),2)*pow(sin(th5),2) +
						(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*pow(cos(th5)*cos(th6)*sin(th4) + cos(th4)*sin(th6),2) +
						L4*(LL6*m6 + L6*(m6 + m7))*(2*cos(th6)*pow(sin(th4),2) + cos(th5)*sin(2*th4)*sin(th6)));


		M_matrix(2,3)=(((ia6yy + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*sin(th4)*sin(2*th5))/2. - L4*(LL6*m6 + L6*(m6 + m7))*cos(th4)*sin(th5)*sin(th6) -
						(ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*cos(th6)*sin(th5)*(cos(th5)*cos(th6)*sin(th4) + cos(th4)*sin(th6)));



		M_matrix(2,4)=(sin(th6)*(L4*(LL6*m6 + L6*(m6 + m7))*cos(th5)*sin(th4) + (ia6xx - ia6zz + ia7yy + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7))*
						(cos(th5)*cos(th6)*sin(th4) + cos(th4)*sin(th6))));


		M_matrix(2,5)=((ia6yy + ia7yy + im6zz*kr6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*(m6 + m7) + L4*(LL6*m6 + L6*(m6 + m7))*cos(th6))*sin(th4)*sin(th5));

		M_matrix(2,6)=0;





		M_matrix(3,0)=(cos(th2)*((ia6yy + ia7yy + pow(L6,2)*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*cos(th5)*sin(th4)*sin(th5) + L6*LL6*m6*sin(th4)*sin(2*th5) -
						L4*(LL6*m6 + L6*(m6 + m7))*cos(th4)*sin(th5)*sin(th6)) + sin(th2)*
						(cos(th3)*sin(th5)*((ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*cos(th4)*cos(th5) +
							L4*(LL6*m6 + L6*(m6 + m7))*sin(th4)*sin(th6)) + sin(th3)*(ia4yy + ia5zz + im4zz*kr4 + pow(L4,2)*m4 + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*m5 +
							pow(L4,2)*m6 + pow(L4,2)*m7 + (ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*pow(cos(th5),2) + 2*L4*L6*m6*cos(th6) +
							2*L4*LL6*m6*cos(th6) + 2*L4*L6*m7*cos(th6) + L2*cos(th4)*(LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(th6)) -
							L2*(LL6*m6 + L6*(m6 + m7))*cos(th5)*sin(th4)*sin(th6))));


		M_matrix(3,1)=(-(sin(th3)*sin(th5)*((ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*cos(th4)*cos(th5) +
							L4*(LL6*m6 + L6*(m6 + m7))*sin(th4)*sin(th6))) + cos(th3)*(ia4yy + ia5zz + im4zz*kr4 + pow(L4,2)*m4 + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*m5 +
						pow(L4,2)*m6 + pow(L4,2)*m7 + (ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*pow(cos(th5),2) + 2*L4*L6*m6*cos(th6) +
						2*L4*LL6*m6*cos(th6) + 2*L4*L6*m7*cos(th6) + L2*cos(th4)*(LL4*m4 + L4*(m4 + m5 + m6 + m7) + (LL6*m6 + L6*(m6 + m7))*cos(th6)) -
						L2*(LL6*m6 + L6*(m6 + m7))*cos(th5)*sin(th4)*sin(th6)));



		M_matrix(3,2)=(sin(th5)*((ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*cos(th5)*sin(th4) - L4*(LL6*m6 + L6*(m6 + m7))*cos(th4)*sin(th6)));


		M_matrix(3,3)=(ia4yy + ia5zz + im4zz*pow(kr4,2) + pow(L4,2)*m4 + 2*L4*LL4*m4 + pow(LL4,2)*m4 + pow(L4,2)*m5 + pow(L4,2)*m6 + pow(L4,2)*m7 +
						(ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7)*pow(cos(th5),2) + 2*L4*(LL6*m6 + L6*(m6 + m7))*cos(th6));


		M_matrix(3,4)=(-(L4*(LL6*m6 + L6*(m6 + m7))*sin(th5)*sin(th6)));


		M_matrix(3,5)=cos(th5)*(ia6yy + ia7yy + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(th6));


		M_matrix(3,6)=0;





		// j5
		M_matrix(4,0)=(cos(th2)*((ia5yy + ia6zz + im5zz*kr5)*cos(th4) + L4*(LL6*m6 + L6*(m6 + m7))*cos(th5)*sin(th4)*sin(th6)) -
						sin(th2)*((LL6*m6 + L6*(m6 + m7))*(L4 + L2*cos(th4))*sin(th3)*sin(th5)*sin(th6) +
						cos(th3)*((ia5yy + ia6zz + im5zz*kr5)*sin(th4) - (LL6*m6 + L6*(m6 + m7))*(L2 + L4*cos(th4))*cos(th5)*sin(th6))));


		M_matrix(4,1)=((-(LL6*m6) - L6*(m6 + m7))*cos(th3)*(L4 + L2*cos(th4))*sin(th5)*sin(th6) +
					sin(th3)*((ia5yy + ia6zz + im5zz*kr5)*sin(th4) - (LL6*m6 + L6*(m6 + m7))*(L2 + L4*cos(th4))*cos(th5)*sin(th6)));


		M_matrix(4,2)=((ia5yy + ia6zz + im5zz*kr5)*cos(th4) + L4*(LL6*m6 + L6*(m6 + m7))*cos(th5)*sin(th4)*sin(th6));

		M_matrix(4,3)=(-(L4*(LL6*m6 + L6*(m6 + m7))*sin(th5)*sin(th6)));

		M_matrix(4,4)=(ia5yy + ia6zz + im5zz*pow(kr5,2));


		M_matrix(4,5)=0;


		M_matrix(4,6)=0;





		//j6

		M_matrix(5,0)=(cos(th5)*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + (LL6*m6 + L6*(m6 + m7))*(L4 + L2*cos(th4))*cos(th6))*
						sin(th2)*sin(th3) + cos(th3)*(L2*(LL6*m6 + L6*(m6 + m7))*cos(th6) +
						cos(th4)*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(th6)))*sin(th2)*sin(th5)\
						+ sin(th4)*(cos(th2)*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(th6))*
							sin(th5) - L2*(LL6*m6 + L6*(m6 + m7))*sin(th2)*sin(th3)*sin(th6)));


		M_matrix(5,1)=((-(L2*(LL6*m6 + L6*(m6 + m7))*cos(th6)) - cos(th4)*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 +
							L4*(LL6*m6 + L6*(m6 + m7))*cos(th6)))*sin(th3)*sin(th5) +
						cos(th3)*(cos(th5)*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 +
							(LL6*m6 + L6*(m6 + m7))*(L4 + L2*cos(th4))*cos(th6)) - L2*(LL6*m6 + L6*(m6 + m7))*sin(th4)*sin(th6)));


		M_matrix(5,2)=((ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(th6))*sin(th4)*sin(th5));


		M_matrix(5,3)=(cos(th5)*(ia6yy + ia7yy + im6zz*kr6 + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7 + L4*(LL6*m6 + L6*(m6 + m7))*cos(th6)));


		M_matrix(5,4)=0;



		M_matrix(5,5)=(ia6yy + ia7yy + im6zz*pow(kr6,2) + pow(L6,2)*m6 + 2*L6*LL6*m6 + pow(LL6,2)*m6 + pow(L6,2)*m7);



		M_matrix(5,6)=0;






		M_matrix(6,0)=((ia7zz + im7zz*kr7)*(cos(th2)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)) +
						sin(th2)*(sin(th3)*sin(th5)*sin(th6) - cos(th3)*(cos(th6)*sin(th4) + cos(th4)*cos(th5)*sin(th6)))));


		M_matrix(6,1)=((ia7zz + im7zz*kr7)*(cos(th6)*sin(th3)*sin(th4) + (cos(th4)*cos(th5)*sin(th3) + cos(th3)*sin(th5))*sin(th6)));


		M_matrix(6,2)=((ia7zz + im7zz*kr7)*(cos(th4)*cos(th6) - cos(th5)*sin(th4)*sin(th6)));


		M_matrix(6,3)=((ia7zz + im7zz*kr7)*sin(th5)*sin(th6));

		M_matrix(6,4)=((ia7zz + im7zz*kr7)*cos(th6));


		M_matrix(6,5)=0;



		M_matrix(6,6)=(ia7zz + im7zz*pow(kr7,2));
}


void rotation_mat( double d_ang1, double d_ang2, double d_ang3, double d_ang4, double d_ang5, double d_ang6, double d_ang7){
    //input : desired angles
    //output : rotation matrix frame: base to d & euler angles
    MatrixXd ROT01(3,3);
    MatrixXd ROT12(3,3);
    MatrixXd ROT23(3,3);
    MatrixXd ROT34(3,3);
    MatrixXd ROT45(3,3);
    MatrixXd ROT56(3,3);
    MatrixXd ROT67(3,3);
    MatrixXd ROT7E(3,3);

    ROT01 << 	cos(d_ang1), 	0, 	-sin(d_ang1),
                sin(d_ang1), 	0, 	cos(d_ang1),
                    0, 			-1, 		0;

    ROT12 << 	cos(d_ang2), 0, 	sin(d_ang2),
                sin(d_ang2), 0,     -cos(d_ang2),
                	0,		 1,		 	0;

    ROT23 << 	cos(d_ang3), 	0, 	-sin(d_ang3),
                sin(d_ang3), 	0, 	cos(d_ang3),
                    0, 			-1, 		0;

		ROT34 << 	cos(d_ang4), 0, 	sin(d_ang4),
                    sin(d_ang4), 0,     -cos(d_ang4),
                    	0,		 1,		 	0;

		ROT45 << 	cos(d_ang5), 	0, 	-sin(d_ang5),
                    sin(d_ang5), 	0, 	cos(d_ang5),
                        0, 			-1, 		0;

		ROT56 << 	cos(d_ang6), 0, 	sin(d_ang6),
                    sin(d_ang6), 0,     -cos(d_ang6),
                    	0,		 1,		 	0;

		ROT67 << 	cos(d_ang7), -sin(d_ang7), 	0,
                    sin(d_ang7), cos(d_ang7), 	0,
                        0, 		0, 		1;

    ROT7E << 	1, 	0, 	0,
                    0, 	1, 	0,
                    0, 	0, 	1;

    ROT_0d = ROT01*ROT12*ROT23*ROT34*ROT45*ROT56*ROT67*ROT7E;




}

/*
void jacob_diffjacob (double ang1, double ang2, double ang3, double ang4, double ang5, double ang6, double ang7, double dth1, double dth2, double dth3, double dth4, double dth5, double dth6, double dth7){
	//Update Jacobian matrix
			//20190406 Results of Jacobian 6x7 - (correct)



			Jacob(0,0) = - 1.0*d5*(sin(ang4)*(cos(ang1)*sin(ang3) + 1.0*cos(ang2)*cos(ang3)*sin(ang1)) + 1.0*cos(ang4)*sin(ang1)*sin(ang2)) - 1.0*d7*(cos(ang6)*(sin(ang4)*(cos(ang1)*sin(ang3) + 1.0*cos(ang2)*cos(ang3)*sin(ang1)) + 1.0*cos(ang4)*sin(ang1)*sin(ang2)) + sin(ang6)*(cos(ang5)*(cos(ang4)*(cos(ang1)*sin(ang3) + 1.0*cos(ang2)*cos(ang3)*sin(ang1)) - sin(ang1)*sin(ang2)*sin(ang4)) + sin(ang5)*(cos(ang1)*cos(ang3) - cos(ang2)*sin(ang1)*sin(ang3)))) - d3*sin(ang1)*sin(ang2);
			Jacob(0,1) = 1.0*d5*(1.0*cos(ang1)*cos(ang2)*cos(ang4) - 1.0*cos(ang1)*cos(ang3)*sin(ang2)*sin(ang4)) - 1.0*d7*(sin(ang6)*(cos(ang5)*(cos(ang1)*cos(ang2)*sin(ang4) + 1.0*cos(ang1)*cos(ang3)*cos(ang4)*sin(ang2)) - cos(ang1)*sin(ang2)*sin(ang3)*sin(ang5)) - cos(ang6)*(1.0*cos(ang1)*cos(ang2)*cos(ang4) - 1.0*cos(ang1)*cos(ang3)*sin(ang2)*sin(ang4))) + d3*cos(ang1)*cos(ang2);
			Jacob(0,2) = 1.0*d7*(sin(ang6)*(sin(ang5)*(sin(ang1)*sin(ang3) - cos(ang1)*cos(ang2)*cos(ang3)) - cos(ang4)*cos(ang5)*(cos(ang3)*sin(ang1) + 1.0*cos(ang1)*cos(ang2)*sin(ang3))) - cos(ang6)*sin(ang4)*(cos(ang3)*sin(ang1) + 1.0*cos(ang1)*cos(ang2)*sin(ang3))) - 1.0*d5*sin(ang4)*(cos(ang3)*sin(ang1) + 1.0*cos(ang1)*cos(ang2)*sin(ang3));
			Jacob(0,3) = - 1.0*d5*(cos(ang4)*(sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) + 1.0*cos(ang1)*sin(ang2)*sin(ang4)) - 1.0*d7*(cos(ang6)*(cos(ang4)*(sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) + 1.0*cos(ang1)*sin(ang2)*sin(ang4)) - cos(ang5)*sin(ang6)*(sin(ang4)*(sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) - cos(ang1)*cos(ang4)*sin(ang2)));
			Jacob(0,4) = 1.0*d7*sin(ang6)*(sin(ang5)*(cos(ang4)*(sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) + cos(ang1)*sin(ang2)*sin(ang4)) - cos(ang5)*(cos(ang3)*sin(ang1) + cos(ang1)*cos(ang2)*sin(ang3)));
			Jacob(0,5) = 1.0*d7*(sin(ang6)*(sin(ang4)*(sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) - 1.0*cos(ang1)*cos(ang4)*sin(ang2)) - cos(ang6)*(cos(ang5)*(cos(ang4)*(sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) + cos(ang1)*sin(ang2)*sin(ang4)) + sin(ang5)*(cos(ang3)*sin(ang1) + cos(ang1)*cos(ang2)*sin(ang3))));
			Jacob(0,6) = 0;

			Jacob(1,0) = d3*cos(ang1)*sin(ang2) - d5*(sin(ang4)*(sin(ang1)*sin(ang3) - cos(ang1)*cos(ang2)*cos(ang3)) - cos(ang1)*cos(ang4)*sin(ang2)) - d7*(sin(ang6)*(cos(ang5)*(cos(ang4)*(sin(ang1)*sin(ang3) - cos(ang1)*cos(ang2)*cos(ang3)) + 1.0*cos(ang1)*sin(ang2)*sin(ang4)) + sin(ang5)*(cos(ang3)*sin(ang1) + 1.0*cos(ang1)*cos(ang2)*sin(ang3))) + cos(ang6)*(sin(ang4)*(sin(ang1)*sin(ang3) - cos(ang1)*cos(ang2)*cos(ang3)) - cos(ang1)*cos(ang4)*sin(ang2)));
			Jacob(1,1) = d5*(cos(ang2)*cos(ang4)*sin(ang1) - cos(ang3)*sin(ang1)*sin(ang2)*sin(ang4)) + d7*(cos(ang6)*(cos(ang2)*cos(ang4)*sin(ang1) - cos(ang3)*sin(ang1)*sin(ang2)*sin(ang4)) - sin(ang6)*(cos(ang5)*(1.0*cos(ang2)*sin(ang1)*sin(ang4) + cos(ang3)*cos(ang4)*sin(ang1)*sin(ang2)) - 1.0*sin(ang1)*sin(ang2)*sin(ang3)*sin(ang5))) + d3*cos(ang2)*sin(ang1);
			Jacob(1,2) = d5*sin(ang4)*(cos(ang1)*cos(ang3) - cos(ang2)*sin(ang1)*sin(ang3)) - d7*(sin(ang6)*(sin(ang5)*(cos(ang1)*sin(ang3) + 1.0*cos(ang2)*cos(ang3)*sin(ang1)) - cos(ang4)*cos(ang5)*(cos(ang1)*cos(ang3) - cos(ang2)*sin(ang1)*sin(ang3))) - cos(ang6)*sin(ang4)*(cos(ang1)*cos(ang3) - cos(ang2)*sin(ang1)*sin(ang3)));
			Jacob(1,3) = d5*(cos(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) - sin(ang1)*sin(ang2)*sin(ang4)) + d7*(cos(ang6)*(cos(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) - sin(ang1)*sin(ang2)*sin(ang4)) - cos(ang5)*sin(ang6)*(sin(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) + 1.0*cos(ang4)*sin(ang1)*sin(ang2)));
			Jacob(1,4) = -d7*sin(ang6)*(sin(ang5)*(cos(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) - 1.0*sin(ang1)*sin(ang2)*sin(ang4)) - cos(ang5)*(cos(ang1)*cos(ang3) - 1.0*cos(ang2)*sin(ang1)*sin(ang3)));
			Jacob(1,5) = d7*(cos(ang6)*(cos(ang5)*(cos(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) - 1.0*sin(ang1)*sin(ang2)*sin(ang4)) + sin(ang5)*(cos(ang1)*cos(ang3) - 1.0*cos(ang2)*sin(ang1)*sin(ang3))) - sin(ang6)*(sin(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) + cos(ang4)*sin(ang1)*sin(ang2)));
			Jacob(1,6) = 0;

			Jacob(2,0) = 0;
			Jacob(2,1) = 1.0*d7*(sin(ang6)*(cos(ang5)*(sin(ang2)*sin(ang4) - cos(ang2)*cos(ang3)*cos(ang4)) + 1.0*cos(ang2)*sin(ang3)*sin(ang5)) - 1.0*cos(ang6)*(cos(ang4)*sin(ang2) + 1.0*cos(ang2)*cos(ang3)*sin(ang4))) - d3*sin(ang2) - d5*(cos(ang4)*sin(ang2) + 1.0*cos(ang2)*cos(ang3)*sin(ang4));
			Jacob(2,2) = 1.0*d7*(sin(ang6)*(1.0*cos(ang3)*sin(ang2)*sin(ang5) + cos(ang4)*cos(ang5)*sin(ang2)*sin(ang3)) + 1.0*cos(ang6)*sin(ang2)*sin(ang3)*sin(ang4)) + 1.0*d5*sin(ang2)*sin(ang3)*sin(ang4);
			Jacob(2,3) = - d5*(cos(ang2)*sin(ang4) + 1.0*cos(ang3)*cos(ang4)*sin(ang2)) - 1.0*d7*(1.0*cos(ang6)*(cos(ang2)*sin(ang4) + 1.0*cos(ang3)*cos(ang4)*sin(ang2)) + cos(ang5)*sin(ang6)*(cos(ang2)*cos(ang4) - cos(ang3)*sin(ang2)*sin(ang4)));
			Jacob(2,4) = 1.0*d7*sin(ang6)*(sin(ang5)*(cos(ang2)*sin(ang4) + cos(ang3)*cos(ang4)*sin(ang2)) + 1.0*cos(ang5)*sin(ang2)*sin(ang3));
			Jacob(2,5) = -1.0*d7*(cos(ang6)*(cos(ang5)*(cos(ang2)*sin(ang4) + cos(ang3)*cos(ang4)*sin(ang2)) - 1.0*sin(ang2)*sin(ang3)*sin(ang5)) + 1.0*sin(ang6)*(cos(ang2)*cos(ang4) - 1.0*cos(ang3)*sin(ang2)*sin(ang4)));
			Jacob(2,6) = 0;

			Jacob(3,0) = -1.0*sin(ang1);
			Jacob(3,1) = cos(ang1)*sin(ang2);
			Jacob(3,2) = - 1.0*cos(ang3)*sin(ang1) - 1.0*cos(ang1)*cos(ang2)*sin(ang3);
			Jacob(3,3) = 1.0*cos(ang1)*cos(ang4)*sin(ang2) - 1.0*sin(ang4)*(1.0*sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3));
			Jacob(3,4) = sin(ang5)*(1.0*cos(ang4)*(1.0*sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) + 1.0*cos(ang1)*sin(ang2)*sin(ang4)) - 1.0*cos(ang5)*(1.0*cos(ang3)*sin(ang1) + 1.0*cos(ang1)*cos(ang2)*sin(ang3));
			Jacob(3,5) = - 1.0*cos(ang6)*(1.0*sin(ang4)*(1.0*sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) - 1.0*cos(ang1)*cos(ang4)*sin(ang2)) - 1.0*sin(ang6)*(1.0*sin(ang5)*(1.0*cos(ang3)*sin(ang1) + 1.0*cos(ang1)*cos(ang2)*sin(ang3)) + 1.0*cos(ang5)*(1.0*cos(ang4)*(1.0*sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) + 1.0*cos(ang1)*sin(ang2)*sin(ang4)));
			Jacob(3,6) = - 1.0*cos(ang6)*(1.0*sin(ang4)*(1.0*sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) - 1.0*cos(ang1)*cos(ang4)*sin(ang2)) - 1.0*sin(ang6)*(1.0*sin(ang5)*(1.0*cos(ang3)*sin(ang1) + 1.0*cos(ang1)*cos(ang2)*sin(ang3)) + 1.0*cos(ang5)*(1.0*cos(ang4)*(1.0*sin(ang1)*sin(ang3) - 1.0*cos(ang1)*cos(ang2)*cos(ang3)) + 1.0*cos(ang1)*sin(ang2)*sin(ang4)));


			Jacob(4,0) = cos(ang1);
			Jacob(4,1) = sin(ang1)*sin(ang2);
			Jacob(4,2) = cos(ang1)*cos(ang3) - 1.0*cos(ang2)*sin(ang1)*sin(ang3);
			Jacob(4,3) = sin(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) + 1.0*cos(ang4)*sin(ang1)*sin(ang2);
			Jacob(4,4) = cos(ang5)*(cos(ang1)*cos(ang3) - 1.0*cos(ang2)*sin(ang1)*sin(ang3)) - 1.0*sin(ang5)*(cos(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) - 1.0*sin(ang1)*sin(ang2)*sin(ang4));
			Jacob(4,5) = sin(ang6)*(cos(ang5)*(cos(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) - 1.0*sin(ang1)*sin(ang2)*sin(ang4)) + sin(ang5)*(cos(ang1)*cos(ang3) - 1.0*cos(ang2)*sin(ang1)*sin(ang3))) + cos(ang6)*(1.0*sin(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) + 1.0*cos(ang4)*sin(ang1)*sin(ang2));
			Jacob(4,6) = sin(ang6)*(cos(ang5)*(cos(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) - 1.0*sin(ang1)*sin(ang2)*sin(ang4)) + sin(ang5)*(cos(ang1)*cos(ang3) - 1.0*cos(ang2)*sin(ang1)*sin(ang3))) + cos(ang6)*(1.0*sin(ang4)*(cos(ang1)*sin(ang3) + cos(ang2)*cos(ang3)*sin(ang1)) + 1.0*cos(ang4)*sin(ang1)*sin(ang2));

			Jacob(5,0) = 0;
			Jacob(5,1) = 1.0*cos(ang2);
			Jacob(5,2) = 1.0*sin(ang2)*sin(ang3);
			Jacob(5,3) = 1.0*cos(ang2)*cos(ang4) - 1.0*cos(ang3)*sin(ang2)*sin(ang4);
			Jacob(5,4) = sin(ang5)*(1.0*cos(ang2)*sin(ang4) + 1.0*cos(ang3)*cos(ang4)*sin(ang2)) + 1.0*cos(ang5)*sin(ang2)*sin(ang3);
			Jacob(5,5) = cos(ang6)*(1.0*cos(ang2)*cos(ang4) - 1.0*cos(ang3)*sin(ang2)*sin(ang4)) - 1.0*sin(ang6)*(1.0*cos(ang5)*(1.0*cos(ang2)*sin(ang4) + 1.0*cos(ang3)*cos(ang4)*sin(ang2)) - 1.0*sin(ang2)*sin(ang3)*sin(ang5));
			Jacob(5,6) = cos(ang6)*(1.0*cos(ang2)*cos(ang4) - 1.0*cos(ang3)*sin(ang2)*sin(ang4)) - 1.0*sin(ang6)*(1.0*cos(ang5)*(1.0*cos(ang2)*sin(ang4) + 1.0*cos(ang3)*cos(ang4)*sin(ang2)) - 1.0*sin(ang2)*sin(ang3)*sin(ang5));

			Jacob_3(0,0)=Jacob(0,0);
			Jacob_3(0,1)=Jacob(0,1);
			Jacob_3(0,2)=Jacob(0,2);
			Jacob_3(0,3)=Jacob(0,3);
			Jacob_3(0,4)=Jacob(0,4);
			Jacob_3(0,5)=Jacob(0,5);
			Jacob_3(0,6)=Jacob(0,6);

			Jacob_3(1,0)=Jacob(1,0);
			Jacob_3(1,1)=Jacob(1,1);
			Jacob_3(1,2)=Jacob(1,2);
			Jacob_3(1,3)=Jacob(1,3);
			Jacob_3(1,4)=Jacob(1,4);
			Jacob_3(1,5)=Jacob(1,5);
			Jacob_3(1,6)=Jacob(1,6);

			Jacob_3(2,0)=Jacob(2,0);
			Jacob_3(2,1)=Jacob(2,1);
			Jacob_3(2,2)=Jacob(2,2);
			Jacob_3(2,3)=Jacob(2,3);
			Jacob_3(2,4)=Jacob(2,4);
			Jacob_3(2,5)=Jacob(2,5);
			Jacob_3(2,6)=Jacob(2,6);


			//diff jacobian
			diff_jacob(0,0)=-1.*d7*dth5*(cos(ang1)*(cos(ang3)*cos(ang5) - 1.*cos(ang4)*sin(ang3)*sin(ang5)) + sin(ang1)*(sin(ang2)*sin(ang4)*sin(ang5) + cos(ang2)*(-1.*cos(ang5)*sin(ang3) - 1.*cos(ang3)*cos(ang4)*sin(ang5))))*sin(ang6) +
							dth4*(sin(ang1)*sin(ang2)*(1.*d5*sin(ang4) + 1.*d7*cos(ang6)*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang1)*sin(ang3)*(-1.*d5*cos(ang4) - 1.*d7*cos(ang4)*cos(ang6) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) +
								cos(ang2)*cos(ang3)*sin(ang1)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth3*(cos(ang1)*(1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(-1.*d5*sin(ang4) - 1.*d7*cos(ang6)*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))) +
								cos(ang2)*sin(ang1)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*(1.*d5*sin(ang4) + 1.*d7*cos(ang6)*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth2*sin(ang1)*(-1.*cos(ang2)*(d3 + cos(ang4)*(1.*d5 + 1.*d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) + sin(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)))) -
							1.*d7*dth6*(sin(ang1)*sin(ang2)*(-1.*cos(ang5)*cos(ang6)*sin(ang4) - 1.*cos(ang4)*sin(ang6)) + cos(ang1)*(cos(ang4)*cos(ang5)*cos(ang6)*sin(ang3) + cos(ang3)*cos(ang6)*sin(ang5) - sin(ang3)*sin(ang4)*sin(ang6)) -
								1.*cos(ang2)*sin(ang1)*(cos(ang6)*sin(ang3)*sin(ang5) + cos(ang3)*(-1.*cos(ang4)*cos(ang5)*cos(ang6) + 1.*sin(ang4)*sin(ang6)))) +
							dth1*(sin(ang1)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))) -
								1.*cos(ang1)*(sin(ang2)*(d3 + cos(ang4)*(1.*d5 + 1.*d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) + cos(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(1.*d5*sin(ang4) + 1.*d7*cos(ang6)*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)))));



			diff_jacob(0,1)=1.*d7*dth5*cos(ang1)*(cos(ang5)*sin(ang2)*sin(ang3) + (1.*cos(ang3)*cos(ang4)*sin(ang2) + cos(ang2)*sin(ang4))*sin(ang5))*sin(ang6) +
							dth3*cos(ang1)*sin(ang2)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))) -
							1.*d7*dth6*cos(ang1)*(-(cos(ang6)*sin(ang2)*sin(ang3)*sin(ang5)) + cos(ang2)*(cos(ang5)*cos(ang6)*sin(ang4) + 1.*cos(ang4)*sin(ang6)) + cos(ang3)*sin(ang2)*(1.*cos(ang4)*cos(ang5)*cos(ang6) - 1.*sin(ang4)*sin(ang6))) +
							dth4*cos(ang1)*(cos(ang2)*((-1.*d5 - 1.*d7*cos(ang6))*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang3)*sin(ang2)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth2*cos(ang1)*(sin(ang2)*(-1.*d3 + cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) + cos(ang2)*(1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(-1.*d5*sin(ang4) - 1.*d7*cos(ang6)*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth1*sin(ang1)*(cos(ang2)*(-1.*d3 + cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) + sin(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(1.*d5*sin(ang4) + 1.*d7*cos(ang6)*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))));



			diff_jacob(0,2)=1.*d7*dth5*(sin(ang1)*(cos(ang5)*sin(ang3) + cos(ang3)*cos(ang4)*sin(ang5)) - 1.*cos(ang1)*cos(ang2)*(cos(ang3)*cos(ang5) - 1.*cos(ang4)*sin(ang3)*sin(ang5)))*sin(ang6) +
							dth4*(cos(ang3)*sin(ang1) + 1.*cos(ang1)*cos(ang2)*sin(ang3))*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) +
							1.*d7*dth6*(cos(ang6)*(-(cos(ang3)*(cos(ang4)*cos(ang5)*sin(ang1) + cos(ang1)*cos(ang2)*sin(ang5))) + sin(ang3)*(-1.*cos(ang1)*cos(ang2)*cos(ang4)*cos(ang5) + sin(ang1)*sin(ang5))) + (cos(ang3)*sin(ang1) + 1.*cos(ang1)*cos(ang2)*sin(ang3))*sin(ang4)*sin(ang6)) +
							dth2*cos(ang1)*sin(ang2)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))) +
							dth3*(cos(ang1)*cos(ang2)*(1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(-1.*d5*sin(ang4) - 1.*d7*cos(ang6)*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))) +
								sin(ang1)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*(1.*d5*sin(ang4) + 1.*d7*cos(ang6)*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth1*(cos(ang1)*(1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(-1.*d5*sin(ang4) - 1.*d7*cos(ang6)*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))) +
								cos(ang2)*sin(ang1)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*(1.*d5*sin(ang4) + 1.*d7*cos(ang6)*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))));



			diff_jacob(0,3)=-1.*d7*dth5*(-(cos(ang1)*cos(ang4)*sin(ang2)) + (-1.*cos(ang1)*cos(ang2)*cos(ang3) + sin(ang1)*sin(ang3))*sin(ang4))*sin(ang5)*sin(ang6) +
							dth3*(cos(ang3)*sin(ang1) + 1.*cos(ang1)*cos(ang2)*sin(ang3))*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) +
							dth6*(-1.*d7*cos(ang5)*cos(ang6)*(-1.*sin(ang1)*sin(ang3)*sin(ang4) + cos(ang1)*(cos(ang4)*sin(ang2) + 1.*cos(ang2)*cos(ang3)*sin(ang4))) + 1.*d7*(cos(ang4)*sin(ang1)*sin(ang3) + cos(ang1)*(-1.*cos(ang2)*cos(ang3)*cos(ang4) + 1.*sin(ang2)*sin(ang4)))*sin(ang6)) +
							dth1*(sin(ang1)*sin(ang2)*(1.*d5*sin(ang4) + 1.*d7*cos(ang6)*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang1)*sin(ang3)*(-1.*d5*cos(ang4) - 1.*d7*cos(ang4)*cos(ang6) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) +
								cos(ang2)*cos(ang3)*sin(ang1)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth2*cos(ang1)*(cos(ang2)*((-1.*d5 - 1.*d7*cos(ang6))*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang3)*sin(ang2)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth4*(sin(ang1)*sin(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang1)*
								(cos(ang4)*((-1.*d5 - 1.*d7*cos(ang6))*sin(ang2) - 1.*d7*cos(ang2)*cos(ang3)*cos(ang5)*sin(ang6)) + sin(ang4)*(cos(ang2)*cos(ang3)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang2)*sin(ang6))));



			diff_jacob(0,4)=1.*d7*dth6*cos(ang6)*(-(cos(ang5)*(cos(ang3)*sin(ang1) + cos(ang1)*cos(ang2)*sin(ang3))) + (cos(ang4)*sin(ang1)*sin(ang3) + cos(ang1)*(-1.*cos(ang2)*cos(ang3)*cos(ang4) + sin(ang2)*sin(ang4)))*sin(ang5)) +
							1.*d7*dth4*(-1.*sin(ang1)*sin(ang3)*sin(ang4) + cos(ang1)*(cos(ang4)*sin(ang2) + 1.*cos(ang2)*cos(ang3)*sin(ang4)))*sin(ang5)*sin(ang6) + 1.*d7*dth2*cos(ang1)*(cos(ang5)*sin(ang2)*sin(ang3) + (1.*cos(ang3)*cos(ang4)*sin(ang2) + cos(ang2)*sin(ang4))*sin(ang5))*sin(ang6) +
							1.*d7*dth3*(sin(ang1)*(cos(ang5)*sin(ang3) + cos(ang3)*cos(ang4)*sin(ang5)) + cos(ang1)*cos(ang2)*(-1.*cos(ang3)*cos(ang5) + 1.*cos(ang4)*sin(ang3)*sin(ang5)))*sin(ang6) -
							1.*d7*dth1*(cos(ang1)*(cos(ang3)*cos(ang5) - 1.*cos(ang4)*sin(ang3)*sin(ang5)) + sin(ang1)*(sin(ang2)*sin(ang4)*sin(ang5) + cos(ang2)*(-1.*cos(ang5)*sin(ang3) - 1.*cos(ang3)*cos(ang4)*sin(ang5))))*sin(ang6) +
							1.*d7*dth5*(sin(ang1)*(cos(ang4)*cos(ang5)*sin(ang3) + cos(ang3)*sin(ang5)) + cos(ang1)*(cos(ang5)*sin(ang2)*sin(ang4) + cos(ang2)*(-1.*cos(ang3)*cos(ang4)*cos(ang5) + sin(ang3)*sin(ang5))))*sin(ang6);



			diff_jacob(0,5)=-1.*d7*dth5*cos(ang6)*(cos(ang5)*(cos(ang3)*sin(ang1) + cos(ang1)*cos(ang2)*sin(ang3)) - (cos(ang4)*sin(ang1)*sin(ang3) + cos(ang1)*(-1.*cos(ang2)*cos(ang3)*cos(ang4) + sin(ang2)*sin(ang4)))*sin(ang5)) +
								1.*d7*dth3*(-(cos(ang6)*(cos(ang4)*cos(ang5)*(cos(ang3)*sin(ang1) + 1.*cos(ang1)*cos(ang2)*sin(ang3)) + (cos(ang1)*cos(ang2)*cos(ang3) - sin(ang1)*sin(ang3))*sin(ang5))) + (cos(ang3)*sin(ang1) + 1.*cos(ang1)*cos(ang2)*sin(ang3))*sin(ang4)*sin(ang6)) +
								1.*d7*dth6*(cos(ang6)*(sin(ang1)*sin(ang3)*sin(ang4) + cos(ang1)*(-1.*cos(ang4)*sin(ang2) - 1.*cos(ang2)*cos(ang3)*sin(ang4))) +
									(sin(ang1)*(cos(ang4)*cos(ang5)*sin(ang3) + cos(ang3)*sin(ang5)) + cos(ang1)*(cos(ang5)*sin(ang2)*sin(ang4) + cos(ang2)*(-1.*cos(ang3)*cos(ang4)*cos(ang5) + sin(ang3)*sin(ang5))))*sin(ang6)) +
								1.*d7*dth2*cos(ang1)*(cos(ang6)*sin(ang2)*sin(ang3)*sin(ang5) + cos(ang2)*(-1.*cos(ang5)*cos(ang6)*sin(ang4) - 1.*cos(ang4)*sin(ang6)) + cos(ang3)*sin(ang2)*(-1.*cos(ang4)*cos(ang5)*cos(ang6) + 1.*sin(ang4)*sin(ang6))) +
								1.*d7*dth4*(sin(ang1)*sin(ang3)*(cos(ang5)*cos(ang6)*sin(ang4) + cos(ang4)*sin(ang6)) - 1.*cos(ang1)*(cos(ang4)*(cos(ang5)*cos(ang6)*sin(ang2) + 1.*cos(ang2)*cos(ang3)*sin(ang6)) + sin(ang4)*(1.*cos(ang2)*cos(ang3)*cos(ang5)*cos(ang6) - 1.*sin(ang2)*sin(ang6)))) +
								1.*d7*dth1*(sin(ang1)*sin(ang2)*(cos(ang5)*cos(ang6)*sin(ang4) + 1.*cos(ang4)*sin(ang6)) - cos(ang1)*(cos(ang4)*cos(ang5)*cos(ang6)*sin(ang3) + cos(ang3)*cos(ang6)*sin(ang5) - sin(ang3)*sin(ang4)*sin(ang6)) +
									cos(ang2)*sin(ang1)*(cos(ang6)*sin(ang3)*sin(ang5) + cos(ang3)*(-1.*cos(ang4)*cos(ang5)*cos(ang6) + 1.*sin(ang4)*sin(ang6))));



			diff_jacob(0,6)=0;




			diff_jacob(1,0)=-1.*d7*dth5*(-1.*cos(ang4)*sin(ang1)*sin(ang3)*sin(ang5) + cos(ang3)*(cos(ang5)*sin(ang1) + cos(ang1)*cos(ang2)*cos(ang4)*sin(ang5)) + cos(ang1)*(1.*cos(ang2)*cos(ang5)*sin(ang3) - 1.*sin(ang2)*sin(ang4)*sin(ang5)))*sin(ang6) +
							dth6*(-(d7*cos(ang6)*(sin(ang1)*(cos(ang4)*cos(ang5)*sin(ang3) + cos(ang3)*sin(ang5)) + cos(ang1)*(1.*cos(ang5)*sin(ang2)*sin(ang4) + cos(ang2)*(-1.*cos(ang3)*cos(ang4)*cos(ang5) + 1.*sin(ang3)*sin(ang5))))) +
								d7*(sin(ang1)*sin(ang3)*sin(ang4) - cos(ang1)*(cos(ang4)*sin(ang2) + cos(ang2)*cos(ang3)*sin(ang4)))*sin(ang6)) +
							dth3*(sin(ang3)*(d7*sin(ang1)*sin(ang5)*sin(ang6) + cos(ang1)*cos(ang2)*((d5 - d7*cos(ang6))*sin(ang4) - d7*cos(ang4)*cos(ang5)*sin(ang6))) +
								cos(ang3)*(-1.*d7*cos(ang1)*cos(ang2)*sin(ang5)*sin(ang6) + sin(ang1)*((d5 - 1.*d7*cos(ang6))*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth2*cos(ang1)*(cos(ang2)*(d3 + cos(ang4)*(d5 + 1.*d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) - 1.*sin(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth4*(-(sin(ang1)*sin(ang3)*(cos(ang4)*(d5 + d7*cos(ang6)) - d7*cos(ang5)*sin(ang4)*sin(ang6))) + cos(ang1)*(sin(ang2)*((-1.*d5 - 1.*d7*cos(ang6))*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) +
									cos(ang2)*cos(ang3)*(cos(ang4)*(d5 + d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)))) + dth1*(-(cos(ang1)*(d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6)))) -
								1.*sin(ang1)*(sin(ang2)*(d3 + cos(ang4)*(d5 + d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) + cos(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6)))));


			diff_jacob(1,1)=d7*dth5*sin(ang1)*(1.*cos(ang5)*sin(ang2)*sin(ang3) + (cos(ang3)*cos(ang4)*sin(ang2) + 1.*cos(ang2)*sin(ang4))*sin(ang5))*sin(ang6) + dth3*sin(ang1)*sin(ang2)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6))) -
							1.*d7*dth6*sin(ang1)*(-1.*cos(ang6)*sin(ang2)*sin(ang3)*sin(ang5) + cos(ang2)*(1.*cos(ang5)*cos(ang6)*sin(ang4) + cos(ang4)*sin(ang6)) + cos(ang3)*sin(ang2)*(cos(ang4)*cos(ang5)*cos(ang6) - 1.*sin(ang4)*sin(ang6))) +
							dth4*sin(ang1)*(cos(ang2)*((-1.*d5 - 1.*d7*cos(ang6))*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang3)*sin(ang2)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + d7*cos(ang5)*sin(ang4)*sin(ang6))) -
							1.*dth2*sin(ang1)*(sin(ang2)*(d3 + cos(ang4)*(d5 + d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) + cos(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth1*cos(ang1)*(cos(ang2)*(d3 + cos(ang4)*(d5 + d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) - 1.*sin(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))));



			diff_jacob(1,2)=-(d7*dth5*(cos(ang1)*(cos(ang5)*sin(ang3) + cos(ang3)*cos(ang4)*sin(ang5)) + cos(ang2)*sin(ang1)*(1.*cos(ang3)*cos(ang5) - cos(ang4)*sin(ang3)*sin(ang5)))*sin(ang6)) +
							dth4*(cos(ang1)*cos(ang3) - cos(ang2)*sin(ang1)*sin(ang3))*(cos(ang4)*(d5 + d7*cos(ang6)) - d7*cos(ang5)*sin(ang4)*sin(ang6)) + dth2*sin(ang1)*sin(ang2)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6))) +
							dth1*(-1.*sin(ang3)*(-1.*d7*sin(ang1)*sin(ang5)*sin(ang6) + cos(ang1)*cos(ang2)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6))) -
								1.*cos(ang3)*(1.*d7*cos(ang1)*cos(ang2)*sin(ang5)*sin(ang6) + sin(ang1)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth3*(-1.*cos(ang2)*sin(ang1)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6))) -
								1.*cos(ang1)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((d5 + d7*cos(ang6))*sin(ang4) + d7*cos(ang4)*cos(ang5)*sin(ang6)))) -
							d7*dth6*(cos(ang2)*sin(ang1)*(cos(ang4)*cos(ang5)*cos(ang6)*sin(ang3) + 1.*cos(ang3)*cos(ang6)*sin(ang5) - 1.*sin(ang3)*sin(ang4)*sin(ang6)) + cos(ang1)*(cos(ang6)*sin(ang3)*sin(ang5) + cos(ang3)*(-(cos(ang4)*cos(ang5)*cos(ang6)) + sin(ang4)*sin(ang6))));



			diff_jacob(1,3)=d7*dth5*(1.*cos(ang4)*sin(ang1)*sin(ang2) + (cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4))*sin(ang5)*sin(ang6) + dth3*(cos(ang1)*cos(ang3) - cos(ang2)*sin(ang1)*sin(ang3))*(cos(ang4)*(d5 + d7*cos(ang6)) - d7*cos(ang5)*sin(ang4)*sin(ang6)) +
							d7*dth6*(-(cos(ang5)*cos(ang6)*(1.*cos(ang4)*sin(ang1)*sin(ang2) + (cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4))) + (-(cos(ang4)*(cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))) + sin(ang1)*sin(ang2)*sin(ang4))*sin(ang6)) +
							dth2*sin(ang1)*(cos(ang2)*((-1.*d5 - 1.*d7*cos(ang6))*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang3)*sin(ang2)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + d7*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth4*(-1.*sin(ang4)*(cos(ang2)*cos(ang3)*(d5 + d7*cos(ang6))*sin(ang1) + cos(ang1)*(d5 + d7*cos(ang6))*sin(ang3) - 1.*d7*cos(ang5)*sin(ang1)*sin(ang2)*sin(ang6)) -
								1.*cos(ang4)*(1.*d7*cos(ang1)*cos(ang5)*sin(ang3)*sin(ang6) + sin(ang1)*((d5 + d7*cos(ang6))*sin(ang2) + 1.*d7*cos(ang2)*cos(ang3)*cos(ang5)*sin(ang6)))) +
							dth1*(-(sin(ang1)*sin(ang3)*(cos(ang4)*(d5 + d7*cos(ang6)) - d7*cos(ang5)*sin(ang4)*sin(ang6))) + cos(ang1)*(sin(ang2)*((-1.*d5 - 1.*d7*cos(ang6))*sin(ang4) - 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) +
									cos(ang2)*cos(ang3)*(cos(ang4)*(d5 + d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6))));



			diff_jacob(1,4)=-(d7*dth6*cos(ang6)*(cos(ang1)*(-(cos(ang3)*cos(ang5)) + cos(ang4)*sin(ang3)*sin(ang5)) + sin(ang1)*(-1.*sin(ang2)*sin(ang4)*sin(ang5) + cos(ang2)*(1.*cos(ang5)*sin(ang3) + cos(ang3)*cos(ang4)*sin(ang5))))) -
							d7*dth4*(-1.*cos(ang4)*sin(ang1)*sin(ang2) - (cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4))*sin(ang5)*sin(ang6) + 1.*d7*dth2*sin(ang1)*(1.*cos(ang5)*sin(ang2)*sin(ang3) + (1.*cos(ang3)*cos(ang4)*sin(ang2) + 1.*cos(ang2)*sin(ang4))*sin(ang5))*sin(ang6) -
							d7*dth5*(-1.*cos(ang5)*sin(ang1)*sin(ang2)*sin(ang4) + cos(ang1)*(cos(ang4)*cos(ang5)*sin(ang3) + cos(ang3)*sin(ang5)) + cos(ang2)*sin(ang1)*(cos(ang3)*cos(ang4)*cos(ang5) - 1.*sin(ang3)*sin(ang5)))*sin(ang6) -
							d7*dth3*(cos(ang1)*(cos(ang5)*sin(ang3) + cos(ang3)*cos(ang4)*sin(ang5)) + cos(ang2)*sin(ang1)*(1.*cos(ang3)*cos(ang5) - cos(ang4)*sin(ang3)*sin(ang5)))*sin(ang6) -
							1.*d7*dth1*(-1.*cos(ang4)*sin(ang1)*sin(ang3)*sin(ang5) + cos(ang3)*(cos(ang5)*sin(ang1) + cos(ang1)*cos(ang2)*cos(ang4)*sin(ang5)) + cos(ang1)*(1.*cos(ang2)*cos(ang5)*sin(ang3) - 1.*sin(ang2)*sin(ang4)*sin(ang5)))*sin(ang6);



			diff_jacob(1,5)=d7*dth5*cos(ang6)*(cos(ang5)*(cos(ang1)*cos(ang3) - 1.*cos(ang2)*sin(ang1)*sin(ang3)) - 1.*(cos(ang2)*cos(ang3)*cos(ang4)*sin(ang1) + cos(ang1)*cos(ang4)*sin(ang3) - 1.*sin(ang1)*sin(ang2)*sin(ang4))*sin(ang5)) +
							d7*dth4*(cos(ang5)*cos(ang6)*(-1.*cos(ang4)*sin(ang1)*sin(ang2) - (cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4)) + (-(cos(ang4)*(cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))) + sin(ang1)*sin(ang2)*sin(ang4))*sin(ang6)) +
							d7*dth1*(cos(ang6)*(-(sin(ang1)*(cos(ang4)*cos(ang5)*sin(ang3) + cos(ang3)*sin(ang5))) + cos(ang1)*(-1.*cos(ang5)*sin(ang2)*sin(ang4) + cos(ang2)*(cos(ang3)*cos(ang4)*cos(ang5) - 1.*sin(ang3)*sin(ang5)))) +
								(sin(ang1)*sin(ang3)*sin(ang4) - cos(ang1)*(cos(ang4)*sin(ang2) + cos(ang2)*cos(ang3)*sin(ang4)))*sin(ang6)) +
							d7*dth6*(-(cos(ang6)*(cos(ang4)*sin(ang1)*sin(ang2) + (cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4))) -
								(-1.*cos(ang5)*sin(ang1)*sin(ang2)*sin(ang4) + cos(ang1)*(cos(ang4)*cos(ang5)*sin(ang3) + cos(ang3)*sin(ang5)) + cos(ang2)*sin(ang1)*(cos(ang3)*cos(ang4)*cos(ang5) - 1.*sin(ang3)*sin(ang5)))*sin(ang6)) -
							1.*d7*dth2*sin(ang1)*(-1.*cos(ang6)*sin(ang2)*sin(ang3)*sin(ang5) + cos(ang2)*(1.*cos(ang5)*cos(ang6)*sin(ang4) + cos(ang4)*sin(ang6)) + cos(ang3)*sin(ang2)*(cos(ang4)*cos(ang5)*cos(ang6) - 1.*sin(ang4)*sin(ang6))) +
							d7*dth3*(cos(ang2)*sin(ang1)*(-(cos(ang4)*cos(ang5)*cos(ang6)*sin(ang3)) - 1.*cos(ang3)*cos(ang6)*sin(ang5) + sin(ang3)*sin(ang4)*sin(ang6)) - cos(ang1)*(cos(ang6)*sin(ang3)*sin(ang5) + cos(ang3)*(-(cos(ang4)*cos(ang5)*cos(ang6)) + sin(ang4)*sin(ang6))));



			diff_jacob(1,6)=0;





			diff_jacob(2,0)=0;


			diff_jacob(2,1)=1.*d7*dth5*(-1.*sin(ang2)*sin(ang4)*sin(ang5) + cos(ang2)*(1.*cos(ang5)*sin(ang3) + 1.*cos(ang3)*cos(ang4)*sin(ang5)))*sin(ang6) + dth3*cos(ang2)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))) +
							dth4*(sin(ang2)*((d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang2)*cos(ang3)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth2*(-1.*cos(ang2)*(d3 + cos(ang4)*(d5 + 1.*d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) + sin(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)))) +
							1.*d7*dth6*(sin(ang2)*(cos(ang5)*cos(ang6)*sin(ang4) + 1.*cos(ang4)*sin(ang6)) - 1.*cos(ang2)*(-1.*cos(ang6)*sin(ang3)*sin(ang5) + cos(ang3)*(cos(ang4)*cos(ang5)*cos(ang6) - 1.*sin(ang4)*sin(ang6))));



			diff_jacob(2,2)=1.*d7*dth5*sin(ang2)*(1.*cos(ang3)*cos(ang5) - 1.*cos(ang4)*sin(ang3)*sin(ang5))*sin(ang6) + dth4*sin(ang2)*sin(ang3)*(cos(ang4)*(1.*d5 + 1.*d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) +
							1.*d7*dth6*sin(ang2)*(cos(ang4)*cos(ang5)*cos(ang6)*sin(ang3) + 1.*cos(ang3)*cos(ang6)*sin(ang5) - 1.*sin(ang3)*sin(ang4)*sin(ang6)) +
							dth3*sin(ang2)*(-1.*d7*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6))) +
							dth2*cos(ang2)*(1.*d7*cos(ang3)*sin(ang5)*sin(ang6) + sin(ang3)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)));



			diff_jacob(2,3)=1.*d7*dth5*(cos(ang2)*cos(ang4) - 1.*cos(ang3)*sin(ang2)*sin(ang4))*sin(ang5)*sin(ang6) + dth3*sin(ang2)*sin(ang3)*(cos(ang4)*(1.*d5 + 1.*d7*cos(ang6)) - 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)) -
							1.*d7*dth6*(-1.*cos(ang3)*sin(ang2)*(cos(ang5)*cos(ang6)*sin(ang4) + 1.*cos(ang4)*sin(ang6)) + cos(ang2)*(cos(ang4)*cos(ang5)*cos(ang6) - 1.*sin(ang4)*sin(ang6))) +
							dth4*(cos(ang3)*sin(ang2)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang2)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth2*(sin(ang2)*((1.*d5 + 1.*d7*cos(ang6))*sin(ang4) + 1.*d7*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang2)*cos(ang3)*(cos(ang4)*(-1.*d5 - 1.*d7*cos(ang6)) + 1.*d7*cos(ang5)*sin(ang4)*sin(ang6)));




			diff_jacob(2,4)=1.*d7*dth6*cos(ang6)*(1.*cos(ang5)*sin(ang2)*sin(ang3) + (cos(ang3)*cos(ang4)*sin(ang2) + cos(ang2)*sin(ang4))*sin(ang5)) + 1.*d7*dth4*(cos(ang2)*cos(ang4) - 1.*cos(ang3)*sin(ang2)*sin(ang4))*sin(ang5)*sin(ang6) +
							1.*d7*dth3*sin(ang2)*(1.*cos(ang3)*cos(ang5) - 1.*cos(ang4)*sin(ang3)*sin(ang5))*sin(ang6) + 1.*d7*dth5*(cos(ang3)*cos(ang4)*cos(ang5)*sin(ang2) + cos(ang2)*cos(ang5)*sin(ang4) - 1.*sin(ang2)*sin(ang3)*sin(ang5))*sin(ang6) +
							1.*d7*dth2*(-(sin(ang2)*sin(ang4)*sin(ang5)) + cos(ang2)*(1.*cos(ang5)*sin(ang3) + cos(ang3)*cos(ang4)*sin(ang5)))*sin(ang6);



			diff_jacob(2,5)=-1.*d7*dth5*cos(ang6)*(-1.*cos(ang5)*sin(ang2)*sin(ang3) - (cos(ang3)*cos(ang4)*sin(ang2) + cos(ang2)*sin(ang4))*sin(ang5)) + 1.*d7*dth3*sin(ang2)*(cos(ang4)*cos(ang5)*cos(ang6)*sin(ang3) + 1.*cos(ang3)*cos(ang6)*sin(ang5) - 1.*sin(ang3)*sin(ang4)*sin(ang6)) +
							dth6*(-1.*d7*cos(ang6)*(cos(ang2)*cos(ang4) - 1.*cos(ang3)*sin(ang2)*sin(ang4)) + 1.*d7*(cos(ang3)*cos(ang4)*cos(ang5)*sin(ang2) + cos(ang2)*cos(ang5)*sin(ang4) - 1.*sin(ang2)*sin(ang3)*sin(ang5))*sin(ang6)) -
							1.*d7*dth4*(cos(ang3)*sin(ang2)*(-1.*cos(ang5)*cos(ang6)*sin(ang4) - 1.*cos(ang4)*sin(ang6)) + cos(ang2)*(cos(ang4)*cos(ang5)*cos(ang6) - 1.*sin(ang4)*sin(ang6))) -
							1.*d7*dth2*(sin(ang2)*(-1.*cos(ang5)*cos(ang6)*sin(ang4) - 1.*cos(ang4)*sin(ang6)) + cos(ang2)*(-1.*cos(ang6)*sin(ang3)*sin(ang5) + cos(ang3)*(cos(ang4)*cos(ang5)*cos(ang6) - 1.*sin(ang4)*sin(ang6))));




			diff_jacob(2,6)=0;




			diff_jacob(3,0)=-1.*dth1*cos(ang1);



			diff_jacob(3,1)=dth2*cos(ang1)*cos(ang2) - dth1*sin(ang1)*sin(ang2);



			diff_jacob(3,2)=1.*dth2*cos(ang1)*sin(ang2)*sin(ang3) + dth3*(-1.*cos(ang1)*cos(ang2)*cos(ang3) + 1.*sin(ang1)*sin(ang3)) + dth1*(-1.*cos(ang1)*cos(ang3) + 1.*cos(ang2)*sin(ang1)*sin(ang3));



			diff_jacob(3,3)=dth3*(-1.*cos(ang3)*sin(ang1) - 1.*cos(ang1)*cos(ang2)*sin(ang3))*sin(ang4) + dth2*cos(ang1)*(1.*cos(ang2)*cos(ang4) - 1.*cos(ang3)*sin(ang2)*sin(ang4)) + dth1*(-1.*cos(ang4)*sin(ang1)*sin(ang2) + (-1.*cos(ang2)*cos(ang3)*sin(ang1) - 1.*cos(ang1)*sin(ang3))*sin(ang4)) +
							dth4*(-1.*cos(ang4)*sin(ang1)*sin(ang3) + cos(ang1)*(1.*cos(ang2)*cos(ang3)*cos(ang4) - 1.*sin(ang2)*sin(ang4)));



			diff_jacob(3,4)=1.*dth4*(-1.*sin(ang1)*sin(ang3)*sin(ang4) + cos(ang1)*(1.*cos(ang4)*sin(ang2) + 1.*cos(ang2)*cos(ang3)*sin(ang4)))*sin(ang5) + 1.*dth2*cos(ang1)*(1.*cos(ang5)*sin(ang2)*sin(ang3) + (1.*cos(ang3)*cos(ang4)*sin(ang2) + 1.*cos(ang2)*sin(ang4))*sin(ang5)) +
							dth3*(sin(ang1)*(1.*cos(ang5)*sin(ang3) + 1.*cos(ang3)*cos(ang4)*sin(ang5)) + cos(ang1)*cos(ang2)*(-1.*cos(ang3)*cos(ang5) + 1.*cos(ang4)*sin(ang3)*sin(ang5))) +
							dth1*(cos(ang1)*(-1.*cos(ang3)*cos(ang5) + 1.*cos(ang4)*sin(ang3)*sin(ang5)) + sin(ang1)*(-1.*sin(ang2)*sin(ang4)*sin(ang5) + cos(ang2)*(1.*cos(ang5)*sin(ang3) + 1.*cos(ang3)*cos(ang4)*sin(ang5)))) +
							dth5*(sin(ang1)*(1.*cos(ang4)*cos(ang5)*sin(ang3) + 1.*cos(ang3)*sin(ang5)) + cos(ang1)*(1.*cos(ang5)*sin(ang2)*sin(ang4) + cos(ang2)*(-1.*cos(ang3)*cos(ang4)*cos(ang5) + 1.*sin(ang3)*sin(ang5))));



			diff_jacob(3,5)=-1.*dth5*(-1.*cos(ang4)*sin(ang1)*sin(ang3)*sin(ang5) + cos(ang3)*(1.*cos(ang5)*sin(ang1) + 1.*cos(ang1)*cos(ang2)*cos(ang4)*sin(ang5)) + cos(ang1)*(1.*cos(ang2)*cos(ang5)*sin(ang3) - 1.*sin(ang2)*sin(ang4)*sin(ang5)))*sin(ang6) +
							dth1*(-1.*cos(ang1)*cos(ang6)*sin(ang3)*sin(ang4) + 1.*cos(ang5)*sin(ang1)*sin(ang2)*sin(ang4)*sin(ang6) - 1.*cos(ang1)*cos(ang3)*sin(ang5)*sin(ang6) +
								cos(ang4)*(-1.*cos(ang6)*sin(ang1)*sin(ang2) + cos(ang5)*(-1.*cos(ang2)*cos(ang3)*sin(ang1) - 1.*cos(ang1)*sin(ang3))*sin(ang6)) + cos(ang2)*sin(ang1)*(-1.*cos(ang3)*cos(ang6)*sin(ang4) + 1.*sin(ang3)*sin(ang5)*sin(ang6))) +
							dth3*(cos(ang3)*(-1.*cos(ang6)*sin(ang1)*sin(ang4) + (-1.*cos(ang4)*cos(ang5)*sin(ang1) - 1.*cos(ang1)*cos(ang2)*sin(ang5))*sin(ang6)) + sin(ang3)*(1.*sin(ang1)*sin(ang5)*sin(ang6) + cos(ang1)*cos(ang2)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth2*cos(ang1)*(cos(ang2)*(1.*cos(ang4)*cos(ang6) - 1.*cos(ang5)*sin(ang4)*sin(ang6)) + sin(ang2)*(1.*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth6*(sin(ang1)*(-1.*cos(ang4)*cos(ang5)*cos(ang6)*sin(ang3) - 1.*cos(ang3)*cos(ang6)*sin(ang5) + 1.*sin(ang3)*sin(ang4)*sin(ang6)) +
								cos(ang1)*(sin(ang2)*(-1.*cos(ang5)*cos(ang6)*sin(ang4) - 1.*cos(ang4)*sin(ang6)) + cos(ang2)*(1.*cos(ang3)*cos(ang4)*cos(ang5)*cos(ang6) - 1.*cos(ang6)*sin(ang3)*sin(ang5) - 1.*cos(ang3)*sin(ang4)*sin(ang6)))) +
							dth4*(sin(ang1)*sin(ang3)*(-1.*cos(ang4)*cos(ang6) + 1.*cos(ang5)*sin(ang4)*sin(ang6)) + cos(ang1)*(sin(ang2)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang2)*cos(ang3)*(1.*cos(ang4)*cos(ang6) - 1.*cos(ang5)*sin(ang4)*sin(ang6))));



			diff_jacob(3,6)=-1.*dth5*(-1.*cos(ang4)*sin(ang1)*sin(ang3)*sin(ang5) + cos(ang3)*(1.*cos(ang5)*sin(ang1) + 1.*cos(ang1)*cos(ang2)*cos(ang4)*sin(ang5)) + cos(ang1)*(1.*cos(ang2)*cos(ang5)*sin(ang3) - 1.*sin(ang2)*sin(ang4)*sin(ang5)))*sin(ang6) +
							dth1*(-1.*cos(ang1)*cos(ang6)*sin(ang3)*sin(ang4) + 1.*cos(ang5)*sin(ang1)*sin(ang2)*sin(ang4)*sin(ang6) - 1.*cos(ang1)*cos(ang3)*sin(ang5)*sin(ang6) +
								cos(ang4)*(-1.*cos(ang6)*sin(ang1)*sin(ang2) + cos(ang5)*(-1.*cos(ang2)*cos(ang3)*sin(ang1) - 1.*cos(ang1)*sin(ang3))*sin(ang6)) + cos(ang2)*sin(ang1)*(-1.*cos(ang3)*cos(ang6)*sin(ang4) + 1.*sin(ang3)*sin(ang5)*sin(ang6))) +
							dth3*(cos(ang3)*(-1.*cos(ang6)*sin(ang1)*sin(ang4) + (-1.*cos(ang4)*cos(ang5)*sin(ang1) - 1.*cos(ang1)*cos(ang2)*sin(ang5))*sin(ang6)) + sin(ang3)*(1.*sin(ang1)*sin(ang5)*sin(ang6) + cos(ang1)*cos(ang2)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth2*cos(ang1)*(cos(ang2)*(1.*cos(ang4)*cos(ang6) - 1.*cos(ang5)*sin(ang4)*sin(ang6)) + sin(ang2)*(1.*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth6*(sin(ang1)*(-1.*cos(ang4)*cos(ang5)*cos(ang6)*sin(ang3) - 1.*cos(ang3)*cos(ang6)*sin(ang5) + 1.*sin(ang3)*sin(ang4)*sin(ang6)) +
								cos(ang1)*(sin(ang2)*(-1.*cos(ang5)*cos(ang6)*sin(ang4) - 1.*cos(ang4)*sin(ang6)) + cos(ang2)*(1.*cos(ang3)*cos(ang4)*cos(ang5)*cos(ang6) - 1.*cos(ang6)*sin(ang3)*sin(ang5) - 1.*cos(ang3)*sin(ang4)*sin(ang6)))) +
							dth4*(sin(ang1)*sin(ang3)*(-1.*cos(ang4)*cos(ang6) + 1.*cos(ang5)*sin(ang4)*sin(ang6)) + cos(ang1)*(sin(ang2)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang2)*cos(ang3)*(1.*cos(ang4)*cos(ang6) - 1.*cos(ang5)*sin(ang4)*sin(ang6))));




			diff_jacob(4,0)=-(dth1*sin(ang1));
			diff_jacob(4,1)=dth2*cos(ang2)*sin(ang1) + dth1*cos(ang1)*sin(ang2);
			diff_jacob(4,2)=1.*dth2*sin(ang1)*sin(ang2)*sin(ang3) + dth3*(-1.*cos(ang2)*cos(ang3)*sin(ang1) - cos(ang1)*sin(ang3)) + dth1*(-(cos(ang3)*sin(ang1)) - 1.*cos(ang1)*cos(ang2)*sin(ang3));



			diff_jacob(4,3)=dth3*(cos(ang1)*cos(ang3) - cos(ang2)*sin(ang1)*sin(ang3))*sin(ang4) + dth2*sin(ang1)*(1.*cos(ang2)*cos(ang4) - cos(ang3)*sin(ang2)*sin(ang4)) + dth4*(cos(ang2)*cos(ang3)*cos(ang4)*sin(ang1) + cos(ang1)*cos(ang4)*sin(ang3) - 1.*sin(ang1)*sin(ang2)*sin(ang4)) +
			dth1*(-(sin(ang1)*sin(ang3)*sin(ang4)) + cos(ang1)*(1.*cos(ang4)*sin(ang2) + cos(ang2)*cos(ang3)*sin(ang4)));




			diff_jacob(4,4)=dth4*(1.*cos(ang4)*sin(ang1)*sin(ang2) + 1.*(cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4))*sin(ang5) +
							dth5*(-1.*cos(ang5)*(cos(ang2)*cos(ang3)*cos(ang4)*sin(ang1) + cos(ang1)*cos(ang4)*sin(ang3) - 1.*sin(ang1)*sin(ang2)*sin(ang4)) - 1.*(cos(ang1)*cos(ang3) - 1.*cos(ang2)*sin(ang1)*sin(ang3))*sin(ang5)) +
							dth2*sin(ang1)*(1.*cos(ang5)*sin(ang2)*sin(ang3) + (1.*cos(ang3)*cos(ang4)*sin(ang2) + 1.*cos(ang2)*sin(ang4))*sin(ang5)) +
							dth1*(-1.*cos(ang5)*(cos(ang3)*sin(ang1) + 1.*cos(ang1)*cos(ang2)*sin(ang3)) - 1.*(-1.*cos(ang4)*sin(ang1)*sin(ang3) + cos(ang1)*(cos(ang2)*cos(ang3)*cos(ang4) - 1.*sin(ang2)*sin(ang4)))*sin(ang5)) +
							dth3*(-1.*cos(ang1)*(cos(ang5)*sin(ang3) + 1.*cos(ang3)*cos(ang4)*sin(ang5)) + cos(ang2)*sin(ang1)*(-1.*cos(ang3)*cos(ang5) + 1.*cos(ang4)*sin(ang3)*sin(ang5)));




			diff_jacob(4,5)=dth5*(cos(ang5)*(cos(ang1)*cos(ang3) - 1.*cos(ang2)*sin(ang1)*sin(ang3)) - 1.*(cos(ang2)*cos(ang3)*cos(ang4)*sin(ang1) + cos(ang1)*cos(ang4)*sin(ang3) - 1.*sin(ang1)*sin(ang2)*sin(ang4))*sin(ang5))*sin(ang6) +
							dth6*(cos(ang5)*cos(ang6)*(cos(ang2)*cos(ang3)*cos(ang4)*sin(ang1) + cos(ang1)*cos(ang4)*sin(ang3) - 1.*sin(ang1)*sin(ang2)*sin(ang4)) + cos(ang6)*(cos(ang1)*cos(ang3) - 1.*cos(ang2)*sin(ang1)*sin(ang3))*sin(ang5) - 1.*cos(ang4)*sin(ang1)*sin(ang2)*sin(ang6) -
								1.*(cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4)*sin(ang6)) + dth4*(cos(ang6)*(1.*cos(ang4)*(cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3)) - 1.*sin(ang1)*sin(ang2)*sin(ang4)) +
								cos(ang5)*(-1.*cos(ang4)*sin(ang1)*sin(ang2) - (cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4))*sin(ang6)) +
							dth1*(cos(ang6)*(-1.*sin(ang1)*sin(ang3)*sin(ang4) + cos(ang1)*(1.*cos(ang4)*sin(ang2) + 1.*cos(ang2)*cos(ang3)*sin(ang4))) +
								(-(sin(ang1)*(cos(ang4)*cos(ang5)*sin(ang3) + cos(ang3)*sin(ang5))) + cos(ang1)*(-1.*cos(ang5)*sin(ang2)*sin(ang4) + cos(ang2)*(cos(ang3)*cos(ang4)*cos(ang5) - 1.*sin(ang3)*sin(ang5))))*sin(ang6)) +
							dth2*sin(ang1)*(cos(ang2)*(1.*cos(ang4)*cos(ang6) - 1.*cos(ang5)*sin(ang4)*sin(ang6)) + sin(ang2)*(1.*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth3*(cos(ang2)*sin(ang1)*(-1.*cos(ang6)*sin(ang3)*sin(ang4) - 1.*(cos(ang4)*cos(ang5)*sin(ang3) + 1.*cos(ang3)*sin(ang5))*sin(ang6)) + cos(ang1)*(-(sin(ang3)*sin(ang5)*sin(ang6)) + cos(ang3)*(1.*cos(ang6)*sin(ang4) + cos(ang4)*cos(ang5)*sin(ang6))));




			diff_jacob(4,6)=dth5*(cos(ang5)*(cos(ang1)*cos(ang3) - 1.*cos(ang2)*sin(ang1)*sin(ang3)) - 1.*(cos(ang2)*cos(ang3)*cos(ang4)*sin(ang1) + cos(ang1)*cos(ang4)*sin(ang3) - 1.*sin(ang1)*sin(ang2)*sin(ang4))*sin(ang5))*sin(ang6) +
							dth6*(cos(ang5)*cos(ang6)*(cos(ang2)*cos(ang3)*cos(ang4)*sin(ang1) + cos(ang1)*cos(ang4)*sin(ang3) - 1.*sin(ang1)*sin(ang2)*sin(ang4)) + cos(ang6)*(cos(ang1)*cos(ang3) - 1.*cos(ang2)*sin(ang1)*sin(ang3))*sin(ang5) - 1.*cos(ang4)*sin(ang1)*sin(ang2)*sin(ang6) -
								1.*(cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4)*sin(ang6)) + dth4*(cos(ang6)*(1.*cos(ang4)*(cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3)) - 1.*sin(ang1)*sin(ang2)*sin(ang4)) +
								cos(ang5)*(-1.*cos(ang4)*sin(ang1)*sin(ang2) - (cos(ang2)*cos(ang3)*sin(ang1) + cos(ang1)*sin(ang3))*sin(ang4))*sin(ang6)) +
							dth1*(cos(ang6)*(-1.*sin(ang1)*sin(ang3)*sin(ang4) + cos(ang1)*(1.*cos(ang4)*sin(ang2) + 1.*cos(ang2)*cos(ang3)*sin(ang4))) +
								(-(sin(ang1)*(cos(ang4)*cos(ang5)*sin(ang3) + cos(ang3)*sin(ang5))) + cos(ang1)*(-1.*cos(ang5)*sin(ang2)*sin(ang4) + cos(ang2)*(cos(ang3)*cos(ang4)*cos(ang5) - 1.*sin(ang3)*sin(ang5))))*sin(ang6)) +
							dth2*sin(ang1)*(cos(ang2)*(1.*cos(ang4)*cos(ang6) - 1.*cos(ang5)*sin(ang4)*sin(ang6)) + sin(ang2)*(1.*sin(ang3)*sin(ang5)*sin(ang6) + cos(ang3)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)))) +
							dth3*(cos(ang2)*sin(ang1)*(-1.*cos(ang6)*sin(ang3)*sin(ang4) - 1.*(cos(ang4)*cos(ang5)*sin(ang3) + 1.*cos(ang3)*sin(ang5))*sin(ang6)) + cos(ang1)*(-(sin(ang3)*sin(ang5)*sin(ang6)) + cos(ang3)*(1.*cos(ang6)*sin(ang4) + cos(ang4)*cos(ang5)*sin(ang6))));




			diff_jacob(5,0)=0;
			diff_jacob(5,1)=-1.*dth2*sin(ang2);
			diff_jacob(5,2)=1.*dth3*cos(ang3)*sin(ang2) + 1.*dth2*cos(ang2)*sin(ang3);
			diff_jacob(5,3)=1.*dth3*sin(ang2)*sin(ang3)*sin(ang4) + dth4*(-1.*cos(ang3)*cos(ang4)*sin(ang2) - 1.*cos(ang2)*sin(ang4)) + dth2*(-1.*cos(ang4)*sin(ang2) - 1.*cos(ang2)*cos(ang3)*sin(ang4));



			diff_jacob(5,4)=dth4*(1.*cos(ang2)*cos(ang4) - 1.*cos(ang3)*sin(ang2)*sin(ang4))*sin(ang5) + dth3*sin(ang2)*(1.*cos(ang3)*cos(ang5) - 1.*cos(ang4)*sin(ang3)*sin(ang5)) + dth5*(1.*cos(ang3)*cos(ang4)*cos(ang5)*sin(ang2) + 1.*cos(ang2)*cos(ang5)*sin(ang4) - 1.*sin(ang2)*sin(ang3)*sin(ang5)) +
			dth2*(-1.*sin(ang2)*sin(ang4)*sin(ang5) + cos(ang2)*(1.*cos(ang5)*sin(ang3) + 1.*cos(ang3)*cos(ang4)*sin(ang5)));



			diff_jacob(5,5)=1.*dth5*(1.*cos(ang5)*sin(ang2)*sin(ang3) + (1.*cos(ang3)*cos(ang4)*sin(ang2) + 1.*cos(ang2)*sin(ang4))*sin(ang5))*sin(ang6) + dth3*sin(ang2)*(1.*cos(ang6)*sin(ang3)*sin(ang4) + (1.*cos(ang4)*cos(ang5)*sin(ang3) + 1.*cos(ang3)*sin(ang5))*sin(ang6)) +
							dth6*(1.*cos(ang6)*sin(ang2)*sin(ang3)*sin(ang5) + cos(ang2)*(-1.*cos(ang5)*cos(ang6)*sin(ang4) - 1.*cos(ang4)*sin(ang6)) + cos(ang3)*sin(ang2)*(-1.*cos(ang4)*cos(ang5)*cos(ang6) + 1.*sin(ang4)*sin(ang6))) +
							dth4*(cos(ang2)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang3)*sin(ang2)*(-1.*cos(ang4)*cos(ang6) + 1.*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth2*(1.*cos(ang5)*sin(ang2)*sin(ang4)*sin(ang6) + cos(ang4)*(-1.*cos(ang6)*sin(ang2) - 1.*cos(ang2)*cos(ang3)*cos(ang5)*sin(ang6)) + cos(ang2)*(-1.*cos(ang3)*cos(ang6)*sin(ang4) + 1.*sin(ang3)*sin(ang5)*sin(ang6)));



			diff_jacob(5,6)=1.*dth5*(1.*cos(ang5)*sin(ang2)*sin(ang3) + (1.*cos(ang3)*cos(ang4)*sin(ang2) + 1.*cos(ang2)*sin(ang4))*sin(ang5))*sin(ang6) + dth3*sin(ang2)*(1.*cos(ang6)*sin(ang3)*sin(ang4) + (1.*cos(ang4)*cos(ang5)*sin(ang3) + 1.*cos(ang3)*sin(ang5))*sin(ang6)) +
							dth6*(1.*cos(ang6)*sin(ang2)*sin(ang3)*sin(ang5) + cos(ang2)*(-1.*cos(ang5)*cos(ang6)*sin(ang4) - 1.*cos(ang4)*sin(ang6)) + cos(ang3)*sin(ang2)*(-1.*cos(ang4)*cos(ang5)*cos(ang6) + 1.*sin(ang4)*sin(ang6))) +
							dth4*(cos(ang2)*(-1.*cos(ang6)*sin(ang4) - 1.*cos(ang4)*cos(ang5)*sin(ang6)) + cos(ang3)*sin(ang2)*(-1.*cos(ang4)*cos(ang6) + 1.*cos(ang5)*sin(ang4)*sin(ang6))) +
							dth2*(1.*cos(ang5)*sin(ang2)*sin(ang4)*sin(ang6) + cos(ang4)*(-1.*cos(ang6)*sin(ang2) - 1.*cos(ang2)*cos(ang3)*cos(ang5)*sin(ang6)) + cos(ang2)*(-1.*cos(ang3)*cos(ang6)*sin(ang4) + 1.*sin(ang3)*sin(ang5)*sin(ang6)));

			diff_jacob_3(0,0)=diff_jacob(0,0);
			diff_jacob_3(0,1)=diff_jacob(0,1);
			diff_jacob_3(0,2)=diff_jacob(0,2);
			diff_jacob_3(0,3)=diff_jacob(0,3);
			diff_jacob_3(0,4)=diff_jacob(0,4);
			diff_jacob_3(0,5)=diff_jacob(0,5);
			diff_jacob_3(0,6)=diff_jacob(0,6);

			diff_jacob_3(1,0)=diff_jacob(1,0);
			diff_jacob_3(1,1)=diff_jacob(1,1);
			diff_jacob_3(1,2)=diff_jacob(1,2);
			diff_jacob_3(1,3)=diff_jacob(1,3);
			diff_jacob_3(1,4)=diff_jacob(1,4);
			diff_jacob_3(1,5)=diff_jacob(1,5);
			diff_jacob_3(1,6)=diff_jacob(1,6);

			diff_jacob_3(2,0)=diff_jacob(2,0);
			diff_jacob_3(2,1)=diff_jacob(2,1);
			diff_jacob_3(2,2)=diff_jacob(2,2);
			diff_jacob_3(2,3)=diff_jacob(2,3);
			diff_jacob_3(2,4)=diff_jacob(2,4);
			diff_jacob_3(2,5)=diff_jacob(2,5);
			diff_jacob_3(2,6)=diff_jacob(2,6);

} */


// DIFFERENTIATION OF JACOBIAN
void jacob_diffjacob (){
	//Update Jacobian matrix
			//20190406 Results of Jacobian 6x7 - (correct)

			Jacob(0,0) = - 1.0*d5*(sin(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + 1.0*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) + 1.0*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1])) - 1.0*d7*(cos(current_joint_states[5])*(sin(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + 1.0*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) + 1.0*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1])) + sin(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + 1.0*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + sin(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2])))) - d3*sin(current_joint_states[0])*sin(current_joint_states[1]);
			Jacob(0,1) = 1.0*d5*(1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3])) - 1.0*d7*(sin(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[3]) + 1.0*cos(current_joint_states[0])*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1])) - cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4])) - cos(current_joint_states[5])*(1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3]))) + d3*cos(current_joint_states[0])*cos(current_joint_states[1]);
			Jacob(0,2) = 1.0*d7*(sin(current_joint_states[5])*(sin(current_joint_states[4])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) - cos(current_joint_states[3])*cos(current_joint_states[4])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))) - cos(current_joint_states[5])*sin(current_joint_states[3])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))) - 1.0*d5*sin(current_joint_states[3])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]));
			Jacob(0,3) = - 1.0*d5*(cos(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) + 1.0*cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) - 1.0*d7*(cos(current_joint_states[5])*(cos(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) + 1.0*cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) - cos(current_joint_states[4])*sin(current_joint_states[5])*(sin(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) - cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[1])));
			Jacob(0,4) = 1.0*d7*sin(current_joint_states[5])*(sin(current_joint_states[4])*(cos(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) + cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) - cos(current_joint_states[4])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2])));
			Jacob(0,5) = 1.0*d7*(sin(current_joint_states[5])*(sin(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[1])) - cos(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) + cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + sin(current_joint_states[4])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))));
			Jacob(0,6) = 0;

			Jacob(1,0) = d3*cos(current_joint_states[0])*sin(current_joint_states[1]) - d5*(sin(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) - cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[1])) - d7*(sin(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) + 1.0*cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + sin(current_joint_states[4])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))) + cos(current_joint_states[5])*(sin(current_joint_states[3])*(sin(current_joint_states[0])*sin(current_joint_states[2]) - cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) - cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[1])));
			Jacob(1,1) = d5*(cos(current_joint_states[1])*cos(current_joint_states[3])*sin(current_joint_states[0]) - cos(current_joint_states[2])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + d7*(cos(current_joint_states[5])*(cos(current_joint_states[1])*cos(current_joint_states[3])*sin(current_joint_states[0]) - cos(current_joint_states[2])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) - sin(current_joint_states[5])*(cos(current_joint_states[4])*(1.0*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[3]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1])) - 1.0*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]))) + d3*cos(current_joint_states[1])*sin(current_joint_states[0]);
			Jacob(1,2) = d5*sin(current_joint_states[3])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2])) - d7*(sin(current_joint_states[5])*(sin(current_joint_states[4])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + 1.0*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - cos(current_joint_states[3])*cos(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))) - cos(current_joint_states[5])*sin(current_joint_states[3])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2])));
			Jacob(1,3) = d5*(cos(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + d7*(cos(current_joint_states[5])*(cos(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) - cos(current_joint_states[4])*sin(current_joint_states[5])*(sin(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) + 1.0*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1])));
			Jacob(1,4) = -d7*sin(current_joint_states[5])*(sin(current_joint_states[4])*(cos(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - 1.0*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) - cos(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.0*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2])));
			Jacob(1,5) = d7*(cos(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - 1.0*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + sin(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.0*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))) - sin(current_joint_states[5])*(sin(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) + cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1])));
			Jacob(1,6) = 0;

			Jacob(2,0) = 0;
			Jacob(2,1) = 1.0*d7*(sin(current_joint_states[5])*(cos(current_joint_states[4])*(sin(current_joint_states[1])*sin(current_joint_states[3]) - cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3])) + 1.0*cos(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4])) - 1.0*cos(current_joint_states[5])*(cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.0*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3]))) - d3*sin(current_joint_states[1]) - d5*(cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.0*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3]));
			Jacob(2,2) = 1.0*d7*(sin(current_joint_states[5])*(1.0*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[4]) + cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2])) + 1.0*cos(current_joint_states[5])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[3])) + 1.0*d5*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[3]);
			Jacob(2,3) = - d5*(cos(current_joint_states[1])*sin(current_joint_states[3]) + 1.0*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1])) - 1.0*d7*(1.0*cos(current_joint_states[5])*(cos(current_joint_states[1])*sin(current_joint_states[3]) + 1.0*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1])) + cos(current_joint_states[4])*sin(current_joint_states[5])*(cos(current_joint_states[1])*cos(current_joint_states[3]) - cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3])));
			Jacob(2,4) = 1.0*d7*sin(current_joint_states[5])*(sin(current_joint_states[4])*(cos(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1])) + 1.0*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]));
			Jacob(2,5) = -1.0*d7*(cos(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1])) - 1.0*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4])) + 1.0*sin(current_joint_states[5])*(cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.0*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3])));
			Jacob(2,6) = 0;

			Jacob(3,0) = -1.0*sin(current_joint_states[0]);
			Jacob(3,1) = cos(current_joint_states[0])*sin(current_joint_states[1]);
			Jacob(3,2) = - 1.0*cos(current_joint_states[2])*sin(current_joint_states[0]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]);
			Jacob(3,3) = 1.0*cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[1]) - 1.0*sin(current_joint_states[3])*(1.0*sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2]));
			Jacob(3,4) = sin(current_joint_states[4])*(1.0*cos(current_joint_states[3])*(1.0*sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) + 1.0*cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) - 1.0*cos(current_joint_states[4])*(1.0*cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]));
			Jacob(3,5) = - 1.0*cos(current_joint_states[5])*(1.0*sin(current_joint_states[3])*(1.0*sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[1])) - 1.0*sin(current_joint_states[5])*(1.0*sin(current_joint_states[4])*(1.0*cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2])) + 1.0*cos(current_joint_states[4])*(1.0*cos(current_joint_states[3])*(1.0*sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) + 1.0*cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])));
			Jacob(3,6) = - 1.0*cos(current_joint_states[5])*(1.0*sin(current_joint_states[3])*(1.0*sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[1])) - 1.0*sin(current_joint_states[5])*(1.0*sin(current_joint_states[4])*(1.0*cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2])) + 1.0*cos(current_joint_states[4])*(1.0*cos(current_joint_states[3])*(1.0*sin(current_joint_states[0])*sin(current_joint_states[2]) - 1.0*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2])) + 1.0*cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])));


			Jacob(4,0) = cos(current_joint_states[0]);
			Jacob(4,1) = sin(current_joint_states[0])*sin(current_joint_states[1]);
			Jacob(4,2) = cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.0*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]);
			Jacob(4,3) = sin(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) + 1.0*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]);
			Jacob(4,4) = cos(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.0*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2])) - 1.0*sin(current_joint_states[4])*(cos(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - 1.0*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3]));
			Jacob(4,5) = sin(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - 1.0*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + sin(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.0*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))) + cos(current_joint_states[5])*(1.0*sin(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) + 1.0*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]));
			Jacob(4,6) = sin(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) - 1.0*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + sin(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.0*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))) + cos(current_joint_states[5])*(1.0*sin(current_joint_states[3])*(cos(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])) + 1.0*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]));

			Jacob(5,0) = 0;
			Jacob(5,1) = 1.0*cos(current_joint_states[1]);
			Jacob(5,2) = 1.0*sin(current_joint_states[1])*sin(current_joint_states[2]);
			Jacob(5,3) = 1.0*cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.0*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3]);
			Jacob(5,4) = sin(current_joint_states[4])*(1.0*cos(current_joint_states[1])*sin(current_joint_states[3]) + 1.0*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1])) + 1.0*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]);
			Jacob(5,5) = cos(current_joint_states[5])*(1.0*cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.0*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3])) - 1.0*sin(current_joint_states[5])*(1.0*cos(current_joint_states[4])*(1.0*cos(current_joint_states[1])*sin(current_joint_states[3]) + 1.0*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1])) - 1.0*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]));
			Jacob(5,6) = cos(current_joint_states[5])*(1.0*cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.0*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3])) - 1.0*sin(current_joint_states[5])*(1.0*cos(current_joint_states[4])*(1.0*cos(current_joint_states[1])*sin(current_joint_states[3]) + 1.0*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1])) - 1.0*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]));

			Jacob_3(0,0)=Jacob(0,0);
			Jacob_3(0,1)=Jacob(0,1);
			Jacob_3(0,2)=Jacob(0,2);
			Jacob_3(0,3)=Jacob(0,3);
			Jacob_3(0,4)=Jacob(0,4);
			Jacob_3(0,5)=Jacob(0,5);
			Jacob_3(0,6)=Jacob(0,6);

			Jacob_3(1,0)=Jacob(1,0);
			Jacob_3(1,1)=Jacob(1,1);
			Jacob_3(1,2)=Jacob(1,2);
			Jacob_3(1,3)=Jacob(1,3);
			Jacob_3(1,4)=Jacob(1,4);
			Jacob_3(1,5)=Jacob(1,5);
			Jacob_3(1,6)=Jacob(1,6);

			Jacob_3(2,0)=Jacob(2,0);
			Jacob_3(2,1)=Jacob(2,1);
			Jacob_3(2,2)=Jacob(2,2);
			Jacob_3(2,3)=Jacob(2,3);
			Jacob_3(2,4)=Jacob(2,4);
			Jacob_3(2,5)=Jacob(2,5);
			Jacob_3(2,6)=Jacob(2,6);


			//diff jacobian
			diff_jacob(0,0)=-1.*d7*dth5*(cos(current_joint_states[0])*(cos(current_joint_states[2])*cos(current_joint_states[4]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])) + sin(current_joint_states[0])*(sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[4])*sin(current_joint_states[2]) - 1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4]))))*sin(current_joint_states[5]) +
							dth4*(sin(current_joint_states[0])*sin(current_joint_states[1])*(1.*d5*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[0])*sin(current_joint_states[2])*(-1.*d5*cos(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[5]) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
								cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth3*(cos(current_joint_states[0])*(1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(-1.*d5*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) +
								cos(current_joint_states[1])*sin(current_joint_states[0])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*(1.*d5*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth2*sin(current_joint_states[0])*(-1.*cos(current_joint_states[1])*(d3 + cos(current_joint_states[3])*(1.*d5 + 1.*d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + sin(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) -
							1.*d7*dth6*(sin(current_joint_states[0])*sin(current_joint_states[1])*(-1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[2]) + cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[4]) - sin(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])) -
								1.*cos(current_joint_states[1])*sin(current_joint_states[0])*(cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) + 1.*sin(current_joint_states[3])*sin(current_joint_states[5])))) +
							dth1*(sin(current_joint_states[0])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) -
								1.*cos(current_joint_states[0])*(sin(current_joint_states[1])*(d3 + cos(current_joint_states[3])*(1.*d5 + 1.*d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(1.*d5*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))));



			diff_jacob(0,1)=1.*d7*dth5*cos(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]))*sin(current_joint_states[5]) +
							dth3*cos(current_joint_states[0])*sin(current_joint_states[1])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) -
							1.*d7*dth6*cos(current_joint_states[0])*(-(cos(current_joint_states[5])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4])) + cos(current_joint_states[1])*(cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(1.*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth4*cos(current_joint_states[0])*(cos(current_joint_states[1])*((-1.*d5 - 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth2*cos(current_joint_states[0])*(sin(current_joint_states[1])*(-1.*d3 + cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(-1.*d5*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth1*sin(current_joint_states[0])*(cos(current_joint_states[1])*(-1.*d3 + cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + sin(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(1.*d5*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))));



			diff_jacob(0,2)=1.*d7*dth5*(sin(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])) - 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*(cos(current_joint_states[2])*cos(current_joint_states[4]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])))*sin(current_joint_states[5]) +
							dth4*(cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
							1.*d7*dth6*(cos(current_joint_states[5])*(-(cos(current_joint_states[2])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[4]))) + sin(current_joint_states[2])*(-1.*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[3])*cos(current_joint_states[4]) + sin(current_joint_states[0])*sin(current_joint_states[4]))) + (cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))*sin(current_joint_states[3])*sin(current_joint_states[5])) +
							dth2*cos(current_joint_states[0])*sin(current_joint_states[1])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) +
							dth3*(cos(current_joint_states[0])*cos(current_joint_states[1])*(1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(-1.*d5*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) +
								sin(current_joint_states[0])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*(1.*d5*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth1*(cos(current_joint_states[0])*(1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(-1.*d5*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) +
								cos(current_joint_states[1])*sin(current_joint_states[0])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*(1.*d5*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))));



			diff_jacob(0,3)=-1.*d7*dth5*(-(cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[1])) + (-1.*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2]) + sin(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]))*sin(current_joint_states[4])*sin(current_joint_states[5]) +
							dth3*(cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
							dth6*(-1.*d7*cos(current_joint_states[4])*cos(current_joint_states[5])*(-1.*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3]) + cos(current_joint_states[0])*(cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3]))) + 1.*d7*(cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[0])*(-1.*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3]) + 1.*sin(current_joint_states[1])*sin(current_joint_states[3])))*sin(current_joint_states[5])) +
							dth1*(sin(current_joint_states[0])*sin(current_joint_states[1])*(1.*d5*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[0])*sin(current_joint_states[2])*(-1.*d5*cos(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[5]) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
								cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth2*cos(current_joint_states[0])*(cos(current_joint_states[1])*((-1.*d5 - 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth4*(sin(current_joint_states[0])*sin(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[0])*
								(cos(current_joint_states[3])*((-1.*d5 - 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[1]) - 1.*d7*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[4])*sin(current_joint_states[5])) + sin(current_joint_states[3])*(cos(current_joint_states[1])*cos(current_joint_states[2])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[5]))));



			diff_jacob(0,4)=1.*d7*dth6*cos(current_joint_states[5])*(-(cos(current_joint_states[4])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))) + (cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[0])*(-1.*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3]) + sin(current_joint_states[1])*sin(current_joint_states[3])))*sin(current_joint_states[4])) +
							1.*d7*dth4*(-1.*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3]) + cos(current_joint_states[0])*(cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3])))*sin(current_joint_states[4])*sin(current_joint_states[5]) + 1.*d7*dth2*cos(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]))*sin(current_joint_states[5]) +
							1.*d7*dth3*(sin(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[0])*cos(current_joint_states[1])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[4]) + 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])))*sin(current_joint_states[5]) -
							1.*d7*dth1*(cos(current_joint_states[0])*(cos(current_joint_states[2])*cos(current_joint_states[4]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])) + sin(current_joint_states[0])*(sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[4])*sin(current_joint_states[2]) - 1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4]))))*sin(current_joint_states[5]) +
							1.*d7*dth5*(sin(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*sin(current_joint_states[4])) + cos(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) + sin(current_joint_states[2])*sin(current_joint_states[4]))))*sin(current_joint_states[5]);



			diff_jacob(0,5)=-1.*d7*dth5*cos(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2])) - (cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[0])*(-1.*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3]) + sin(current_joint_states[1])*sin(current_joint_states[3])))*sin(current_joint_states[4])) +
								1.*d7*dth3*(-(cos(current_joint_states[5])*(cos(current_joint_states[3])*cos(current_joint_states[4])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2])) + (cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2]) - sin(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[4]))) + (cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))*sin(current_joint_states[3])*sin(current_joint_states[5])) +
								1.*d7*dth6*(cos(current_joint_states[5])*(sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3]) + cos(current_joint_states[0])*(-1.*cos(current_joint_states[3])*sin(current_joint_states[1]) - 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3]))) +
									(sin(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*sin(current_joint_states[4])) + cos(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) + sin(current_joint_states[2])*sin(current_joint_states[4]))))*sin(current_joint_states[5])) +
								1.*d7*dth2*cos(current_joint_states[0])*(cos(current_joint_states[5])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) + 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
								1.*d7*dth4*(sin(current_joint_states[0])*sin(current_joint_states[2])*(cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) + cos(current_joint_states[3])*sin(current_joint_states[5])) - 1.*cos(current_joint_states[0])*(cos(current_joint_states[3])*(cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[5])) + sin(current_joint_states[3])*(1.*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[5])))) +
								1.*d7*dth1*(sin(current_joint_states[0])*sin(current_joint_states[1])*(cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) - cos(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[2]) + cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[4]) - sin(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
									cos(current_joint_states[1])*sin(current_joint_states[0])*(cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) + 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))));



			diff_jacob(0,6)=0;




			diff_jacob(1,0)=-1.*d7*dth5*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(cos(current_joint_states[4])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[0])*(1.*cos(current_joint_states[1])*cos(current_joint_states[4])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4])))*sin(current_joint_states[5]) +
							dth6*(-(d7*cos(current_joint_states[5])*(sin(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*sin(current_joint_states[4])) + cos(current_joint_states[0])*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) + 1.*sin(current_joint_states[2])*sin(current_joint_states[4]))))) +
								d7*(sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3]) - cos(current_joint_states[0])*(cos(current_joint_states[3])*sin(current_joint_states[1]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3])))*sin(current_joint_states[5])) +
							dth3*(sin(current_joint_states[2])*(d7*sin(current_joint_states[0])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[0])*cos(current_joint_states[1])*((d5 - d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) - d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) +
								cos(current_joint_states[2])*(-1.*d7*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[0])*((d5 - 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth2*cos(current_joint_states[0])*(cos(current_joint_states[1])*(d3 + cos(current_joint_states[3])*(d5 + 1.*d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) - 1.*sin(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth4*(-(sin(current_joint_states[0])*sin(current_joint_states[2])*(cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) + cos(current_joint_states[0])*(sin(current_joint_states[1])*((-1.*d5 - 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) +
									cos(current_joint_states[1])*cos(current_joint_states[2])*(cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])))) + dth1*(-(cos(current_joint_states[0])*(d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) -
								1.*sin(current_joint_states[0])*(sin(current_joint_states[1])*(d3 + cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))));


			diff_jacob(1,1)=d7*dth5*sin(current_joint_states[0])*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]))*sin(current_joint_states[5]) + dth3*sin(current_joint_states[0])*sin(current_joint_states[1])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) -
							1.*d7*dth6*sin(current_joint_states[0])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) + cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth4*sin(current_joint_states[0])*(cos(current_joint_states[1])*((-1.*d5 - 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) -
							1.*dth2*sin(current_joint_states[0])*(sin(current_joint_states[1])*(d3 + cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth1*cos(current_joint_states[0])*(cos(current_joint_states[1])*(d3 + cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) - 1.*sin(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))));



			diff_jacob(1,2)=-(d7*dth5*(cos(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[1])*sin(current_joint_states[0])*(1.*cos(current_joint_states[2])*cos(current_joint_states[4]) - cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])))*sin(current_joint_states[5])) +
							dth4*(cos(current_joint_states[0])*cos(current_joint_states[2]) - cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))*(cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + dth2*sin(current_joint_states[0])*sin(current_joint_states[1])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) +
							dth1*(-1.*sin(current_joint_states[2])*(-1.*d7*sin(current_joint_states[0])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[0])*cos(current_joint_states[1])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) -
								1.*cos(current_joint_states[2])*(1.*d7*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[0])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth3*(-1.*cos(current_joint_states[1])*sin(current_joint_states[0])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) -
								1.*cos(current_joint_states[0])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) -
							d7*dth6*(cos(current_joint_states[1])*sin(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[4]) - 1.*sin(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[0])*(cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(-(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])) + sin(current_joint_states[3])*sin(current_joint_states[5]))));



			diff_jacob(1,3)=d7*dth5*(1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) + (cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]))*sin(current_joint_states[4])*sin(current_joint_states[5]) + dth3*(cos(current_joint_states[0])*cos(current_joint_states[2]) - cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))*(cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
							d7*dth6*(-(cos(current_joint_states[4])*cos(current_joint_states[5])*(1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) + (cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]))) + (-(cos(current_joint_states[3])*(cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))) + sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[5])) +
							dth2*sin(current_joint_states[0])*(cos(current_joint_states[1])*((-1.*d5 - 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth4*(-1.*sin(current_joint_states[3])*(cos(current_joint_states[1])*cos(current_joint_states[2])*(d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[0]) + cos(current_joint_states[0])*(d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[2]) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[5])) -
								1.*cos(current_joint_states[3])*(1.*d7*cos(current_joint_states[0])*cos(current_joint_states[4])*sin(current_joint_states[2])*sin(current_joint_states[5]) + sin(current_joint_states[0])*((d5 + d7*cos(current_joint_states[5]))*sin(current_joint_states[1]) + 1.*d7*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth1*(-(sin(current_joint_states[0])*sin(current_joint_states[2])*(cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) + cos(current_joint_states[0])*(sin(current_joint_states[1])*((-1.*d5 - 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) - 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) +
									cos(current_joint_states[1])*cos(current_joint_states[2])*(cos(current_joint_states[3])*(d5 + d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))));



			diff_jacob(1,4)=-(d7*dth6*cos(current_joint_states[5])*(cos(current_joint_states[0])*(-(cos(current_joint_states[2])*cos(current_joint_states[4])) + cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])) + sin(current_joint_states[0])*(-1.*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(1.*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4]))))) -
							d7*dth4*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) - (cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]))*sin(current_joint_states[4])*sin(current_joint_states[5]) + 1.*d7*dth2*sin(current_joint_states[0])*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]))*sin(current_joint_states[5]) -
							d7*dth5*(-1.*cos(current_joint_states[4])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*sin(current_joint_states[4])) + cos(current_joint_states[1])*sin(current_joint_states[0])*(cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) - 1.*sin(current_joint_states[2])*sin(current_joint_states[4])))*sin(current_joint_states[5]) -
							d7*dth3*(cos(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[1])*sin(current_joint_states[0])*(1.*cos(current_joint_states[2])*cos(current_joint_states[4]) - cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])))*sin(current_joint_states[5]) -
							1.*d7*dth1*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(cos(current_joint_states[4])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[0])*(1.*cos(current_joint_states[1])*cos(current_joint_states[4])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4])))*sin(current_joint_states[5]);



			diff_jacob(1,5)=d7*dth5*cos(current_joint_states[5])*(cos(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2])) - 1.*(cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4])) +
							d7*dth4*(cos(current_joint_states[4])*cos(current_joint_states[5])*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) - (cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3])) + (-(cos(current_joint_states[3])*(cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))) + sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[5])) +
							d7*dth1*(cos(current_joint_states[5])*(-(sin(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*sin(current_joint_states[4]))) + cos(current_joint_states[0])*(-1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[1])*(cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) - 1.*sin(current_joint_states[2])*sin(current_joint_states[4])))) +
								(sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3]) - cos(current_joint_states[0])*(cos(current_joint_states[3])*sin(current_joint_states[1]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3])))*sin(current_joint_states[5])) +
							d7*dth6*(-(cos(current_joint_states[5])*(cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) + (cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]))) -
								(-1.*cos(current_joint_states[4])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*sin(current_joint_states[4])) + cos(current_joint_states[1])*sin(current_joint_states[0])*(cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) - 1.*sin(current_joint_states[2])*sin(current_joint_states[4])))*sin(current_joint_states[5])) -
							1.*d7*dth2*sin(current_joint_states[0])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) + cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							d7*dth3*(cos(current_joint_states[1])*sin(current_joint_states[0])*(-(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[2])) - 1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[4]) + sin(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])) - cos(current_joint_states[0])*(cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(-(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])) + sin(current_joint_states[3])*sin(current_joint_states[5]))));



			diff_jacob(1,6)=0;





			diff_jacob(2,0)=0;


			diff_jacob(2,1)=1.*d7*dth5*(-1.*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(1.*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])))*sin(current_joint_states[5]) + dth3*cos(current_joint_states[1])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) +
							dth4*(sin(current_joint_states[1])*((d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[1])*cos(current_joint_states[2])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth2*(-1.*cos(current_joint_states[1])*(d3 + cos(current_joint_states[3])*(d5 + 1.*d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + sin(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							1.*d7*dth6*(sin(current_joint_states[1])*(cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) - 1.*cos(current_joint_states[1])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))));



			diff_jacob(2,2)=1.*d7*dth5*sin(current_joint_states[1])*(1.*cos(current_joint_states[2])*cos(current_joint_states[4]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4]))*sin(current_joint_states[5]) + dth4*sin(current_joint_states[1])*sin(current_joint_states[2])*(cos(current_joint_states[3])*(1.*d5 + 1.*d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
							1.*d7*dth6*sin(current_joint_states[1])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[4]) - 1.*sin(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
							dth3*sin(current_joint_states[1])*(-1.*d7*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))) +
							dth2*cos(current_joint_states[1])*(1.*d7*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + sin(current_joint_states[2])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])));



			diff_jacob(2,3)=1.*d7*dth5*(cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4])*sin(current_joint_states[5]) + dth3*sin(current_joint_states[1])*sin(current_joint_states[2])*(cos(current_joint_states[3])*(1.*d5 + 1.*d7*cos(current_joint_states[5])) - 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) -
							1.*d7*dth6*(-1.*cos(current_joint_states[2])*sin(current_joint_states[1])*(cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth4*(cos(current_joint_states[2])*sin(current_joint_states[1])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth2*(sin(current_joint_states[1])*((1.*d5 + 1.*d7*cos(current_joint_states[5]))*sin(current_joint_states[3]) + 1.*d7*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[1])*cos(current_joint_states[2])*(cos(current_joint_states[3])*(-1.*d5 - 1.*d7*cos(current_joint_states[5])) + 1.*d7*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])));




			diff_jacob(2,4)=1.*d7*dth6*cos(current_joint_states[5])*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4])) + 1.*d7*dth4*(cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4])*sin(current_joint_states[5]) +
							1.*d7*dth3*sin(current_joint_states[1])*(1.*cos(current_joint_states[2])*cos(current_joint_states[4]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4]))*sin(current_joint_states[5]) + 1.*d7*dth5*(cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[1]) + cos(current_joint_states[1])*cos(current_joint_states[4])*sin(current_joint_states[3]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]))*sin(current_joint_states[5]) +
							1.*d7*dth2*(-(sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[1])*(1.*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])))*sin(current_joint_states[5]);



			diff_jacob(2,5)=-1.*d7*dth5*cos(current_joint_states[5])*(-1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) - (cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4])) + 1.*d7*dth3*sin(current_joint_states[1])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[4]) - 1.*sin(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
							dth6*(-1.*d7*cos(current_joint_states[5])*(cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3])) + 1.*d7*(cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[1]) + cos(current_joint_states[1])*cos(current_joint_states[4])*sin(current_joint_states[3]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]))*sin(current_joint_states[5])) -
							1.*d7*dth4*(cos(current_joint_states[2])*sin(current_joint_states[1])*(-1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))) -
							1.*d7*dth2*(sin(current_joint_states[1])*(-1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))));




			diff_jacob(2,6)=0;




			diff_jacob(3,0)=-1.*dth1*cos(current_joint_states[0]);



			diff_jacob(3,1)=dth2*cos(current_joint_states[0])*cos(current_joint_states[1]) - dth1*sin(current_joint_states[0])*sin(current_joint_states[1]);



			diff_jacob(3,2)=1.*dth2*cos(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[2]) + dth3*(-1.*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[2]) + 1.*sin(current_joint_states[0])*sin(current_joint_states[2])) + dth1*(-1.*cos(current_joint_states[0])*cos(current_joint_states[2]) + 1.*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]));



			diff_jacob(3,3)=dth3*(-1.*cos(current_joint_states[2])*sin(current_joint_states[0]) - 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]))*sin(current_joint_states[3]) + dth2*cos(current_joint_states[0])*(1.*cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3])) + dth1*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) + (-1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) - 1.*cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3])) +
							dth4*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[0])*(1.*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[3])));



			diff_jacob(3,4)=1.*dth4*(-1.*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3]) + cos(current_joint_states[0])*(1.*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3])))*sin(current_joint_states[4]) + 1.*dth2*cos(current_joint_states[0])*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4])) +
							dth3*(sin(current_joint_states[0])*(1.*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[0])*cos(current_joint_states[1])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[4]) + 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4]))) +
							dth1*(cos(current_joint_states[0])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[4]) + 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])) + sin(current_joint_states[0])*(-1.*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(1.*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])))) +
							dth5*(sin(current_joint_states[0])*(1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*sin(current_joint_states[4])) + cos(current_joint_states[0])*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) + 1.*sin(current_joint_states[2])*sin(current_joint_states[4]))));



			diff_jacob(3,5)=-1.*dth5*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(1.*cos(current_joint_states[4])*sin(current_joint_states[0]) + 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[0])*(1.*cos(current_joint_states[1])*cos(current_joint_states[4])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4])))*sin(current_joint_states[5]) +
							dth1*(-1.*cos(current_joint_states[0])*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[3]) + 1.*cos(current_joint_states[4])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[5]) - 1.*cos(current_joint_states[0])*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) +
								cos(current_joint_states[3])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[0])*sin(current_joint_states[1]) + cos(current_joint_states[4])*(-1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) - 1.*cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[5])) + cos(current_joint_states[1])*sin(current_joint_states[0])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]))) +
							dth3*(cos(current_joint_states[2])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[0])*sin(current_joint_states[3]) + (-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[0]) - 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[4]))*sin(current_joint_states[5])) + sin(current_joint_states[2])*(1.*sin(current_joint_states[0])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[0])*cos(current_joint_states[1])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth2*cos(current_joint_states[0])*(cos(current_joint_states[1])*(1.*cos(current_joint_states[3])*cos(current_joint_states[5]) - 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + sin(current_joint_states[1])*(1.*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth6*(sin(current_joint_states[0])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[2]) - 1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[4]) + 1.*sin(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
								cos(current_joint_states[0])*(sin(current_joint_states[1])*(-1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(1.*cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[4]) - 1.*cos(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])))) +
							dth4*(sin(current_joint_states[0])*sin(current_joint_states[2])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[5]) + 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[0])*(sin(current_joint_states[1])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[1])*cos(current_joint_states[2])*(1.*cos(current_joint_states[3])*cos(current_joint_states[5]) - 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))));



			diff_jacob(3,6)=-1.*dth5*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[2])*(1.*cos(current_joint_states[4])*sin(current_joint_states[0]) + 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[0])*(1.*cos(current_joint_states[1])*cos(current_joint_states[4])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4])))*sin(current_joint_states[5]) +
							dth1*(-1.*cos(current_joint_states[0])*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[3]) + 1.*cos(current_joint_states[4])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[5]) - 1.*cos(current_joint_states[0])*cos(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) +
								cos(current_joint_states[3])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[0])*sin(current_joint_states[1]) + cos(current_joint_states[4])*(-1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) - 1.*cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[5])) + cos(current_joint_states[1])*sin(current_joint_states[0])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]))) +
							dth3*(cos(current_joint_states[2])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[0])*sin(current_joint_states[3]) + (-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[0]) - 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[4]))*sin(current_joint_states[5])) + sin(current_joint_states[2])*(1.*sin(current_joint_states[0])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[0])*cos(current_joint_states[1])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth2*cos(current_joint_states[0])*(cos(current_joint_states[1])*(1.*cos(current_joint_states[3])*cos(current_joint_states[5]) - 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + sin(current_joint_states[1])*(1.*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth6*(sin(current_joint_states[0])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[2]) - 1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[4]) + 1.*sin(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])) +
								cos(current_joint_states[0])*(sin(current_joint_states[1])*(-1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(1.*cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) - 1.*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[4]) - 1.*cos(current_joint_states[2])*sin(current_joint_states[3])*sin(current_joint_states[5])))) +
							dth4*(sin(current_joint_states[0])*sin(current_joint_states[2])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[5]) + 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[0])*(sin(current_joint_states[1])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[1])*cos(current_joint_states[2])*(1.*cos(current_joint_states[3])*cos(current_joint_states[5]) - 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))));




			diff_jacob(4,0)=-(dth1*sin(current_joint_states[0]));
			diff_jacob(4,1)=dth2*cos(current_joint_states[1])*sin(current_joint_states[0]) + dth1*cos(current_joint_states[0])*sin(current_joint_states[1]);
			diff_jacob(4,2)=1.*dth2*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[2]) + dth3*(-1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) - cos(current_joint_states[0])*sin(current_joint_states[2])) + dth1*(-(cos(current_joint_states[2])*sin(current_joint_states[0])) - 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2]));



			diff_jacob(4,3)=dth3*(cos(current_joint_states[0])*cos(current_joint_states[2]) - cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]) + dth2*sin(current_joint_states[0])*(1.*cos(current_joint_states[1])*cos(current_joint_states[3]) - cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3])) + dth4*(cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) +
			dth1*(-(sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3])) + cos(current_joint_states[0])*(1.*cos(current_joint_states[3])*sin(current_joint_states[1]) + cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3])));




			diff_jacob(4,4)=dth4*(1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) + 1.*(cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]))*sin(current_joint_states[4]) +
							dth5*(-1.*cos(current_joint_states[4])*(cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) - 1.*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[4])) +
							dth2*sin(current_joint_states[0])*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4])) +
							dth1*(-1.*cos(current_joint_states[4])*(cos(current_joint_states[2])*sin(current_joint_states[0]) + 1.*cos(current_joint_states[0])*cos(current_joint_states[1])*sin(current_joint_states[2])) - 1.*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[2]) + cos(current_joint_states[0])*(cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[3])))*sin(current_joint_states[4])) +
							dth3*(-1.*cos(current_joint_states[0])*(cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])) + cos(current_joint_states[1])*sin(current_joint_states[0])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[4]) + 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])));




			diff_jacob(4,5)=dth5*(cos(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2])) - 1.*(cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]))*sin(current_joint_states[5]) +
							dth6*(cos(current_joint_states[4])*cos(current_joint_states[5])*(cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + cos(current_joint_states[5])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[4]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[5]) -
								1.*(cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3])*sin(current_joint_states[5])) + dth4*(cos(current_joint_states[5])*(1.*cos(current_joint_states[3])*(cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2])) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) +
								cos(current_joint_states[4])*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) - (cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]))*sin(current_joint_states[5])) +
							dth1*(cos(current_joint_states[5])*(-1.*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3]) + cos(current_joint_states[0])*(1.*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3]))) +
								(-(sin(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*sin(current_joint_states[4]))) + cos(current_joint_states[0])*(-1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[1])*(cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) - 1.*sin(current_joint_states[2])*sin(current_joint_states[4]))))*sin(current_joint_states[5])) +
							dth2*sin(current_joint_states[0])*(cos(current_joint_states[1])*(1.*cos(current_joint_states[3])*cos(current_joint_states[5]) - 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + sin(current_joint_states[1])*(1.*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth3*(cos(current_joint_states[1])*sin(current_joint_states[0])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[3]) - 1.*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*sin(current_joint_states[4]))*sin(current_joint_states[5])) + cos(current_joint_states[0])*(-(sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[2])*(1.*cos(current_joint_states[5])*sin(current_joint_states[3]) + cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))));




			diff_jacob(4,6)=dth5*(cos(current_joint_states[4])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2])) - 1.*(cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]))*sin(current_joint_states[5]) +
							dth6*(cos(current_joint_states[4])*cos(current_joint_states[5])*(cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[0]) + cos(current_joint_states[0])*cos(current_joint_states[3])*sin(current_joint_states[2]) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) + cos(current_joint_states[5])*(cos(current_joint_states[0])*cos(current_joint_states[2]) - 1.*cos(current_joint_states[1])*sin(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[4]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[5]) -
								1.*(cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3])*sin(current_joint_states[5])) + dth4*(cos(current_joint_states[5])*(1.*cos(current_joint_states[3])*(cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2])) - 1.*sin(current_joint_states[0])*sin(current_joint_states[1])*sin(current_joint_states[3])) +
								cos(current_joint_states[4])*(-1.*cos(current_joint_states[3])*sin(current_joint_states[0])*sin(current_joint_states[1]) - (cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[0]) + cos(current_joint_states[0])*sin(current_joint_states[2]))*sin(current_joint_states[3]))*sin(current_joint_states[5])) +
							dth1*(cos(current_joint_states[5])*(-1.*sin(current_joint_states[0])*sin(current_joint_states[2])*sin(current_joint_states[3]) + cos(current_joint_states[0])*(1.*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3]))) +
								(-(sin(current_joint_states[0])*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + cos(current_joint_states[2])*sin(current_joint_states[4]))) + cos(current_joint_states[0])*(-1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3]) + cos(current_joint_states[1])*(cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4]) - 1.*sin(current_joint_states[2])*sin(current_joint_states[4]))))*sin(current_joint_states[5])) +
							dth2*sin(current_joint_states[0])*(cos(current_joint_states[1])*(1.*cos(current_joint_states[3])*cos(current_joint_states[5]) - 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5])) + sin(current_joint_states[1])*(1.*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5]) + cos(current_joint_states[2])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])))) +
							dth3*(cos(current_joint_states[1])*sin(current_joint_states[0])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[3]) - 1.*(cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*sin(current_joint_states[4]))*sin(current_joint_states[5])) + cos(current_joint_states[0])*(-(sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[2])*(1.*cos(current_joint_states[5])*sin(current_joint_states[3]) + cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5]))));




			diff_jacob(5,0)=0;
			diff_jacob(5,1)=-1.*dth2*sin(current_joint_states[1]);
			diff_jacob(5,2)=1.*dth3*cos(current_joint_states[2])*sin(current_joint_states[1]) + 1.*dth2*cos(current_joint_states[1])*sin(current_joint_states[2]);
			diff_jacob(5,3)=1.*dth3*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[3]) + dth4*(-1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) - 1.*cos(current_joint_states[1])*sin(current_joint_states[3])) + dth2*(-1.*cos(current_joint_states[3])*sin(current_joint_states[1]) - 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*sin(current_joint_states[3]));



			diff_jacob(5,4)=dth4*(1.*cos(current_joint_states[1])*cos(current_joint_states[3]) - 1.*cos(current_joint_states[2])*sin(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]) + dth3*sin(current_joint_states[1])*(1.*cos(current_joint_states[2])*cos(current_joint_states[4]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[2])*sin(current_joint_states[4])) + dth5*(1.*cos(current_joint_states[2])*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*cos(current_joint_states[4])*sin(current_joint_states[3]) - 1.*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4])) +
			dth2*(-1.*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(1.*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[4])));



			diff_jacob(5,5)=1.*dth5*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]))*sin(current_joint_states[5]) + dth3*sin(current_joint_states[1])*(1.*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[3]) + (1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*sin(current_joint_states[4]))*sin(current_joint_states[5])) +
							dth6*(1.*cos(current_joint_states[5])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) + 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth4*(cos(current_joint_states[1])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[5]) + 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth2*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[5]) + cos(current_joint_states[3])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[1]) - 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5])));



			diff_jacob(5,6)=1.*dth5*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[2]) + (1.*cos(current_joint_states[2])*cos(current_joint_states[3])*sin(current_joint_states[1]) + 1.*cos(current_joint_states[1])*sin(current_joint_states[3]))*sin(current_joint_states[4]))*sin(current_joint_states[5]) + dth3*sin(current_joint_states[1])*(1.*cos(current_joint_states[5])*sin(current_joint_states[2])*sin(current_joint_states[3]) + (1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[2]) + 1.*cos(current_joint_states[2])*sin(current_joint_states[4]))*sin(current_joint_states[5])) +
							dth6*(1.*cos(current_joint_states[5])*sin(current_joint_states[1])*sin(current_joint_states[2])*sin(current_joint_states[4]) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[4])*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[4])*cos(current_joint_states[5]) + 1.*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth4*(cos(current_joint_states[1])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[3]) - 1.*cos(current_joint_states[3])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[2])*sin(current_joint_states[1])*(-1.*cos(current_joint_states[3])*cos(current_joint_states[5]) + 1.*cos(current_joint_states[4])*sin(current_joint_states[3])*sin(current_joint_states[5]))) +
							dth2*(1.*cos(current_joint_states[4])*sin(current_joint_states[1])*sin(current_joint_states[3])*sin(current_joint_states[5]) + cos(current_joint_states[3])*(-1.*cos(current_joint_states[5])*sin(current_joint_states[1]) - 1.*cos(current_joint_states[1])*cos(current_joint_states[2])*cos(current_joint_states[4])*sin(current_joint_states[5])) + cos(current_joint_states[1])*(-1.*cos(current_joint_states[2])*cos(current_joint_states[5])*sin(current_joint_states[3]) + 1.*sin(current_joint_states[2])*sin(current_joint_states[4])*sin(current_joint_states[5])));

			diff_jacob_3(0,0)=diff_jacob(0,0);
			diff_jacob_3(0,1)=diff_jacob(0,1);
			diff_jacob_3(0,2)=diff_jacob(0,2);
			diff_jacob_3(0,3)=diff_jacob(0,3);
			diff_jacob_3(0,4)=diff_jacob(0,4);
			diff_jacob_3(0,5)=diff_jacob(0,5);
			diff_jacob_3(0,6)=diff_jacob(0,6);

			diff_jacob_3(1,0)=diff_jacob(1,0);
			diff_jacob_3(1,1)=diff_jacob(1,1);
			diff_jacob_3(1,2)=diff_jacob(1,2);
			diff_jacob_3(1,3)=diff_jacob(1,3);
			diff_jacob_3(1,4)=diff_jacob(1,4);
			diff_jacob_3(1,5)=diff_jacob(1,5);
			diff_jacob_3(1,6)=diff_jacob(1,6);

			diff_jacob_3(2,0)=diff_jacob(2,0);
			diff_jacob_3(2,1)=diff_jacob(2,1);
			diff_jacob_3(2,2)=diff_jacob(2,2);
			diff_jacob_3(2,3)=diff_jacob(2,3);
			diff_jacob_3(2,4)=diff_jacob(2,4);
			diff_jacob_3(2,5)=diff_jacob(2,5);
			diff_jacob_3(2,6)=diff_jacob(2,6);

}
//***************************************Friction compensate******************************************

double friction_LuGre(int axis, double ang_v) // axis , ang_vel
{
	double al_0[7] = {0,0,0,0,0,0,0};
	double al_1[7] = {0,0,0,0,0,0,0};
	double av_s[7] = {0,0,0,0,0,0,0};
	double sig_0[7] = {0,0,0,0,0,0,0};
	double sig_1[7] = {0,0,0,0,0,0,0};
	double sig_2[7] = {0,0,0,0,0,0,0};
	double c_time = 0.001; // control hz

	if (axis == 0)
		{
		//1j

		al_0[0] = 30.5165;
		al_1[0] = -15.9017;
		av_s[0] = 0.183;
		sig_2[0] = 38.2933;

		sig_0[0] = 8090;//1009;
		sig_1[0] =200;//13.3075;

		}


	if (axis == 1)
		{
		//2j seems good but not sure...

		al_0[1] = 47.4133;
		al_1[1] = -0.9973;
		av_s[1] = 0.0027;
		sig_2[1] = 226.4123;


		sig_0[1] = 1890700;//*0.1;//1009;
		sig_1[1] = 302040*0.02;

		}


	if (axis == 2)
		{
		//3j done!
		//6factor done!

		al_0[2] =45.6405;
		al_1[2] = -3.1568;
		av_s[2] =-0.0566;
		sig_2[2] = 110.8247;

		sig_0[2] = 1174059;// high -> initail moving hard
		sig_1[2] = 2336.62;// high -> torque saturation occured + motor sound weired at reversal velocity
		}


	if (axis == 3)

		{
		//4j done!

		al_0[3] = 40.7462; //44.7462;
		al_1[3] = -8.5697;
		av_s[3] = 0.0897;
		sig_2[3] = 86.2281;

		sig_0[3] = 1047400*0.7;
		sig_1[3] = 1626.3;//334800*0.008;//*0.003;//*0.0106;

		}



	if (axis == 4)

		{
		//5j done!

		al_0[4] = 40;
		al_1[4] = 0.9945;
		av_s[4] = -0.00000013723;
		sig_2[4] =100;

		sig_0[4] =500000;
		sig_1[4] =1500;
		}


	if (axis == 5)

		{
		//6j done!

		al_0[5] = 13.0009;
		al_1[5] = -4.7344;
		av_s[5] = -0.0198;
		sig_2[5] = 47.0259;

		sig_0[5] = 106490;
		sig_1[5] =520.9762;

		}

	if (axis == 6)

		{
		//7j done!

		al_0[6] = 4.2048;
		al_1[6] = 0.3953;
		av_s[6] = 0.0208;
		sig_2[6] = 9.1313;//26.5761;

		sig_0[6] = 136320*0.8;//104668;//646680;//
		sig_1[6] =2170.3*0.07;//550;//1800;//

		}




		fun_s[axis] = al_0[axis]+ al_1[axis]*exp(-pow((ang_v/av_s[axis]),2));

		z[axis] = (c_time*(prev_z_dot[axis])) + prev_z[axis];

		z_dot[axis] = (ang_v - ((sig_0[axis] * fabs(ang_v))*z[axis]) / fun_s[axis])*exp(-fabs(ang_v)/0.1);




		Friction_LG[axis] = sig_1[axis]*z_dot[axis] + sig_0[axis] *z[axis] + sig_2[axis]*ang_v ;

		prev_z[axis]= z[axis]; //save previous z value
		prev_z_dot[axis] = z_dot[axis];  //save previous z_dot value


}

/*
double frictioncomptorque(int axis , double dq) // stribeck model
{



				//motor driver continuos current
			Icon[0]=2.744;	//motor 1 driver continuous current (A)
			Icon[1]=8.033;	//motor 2 driver continuous current (A)
			Icon[2]=4.101;	//motor 3 driver continuous current (A)
			Icon[3]=4.101;	//motor 4 driver continuous current (A)
			Icon[4]=2.744;	//motor 5 driver continuous current (A)
			Icon[5]=2.63;	//motor 6 driver continuous current (A)
			Icon[6]=1.131;	//motor 7 driver continuous current (A)

			//Torque limits condsidering allowable starting peak torque
			T_limit[0] = 229;	//J1
			T_limit[1] = 841;	//J2
			T_limit[2] = 484;	//J3
			T_limit[3] = 484;	//J4
			T_limit[4] = 229;	//J5
			T_limit[5] = 229;	//J6
			T_limit[6] = 70;	//J7


	//Friction compensate current (A) limit +/-
	fric_amp_lim[0] = 0.6;	//Joint 1
	fric_amp_lim[1] = 1.4;	//Joint 2
	fric_amp_lim[2] = 0.95;	//Joint 3
	fric_amp_lim[3] = 0.65;	//Joint 4
	fric_amp_lim[4] = 0.65;	//Joint 5
	fric_amp_lim[5] = 0.5;	//Joint 6
	fric_amp_lim[6] = 0.3;	//Joint 7

	//input dq [rad/s] -> output Friction_I current [A]
	double Af, Bf, Cf, Df;
	//A, B, C, D are fitting coefficients of friction model
	//F = A*sgn(dq) + B*dq + C*(1-exp(-dq/D))

	if (axis == 0)
	{
		if (dq>=0)
		{
			//positive velocity case:
			Af = 0.2405;
			Bf = -0.4581;
			Cf = 0.1674;
			Df = 0.1287;
		}
		if (dq<0)
		{
			//negative velocity case:
			Af = 0.2252;
			Bf = 14.8581;
			Cf = -52.9605;
			Df = 4.0048;
		}
	}
	if (axis == 1)
	{
		if (dq>=0)
		{
			//positive velocity case:
			Af = 0.4115;
			Bf = 1.5737;
			Cf = 0.2083;
			Df = 0.0861;
		}
		if (dq<0)
		{
			//negative velocity case:
			Af = 0.44;
			Bf = 10.533;
			Cf = -19.963;
			Df = 2.555;
		}
	}
	if (axis == 2)
	{
		if (dq>=0)
		{
			//positive velocity case:
			Af = 0.3346;
			Bf = -3.5017;
			Cf = 18.095;
			Df = 3.6844;
		}
		if (dq<0)
		{
			//negative velocity case:
			Af = 0.3483;
			Bf = 1.9422;
			Cf = -4.4092;
			Df = 5.3752;
		}
	}
	if (axis == 3)
	{
		if (dq>=0)
		{
			//positive velocity case:
			Af = 0.2243;
			Bf = 2.0852;
			Cf = -5.3291;
			Df = 4.608;
		}
		if (dq<0)
		{
			//negative velocity case:
			Af = 0.246;
			Bf = 0.841;
			Cf = -0.088;
			Df = -0.155;
		}
	}
	if (axis == 4)
	{
		if (dq>=0)
		{
			//positive velocity case:
			Af = -0.0735;
			Bf = 0.7954;
			Cf = 0.3168;
			Df = 0.0000001;
		}
		if (dq<0)
		{
			//negative velocity case:
			Af = 0.131;
			Bf = 0.696;
			Cf = -0.142;
			Df = -0.0002;
		}
	}
	if (axis == 5)
	{
		if (dq>=0)
		{
			//positive velocity case:
			Af = 0.166;
			Bf = 0.557;
			Cf = 0.078;
			Df = 0.109;
		}
		if (dq<0)
		{
			//negative velocity case:
			Af = 0.18;
			Bf = -0.115;
			Cf = -0.794;
			Df = -0.819;
		}
	}
	if (axis == 6)
	{
		if (dq>=0)
		{
			//positive velocity case:
			Af = 0.172;
			Bf = 0.137;
			Cf = 0.048;
			Df = 0.144;
		}
		if (dq<0)
		{
			//negative velocity case:
			Af = 0.166;
			Bf = 5.087;
			Cf = -56.387;
			Df = 11.932;
		}
	}


	//printf("Af %.5f Bf  %.5f Cf %.5f Df %.5f \n", Af, Bf, Cf, Df);

	//Friction torque [Nm]
	if (dq>0) Friction_I = Af + Bf*dq + Cf*(1-exp(-dq/Df));
	if (dq<0) Friction_I = -Af + Bf*dq + Cf*(1-exp(-dq/Df));
	if (dq == 0) Friction_I = Bf*dq + Cf*(1-exp(-dq/Df));
	//Friction_I = A*sgn(dq) + B*dq + C*(1-exp(-dq/D));

	if (axis == 3) Friction_I = 0.8*Friction_I;
	if (axis == 1) Friction_I = 0.8*Friction_I;

    if(Friction_I>fric_amp_lim[axis])
    {
        Friction_I=fric_amp_lim[axis];
    }

    else if (Friction_I<-fric_amp_lim[axis])
    {
        Friction_I=-fric_amp_lim[axis];
    }

    else
    {
        Friction_I = Friction_I;
    }

	//printf("t1 %.5f t2  %.5f t3 %.5f dq %.5f Friction com %.5f \n", Af, Bf*dq, Cf*(1-exp(-dq/Df)), dq, Friction_I);
	//return Friction_I;
        //Firction_tq[axis] = Friction_I*Kt[axis]*160;//unit: Nm 1000/Icon[axis];
}
*/


void Euler_cal(const Eigen::MatrixXd &r_data ) // ouput: current alpha beta gamma
{
	// euler calculator  ZYX Tait-Bryan Angle Conversions -> pi/2 makes singularity
	current_beta =atan2(-r_data(2,0),sqrt(pow(r_data(0,0),2)+pow(r_data(1,0),2))); // beta for calculating euler angle in rotation matrix
	if(current_beta==pi/2){
			current_alpha=0;
			current_gamma=atan2(r_data(0,1),r_data(1,1));
		}

	else if(current_beta==-pi/2){
		current_alpha=0;
		current_gamma=-atan2(r_data(0,1),r_data(1,1));
	  }

	else{
			current_alpha =atan2(r_data(1,0)/cos(current_beta),r_data(0,0)/cos(current_beta)); // alpha for calculating euler angle in rotation matrix
			current_gamma =atan2(r_data(2,1)/cos(current_beta),r_data(2,2)/cos(current_beta));
		}

//printf("frfunc: %.2f %.2f %.2f %.2f %.2f %.2f  \n", robot_current_xyz[0], robot_current_xyz[1], robot_current_xyz[2], current_alpha, current_beta, current_gamma);

}


void demo(void *arg)
{

	ros::NodeHandle nh;                            // Node handle declaration for communication with ROS
	ros::Publisher pub_torque = nh.advertise<ndr_downscale::seven>("downscale_cal_torque", 100);



	//***********************************Local constants******************************************
	//Link masses


	//PD control gains

      /*  Kp[0]=900;	//P gain for joint 1
        Kp[1]=900;	//P gain for joint 2
        Kp[2]=900;	//P gain for joint 3
        Kp[3]=400;	//P gain for joint 4
        Kp[4]=400;	//P gain for joint 5
        Kp[5]=400;	//P gain for joint 6
        Kp[6]=100;	//P gain for joint 7

        Kd[0]=60;	//D gain for joint 1
        Kd[1]=60;	//D gain for joint 2
        Kd[2]=60;	//D gain for joint 3
        Kd[3]=20;	//D gain for joint 4
        Kd[4]=40;	//D gain for joint 5
        Kd[5]=40;	//D gain for joint 6
        Kd[6]=20;	//D gain for joint 7
		*/

   // double Kkt[7]={0.3656,0.4401,0.4626,0.4626,0.3658,0.3658,0.2325};//motor torque constants (Nm/A)

		//***********************************Local variables******************************************
		//Given init homogeneous transformation matrix -0.37


		/*
		//Desired position in radian
		desired_theta(0,0) = q[0];		//Joint 1 desired angle
		desired_theta(1,0) = q[1];		//Joint 2 desired angle
		desired_theta(2,0) = q[2];		//Joint 3 desired angle
		desired_theta(3,0) = q[3];		//Joint 4 desired angle
		desired_theta(4,0) = q[4];		//Joint 5 desired angle
		desired_theta(5,0) = q[5];		//Joint 6 desired angle
		desired_theta(6,0) = q[6];		//Joint 7 desired angle
		*/

	    /*
		//Desired position in radian
		desired_theta(0,0) = 0;			//Joint 1 desired angle
		desired_theta(1,0) = 0.0917;		//Joint 2 desired angle
		desired_theta(2,0) = 0;			//Joint 3 desired angle
		desired_theta(3,0) = 1.6164;		//Joint 4 desired angle
		desired_theta(4,0) = 0;			//Joint 5 desired angle
		desired_theta(5,0) = 1.4335;		//Joint 6 desired angle
		desired_theta(6,0) = -1.5708;		//Joint 7 desired angle
	    */

		/*
		desired_theta(0,0) = 0*pi/180;		//Joint 1 desired angle
		desired_theta(1,0) = 0*pi/180;		//Joint 2 desired angle
		desired_theta(2,0) = 0*pi/180;		//Joint 3 desired angle
		desired_theta(3,0) = 30*pi/180;		//Joint 4 desired angle
		desired_theta(4,0) = 0*pi/180;		//Joint 5 desired angle
		desired_theta(5,0) = 60*pi/180;		//Joint 6 desired angle
		desired_theta(6,0) = 0*pi/180;		//Joint 7 desired angle
		*/

	    //init joint error:
		theta_err1(0,0) = 0;
		theta_err1(1,0) = 0;
		theta_err1(2,0) = 0;
		theta_err1(3,0) = 0;
		theta_err1(4,0) = 0;
		theta_err1(5,0) = 0;
		theta_err1(6,0) = 0;

		theta_err2(0,0) = 0;
		theta_err2(1,0) = 0;
		theta_err2(2,0) = 0;
		theta_err2(3,0) = 0;
		theta_err2(4,0) = 0;
		theta_err2(5,0) = 0;
		theta_err2(6,0) = 0;

    //init joint prev_vel
    p_current_dtheta(0,0)=0;
    p_current_dtheta(1,0)=0;
    p_current_dtheta(2,0)=0;
    p_current_dtheta(3,0)=0;
    p_current_dtheta(4,0)=0;
    p_current_dtheta(5,0)=0;
    p_current_dtheta(6,0)=0;


	/*
			//****************************Stiffness control parameters************************************

			//Cartesian stiffness matrix 6x6 [N/m]
			KxVec << 2500, 10, 2500, 1, 1, 1;
			Kx = KxVec.asDiagonal();

			//Cartesian damping matrix 6x6
			DxVec << 20, 20, 20, 20, 20, 20;
			Dx = DxVec.asDiagonal();

			//Weighting matrix 7x7
			K0Vec << 1000, 1000, 1000, 1000, 100, 100, 100;
			K0 = K0Vec.asDiagonal();
	*/


			I7 = I7.setIdentity(7,7);






	//Null torque
		KnVec << 1000, 1000, 1000, 100, 40, 40,10;//500, 500, 200, 100, 10, 10,10;
		KnVec << 400 ,400, 400,20, 4,4,2;//200, 200, 200, 20, 10, 10,10;
			Kn = KnVec.asDiagonal();

		DnVec << 20, 20, 20, 20, 1, 1,1;
			Dn = DnVec.asDiagonal();
        // impedance model
		//Given desired mass matrix
		Md_matrix_6 << 	10, 0, 	0,0,0,0,
	                     0, 10, 0,0,0,0,
	                     0, 0, 10,0,0,0,
	                     0, 0, 	0,10,0,0,
	                     0, 0, 	0,0,10,0,
	                     0, 0, 	0,0,0,10;
	        //Given desired damper matrix
		kd_matrix_6 << 	20, 0,	0,0,0,0,
	                     0, 20, 0,0,0,0,
	                     0, 0, 20,0,0,0,
	                     0, 0, 	0,10,0,0,
	                     0, 0, 	0,0,10,0,
	                     0, 0, 	0,0,0,10;

	        //Given desired spring matrix
		kp_matrix_6 << 	500, 0, 0,0,0,0,
	                     0, 500, 0,0,0,0,
	                     0, 0, 500,0,0,0,
	                     0, 0, 	0,100,0,0,
	                     0, 0, 	0,0,100,0,
	                     0, 0, 	0,0,0,100;

		// cartesian rotation gain
		Md_matrix_3 << 	Md_matrix_6(3,3), 0, 0,
	                     0, Md_matrix_6(4,4), 0,
	                     0, 0, Md_matrix_6(5,5);

	        //Given desired damper matrix
		kd_matrix_3 << 	kd_matrix_6(3,3), 0, 	0,
	                     0, kd_matrix_6(4,4), 0,
	                     0, 0, kd_matrix_6(5,5);

	        //Given desired spring matrix
		kp_matrix_3 << 	kp_matrix_6(3,3), 0, 0,
	                     0, kp_matrix_6(4,4), 0,
	                     0, 0, kp_matrix_6(5,5);

		h_A << 0,
				0,
				0,
				0,
				0,
				0;


    //Given init homogeneous transformation matrix

		/*P_INIT << 	1, 	0.9211, 	0.3894, 	0.8,
                        0, 	-0.9211, 	-0.3894, 	0,
                        1, 	0.3894, 	-0.9211, 	0.5,
                        0, 	0, 		0, 		1; */

	/*
	P_INIT << 	1, 	0, 	0, 	0.7,
                0, 	-1, 0, 	0.1,
                0, 0, 	-1,	0.3,
                0, 	0, 	0, 	1;   */

/*
   	P_INIT << 1, 0, 0, 0,
			0, -1, 0, 0.5,
			0, 0, -1, 0.5,
				0, 0, 0, 1;

	SI_FIX = 1;
*/

	P_INIT << 	1, 	0, 	0, 	0,
                0, 	-1, 0, 	0.757,
                0, 0, 	-1,	0.3,
                0, 	0, 	0, 	1;


	SI_FIX = 0;

	robot_initial_rotation << P_INIT(0, 0), P_INIT(0, 1), P_INIT(0, 2),
		P_INIT(1, 0), P_INIT(1, 1), P_INIT(1, 2),
		P_INIT(2, 0), P_INIT(2, 1), P_INIT(2, 2);



	robot_current_rotation = robot_initial_rotation;

	// initail rotation ??/
	hd_del_rotation << 1, 0, 0,
						0, 1, 0,
						0, 0, 1;

	robot_initial_xyz[0] = P_INIT(0, 3);
	robot_initial_xyz[1] = P_INIT(1, 3);
	robot_initial_xyz[2] = P_INIT(2, 3);

	//Desired ang_vel in radian
	desired_dtheta(0,0)=0;
	desired_dtheta(1,0)=0;
	desired_dtheta(2,0)=0;
	desired_dtheta(3,0)=0;
	desired_dtheta(4,0)=0;
	desired_dtheta(5,0)=0;
	desired_dtheta(6,0)=0;

    //InvK7(P_INIT,SI_FIX);

	//ros::Rate rate(1000.); // loop frequency[Hz]
	//ros::spinOnce();




	//printf(" waiting 0.001 sec...\n");

	//ros::Duration(0.001).sleep();

	//printf("============================== initial pos==============================\n");


	//double time_taken;
	//time_taken = 0.001;

	//int VelIsGet[7];
	//VelIsGet[4] = -1;
	//VelIsGet[5] = -1;
	//int startup = 10;	//Repeat the computation several times without torque command to pmac


	double n=0;

	double f_time = 0.002;
	double dt = 0.002;

	//double m_time=0;
	//int m_mode=0; //seclect mode
	//double m_count=0; //mode count
	//double time_perd = 0.05;

	RTIME now, previous;
	double SI_del = SI_FIX;
	rt_task_set_periodic(NULL, TM_NOW, (int)ms(2)); // 0.5khz
	previous = rt_timer_read();
	unsigned int runcount = 0;
    //***************************Stiffness control infinite loop**********************************
	while (1)//ros::ok()
	{
			rt_task_wait_period(NULL);

			runcount++;

			// DataSave(runcount);


			// rate.sleep();
			clock_t t;
			t = clock();
			double time = double(t) / CLOCKS_PER_SEC; //time


  		// Rotation master from device
			hd_current_rotation << hd_rotation_matrix[0], hd_rotation_matrix[3], hd_rotation_matrix[6],
			hd_rotation_matrix[1], hd_rotation_matrix[4], hd_rotation_matrix[7],
			hd_rotation_matrix[2], hd_rotation_matrix[5], hd_rotation_matrix[8];




			pre_dvelocity = c_dvelocity;
			prev_desired_theta = desired_theta;
			prev_d_position = d_position;


		if (Buttons[0] == 1 && Inkwell == 1)
		{
			if (button1_click_time == 0)
			{
					hd_initial_xyz[0] = hd_current_xyz[0];
					hd_initial_xyz[1] = hd_current_xyz[1];
					hd_initial_xyz[2] = hd_current_xyz[2];
					//robot_initial_xyz[0] = robot_current_xyz[0];
					//robot_initial_xyz[1] = robot_current_xyz[1];
					//robot_initial_xyz[2] = robot_current_xyz[2];


					hd_initial_rotation << hd_rotation_matrix[0], hd_rotation_matrix[3], hd_rotation_matrix[6],
					hd_rotation_matrix[1], hd_rotation_matrix[4], hd_rotation_matrix[7],
					hd_rotation_matrix[2], hd_rotation_matrix[5], hd_rotation_matrix[8];

				//hd_initial_rotation = hd_del_rotation;

			}
					hd_del_xyz[0] = hd_current_xyz[0] - hd_initial_xyz[0];
					hd_del_xyz[1] = hd_current_xyz[1] - hd_initial_xyz[1];
					hd_del_xyz[2] = hd_current_xyz[2] - hd_initial_xyz[2];
					haptic_del_xyz[0] = haptic_del_xyz[0] - hd_del_xyz[2];
					haptic_del_xyz[1] = haptic_del_xyz[1] - hd_del_xyz[0];
					haptic_del_xyz[2] = haptic_del_xyz[2] + hd_del_xyz[1];
					hd_initial_xyz[0] = hd_current_xyz[0];
					hd_initial_xyz[1] = hd_current_xyz[1];
					hd_initial_xyz[2] = hd_current_xyz[2];

			button1_click_time++;




		}

		else {
				//robot_initial_xyz[0] = robot_current_xyz[0];
				//robot_initial_xyz[1] = robot_current_xyz[1];
				//robot_initial_xyz[2] = robot_current_xyz[2];
				button1_click_time = 0;
				hd_del_xyz[0] = 0;
				hd_del_xyz[1] = 0;
				hd_del_xyz[2] = 0;

				hd_del_rotation << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;

				//haptic_del_xyz[0] = haptic_del_xyz[0] - hd_del_xyz[2];
				//haptic_del_xyz[1] = haptic_del_xyz[1] - hd_del_xyz[0];
				//haptic_del_xyz[2] = haptic_del_xyz[2] + hd_del_xyz[1];
		}

		if (Buttons[0] == 1 && Inkwell == 0)
		{
			if (button1_si_click_time == 0)
			{
				SI_del = SI_del + 0.0005;

			}

			//button1_si_click_time++;
		}
		else {
			button1_si_click_time = 0;
		}

		if (Buttons[1] == 1 && Inkwell == 0)
		{
			if (button2_si_click_time == 0)
			{
				SI_del = SI_del - 0.0005;

			}

			//button2_si_click_time++;

		}
		else {
			button2_si_click_time = 0;
		}


		//******************************************************************desired position , orientation *********************************************************************************



		// desired position & orientation from master device
		P_END << 	robot_initial_rotation(0,0), 	robot_initial_rotation(0,1), 	robot_initial_rotation(0,2), 	robot_initial_xyz[0],// + haptic_del_xyz[0]*0.003,
					robot_initial_rotation(1,0), 	robot_initial_rotation(1,1), 	robot_initial_rotation(1,2), 	robot_initial_xyz[1],// + haptic_del_xyz[1]*0.003,
					robot_initial_rotation(2,0), 	robot_initial_rotation(2,1), 	robot_initial_rotation(2,2), 	robot_initial_xyz[2]+ haptic_del_xyz[2]*0.001,
					0, 														  0, 						   	  0, 												  1;


		// desired point XYZ from forwd7 fucntion  + haptic device
		dr_posX =P_END(0,3);//robot_current_xyz[0] ;//+ haptic_del_xyz[0]*0.003; //x
        dr_posY =P_END(1,3);//robot_current_xyz[1] ;//+ haptic_del_xyz[1]*0.003; //y
        dr_posZ =P_END(2,3);//robot_current_xyz[2]+ haptic_del_xyz[2]*0.003; //z;

		P_COM = P_END;

		InvK7(P_COM, SI_del); // OUTPUT:  desired angle q[0] to q[6]


        //***********************************************************************************************************************************
		// euler angle for inital pos
		// gamma -> rotation of x axis


		 // euler calculator  ZYX Tait-Bryan Angle Conversions -> pi/2 makes singularity
		Euler_cal(P_END); // output: current_alpha, current_beta, current_gamma
		/*
		current_beta =atan2(-P_END(2,0),sqrt(pow(P_END(0,0),2)+pow(P_END(1,0),2))); // beta for calculating euler angle in rotation matrix

		if(current_beta==pi/2){
			current_alpha=0;
			current_gamma=atan2(P_END(0,1),P_END(1,1));
		}

		else if(current_beta==-pi/2){
			current_alpha=0;
			current_gamma=-atan2(P_END(0,1),P_END(1,1));
		}
    	else{
		current_alpha =atan2(P_END(1,0)/cos(current_beta),P_END(0,0)/cos(current_beta)); // alpha for calculating euler angle in rotation matrix
		current_gamma =atan2(P_END(2,1)/cos(current_beta),P_END(2,2)/cos(current_beta));

		} */
		//init_euler_alpha = current_alpha;
		//init_euler_beta  = current_beta;
		//init_euler_gamma = current_gamma;

		//printf("inital: %.2f %.2f %.2f %.2f %.2f %.2f  \n", posX, posY, posZ, init_euler_alpha,  init_euler_beta, init_euler_gamma);
	    //***********************************************************************************************************************************


		//joint_publish(q);
		//printf("dacc4 = %.2f", d_acc(3,0));
		//Desired position in radian
		desired_theta(0,0) = q[0];		//Joint 1 desired angle
		desired_theta(1,0) = q[1];		//Joint 2 desired angle
		desired_theta(2,0) = q[2];		//Joint 3 desired angle
		desired_theta(3,0) = q[3];		//Joint 4 desired angle
		desired_theta(4,0) = q[4];		//Joint 5 desired angle
		desired_theta(5,0) = q[5];		//Joint 6 desired angle
		desired_theta(6,0) = q[6];		//Joint 7 desired angle


		//Desired ang_vel in radian
		desired_dtheta=(desired_theta - prev_desired_theta)/dt;
		dr_ang_vel = desired_dtheta;



		rotation_mat(q[0],q[1],q[2], q[3],q[4],q[5],q[6]);
		ROT_0dd = ROT_0d; // base to desired rotation


		//computing jacob for desired angle
		//jacob_diffjacob(desired_theta(0,0),desired_theta(1,0),desired_theta(2,0),desired_theta(3,0),desired_theta(4,0),desired_theta(5,0),desired_theta(6,0),dr_ang_vel(0,0),dr_ang_vel(1,0),dr_ang_vel(2,0),dr_ang_vel(3,0),dr_ang_vel(4,0),dr_ang_vel(5,0),dr_ang_vel(6,0));


		//========================================================================================================================================================================
		// euler angle for inital pos
		// gamma -> rotation of x axis
		// euler calculator  ZYX Tait-Bryan Angle Conversions -> pi/2 makes singularity

		//Euler_cal(ROT_0dd); // output: current_alpha, current_beta, current_gamma
		/*
		current_beta =atan2(-ROT_0dd(2,0),sqrt(pow(ROT_0dd(0,0),2)+pow(ROT_0dd(1,0),2))); // beta for calculating euler angle in rotation matrix

		if(current_beta==pi/2){
		current_alpha=0;
		current_gamma=atan2(ROT_0dd(0,1),ROT_0dd(1,1));
		}

		else if(current_beta==-pi/2){
		current_alpha=0;
		current_gamma=-atan2(ROT_0dd(0,1),ROT_0dd(1,1));
		}
		else{
		current_alpha =atan2(ROT_0dd(1,0)/cos(current_beta),ROT_0dd(0,0)/cos(current_beta)); // alpha for calculating euler angle in rotation matrix
		current_gamma =atan2(ROT_0dd(2,1)/cos(current_beta),ROT_0dd(2,2)/cos(current_beta));

		}*/
		//init_euler_alpha = current_alpha;
		//init_euler_beta  = current_beta;
		//init_euler_gamma = current_gamma;

		//printf("inital: %.2f %.2f %.2f %.2f %.2f %.2f  \n", posX, posY, posZ, init_euler_alpha,  init_euler_beta, init_euler_gamma);


		//========================================================================================================================================================================

		//joint_publish(q);
		//printf("dacc4 = %.2f", d_acc(3,0));


		//*****************************************************************desired pos*****************************************************************************************************


		// from fixed point
		d_position(0,0) = dr_posX;
		d_position(1,0) = dr_posY;
		d_position(2,0) = dr_posZ;

		// roatation wiil be calculated in de orientation section
		d_position(3,0) = 0;//init_euler_alpha; // euler angle alpha
		d_position(4,0) = 0;//init_euler_beta; // euler angle beta
		d_position(5,0) = 0;//init_euler_gamma; // euler angle gamma



		// for zyx euler

		/*
		T_angvel_matrix << 1, 0, -sin(init_euler_beta),
			0, cos(init_euler_alpha), cos(init_euler_beta)*sin(init_euler_alpha),
		0, 	-sin(init_euler_alpha), cos(init_euler_beta)*cos(init_euler_alpha);
		*/

		//rotation
		/*
		ROT_pos_d(0,0)=d_position(3,0);
		ROT_pos_d(1,0)=d_position(4,0);
		ROT_pos_d(2,0)=d_position(5,0);

		ROT_pos_d_2 =ROT_pos_d;//ROT_pos_d;//T_rot_matrix*ROT_pos_d;
		d_position(3,0)=ROT_pos_d_2(0,0);
		d_position(4,0)=ROT_pos_d_2(1,0);
		d_position(5,0)=ROT_pos_d_2(2,0);
		*/


		// rotation only (euler)
		// d_position_3(0,0) = d_position(3,0);
		// d_position_3(1,0) = d_position(4,0);
		// d_position_3(2,0) = d_position(5,0);



		//d_vel = Jacob*dr_ang_vel;
		d_vel = (d_position - prev_d_position)/dt;
		/*
		// euler rotation derivtive
		ROT_vel_d(0,0)=d_vel(3,0);
		ROT_vel_d(1,0)=d_vel(4,0);
		ROT_vel_d(2,0)=d_vel(5,0);

		ROT_vel_d_2 =T_angvel_matrix*ROT_vel_d; // convert 'euler angle derivative' to 'angular velicty'
		d_vel(3,0)=ROT_vel_d_2(0,0); // angular velocity
		d_vel(4,0)=ROT_vel_d_2(1,0);
		d_vel(5,0)=ROT_vel_d_2(2,0);

		*/

		c_dvelocity = d_vel; // desired velocity 6x1
		d_acc = (c_dvelocity - pre_dvelocity)/f_time; //(c_dvelocity - pre_dvelocity)/ time_taken

		//******************************************************************Current position , orientation *********************************************************************************

		// ADDED

		p_current_dtheta = current_dtheta ;

		//Update current joint angles: in radian

		th1 = current_joint_states[0];
		th2 = current_joint_states[1];
		th3 = current_joint_states[2];
		th4 = current_joint_states[3];
		th5 = current_joint_states[4];
		th6 = current_joint_states[5];
		th7 = -1.57;// current_joint_states[6];
		//printf("Joint angle errors: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", th1, th2, th3, th4, th5, th6, th7);

		//Update current joint angle velocities: in radian
		dth1=cur_vel[0];
		dth2=cur_vel[1];
		dth3=cur_vel[2];
		dth4=cur_vel[3];
		dth5=cur_vel[4];
		dth6=cur_vel[5];
		dth7=cur_vel[6];

		current_theta(0,0) = th1;
		current_theta(1,0) = th2;
		current_theta(2,0) = th3;
		current_theta(3,0) = th4;
		current_theta(4,0) = th5;
		current_theta(5,0) = th6;
		current_theta(6,0) = th7;

		ang_vel(0,0)=dth1;
		ang_vel(1,0)=dth2;
		ang_vel(2,0)=dth3;
		ang_vel(3,0)=dth4;
		ang_vel(4,0)=dth5;
		ang_vel(5,0)=dth6;
		ang_vel(6,0)=dth7;

		current_dtheta(0,0)=dth1;
		current_dtheta(1,0)=dth2;
		current_dtheta(2,0)=dth3;
		current_dtheta(3,0)=dth4;
		current_dtheta(4,0)=dth5;
		current_dtheta(5,0)=dth6;
		current_dtheta(6,0)=dth7;


		//jacob_diffjacob(th1,th2,th3,th4,th5,th6,th7,dth1,dth2,dth3,dth4,dth5,dth6,dth7); //computing jacob for current angle
		jacob_diffjacob(); //computing jacob for current angle

		//end eff orientation
		rotation_mat(th1, th2, th3, th4, th5,th6,th7); // OUTPUT: ROT_0d,  end eff orientation
		ROT_0e = ROT_0d; // get eef frame orienatioin from base


		//******************************************************************null torque*****************************************************************************************************
		/*
		forwd7(th1,th2,th3,th4,th5,th6,th7); // output: robot_current_xyz[0]. [1] , [2]

		P_COM(0,0) = ROT_0e(0,0);
		P_COM(0,1) = ROT_0e(0,1);
		P_COM(0,2) = ROT_0e(0,2);
		P_COM(1,0) = ROT_0e(1,0);
		P_COM(1,1) = ROT_0e(1,1);
		P_COM(1,2) = ROT_0e(1,2);
		P_COM(2,0) = ROT_0e(2,0);
		P_COM(2,1) = ROT_0e(2,1);
		P_COM(2,2) = ROT_0e(2,2);

		P_COM(3,0) = robot_current_xyz[0]; // from forwd7
		P_COM(3,1) = robot_current_xyz[1];
		P_COM(3,2) = robot_current_xyz[2];

		InvK7(P_COM,SI_del); // output : q[i]
		*/
		//Compute null joint ang, ang_vel error [rad]
			for (int i=0; i<7; i++)
			{
				n_theta_err1(i,0) = current_theta(i,0)- q[i] ;//desired_theta(i,0);//q[i] ;
				n_dtheta_err1(i,0) = current_dtheta(i,0);//desired_dtheta(i,0) - current_dtheta(i,0);
			}




		//==================================================================<DE ORIENTATION SECTION>=================================================================================

		//=========================================================Euler angle from eef frame to desired frame=============================================================================
		// euler angle for inital pos
		// gamma -> rotation of x axis
		// euler calculator  ZYX Tait-Bryan Angle Conversions -> pi/2 makes singularity

		//======================================== euler 'de' ===========================================================
		ROT_ed = ROT_0e.transpose() * ROT_0dd; // ROTATION EEF TO DESIRED
		Euler_cal(ROT_ed); // output: current_alpha, current_beta, current_gamma

		/*
		current_beta =atan2(-ROT_ed(2,0),sqrt(pow(ROT_ed(0,0),2)+pow(ROT_ed(1,0),2))); // beta for calculating euler angle in rotation matrix

		if(current_beta==pi/2){
		current_alpha=0;
		current_gamma=atan2(ROT_ed(0,1),ROT_ed(1,1));
		}

		else if(current_beta==-pi/2){
		current_alpha=0;
		current_gamma=-atan2(ROT_ed(0,1),ROT_ed(1,1));
		}
		else{
		current_alpha =atan2(ROT_ed(1,0)/cos(current_beta),ROT_ed(0,0)/cos(current_beta)); // alpha for calculating euler angle in rotation matrix
		current_gamma =atan2(ROT_ed(2,1)/cos(current_beta),ROT_ed(2,2)/cos(current_beta));

		} */
		eef_des_euler_alpha = current_alpha;
		eef_des_euler_beta  = current_beta;
		eef_des_euler_gamma = current_gamma;



		Trans_Jacob = Jacob.transpose();



		T_de_matrix << 1, 0, -sin(eef_des_euler_beta),
									0, cos(eef_des_euler_alpha), cos(eef_des_euler_beta)*sin(eef_des_euler_alpha),
									0, 	-sin(eef_des_euler_alpha), cos(eef_des_euler_beta)*cos(eef_des_euler_alpha);



		T_e_de_matrix = ROT_0e*T_de_matrix;


		err_position_3(0,0)  = eef_des_euler_alpha;
		err_position_3(1,0)  = eef_des_euler_beta;
		err_position_3(2,0)  = eef_des_euler_gamma;

		err_velocity_3 = (err_position_3 - prev_err_position_3)/f_time; // -> euler de_dot
		prev_err_position_3 =err_position_3;


		T_de_matrix_dot << 1, 0, -cos(eef_des_euler_beta)*err_velocity_3(1,0),
												0, -sin(eef_des_euler_alpha)*err_velocity_3(0,0), -sin(eef_des_euler_beta)*sin(eef_des_euler_alpha)*err_velocity_3(1,0) + cos(eef_des_euler_beta)*cos(eef_des_euler_alpha)*err_velocity_3(0,0) ,
												0, 	-cos(eef_des_euler_alpha)*err_velocity_3(0,0), -sin(eef_des_euler_beta)*cos(eef_des_euler_alpha)*err_velocity_3(1,0) -cos(eef_des_euler_beta)*sin(eef_des_euler_alpha)*err_velocity_3(0,0) ;


		T_e_de_matrix_dot =  T_de_matrix_dot;//ROT_0e*T_de_matrix_dot;



		//*****************************************************************currrent pos*****************************************************************************************************
		forwd7(th1,th2,th3,th4,th5,th6,th7); //output: robot_current_xyz[0],robot_current_xyz[1],robot_current_xyz[2]

		c_position(0,0)=robot_current_xyz[0]; //x
		c_position(1,0)=robot_current_xyz[1]; //y
		c_position(2,0)=robot_current_xyz[2]; //z

		rotation_mat(th1, th2, th3, th4, th5,th6,th7); // output: angle (3x3)m euler angle(current_alpha,beta,gamma)
		//printf("cr_rot_value: %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f   \n",  ROT_0d(0,0), ROT_0d(0,1), ROT_0d(0,2),ROT_0d(1,0), ROT_0d(1,1), ROT_0d(1,2),ROT_0d(2,0), ROT_0d(2,1), ROT_0d(2,2));

		// rotation zero based own frame
		c_position(3,0)=0;//current_alpha;
		c_position(4,0)=0;//current_beta;
		c_position(5,0)=0;//current_gamma;


			/*
			 T_angvel_matrix << 1, 0, -sin(current_beta),
			 					0, cos(current_alpha), cos(current_beta)*sin(current_alpha),
								0, 	-sin(current_alpha), cos(current_beta)*cos(current_alpha);
			 */

			/*
			 ROT_pos_C(0,0)=c_position(3,0);
			 ROT_pos_C(1,0)=c_position(4,0);
			 ROT_pos_C(2,0)=c_position(5,0);

			 ROT_pos_C_2 =ROT_pos_C;//T_angvel_matrix*ROT_pos_C;//ROT_pos_C;//T_rot_matrix*ROT_pos_C;
			 c_position(3,0)=ROT_pos_C_2(0,0);
			 c_position(4,0)=ROT_pos_C_2(1,0);
			 c_position(5,0)=ROT_pos_C_2(2,0);
			*/

			// rotation only
			 //c_position_3(0,0)=robot_current_xyz[0];
             //c_position_3(1,0)=robot_current_xyz[1];
             // c_position_3(2,0)=robot_current_xyz[2];


				/*
			 c_vel = Jacob*ang_vel; // result : velocity & euler angle derivative

			 // euler rotation
			 ROT_vel_C(0,0)=c_vel(3,0);
			 ROT_vel_C(1,0)=c_vel(4,0);
			 ROT_vel_C(2,0)=c_vel(5,0);

			 ROT_vel_C_2 = T_angvel_matrix*ROT_vel_C; // convert 'euler angle derivative' to 'angular velicty'
			 c_vel(3,0)=ROT_vel_C_2(0,0);
			 c_vel(4,0)=ROT_vel_C_2(1,0);
			 c_vel(5,0)=ROT_vel_C_2(2,0);
				*/




			 err_position =  c_position - d_position ; //  error = current - desired

			 err_velocity = c_vel - d_vel  ;



			 //*********************************************** F/T Sensor processing ************************************************************************

		 // TF sensor filter
		 for (int i=0; i<6; i++)
			{
				if(fabs(cur_tau[i])<100)
				{
					cur_tau[i] = 0;
				}
				else
				{
					cur_tau[i]= cur_tau[i];
				}

			}

				FT_TEMP(0,0)=cur_tau[0];
				FT_TEMP(1,0)=cur_tau[1];
				FT_TEMP(2,0)=cur_tau[2];

				FT_xyz =I*FT_TEMP; //;ROT_0d*FT_TEMP;
				h_0e(0,0) =0;//FT_xyz(0,0); // eef x axis force
				h_0e(1,0) =0;//FT_xyz(1,0); // eef y axis force
				h_0e(2,0) =0;//FT_xyz(2,0); // eef z axis force
				h_0e(3,0) =0;//cur_tau[3]; // x moment - not sure
				h_0e(4,0) =0;//cur_tau[4]; // y moment
				h_0e(5,0) =0;//cur_tau[5]; // z moment



			//h_0e_3(0,0) =FT_xyz(0,0); // eef x axis force
			//h_0e_3(1,0) =FT_xyz(1,0); // eef y axis force
			//h_0e_3(2,0) =FT_xyz(2,0); // eef z axis force

			//ORIGINAL
			/*
			FT_TEMP(0,0)=cur_tau[0];
			FT_TEMP(1,0)=cur_tau[1];
			FT_TEMP(2,0)=cur_tau[2];
			FT_TEMP_2(0,0)=cur_tau[3];
			FT_TEMP_2(1,0)=cur_tau[4];
			FT_TEMP_2(2,0)=cur_tau[5];

			// force based on base frame
			FT_xyz =ROT_0d*FT_TEMP;//I*FT_TEMP; //;ROT_0d*FT_TEMP;
			FT_xyz_2 =ROT_ed*FT_TEMP_2;
			h_0e(0,0) =FT_xyz(0,0); // eef x axis force
			h_0e(1,0) =FT_xyz(1,0); // eef y axis force
			h_0e(2,0) =FT_xyz(2,0); // eef z axis force
			h_0e(3,0) =FT_xyz_2(0,0); // x moment - not sure
			h_0e(4,0) =FT_xyz_2(1,0); // y moment
			h_0e(5,0) =FT_xyz_2(2,0); // z moment
			*/

			// moment
			h_0e_3(0,0) =h_0e(3,0); // eef x axis force
			h_0e_3(1,0) =h_0e(4,0); // eef y axis force
			h_0e_3(2,0) =h_0e(5,0); // eef z axis force


	//*******************************************************************************************************************************************

			Mass_Matrix(th1,th2,th3,th4,th5,th6,th7);

			//Gr_Matrix(th1,th2,th3,th4,th5,th6,th7); // ioutput: gravity matrix 7x1

			cartesian_Mass = (Jacob*M_matrix.inverse()*Jacob.transpose()).inverse(); // caresian mass matrix 6x6


			resolved_acc = d_acc - Md_matrix_6.inverse()*(kd_matrix_6*err_velocity + kp_matrix_6*err_position) - Md_matrix_6.inverse()*h_0e;


			acc_o = T_e_de_matrix * Md_matrix_3.inverse()*(-kd_matrix_3*err_velocity_3 - kp_matrix_3*err_position_3) + T_e_de_matrix * Md_matrix_3.inverse()*T_e_de_matrix.transpose()*h_0e_3 + T_e_de_matrix_dot*err_velocity_3;



			resolved_acc(3,0) = d_acc(3,0) + acc_o(0,0);
			resolved_acc(4,0) = d_acc(4,0) + acc_o(1,0);
			resolved_acc(5,0) = d_acc(5,0) + acc_o(2,0);


			input_y =cartesian_Mass*(resolved_acc - diff_jacob*ang_vel)  - h_0e;



			//Null space stiffness


			Prj_matrix = I7-Trans_Jacob*cartesian_Mass*Jacob*M_matrix.inverse();
			Tau_stif =Prj_matrix* (-Kn*n_theta_err1 - Dn*n_dtheta_err1);
			// G_matrix -> already in  subsystem
			Tau_I=Jacob.transpose()*input_y  + Tau_stif;//(I7-Jacob.transpose()*Md_matrix_6*Jacob*M_matrix.inverse())*Tau_stif; // B(q)*y + N(q,q')

			checkval=Jacob.transpose()*input_y;
			checkval_2=input_y;

			// velocity filter
			for (int i=0; i<7; i++)
			{
					if(fabs(cur_vel[i])>=0.6) //|| measure[i]>0.5
		      {
							if(cur_vel[i]>=0.6) // +
							{
								cur_vel[i]=0.6;
							}


							else  // -
							{
								cur_vel[i]=-0.6;
						}

					}

		      else
		      {
		          {cur_vel[i]=cur_vel[i];}
		      }
			}


			//friction torque********************************************************************

			friction_LuGre(0, cur_vel[0]);
			Firction_tq[0] = Friction_LG[0];

			friction_LuGre(1, cur_vel[1]);
			Firction_tq[1] = Friction_LG[1];


			friction_LuGre(2, cur_vel[2]);
			Firction_tq[2] = Friction_LG[2];

			friction_LuGre(3, cur_vel[3]);
			Firction_tq[3] = Friction_LG[3];

			friction_LuGre(4, cur_vel[4]);
			Firction_tq[4] = Friction_LG[4];

			friction_LuGre(5, cur_vel[5]);
			Firction_tq[5] = Friction_LG[5];

			friction_LuGre(6, cur_vel[6]);
			Firction_tq[6] = Friction_LG[6];


			//***********************************************************************************


			//for translational
			//Prj_matrix_3 = I7-Jacob_3.transpose()*cartesian_Mass_3*Jacob_3*M_matrix.inverse();
			//Tau_stif_3 =Prj_matrix_3* (-Kn*n_theta_err1 - Dn*dtheta_err);
			//Tau_I=Jacob_3.transpose()*input_y_3 + G_matrix + Tau_stif;//(I7-Jacob.transpose()*Md_matrix_6*Jacob*M_matrix.inverse())*Tau_stif; // B(q)*y + N(q,q')

			//checkval=Jacob.transpose()*input_y;
			//printf("responce: %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n", checkval(0,0), checkval(1,0),checkval(2,0),checkval(3,0),checkval(4,0),checkval(5,0),checkval(6,0));


			command_tau[0] = Tau_I(0,0)+Firction_tq[0];//0;// Tau(0,0);
			command_tau[1] = Tau_I(1,0)+Firction_tq[1];
			command_tau[2] = Tau_I(2,0)+Firction_tq[2];
			command_tau[3] = Tau_I(3,0)+Firction_tq[3];
			command_tau[4] = Tau_I(4,0)+Firction_tq[4];
			command_tau[5] = Tau_I(5,0)+Firction_tq[5];
			command_tau[6] = Tau_I(6,0)+Firction_tq[6];


			ndr_downscale::seven msga;

			if(runcount <1500)
			{
					msga.a =0;
					msga.b = 0;
					msga.c =0;
					msga.d =0;
					msga.e = 0;
					msga.f =0;
					msga.g = 0;
			}
			else{
					//Nm
					msga.a =command_tau[0];
					msga.b = command_tau[1];
					msga.c =command_tau[2];
					msga.d =command_tau[3];
					msga.e = command_tau[4];
					msga.f = command_tau[5];// target_torque[5];
					msga.g = command_tau[6];  //7th jiont
			}
			pub_torque.publish(msga);


			DataSave(runcount);

			n++;



			//} //end if VelIsGet


			ros::spinOnce();
			t = clock() - t;
			time_taken = ((double)t)/CLOCKS_PER_SEC; // in seconds
	} //end while

}





void printt_run(void *arg)
{
	rt_task_set_periodic(NULL, TM_NOW, (int)ms(100));	// period = 1 (msec)

	while (1)
	{

			//rt_printf("**********************position***************************************\n");
			//rt_printf("angle: %f %f %f %f %f %f %f \n", act_ang[0]  , act_ang[1]  , act_ang[2] , act_ang[3], act_ang[4], act_ang[5], act_ang[6] );
			//rt_printf("\e[32;1m\t j4 ang: %f,  \e[0m\n", 	 	act_ang[3]);
			rt_printf("\e[32;1m\t Disired: %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", d_position(0,0), d_position(1,0), d_position(2,0), d_position(3,0),  d_position(4,0), d_position(5,0));
			rt_printf("\e[32;1m\t Current: %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", c_position(0,0), c_position(1,0), c_position(2,0), c_position(3,0),  c_position(4,0), c_position(5,0));
			rt_printf("\e[32;1m\t poserror: %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", err_position(0,0), err_position(1,0), err_position(2,0), err_position_3(0,0),  err_position_3(1,0), err_position_3(2,0));
			rt_printf("\e[32;1m\t velerror: %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", err_velocity(0,0), err_velocity(1,0), err_velocity(2,0), err_velocity_3(0,0),  err_velocity_3(1,0), err_velocity_3(2,0));
			rt_printf("\e[32;1m\t nullstif: %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", Tau_stif(0,0), Tau_stif(1,0), Tau_stif(2,0), Tau_stif(3,0),  Tau_stif(4,0), Tau_stif(5,0));
			rt_printf("\e[32;1m\t tqinputt: %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", input_y(0,0), input_y(1,0), input_y(2,0), input_y(3,0),  input_y(4,0), input_y(5,0));

			rt_printf("\e[32;1m\t nullerrp: %.2f %.2f %.2f %.2f %.2f %.2f %.2f \e[0m\n", n_theta_err1(0,0), n_theta_err1(1,0), n_theta_err1(2,0), n_theta_err1(3,0),  n_theta_err1(4,0), n_theta_err1(5,0),n_theta_err1(6,0));
			rt_printf("\e[32;1m\t nullerrv: %.2f %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", n_dtheta_err1(0,0), n_dtheta_err1(1,0), n_dtheta_err1(2,0), n_dtheta_err1(3,0),  n_dtheta_err1(4,0), n_dtheta_err1(5,0),n_dtheta_err1(6,0));

			rt_printf("\e[32;1m\t dr_ang: %.2f %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", desired_theta(0,0), desired_theta(1,0), desired_theta(2,0), desired_theta(3,0), desired_theta(4,0),desired_theta(5,0), desired_theta(6,0));
			rt_printf("\e[32;1m\t cr_ang: %.2f %.2f %.2f %.2f %.2f %.2f %.2f  \e[0m\n", current_theta(0,0), current_theta(1,0), current_theta(2,0), current_theta(3,0), current_theta(4,0),current_theta(5,0), current_theta(6,0));

			//rt_printf("vel:  %f %f %f %f %f %f %f\n", act_ang_vel[0]  , act_ang_vel[1]  , act_ang_vel[2] , act_ang_vel[3], act_ang_vel[4], act_ang_vel[5], act_ang_vel[6]  );
			//rt_printf("\e[32;1m\t j4 av: %f,  \e[0m\n", 	 	act_ang_vel[3]);
			//rt_printf("\e[32;1m\t j5 av: %f,  \e[0m\n", 	 	act_ang_vel[4]);
			//rt_printf("\e[32;1m\t j6 av: %f,  \e[0m\n", 	 	act_ang_vel[5]);

			//rt_printf("\e[32;1m\t ActPosdeg[1]: %.3f, ActPosdeg[2]: %.3f  ActPosdeg[3]: %.3f, ActPosdeg[4]: %.3f, ActPosdeg[5]: %.3f, ActPosdeg[6]: %.3f, ActPosdeg[7]: %.3f \e[0m\n", enctodeg[0], enctodeg[1], enctodeg[2], enctodeg[3], enctodeg[4], enctodeg[5], enctodeg[6]);
			//rt_printf("\e[32;1m\t ActualVel[1]: %.3f, ActualVel[2]: %.3f, ActualVel[3]: %.3f, ActualVel[4]: %.3f, ActualVel[5]: %.3f, ActualVel[6]: %.3f, ActualVel[7]: %.3f \e[0m\n", vel_enctodeg[0], vel_enctodeg[1], vel_enctodeg[2], vel_enctodeg[3], vel_enctodeg[4], vel_enctodeg[5], vel_enctodeg[6]);
			//rt_printf("\e[32;1m\t ActualTor[1]: %i, ActualTor[2]: %i, ActualTor[3]: %i, ActualTor[4]: %i, ActualTor[5]: %i, ActualTor[6]: %i, ActualTor[7]: %i \e[0m\n", ActualTor[0], ActualTor[1], ActualTor[2], ActualTor[3], ActualTor[4], ActualTor[5], ActualTor[6]);
			rt_printf("\e[32;1m\t FricTorNm[1]: %f, FricTorNm[2]: %f, FricTorNm[3]: %f, FricTorNm[4]: %f, FricTorNm[5]: %f, FricTorNm[6]: %f FricTorNm[7]: %f \e[0m\n", Firction_tq[0], Firction_tq[1], Firction_tq[2], Firction_tq[3], Firction_tq[4], Firction_tq[5], Firction_tq[6]);
			rt_printf("\e[32;1m\t ComdTorNm[1]: %f, ComdTorNm[2]: %f ComdTorNm[3]: %f, ComdTorNm[4]: %f, ComdTorNm[5]: %f, ComdTorNm[6]: %f, ComdTorNm[7]: %f \e[0m\n", Tau_I(0,0), Tau_I(1,0), Tau_I(2,0), Tau_I(3,0), Tau_I(4,0), Tau_I(5,0), Tau_I(6,0));

			//rt_printf("\e[32;1m\t time: %f,  \e[0m\n", 	 	time);
			rt_printf("\e[32;1m\t time taken: %.5f,  \e[0m\n", 	 	time_taken);

			rt_task_wait_period(NULL);
			//rt_printf("cal_time %f\n", time_taken);

			//rt_task_wait_period(NULL);
	}
}

void catch_signal(int sig = 0)
{
	 //fclose(pData);
	rt_task_delete(&demo_task);
	rt_task_delete(&printt_task);

	 exit(1);
}

int main (int argc, char* argv[])
{

		ros::init( argc, argv, "ndr_downscale_impedance" );

		ros::NodeHandle nh;

		ros::Subscriber sub_jointp = nh.subscribe("downscale_actp", 100, msgCallbacdataangle);
		ros::Subscriber sub_jointv = nh.subscribe("downscale_actv", 100, msgCallbacdataangvel);
		ros::Subscriber sub_torquesensorCAN2ETH = nh.subscribe("downscale_tfsensor", 100, msgCallbackT);

		// Teleoperation
		ros::Subscriber hd_trans = nh.subscribe("/hd_trans", 1, &hd_callback_trans);
		ros::Subscriber hd_buttons = nh.subscribe("/hd_buttons", 1, &hd_callback_buttons);


		rt_print_auto_init(1);

		signal(SIGTERM, catch_signal);

		signal(SIGINT, catch_signal);

		mlockall(MCL_CURRENT|MCL_FUTURE);

		rt_task_create(&demo_task, "trivial", 0, 98, 0);
		rt_task_start(&demo_task, &demo, NULL);

		rt_task_create(&printt_task, "printing_pd", 0, 79, 0);
		rt_task_start(&printt_task, &printt_run, NULL);

		pause();

		//rt_task_delete(&demo_task);


		catch_signal();

		return 0;
}
