#ifndef __XENO__
#define __XENO__
#endif

//-system-/////////////////////////////////////////////////////////////////
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

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <rtdk.h>
#define ms(x) (x*1000000)

// ROS
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <boost/bind.hpp>

#include "ndr_downscale/one.h"
#include "ndr_downscale/two.h"
#include "ndr_downscale/three.h"
#include "ndr_downscale/six.h"
#include "ndr_downscale/seven.h"
#include "ndr_downscale/twentyone.h"

#include <iostream>
#include <sstream>
#include <getopt.h>
#include <list>

// External libraries
#include "../include/Eigen/Dense"
#include "../include/Eigen/Core"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32MultiArray.h"
//time measure
#include <chrono>

// EtherCAT
#include "ecrt.h"

typedef unsigned int UINT32;
typedef int32_t INT32;
typedef int16_t INT16;
typedef uint16_t UINT16;
typedef uint8_t UINT8;
typedef int8_t INT8;

/**********************************************************************************************************/
/**************************EtherCAT devices*******************************************/
/********downscale************/
// Vendor ID & Product Code 
#define ROBOTOUS_VENDOR_ID 0x8EE 
#define ROBOTOUS_PRODUCT_CODE 0x00000002 

// Vendor ID & Product Code 
#define SERVOTRONIX_VENDOR_ID 0x000002E1 
#define SERVOTRONIX_PRODUCT_CODE 0x0 

/********downscale************/
#define NUM_AXIS	(1 + 7)
#define NUM_ROBOTOUS_RFT_EC02_AXES 1
#define NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES 7

#define NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_RX_PDO_ENTRIES 8
#define NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_TX_PDO_ENTRIES 9
#define NUM_ROBOTOUS_RFT_EC02_RX_PDO_ENTRIES 2
#define NUM_ROBOTOUS_RFT_EC02_TX_PDO_ENTRIES 24

#define NUM_MOTION_AXIS (NUM_ROBOTOUS_RFT_EC02_AXES + NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES)
#define MAX_TORQUE 2000


#define NSEC_PER_SEC 1000000000

// SERVOTRONIX CATALOGUE Page. 398
#define STATUSWORD_READY_TO_SWITCH_ON_BIT 		0
#define STATUSWORD_SWITCHED_ON_BIT 				1
#define STATUSWORD_OPERATION_ENABLE_BIT 		2
#define STATUSWORD_FAULT_BIT 					3
#define STATUSWORD_VOLTAGE_ENABLE_BIT 			4
#define STATUSWORD_QUICK_STOP_BIT 				5
#define STATUSWORD_SWITCH_ON_DISABLE_BIT 		6
#define STATUSWORD_NO_USED_WARNING_BIT 			7
#define STATUSWORD_ELMO_NOT_USED_BIT 			8
#define STATUSWORD_REMOTE_BIT 					9
#define STATUSWORD_TARGET_REACHED_BIT 			10
#define STATUSWORD_INTERNAL_LIMIT_ACTIVE_BIT	11


// Operation mode
#define OP_MODE_NO_MODE					0x00
#define OP_MODE_PROFILE_POSITION		0x01
#define OP_MODE_VELOCITY				0x02
#define OP_MODE_PROFILE_VELOCITY		0x03
#define OP_MODE_TORQUE_PROFILE			0x04
#define OP_MODE_HOMING					0x06
#define OP_MODE_INTERPOLATED_POSITION	0x07
#define OP_MODE_CYCLIC_SYNC_POSITION	0x08
#define OP_MODE_CYCLIC_SYNC_VELOCITY	0x09
#define OP_MODE_CYCLIC_SYNC_TORQUE		0x0a

// Operation
#define NONE	0x00
#define INIT 	0x01
#define PRE_OP 	0x02
#define SAFE_OP 0x04
#define OP 		0x08

// Demo mode
#define DEMO_MODE_TORQUE		1
#define DEMO_MODE_POSITION		2

int demo_mode = DEMO_MODE_POSITION;

/********downscale************/
//SERVOTRONIX
// WRITE PDO (Write to Motor driver ~ Motor driver receive RX 8)
#define CONTROL_WORD 0x60400
#define MODE_OF_OPERATION 0x60600
#define TARGET_POSITION 0x607A0
#define PROFILE_VELOCITY 0x60810
#define TARGET_VELOCITY 0x60FF0
#define TARGET_TORQUE 0x60710
#define DIGITAL_OUTPUTS 0x60FE0
#define TORQUE_OFFSET 0x60B20

// READ PDO (Read from Motor driver ~ Motor driver transmit TX 9)
#define STATUS_WORD 0x60410
#define MODE_OF_OPERATION_DISPLAY 0x60610
#define TORQUE_ACTUAL_VALUE 0x60770
#define POSITION_ACTUAL_VALUE 0x60640
#define TORQUE_DEMAND_VALUE 0x60740
#define ANALOG_INPUT1 0x20F20
#define DIGITAL_INPUTS 0x60FD0
#define MANUSPECS_MACHINE_HW_POSITION_EXTERNAL_COMMAND 0x20B60
#define POSITION_FOLLOWING_ERROR_ACTUAL_VALUE 0x60F40

//ROBOTOUS
// WRITE PDO 
#define CONFIG_PARAM1 0x70001
#define CONFIG_PARAM2 0x70002

// READ PDO
#define INPUT_DF1 0x60001
#define INPUT_DF2 0x60002
#define INPUT_DF3 0x60003
#define INPUT_DF4 0x60004
#define INPUT_DF5 0x60005
#define INPUT_DF6 0x60006
#define INPUT_DF7 0x60007
#define INPUT_DF8 0x60008
#define INPUT_DF9 0x60009
#define INPUT_DF10 0x600010
#define INPUT_DF11 0x600011
#define INPUT_DF12 0x600012
#define INPUT_DF13 0x600013
#define INPUT_DF14 0x600014
#define INPUT_DF15 0x600015
#define INPUT_DF16 0x600016
#define INPUT_RAWFX 0x600017
#define INPUT_RAWFY 0x600018
#define INPUT_RAWFZ 0x600019
#define INPUT_RAWTX 0x600020
#define INPUT_RAWTY 0x600021
#define INPUT_RAWTZ 0x600022
#define INPUT_OVER_LOAD_STATUS 0x600023
#define INPUT_ERROR_FLAG 0x600024

//추가 변수
#ifndef PI
#define PI	(3.14159265359)
#endif

#ifndef PI2
#define PI2	(6.28318530718)
#endif


/****************************************************************************/
/********downscale************/
/***************** PDO config****************************/
ec_pdo_entry_info_t Robotous_RFT_EC02_pdo_entries[] = 
{ 
	{0x7000,	1,	32},	/* ConfigParam1				-- RxPdo	0x1600*/
	{0x7000,	2,	32},	/* ConfigParam2*/
	{0x6000,	1,	8},		/* DF1 						-- TxPdo	0x1A00*/
	{0x6000,	2,	8},		/* DF2 */
	{0x6000,	3,	8},		/* DF3 */
	{0x6000,	4,	8},		/* DF4 */
	{0x6000,	5,	8},		/* DF5 */
	{0x6000,	6,	8},		/* DF6 */
	{0x6000,	7,	8},		/* DF7 */
	{0x6000,	8,	8},		/* DF8 */
	{0x6000,	9,	8},		/* DF9 */
	{0x6000,	10,	8},		/* DF10 */
	{0x6000,	11,	8},		/* DF11 */
	{0x6000,	12,	8},		/* DF12 */
	{0x6000,	13,	8},		/* DF13 */
	{0x6000,	14,	8},		/* DF14 */
	{0x6000,	15,	8},		/* DF15 */
	{0x6000,	16,	8},		/* DF16 */
	{0x6000,	17,	16},	/* RawFx */
	{0x6000,	18,	16},	/* RawFy */
	{0x6000,	19,	16},	/* RawFz */
	{0x6000,	20,	16},	/* RawTx */
	{0x6000,	21,	16},	/* RawTy */
	{0x6000,	22,	16},	/* RawTz */
	{0x6000,	23,	8},		/* OverloadStatus */
	{0x6000,	24,	8},		/* ErrorFlag */
};

ec_pdo_info_t Robotous_RFT_EC02_pdos[] = {
    {0x1600,	2,	Robotous_RFT_EC02_pdo_entries + 0},	/* RxPdo	0x1600: 2 entries */
	{0x1A00,	24,	Robotous_RFT_EC02_pdo_entries + 2},	/* TxPdo	0x1A00: 24 entries */
};

ec_sync_info_t Robotous_RFT_EC02_syncs[] = 
{ 
	{0, EC_DIR_OUTPUT	, 0, NULL, EC_WD_ENABLE}, 
	{1, EC_DIR_INPUT	, 0, NULL, EC_WD_ENABLE}, 
	{2, EC_DIR_OUTPUT	, 1, Robotous_RFT_EC02_pdos + 0, EC_WD_ENABLE}, 
	{3, EC_DIR_INPUT	, 1, Robotous_RFT_EC02_pdos + 1, EC_WD_ENABLE}, 
	{0xff} 
};

ec_pdo_entry_info_t Servotronix_motion_control_ltd__CDHD_pdo_entries[] = 
{ 
	{0x6040,	0,	16},	/* ControlWord 				-- RxPdo1	0x1600*/		
	{0x6060,	0,	8},		/* ModeOfOperation */
	{0x607a,	0,	32},	/* TargetPosition 			-- RxPdo2	0x1601*/
	{0x6081,	0,	32},	/* ProfileVelocity */
	{0x60ff,	0,	32},	/* TargetVelocity *			-- RxPdo3	0x1602*/
	{0x6071,	0,	16},	/* TargetTorque 			-- RxPdo4	0x1603*/
	{0x60fe,	1,	32},	/* DigitalOutpus */
	{0x60b2,	0,	16},	/* TorqueOffset */
	{0x6041,	0,	16},	/* StatusWord				-- TxPdo1	0x1A00*/
	{0x6061,	0,	8},		/* ModeOfOperationDisplay*/
	{0x6077,	0,	16},	/* TorqueActualValue */
	{0x6064,	0,	32},	/* PositionActualValue		-- TxPdo2	0x1A01*/
	{0x6074,	0,	16},	/* TorqueDemandValue		-- TxPdo3	0x1A02*/
	{0x20f2,	0,	16},	/* AnalogInput1*/
	{0x60fd,	0,	32},	/* DigitalInputs			-- TxPdo4	0x1A03*/
	{0x20b6,	0,	32},	/* ManuspecMachineHWPositionExternalcommand */
	{0x60f4,	0,	32},	/* FollowingErrorActualValue*/
};

ec_pdo_info_t Servotronix_motion_control_ltd__CDHD_pdos[] = {
    {0x1600,	2,	Servotronix_motion_control_ltd__CDHD_pdo_entries + 0},	/* RxPdo1	0x1600: 2 entries */
	{0x1601,	2,	Servotronix_motion_control_ltd__CDHD_pdo_entries + 2},	/* RxPdo2	0x1601: 2 entries */
	{0x1602,	1,	Servotronix_motion_control_ltd__CDHD_pdo_entries + 4},	/* RxPdo3	0x1602: 1 entries */
	{0x1603,	3,	Servotronix_motion_control_ltd__CDHD_pdo_entries + 5},	/* RxPdo4	0x1603: 3 entries */
	{0x1A00,	3,	Servotronix_motion_control_ltd__CDHD_pdo_entries + 8},	/* TxPdo1	0x1A00: 3 entries */
	{0x1A01,	1,	Servotronix_motion_control_ltd__CDHD_pdo_entries + 11},	/* TxPdo2	0x1A01: 1 entries */
	{0x1A02,	2,	Servotronix_motion_control_ltd__CDHD_pdo_entries + 12},	/* TxPdo3	0x1A02: 2 entries */
	{0x1A03,	3,	Servotronix_motion_control_ltd__CDHD_pdo_entries + 14},	/* TxPdo4	0x1A02: 3 entries */
};

// ec_sync_info_t Servotronix_motion_control_ltd__CDHD_syncs[] = 
// { 
// 	{0, EC_DIR_OUTPUT	, 0, NULL, EC_WD_ENABLE}, 
// 	{1, EC_DIR_INPUT	, 0, NULL, EC_WD_ENABLE}, 
// 	{2, EC_DIR_OUTPUT	, 4, Robotous_RFT_EC02_pdos + 0, EC_WD_ENABLE}, 
// 	{3, EC_DIR_INPUT	, 4, Robotous_RFT_EC02_pdos + 4, EC_WD_ENABLE}, 
// 	{0xff} 
// };

ec_sync_info_t Servotronix_motion_control_ltd__CDHD_syncs[] = {
	{0, EC_DIR_INPUT, 8, Servotronix_motion_control_ltd__CDHD_pdos + 0, EC_WD_ENABLE}, 
	{0xff}
};

/****************************************************************************/
// Struct
/********downscale************/

struct ROBOTOUS_RFT_EC02_IN
{
	uint32_t		ConfigParam1; 	// 0x7000
	uint32_t		ConfigParam2; 	// 0x7000			
};
struct ROBOTOUS_RFT_EC02_OUT
{
    uint8_t		DF1; 	// 0x6000
	uint8_t		DF2; 	// 0x6000
	uint8_t		DF3; 	// 0x6000
	uint8_t		DF4; 	// 0x6000
	uint8_t		DF5; 	// 0x6000
	uint8_t		DF6; 	// 0x6000
	uint8_t		DF7; 	// 0x6000
	uint8_t		DF8; 	// 0x6000
	uint8_t		DF9; 	// 0x6000
	uint8_t		DF10; 	// 0x6000
	uint8_t		DF11; 	// 0x6000
	uint8_t		DF12; 	// 0x6000
	uint8_t		DF13; 	// 0x6000
	uint8_t		DF14; 	// 0x6000
	uint8_t		DF15; 	// 0x6000
	uint8_t		DF16; 	// 0x6000
	int16_t		RawFx; 	// 0x6000
	int16_t		RawFy; 	// 0x6000
	int16_t		RawFz; 	// 0x6000
	int16_t		RawTx; 	// 0x6000
	int16_t		RawTy; 	// 0x6000
	int16_t		RawTz; 	// 0x6000
	uint8_t		OverloadStatus; 	// 0x6000
	uint8_t		ErrorFlag; 	// 0x6000				
};
struct ROBOTOUS_RFT_EC02
{
    ec_slave_config_t* Config;
    
    ROBOTOUS_RFT_EC02_IN 	InParam;
	ROBOTOUS_RFT_EC02_OUT 	OutParam;
	
	uint32_t offConfigParam1;
	uint32_t offConfigParam2;
	uint32_t offDF1;
	uint32_t offDF2;
	uint32_t offDF3;
	uint32_t offDF4;
	uint32_t offDF5;
	uint32_t offDF6;
	uint32_t offDF7;
	uint32_t offDF8;
	uint32_t offDF9;
	uint32_t offDF10;
	uint32_t offDF11;
	uint32_t offDF12;
	uint32_t offDF13;
	uint32_t offDF14;
	uint32_t offDF15;
	uint32_t offDF16;
	uint32_t offRawFx;
	uint32_t offRawFy;
	uint32_t offRawFz;
	uint32_t offRawTx;
	uint32_t offRawTy;
	uint32_t offRawTz;
	uint32_t offOverloadStatus;
	uint32_t offErrorFlag;
	uint32_t bitoffConfigParam1;
	uint32_t bitoffConfigParam2;
	uint32_t bitoffDF1;
	uint32_t bitoffDF2;
	uint32_t bitoffDF3;
	uint32_t bitoffDF4;
	uint32_t bitoffDF5;
	uint32_t bitoffDF6;
	uint32_t bitoffDF7;
	uint32_t bitoffDF8;
	uint32_t bitoffDF9;
	uint32_t bitoffDF10;
	uint32_t bitoffDF11;
	uint32_t bitoffDF12;
	uint32_t bitoffDF13;
	uint32_t bitoffDF14;
	uint32_t bitoffDF15;
	uint32_t bitoffDF16;
	uint32_t bitoffRawFx;
	uint32_t bitoffRawFy;
	uint32_t bitoffRawFz;
	uint32_t bitoffRawTx;
	uint32_t bitoffRawTy;
	uint32_t bitoffRawTz;
	uint32_t bitoffOverloadStatus;
	uint32_t bitoffErrorFlag;	
};

struct SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_IN
{
	//-------------- RxPdo1	0x1600
	uint16_t		ControlWord; 	// 0x6040
	int8_t		ModeOfOperation; 	// 0x6060
	//-------------- RxPdo2	0x1601
	int32_t		TargetPosition; 	// 0x607a
	uint32_t		ProfileVelocity; 	// 0x6081
	//-------------- RxPdo3	0x1602
	int32_t		TargetVelocity; 	// 0x60ff
	//-------------- RxPdo4	0x1603
	int16_t		TargetTorque; 	// 0x6071
	uint32_t		DigitalOutpus; 	// 0x60fe
	int16_t		TorqueOffset; 	// 0x60b2				
};
struct SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_OUT
{
	//-------------- TxPdo1	0x1A00
	uint16_t		StatusWord; 	// 0x6041
	int8_t		ModeOfOperationDisplay; 	// 0x6061
	int16_t		TorqueActualValue; 	// 0x6077
	//-------------- TxPdo2	0x1A01
	int32_t		PositionActualValue; 	// 0x6064
	//-------------- TxPdo3	0x1A02
	int16_t		TorqueDemandValue; 	// 0x6074
	int16_t		AnalogInput1; 	// 0x20f2
	//-------------- TxPdo4	0x1A03
	uint32_t		DigitalInputs; 	// 0x60fd
	int32_t		ManuspecMachineHWPositionExternalcommand; 	// 0x20b6
	int32_t		FollowingErrorActualValue; 	// 0x60f4				
};
struct SERVOTRONIX_MOTION_CONTROL_LTD__CDHD
{
	ec_slave_config_t* Config;
	
	SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_IN 	InParam;
	SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_OUT 	OutParam;
	
	uint32_t offControlWord;
	uint32_t offModeOfOperation;
	uint32_t offTargetPosition;
	uint32_t offProfileVelocity;
	uint32_t offTargetVelocity;
	uint32_t offTargetTorque;
	uint32_t offDigitalOutpus;
	uint32_t offTorqueOffset;
	uint32_t offStatusWord;
	uint32_t offModeOfOperationDisplay;
	uint32_t offTorqueActualValue;
	uint32_t offPositionActualValue;
	uint32_t offTorqueDemandValue;
	uint32_t offAnalogInput1;
	uint32_t offDigitalInputs;
	uint32_t offManuspecMachineHWPositionExternalcommand;
	uint32_t offFollowingErrorActualValue;
	uint32_t bitoffControlWord;
	uint32_t bitoffModeOfOperation;
	uint32_t bitoffTargetPosition;
	uint32_t bitoffProfileVelocity;
	uint32_t bitoffTargetVelocity;
	uint32_t bitoffTargetTorque;
	uint32_t bitoffDigitalOutpus;
	uint32_t bitoffTorqueOffset;
	uint32_t bitoffStatusWord;
	uint32_t bitoffModeOfOperationDisplay;
	uint32_t bitoffTorqueActualValue;
	uint32_t bitoffPositionActualValue;
	uint32_t bitoffTorqueDemandValue;
	uint32_t bitoffAnalogInput1;
	uint32_t bitoffDigitalInputs;
	uint32_t bitoffManuspecMachineHWPositionExternalcommand;
	uint32_t bitoffFollowingErrorActualValue;								
	
};
/****************************************************************************/
// Class

class EthercatMaster
{
    private:
        // Parameters
        ec_master_t* _master;
        
		/********aidin************/
        // ec_pdo_entry_reg_t _rx_domain_regs[NUM_ELMO_GOLD_RX_PDO_ENTRIES*NUM_ELMO_GOLD + 1];
        // ec_pdo_entry_reg_t _tx_domain_regs[NUM_ELMO_GOLD_TX_PDO_ENTRIES*NUM_ELMO_GOLD + NUM_HMS_ANYBUS_TX_PDO_ENTRIES + 1];
		/********downscale************/
		ec_pdo_entry_reg_t _rx_domain_regs[NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_RX_PDO_ENTRIES*NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES + NUM_ROBOTOUS_RFT_EC02_RX_PDO_ENTRIES + 1];
        ec_pdo_entry_reg_t _tx_domain_regs[NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_TX_PDO_ENTRIES*NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES + NUM_ROBOTOUS_RFT_EC02_TX_PDO_ENTRIES + 1];

        ec_domain_t* _rx_domain;
        ec_domain_t* _tx_domain;

        uint8_t* _rx_domain_pd;    // Rx process data
        uint8_t* _tx_domain_pd;    // Tx process data

        // ELMO_GOLD _elmo_gold[NUM_ELMO_GOLD];
        // HMS_ANYBUS _hms_anybus[NUM_HMS_ANYBUS];

		ROBOTOUS_RFT_EC02 _robotous_rft_ec02[NUM_ROBOTOUS_RFT_EC02_AXES];
		SERVOTRONIX_MOTION_CONTROL_LTD__CDHD _servotronix_motion_control_ltd_cdhd[NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES];

    public:
        EthercatMaster();
        ~EthercatMaster();

        int init(const int8_t mode_op);
        
        int initMaster();
        int initSlave();
		int initDomain();		
		int activateMaster();
		void initAxes();

		void syncEthercatMaster();

        int getRxDomainStatus();
        int getTxDomainStatus();
        int getMasterStatus(unsigned int& num_slave, unsigned int& state);
        
        void registerRxDomainEntry(unsigned int vendor_id, unsigned int product_code, unsigned int position, uint16_t entry_index, uint16_t entry_subindex ,unsigned int* offset, unsigned int* bit_offset);
        void registerTxDomainEntry(unsigned int vendor_id, unsigned int product_code, unsigned int position, uint16_t entry_index, uint16_t entry_subindex ,unsigned int* offset, unsigned int* bit_offset);

        void processTxDomain();
        void processRxDomain();

        void readBuffer(const int EntryID, void* data);
        void writeBuffer(const int EntryID, void* data);
        
        void servoOn(const int position);

        int initSubscriber();

		uint64_t getSysTimeDC();
};

EthercatMaster::EthercatMaster():_master{0}, _rx_domain{0}, _tx_domain{0}, _rx_domain_pd{0}, _tx_domain_pd{0}
{
}
EthercatMaster::~EthercatMaster()
{
    ecrt_release_master(this->_master);
}

int EthercatMaster::init(const int8_t mode_op)
{      
    // TODO: Initiate Axes' parameters here
    // for (int i=0; i<NUM_ELMO_GOLD; i++)
    // {
    //     this->_elmo_gold[i].InParam.MaxTorque = 1000;
	// 	this->_elmo_gold[i].InParam.ModeOfOperation = mode_op;															
    // }

	for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
    {
        // this->_servotronix_motion_control_ltd_cdhd[i].InParam.MaxTorque = 2000;
		this->_servotronix_motion_control_ltd_cdhd[i].InParam.ModeOfOperation = mode_op;															
    }
    
    if (this->initMaster() < 0)
    {
        printf("Init Master Failed.\n");
        return -1;
    }					

    if (this->initSlave() == -1)
    {
        printf("Init Slaves Failed.\n");
        return -1;
    }

    if (this->initDomain() == -1)
    {
        printf("Init Domains Failed.\n");
        return -1;
    }

    if (this->activateMaster() == -1)
    {
        return -1;
    }

    return 0;

}

int EthercatMaster::getRxDomainStatus()
{
    ec_domain_state_t ds = {};
	ecrt_domain_state(this->_rx_domain, &ds);

	return (int) ds.wc_state;
}

int EthercatMaster::getTxDomainStatus()
{
	ec_domain_state_t ds = {};
	ecrt_domain_state(this->_tx_domain, &ds);

	return (int) ds.wc_state;
}

int EthercatMaster::getMasterStatus(unsigned int& num_slave, unsigned int & state)
{
    ec_master_state_t ms;
	ecrt_master_state(this->_master, &ms);

	// #define NONE	0x00
	// #define INIT 	0x01
	// #define PRE_OP 	0x02
	// #define SAFE_OP 0x04
	// #define OP 		0x08

    if (ms.link_up)
	{
		num_slave = ms.slaves_responding;

		if (ms.al_states & OP)
			state = OP;
		else if (ms.al_states & SAFE_OP)
			state = SAFE_OP;
		else if (ms.al_states & PRE_OP)
			state = PRE_OP;
		else
			state = INIT;

		return 1;
	}
	else
	{
		num_slave = 0;
		state = 0x00;

		return 0;
	}
}

int EthercatMaster::initMaster()
{
	this->_master = ecrt_request_master(0);
	if (!this->_master)
	{
		fprintf(stderr, "Unable to get requested master.\n");
		return -1;
	}

	return 0;
}

int EthercatMaster::initSlave()
{
	/*****aidin*******/
	// for (int i=0; i<NUM_ELMO_GOLD; i++)
	// {
	// 	this->_elmo_gold[i].Config = ecrt_master_slave_config(this->_master, 0, i, ELMO_VENDOR_ID, ELMO_GOLD_PRODUCT_CODE); // Elmo Gold Vendor ID, Product Code					
	// 	if (this->_elmo_gold[i].Config == NULL)
	// 	{
	// 		printf("There is no configuration on slave ELMO_GOLD!\n");
	// 		return -1;
	// 	}
	// 	if (ecrt_slave_config_pdos(this->_elmo_gold[i].Config, EC_END, Elmo_Gold_syncs))
	// 	{
	// 		printf("Error in configuring PDOs for slave!\n");
	// 		return -1;
	// 	}
	// }
	// for (int i=0; i<NUM_HMS_ANYBUS; i++)
	// {
	// 	this->_hms_anybus[i].Config = ecrt_master_slave_config(this->_master, 0, NUM_ELMO_GOLD + i, HMS_VENDOR_ID, HMS_ANYBUS_PRODUCT_CODE); // HMS Anybus Vendor ID, Product Code			
	// 	if (this->_hms_anybus[i].Config == NULL)
	// 	{
	// 		printf("There is no configuration on slave HMS_ANYBUS!\n");
	// 		return -1;
	// 	}

							
	// 	if (ecrt_slave_config_pdos(this->_hms_anybus[i].Config, EC_END, Hms_Anybus_syncs))
	// 	{
	// 		printf("Error in configuring PDOs for slave!\n");
	// 		return -1;
	// 	}
	// }

	/*****downscale*******/
	for (int i = 0; i < NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
	{
		this->_servotronix_motion_control_ltd_cdhd[i].Config = ecrt_master_slave_config(this->_master, 0, i, SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE); // Servotronix Vendor ID, Product Code					
		if (this->_servotronix_motion_control_ltd_cdhd[i].Config == NULL)
		{
			printf("There is no configuration on slave SERVOTRONIX_MOTION_CONTROL_LTD__CDHD!\n");
			return -1;
		}
		if (ecrt_slave_config_pdos(this->_servotronix_motion_control_ltd_cdhd[i].Config, EC_END, Servotronix_motion_control_ltd__CDHD_syncs))
		{
			printf("Error in configuring PDOs for slave!\n");
			return -1;
		}
	}

    for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++)
	{
		this->_robotous_rft_ec02[i].Config = ecrt_master_slave_config(this->_master, 0, NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES + i, ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE); // HMS Anybus Vendor ID, Product Code			
		if (this->_robotous_rft_ec02[i].Config == NULL)
		{
			printf("There is no configuration on slave ROBOTOUS_RFT_EC02!\n");
			return -1;
		}

							
		if (ecrt_slave_config_pdos(this->_robotous_rft_ec02[i].Config, EC_END, Robotous_RFT_EC02_syncs))
		{
			printf("Error in configuring PDOs for slave!\n");
			return -1;
		}
	}


	return 0;
}

int EthercatMaster::initDomain()
{
    // Create Domain
    this->_rx_domain = ecrt_master_create_domain(_master);
    if (this->_rx_domain == NULL)
    {
        printf("Creating Rxdomain failed!\n");
		return -1;
    }
    this->_tx_domain = ecrt_master_create_domain(_master);
    if (this->_tx_domain == NULL)
    {
        printf("Creating Txdomain failed!\n");
		return -1;
    }
       
	/*****downscale*******/
    for(int i = 0; i < NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
    {

        int position = i;
        // RxDomain
		this->registerRxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6040, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offControlWord, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffControlWord);
        this->registerRxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6060, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offModeOfOperation, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffModeOfOperation);
        this->registerRxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x607A, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offTargetPosition, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffTargetPosition);
		this->registerRxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6081, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offProfileVelocity, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffProfileVelocity);
        this->registerRxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x60FF, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offTargetVelocity, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffTargetVelocity);
        this->registerRxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6071, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offTargetTorque, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffTargetTorque);
		this->registerRxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x60FE, 1, &this->_servotronix_motion_control_ltd_cdhd[i].offDigitalOutpus, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffDigitalOutpus);
        this->registerRxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x60B2, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offTorqueOffset, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffTorqueOffset);
        
		// TxDomain
		this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6041, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offStatusWord, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffStatusWord);
        this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6061, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offModeOfOperationDisplay, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffModeOfOperationDisplay);
        this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6077, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offTorqueActualValue, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffTorqueActualValue);
		this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6064, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offPositionActualValue, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffPositionActualValue);
        this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x6074, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offTorqueDemandValue, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffTorqueDemandValue);
		this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x20F2, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offAnalogInput1, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffAnalogInput1);
		this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x60FD, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offDigitalInputs, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffDigitalInputs);
		this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x20B6, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offManuspecMachineHWPositionExternalcommand, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffManuspecMachineHWPositionExternalcommand);
		this->registerTxDomainEntry(SERVOTRONIX_VENDOR_ID, SERVOTRONIX_PRODUCT_CODE, position, 0x60F4, 0, &this->_servotronix_motion_control_ltd_cdhd[i].offFollowingErrorActualValue, &this->_servotronix_motion_control_ltd_cdhd[i].bitoffFollowingErrorActualValue);
	}

    // for (int i = 0; i < NUM_ROBOTOUS_RFT_EC02_AXES; i++)
    // {
    //     int position = NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES + i;
	// 	// RxDomain
    //     this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x7000, 1, &this->_robotous_rft_ec02[i].offConfigParam1, &this->_robotous_rft_ec02[i].bitoffConfigParam1);	/* ConfigParam1 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x7000, 2, &this->_robotous_rft_ec02[i].offConfigParam2, &this->_robotous_rft_ec02[i].bitoffConfigParam2);	/* ConfigParam2 */ 

	// 	// TxDomain
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 1, &this->_robotous_rft_ec02[i].offDF1, &this->_robotous_rft_ec02[i].bitoffDF1);	/* DF1 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 2, &this->_robotous_rft_ec02[i].offDF2, &this->_robotous_rft_ec02[i].bitoffDF2);	/* DF2 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 3, &this->_robotous_rft_ec02[i].offDF3, &this->_robotous_rft_ec02[i].bitoffDF3);	/* DF3 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 4, &this->_robotous_rft_ec02[i].offDF4, &this->_robotous_rft_ec02[i].bitoffDF4);	/* DF4 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 5, &this->_robotous_rft_ec02[i].offDF5, &this->_robotous_rft_ec02[i].bitoffDF5);	/* DF5 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 6, &this->_robotous_rft_ec02[i].offDF6, &this->_robotous_rft_ec02[i].bitoffDF6);	/* DF6 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 7, &this->_robotous_rft_ec02[i].offDF7, &this->_robotous_rft_ec02[i].bitoffDF7);	/* DF7 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 8, &this->_robotous_rft_ec02[i].offDF8, &this->_robotous_rft_ec02[i].bitoffDF8);	/* DF8 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 9, &this->_robotous_rft_ec02[i].offDF9, &this->_robotous_rft_ec02[i].bitoffDF9);	/* DF9 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 10, &this->_robotous_rft_ec02[i].offDF10, &this->_robotous_rft_ec02[i].bitoffDF10);	/* DF10 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 11, &this->_robotous_rft_ec02[i].offDF11, &this->_robotous_rft_ec02[i].bitoffDF11);	/* DF11 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 12, &this->_robotous_rft_ec02[i].offDF12, &this->_robotous_rft_ec02[i].bitoffDF12);	/* DF12 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 13, &this->_robotous_rft_ec02[i].offDF13, &this->_robotous_rft_ec02[i].bitoffDF13);	/* DF13 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 14, &this->_robotous_rft_ec02[i].offDF14, &this->_robotous_rft_ec02[i].bitoffDF14);	/* DF14 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 15, &this->_robotous_rft_ec02[i].offDF15, &this->_robotous_rft_ec02[i].bitoffDF15);	/* DF15 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 16, &this->_robotous_rft_ec02[i].offDF16, &this->_robotous_rft_ec02[i].bitoffDF16);	/* DF16 */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 17, &this->_robotous_rft_ec02[i].offRawFx, &this->_robotous_rft_ec02[i].bitoffRawFx);	/* RawFx */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 18, &this->_robotous_rft_ec02[i].offRawFy, &this->_robotous_rft_ec02[i].bitoffRawFy);	/* RawFy */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 19, &this->_robotous_rft_ec02[i].offRawFz, &this->_robotous_rft_ec02[i].bitoffRawFz);	/* RawFz */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 20, &this->_robotous_rft_ec02[i].offRawTx, &this->_robotous_rft_ec02[i].bitoffRawTx);	/* RawTx */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 21, &this->_robotous_rft_ec02[i].offRawTy, &this->_robotous_rft_ec02[i].bitoffRawTy);	/* RawTy */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 22, &this->_robotous_rft_ec02[i].offRawTz, &this->_robotous_rft_ec02[i].bitoffRawTz);	/* RawTz */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 23, &this->_robotous_rft_ec02[i].offOverloadStatus, &this->_robotous_rft_ec02[i].bitoffOverloadStatus);	/* OverloadStatus */
	// 	this->registerTxDomainEntry(ROBOTOUS_VENDOR_ID, ROBOTOUS_PRODUCT_CODE, position, 0x6000, 24, &this->_robotous_rft_ec02[i].offErrorFlag, &this->_robotous_rft_ec02[i].bitoffErrorFlag);	/* ErrorFlag */

	// }

    // Init Domain
	if (ecrt_domain_reg_pdo_entry_list(this->_rx_domain, this->_rx_domain_regs) != 0)
	{
		printf("Failed to register Rxdomain entry!\n");
		return -1;
	}

	if (ecrt_domain_reg_pdo_entry_list(this->_tx_domain, this->_tx_domain_regs) != 0)
	{
		printf("Failed to register Txdomain entry!\n");
		return -1;
	}

	return 0;
}

int EthercatMaster::activateMaster()
{
    if (this->_master != NULL)
    {
        printf("Activating master...\n");
		if (ecrt_master_activate(this->_master))
		{
			fprintf(stderr,"Master Activation failed.\n");\
			return -1;
		}
		usleep(1000);

		if (!(this->_rx_domain_pd = ecrt_domain_data(this->_rx_domain)))
		{
			printf("Failed to initialize domain data pointer.\n");
			return -1;
		}

        if (!(this->_tx_domain_pd = ecrt_domain_data(this->_tx_domain)))
		{
			printf("Failed to initialize domain data pointer.\n");
			return -1;
		}

		return 0;
    }
    return -1;
}

uint64_t EthercatMaster::getSysTimeDC()
{
	RTIME time = rt_timer_read();

	if (0 > time) {
		//rt_printf("system_time_base: %lld, time: %llu\n", system_time_base, time);
		return time;
	}
	else {
		return time - 0;
	}
}

void EthercatMaster::syncEthercatMaster()
{
	// set master time in nano-seconds
	ecrt_master_application_time(this->_master, this->getSysTimeDC());

	// sync reference clock to master
	ecrt_master_sync_reference_clock(this->_master);

	// call to sync slaves to ref slave
	ecrt_master_sync_slave_clocks(this->_master);
}

void EthercatMaster::registerRxDomainEntry(unsigned int vendor_id, unsigned int product_code, unsigned int position, uint16_t entry_index, uint16_t entry_subindex ,unsigned int* offset, unsigned int* bit_offset)
{
    static unsigned int rx_domain_idx = 0;

    this->_rx_domain_regs[rx_domain_idx].alias = 0;
	this->_rx_domain_regs[rx_domain_idx].position = position;
	this->_rx_domain_regs[rx_domain_idx].vendor_id = vendor_id;       // Elmo Gold Vendor ID
	this->_rx_domain_regs[rx_domain_idx].product_code = product_code;    // Elmo Gold Product code

	this->_rx_domain_regs[rx_domain_idx].index = entry_index;
	this->_rx_domain_regs[rx_domain_idx].subindex = entry_subindex;
	this->_rx_domain_regs[rx_domain_idx].offset = offset;
	this->_rx_domain_regs[rx_domain_idx].bit_position = bit_offset;

    rx_domain_idx++;
}

void EthercatMaster::registerTxDomainEntry(unsigned int vendor_id, unsigned int product_code, unsigned int position, uint16_t entry_index, uint16_t entry_subindex ,unsigned int* offset, unsigned int* bit_offset)
{
    static unsigned int tx_domain_idx = 0;

    this->_tx_domain_regs[tx_domain_idx].alias = 0;
	this->_tx_domain_regs[tx_domain_idx].position = position;
	this->_tx_domain_regs[tx_domain_idx].vendor_id = vendor_id;       // Elmo Gold Vendor ID
	this->_tx_domain_regs[tx_domain_idx].product_code = product_code;    // Elmo Gold Product code

	this->_tx_domain_regs[tx_domain_idx].index = entry_index;
	this->_tx_domain_regs[tx_domain_idx].subindex = entry_subindex;
	this->_tx_domain_regs[tx_domain_idx].offset = offset;
	this->_tx_domain_regs[tx_domain_idx].bit_position = bit_offset;

    tx_domain_idx++;
}

void EthercatMaster::processRxDomain()
{
	ecrt_master_receive(this->_master); //RECEIVE A FRAME
	ecrt_domain_process(this->_rx_domain);
	
	// TODO: Write control data from axes to servos via master

	for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; ++i)
	{
		servoOn(i);

		EC_WRITE_U16(this->_rx_domain_pd +  this->_servotronix_motion_control_ltd_cdhd[i].offControlWord, this->_servotronix_motion_control_ltd_cdhd[i].InParam.ControlWord);
		EC_WRITE_S8(this->_rx_domain_pd +  this->_servotronix_motion_control_ltd_cdhd[i].offModeOfOperation, this->_servotronix_motion_control_ltd_cdhd[i].InParam.ModeOfOperation);
		EC_WRITE_S32(this->_rx_domain_pd +  this->_servotronix_motion_control_ltd_cdhd[i].offTargetPosition, this->_servotronix_motion_control_ltd_cdhd[i].InParam.TargetPosition);
		EC_WRITE_U32(this->_rx_domain_pd +  this->_servotronix_motion_control_ltd_cdhd[i].offProfileVelocity, this->_servotronix_motion_control_ltd_cdhd[i].InParam.ProfileVelocity);
		EC_WRITE_S32(this->_rx_domain_pd +  this->_servotronix_motion_control_ltd_cdhd[i].offTargetVelocity, this->_servotronix_motion_control_ltd_cdhd[i].InParam.TargetVelocity);
		EC_WRITE_S16(this->_rx_domain_pd +  this->_servotronix_motion_control_ltd_cdhd[i].offTargetTorque, this->_servotronix_motion_control_ltd_cdhd[i].InParam.TargetTorque);
		EC_WRITE_U32(this->_rx_domain_pd +  this->_servotronix_motion_control_ltd_cdhd[i].offDigitalOutpus, this->_servotronix_motion_control_ltd_cdhd[i].InParam.DigitalOutpus);
		EC_WRITE_S16(this->_rx_domain_pd +  this->_servotronix_motion_control_ltd_cdhd[i].offTorqueOffset, this->_servotronix_motion_control_ltd_cdhd[i].InParam.TorqueOffset);
	}

	// for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; ++i)
	// {
	// 	// servoOn(i);

	// 	EC_WRITE_U32(this->_rx_domain_pd +  this->_robotous_rft_ec02[i].offConfigParam1, this->_robotous_rft_ec02[i].InParam.ConfigParam1);
	// 	EC_WRITE_U32(this->_rx_domain_pd +  this->_robotous_rft_ec02[i].offConfigParam2, this->_robotous_rft_ec02[i].InParam.ConfigParam2);	
	// }
				
	ecrt_domain_queue(this->_rx_domain);
	ecrt_master_send(this->_master); //SEND ALL QUEUED DATAGRAMS		
}

void EthercatMaster::processTxDomain()
{
	ecrt_master_receive(this->_master); //RECEIVE A FRAME
	ecrt_domain_process(this->_tx_domain);
	
	// TODO: Read feedback data from servos to axes via master		
	
	for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; ++i)
	{
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.StatusWord = EC_READ_U16(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offStatusWord);
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.ModeOfOperationDisplay = EC_READ_S8(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offModeOfOperationDisplay);
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.TorqueActualValue = EC_READ_S16(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offTorqueActualValue);
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.PositionActualValue = EC_READ_S32(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offPositionActualValue);
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.TorqueDemandValue = EC_READ_S16(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offTorqueDemandValue);
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.AnalogInput1 = EC_READ_S16(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offAnalogInput1);
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.DigitalInputs = EC_READ_U32(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offDigitalInputs);
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.ManuspecMachineHWPositionExternalcommand = EC_READ_S32(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offManuspecMachineHWPositionExternalcommand);
		this->_servotronix_motion_control_ltd_cdhd[i].OutParam.FollowingErrorActualValue = EC_READ_S32(this->_tx_domain_pd + this->_servotronix_motion_control_ltd_cdhd[i].offFollowingErrorActualValue);
	}

    // for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; ++i)
	// {
	// 	this->_robotous_rft_ec02[i].OutParam.DF1 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF1);
	// 	this->_robotous_rft_ec02[i].OutParam.DF2 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF2);
	// 	this->_robotous_rft_ec02[i].OutParam.DF3 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF3);
	// 	this->_robotous_rft_ec02[i].OutParam.DF4 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF4);
	// 	this->_robotous_rft_ec02[i].OutParam.DF5 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF5);
	// 	this->_robotous_rft_ec02[i].OutParam.DF6 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF6);
	// 	this->_robotous_rft_ec02[i].OutParam.DF7 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF7);
	// 	this->_robotous_rft_ec02[i].OutParam.DF8 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF8);
	// 	this->_robotous_rft_ec02[i].OutParam.DF9 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF9);
	// 	this->_robotous_rft_ec02[i].OutParam.DF10 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF10);
	// 	this->_robotous_rft_ec02[i].OutParam.DF11 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF11);
	// 	this->_robotous_rft_ec02[i].OutParam.DF12 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF12);
	// 	this->_robotous_rft_ec02[i].OutParam.DF13 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF13);
	// 	this->_robotous_rft_ec02[i].OutParam.DF14 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF14);
	// 	this->_robotous_rft_ec02[i].OutParam.DF15 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF15);
	// 	this->_robotous_rft_ec02[i].OutParam.DF16 = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offDF16);
	// 	this->_robotous_rft_ec02[i].OutParam.RawFx = EC_READ_S16(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offRawFx);
	// 	this->_robotous_rft_ec02[i].OutParam.RawFy = EC_READ_S16(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offRawFy);
	// 	this->_robotous_rft_ec02[i].OutParam.RawFz = EC_READ_S16(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offRawFz);
	// 	this->_robotous_rft_ec02[i].OutParam.RawTx = EC_READ_S16(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offRawTx);
	// 	this->_robotous_rft_ec02[i].OutParam.RawTy = EC_READ_S16(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offRawTy);
	// 	this->_robotous_rft_ec02[i].OutParam.RawTz = EC_READ_S16(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offRawTz);
	// 	this->_robotous_rft_ec02[i].OutParam.OverloadStatus = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offOverloadStatus);
	// 	this->_robotous_rft_ec02[i].OutParam.ErrorFlag = EC_READ_U8(this->_tx_domain_pd + this->_robotous_rft_ec02[i].offErrorFlag);
	// }

	ecrt_domain_queue(this->_tx_domain);
	ecrt_master_send(this->_master); //SEND ALL QUEUED DATAGRAMS				
}


void EthercatMaster::writeBuffer(const int EntryID, void* data)
{
    switch (EntryID)
	{
		// SERVOTRONIX
		// #define CONTROL_WORD 0x6040			//UINT16
		// #define MODE_OF_OPERATION 0x6060		//INT8
		// #define TARGET_POSITION 0x607A		//INT32
		// #define PROFILE_VELOCITY 0x6081		//UINT32
		// #define TARGET_VELOCITY 0x60FF		//INT32
		// #define TARGET_TORQUE 0x6071			//INT16
		// #define DIGITAL_OUTPUTS 0x60fE		//UINT32
		// #define TORQUE_OFFSET 0x60B2			//INT16

		case CONTROL_WORD:
		{
			uint16_t * const _controlWord = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                this->_servotronix_motion_control_ltd_cdhd[i].InParam.ControlWord = _controlWord[i];
            }
		}
			break;	

		case MODE_OF_OPERATION:
		{
			int8_t * const _modeOfOperation = static_cast<int8_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                this->_servotronix_motion_control_ltd_cdhd[i].InParam.ModeOfOperation = _modeOfOperation[i];
            }
		}
			break;

		case TARGET_POSITION:
		{
			int32_t* _targetPosition = static_cast<int32_t *>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                this->_servotronix_motion_control_ltd_cdhd[i].InParam.TargetPosition = _targetPosition[i];
            }
		}
			break;						

		case PROFILE_VELOCITY:
		{
			uint32_t* _profileVelocity = static_cast<uint32_t *>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                this->_servotronix_motion_control_ltd_cdhd[i].InParam.ProfileVelocity = _profileVelocity[i];
            }
		}
			break;

		case TARGET_VELOCITY:
		{
			int32_t * const _targetVelocity = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                this->_servotronix_motion_control_ltd_cdhd[i].InParam.TargetVelocity = _targetVelocity[i];
            }
		}
			break;						

		case TARGET_TORQUE:
		{
			int16_t * const _targetTorque = static_cast<int16_t * const>(data);		
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                this->_servotronix_motion_control_ltd_cdhd[i].InParam.TargetTorque = _targetTorque[i];
            }
		}
			break;						

		case DIGITAL_OUTPUTS:
		{
			uint32_t * const _digitalOutputs = static_cast<uint32_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                this->_servotronix_motion_control_ltd_cdhd[i].InParam.DigitalOutpus = _digitalOutputs[i];
            }
		}
			break;						

		case TORQUE_OFFSET:
		{
			int16_t * const _torqueOffset = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                this->_servotronix_motion_control_ltd_cdhd[i].InParam.TorqueOffset = _torqueOffset[i];
            }
		}
			break;					

		//ROBOTOUS
		// #define CONFIG_PARAM1 0x7000		//UINT32
		// #define CONFIG_PARAM2 0x7000		//UINT32			

		case CONFIG_PARAM1:
		{
			uint32_t * const _configParam1 = static_cast<uint32_t * const>(data);
            for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++)
            {
                this->_robotous_rft_ec02[i].InParam.ConfigParam1 = _configParam1[i];
            }
		}
			break;

		case CONFIG_PARAM2:
		{
			uint32_t * const _configParam2 = static_cast<uint32_t * const>(data);
            for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++)
            {
                this->_robotous_rft_ec02[i].InParam.ConfigParam2 = _configParam2[i];
            }
		}
			break;

		default:	// Undefined Entry ID					
			break;
	}
}

void EthercatMaster::readBuffer(const int EntryID, void* data)
{
    switch (EntryID)
	{		
		// SERVOTRONIX
		// #define STATUS_WORD 0x6041											//UINT16
		// #define MODE_OF_OPERATION_DISPLAY 0x6061								//INT8
		// #define TORQUE_ACTUAL_VALUE 0x6077									//INT16
		// #define POSITION_ACTUAL_VALUE 0x6064									//INT32
		// #define TORQUE_DEMAND_VALUE 0x6074									//INT16
		// #define ANALOG_INPUT1 0x20F2											//INT16
		// #define DIGITAL_INPUTS 0x60FD										//UINT32
		// #define MANUSPECS_MACHINE_HW_POSITION_EXTERNAL_COMMAND 0x20B6		//INT32
		// #define POSITION_FOLLOWING_ERROR_ACTUAL_VALUE 0x60F4					//INT32

		case STATUS_WORD:
		{
			uint16_t * _statusWord = static_cast<uint16_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _statusWord[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.StatusWord;
            }
		}
			break;	

		case MODE_OF_OPERATION_DISPLAY:
		{
			int8_t * _modeOfOperationDisplay = static_cast<int8_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _modeOfOperationDisplay[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.ModeOfOperationDisplay;
            }
		}
			break;	

		case TORQUE_ACTUAL_VALUE:
		{
			int16_t * _torqueActualValue = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _torqueActualValue[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.TorqueActualValue;
            }
		}
			break;	

		case POSITION_ACTUAL_VALUE:
		{
            int32_t * _positionActualValue = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _positionActualValue[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.PositionActualValue;
            }
			
		}
			break;

		case TORQUE_DEMAND_VALUE:
		{
            int16_t * _torqueDemandValue = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _torqueDemandValue[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.TorqueDemandValue;
            }
			
		}
			break;

		case ANALOG_INPUT1:
		{
            int16_t * _analogInput1 = static_cast<int16_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _analogInput1[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.AnalogInput1;
            }
			
		}
			break;

		case DIGITAL_INPUTS:
		{
            uint32_t * _digitalInputs = static_cast<uint32_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _digitalInputs[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.DigitalInputs;
            }
			
		}
			break;

		case MANUSPECS_MACHINE_HW_POSITION_EXTERNAL_COMMAND:
		{
            int32_t * _manuspecMachineHWPositionExternalcommand = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _manuspecMachineHWPositionExternalcommand[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.ManuspecMachineHWPositionExternalcommand;
            }
			
		}
			break;

		case POSITION_FOLLOWING_ERROR_ACTUAL_VALUE:
		{
            int32_t * _followingErrorActualValue = static_cast<int32_t * const>(data);
            for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
            {
                _followingErrorActualValue[i] = this->_servotronix_motion_control_ltd_cdhd[i].OutParam.FollowingErrorActualValue;
            }
			
		}
			break;

		//ROBOTOUS
		// #define DF1 0x60001		//UINT8
		// #define DF2 0x60002		//UINT8
		// #define DF3 0x60003		//UINT8	
		// #define DF4 0x60004		//UINT8
		// #define DF5 0x60005		//UINT8
		// #define DF6 0x60006		//UINT8
		// #define DF7 0x60007		//UINT8
		// #define DF8 0x60008		//UINT8
		// #define DF9 0x60009		//UINT8
		// #define DF10 0x600010		//UINT8
		// #define DF11 0x600011		//UINT8
		// #define DF12 0x600012		//UINT8
		// #define DF13 0x600013		//UINT8
		// #define DF14 0x600014		//UINT8
		// #define DF15 0x600015		//UINT8
		// #define DF16 0x600016		//UINT8
		// #define RAWFX 0x600017		//INT16
		// #define RAWFY 0x600018		//INT16
		// #define RAWFZ 0x600019		//INT16
		// #define RAWTX 0x600020		//INT16
		// #define RAWTY 0x600021		//INT16
		// #define RAWTZ 0x600022		//INT16
		// #define OVER_LOAD_STATUS 0x600023		//UINT8
		// #define ERROR_FLAG 0x600024			//UINT8

        case INPUT_DF1:
		{
			uint8_t * const _df1 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df1[i] = this->_robotous_rft_ec02[i].OutParam.DF1;
						}

		}
			break;			

		case INPUT_DF2:
		{
			uint8_t * const _df2 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df2[i] = this->_robotous_rft_ec02[i].OutParam.DF2;
						}

		}
			break;

		case INPUT_DF3:
		{
			uint8_t * const _df3 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df3[i] = this->_robotous_rft_ec02[i].OutParam.DF3;
						}

		}
			break;

		case INPUT_DF4:
		{
			uint8_t * const _df4 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df4[i] = this->_robotous_rft_ec02[i].OutParam.DF4;
						}

		}
			break;

		case INPUT_DF5:
		{
			uint8_t * const _df5 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df5[i] = this->_robotous_rft_ec02[i].OutParam.DF5;
						}

		}
			break;

		case INPUT_DF6:
		{
			uint8_t * const _df6 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df6[i] = this->_robotous_rft_ec02[i].OutParam.DF6;
						}

		}
			break;

		case INPUT_DF7:
		{
			uint8_t * const _df7 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df7[i] = this->_robotous_rft_ec02[i].OutParam.DF7;
						}

		}
			break;

		case INPUT_DF8:
		{
			uint8_t * const _df8 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df8[i] = this->_robotous_rft_ec02[i].OutParam.DF8;
						}

		}
			break;

		case INPUT_DF9:
		{
			uint8_t * const _df9 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df9[i] = this->_robotous_rft_ec02[i].OutParam.DF9;
						}

		}
			break;

		case INPUT_DF10:
		{
			uint8_t * const _df10 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df10[i] = this->_robotous_rft_ec02[i].OutParam.DF10;
						}

		}
			break;

		case INPUT_DF11:
		{
			uint8_t * const _df11 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df11[i] = this->_robotous_rft_ec02[i].OutParam.DF11;
						}

		}
			break;

		case INPUT_DF12:
		{
			uint8_t * const _df12 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df12[i] = this->_robotous_rft_ec02[i].OutParam.DF12;
						}

		}
			break;

		case INPUT_DF13:
		{
			uint8_t * const _df13 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df13[i] = this->_robotous_rft_ec02[i].OutParam.DF13;
						}

		}
			break;

		case INPUT_DF14:
		{
			uint8_t * const _df14 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df14[i] = this->_robotous_rft_ec02[i].OutParam.DF14;
						}

		}
			break;

		case INPUT_DF15:
		{
			uint8_t * const _df15 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df15[i] = this->_robotous_rft_ec02[i].OutParam.DF15;
						}

		}
			break;

		case INPUT_DF16:
		{
			uint8_t * const _df16 = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_df16[i] = this->_robotous_rft_ec02[i].OutParam.DF16;
						}

		}
			break;

		case INPUT_RAWFX:
		{
			int16_t * const _rawfx = static_cast<int16_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_rawfx[i] = this->_robotous_rft_ec02[i].OutParam.RawFx;
						}

		}
			break;

		case INPUT_RAWFY:
		{
			int16_t * const _rawfy = static_cast<int16_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_rawfy[i] = this->_robotous_rft_ec02[i].OutParam.RawFy;
						}

		}
			break;

		case INPUT_RAWFZ:
		{
			int16_t * const _rawfz = static_cast<int16_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_rawfz[i] = this->_robotous_rft_ec02[i].OutParam.RawFz;
						}

		}
			break;
		
		case INPUT_RAWTX:
		{
			int16_t * const _rawtx = static_cast<int16_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_rawtx[i] = this->_robotous_rft_ec02[i].OutParam.RawTx;
						}

		}
			break;

		case INPUT_RAWTY:
		{
			int16_t * const _rawty = static_cast<int16_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_rawty[i] = this->_robotous_rft_ec02[i].OutParam.RawTy;
						}

		}
			break;

		case INPUT_RAWTZ:
		{
			int16_t * const _rawtz = static_cast<int16_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_rawtz[i] = this->_robotous_rft_ec02[i].OutParam.RawTz;
						}

		}
			break;

		case INPUT_OVER_LOAD_STATUS:
		{
			uint8_t * const _overLoadStatus = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_overLoadStatus[i] = this->_robotous_rft_ec02[i].OutParam.OverloadStatus;
						}

		}
			break;

		case INPUT_ERROR_FLAG:
		{
			uint8_t * const _errorFlag = static_cast<uint8_t * const>(data);
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++){
							_errorFlag[i] = this->_robotous_rft_ec02[i].OutParam.ErrorFlag;
						}

		}
			break;
		
		default:	// Undefined Entry ID
			break;
	}
}


void EthercatMaster::servoOn(const int position)
{
    if (!(this->_servotronix_motion_control_ltd_cdhd[position].OutParam.StatusWord & (1<<STATUSWORD_OPERATION_ENABLE_BIT)))
	{
		if (!(this->_servotronix_motion_control_ltd_cdhd[position].OutParam.StatusWord & (1<<STATUSWORD_SWITCHED_ON_BIT))) {
			if (!(this->_servotronix_motion_control_ltd_cdhd[position].OutParam.StatusWord & (1<<STATUSWORD_READY_TO_SWITCH_ON_BIT))) {
				if ((this->_servotronix_motion_control_ltd_cdhd[position].OutParam.StatusWord & (1<<STATUSWORD_FAULT_BIT))) {
					this->_servotronix_motion_control_ltd_cdhd[position].InParam.ControlWord = 0x80; //fault reset
				}
				else
				{
					// SERVOTRONIX CATALOGUE Page. 60
					this->_servotronix_motion_control_ltd_cdhd[position].InParam.ControlWord = 0x06; //shutdown
				}
			}
			else
			{
				this->_servotronix_motion_control_ltd_cdhd[position].InParam.ControlWord = 0x07; //switch on
			}
		}
		else
		{
			this->_servotronix_motion_control_ltd_cdhd[position].InParam.ControlWord = 0x0F; //switch on
		}
	}
	else
	{
		this->_servotronix_motion_control_ltd_cdhd[position].InParam.ControlWord = 0x0F; //switch on
	}
}

//***********************MOTOR INIT AND BREAK OFF*******************************
void EthercatMaster::initAxes()
{
	/*
	ServoAxis.h <- Neuromeka built-in function
	const int gearRatio[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const int pulsePerRevolution[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const double ratedTau[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const int dirQ[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const int dirTau[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	const int zeroPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
	*/

	for (int i = 0; i < NUM_AXIS; i++)
	{
		// Axis[i].setGearRatio(1);
		// Axis[i].setPulsePerRevolution(1);
		// Axis[i].setRatedTau(1);

		// Axis[i].setDirQ(1);
		// Axis[i].setDirTau(1);

		// Axis[i].setConversionConstants();

		// Axis[i].setTrajPeriod(period);

		// Axis[i].setTarVelInCnt(0);
		// Axis[i].setTarTorInCnt(0);

    	// Motor break off
		// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOn(i);

		// If 0 -> digital output assigned to the break is off
		// If 1 -> digital output assigned to the break is on
		this->_servotronix_motion_control_ltd_cdhd[i].InParam.DigitalOutpus = true;
        
	}

  	// Use this commands for stopping the motor
	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(0);
	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(1);
	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(2);
	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(3);
	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(4);
	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(5);
    //_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(6);

	this->_servotronix_motion_control_ltd_cdhd[0].InParam.DigitalOutpus = false;
	this->_servotronix_motion_control_ltd_cdhd[1].InParam.DigitalOutpus = false;
	this->_servotronix_motion_control_ltd_cdhd[2].InParam.DigitalOutpus = false;
	this->_servotronix_motion_control_ltd_cdhd[3].InParam.DigitalOutpus = false;
	this->_servotronix_motion_control_ltd_cdhd[4].InParam.DigitalOutpus = false;
	this->_servotronix_motion_control_ltd_cdhd[5].InParam.DigitalOutpus = false;
	// this->_servotronix_motion_control_ltd_cdhd[6].InParam.DigitalOutpus = false;

	// ethercatMaster.processRxDomain();

	// return 1;
}

// //***********************MOTOR INIT AND BREAK OFF*******************************
// int initAxes()
// {
// 	/*
// 	ServoAxis.h <- Neuromeka built-in function
// 	const int gearRatio[NUM_AXIS] = {0,0,0,0,0,0,0,0};
// 	const int pulsePerRevolution[NUM_AXIS] = {0,0,0,0,0,0,0,0};
// 	const double ratedTau[NUM_AXIS] = {0,0,0,0,0,0,0,0};
// 	const int dirQ[NUM_AXIS] = {0,0,0,0,0,0,0,0};
// 	const int dirTau[NUM_AXIS] = {0,0,0,0,0,0,0,0};
// 	const int zeroPos[NUM_AXIS] = {0,0,0,0,0,0,0,0};
// 	*/

// 	for (int i = 0; i < NUM_AXIS; i++)
// 	{
// 		// Axis[i].setGearRatio(1);
// 		// Axis[i].setPulsePerRevolution(1);
// 		// Axis[i].setRatedTau(1);

// 		// Axis[i].setDirQ(1);
// 		// Axis[i].setDirTau(1);

// 		// Axis[i].setConversionConstants();

// 		// Axis[i].setTrajPeriod(period);

// 		// Axis[i].setTarVelInCnt(0);
// 		// Axis[i].setTarTorInCnt(0);

//     	// Motor break off
// 		// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOn(i);

// 		// If 0 -> digital output assigned to the break is off
// 		// If 1 -> digital output assigned to the break is on
// 		this->_servotronix_motion_control_ltd_cdhd[i].InParam.DigitalOutpus = true;
        
// 	}

//   	// Use this commands for stopping the motor
// 	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(0);
// 	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(1);
// 	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(2);
// 	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(3);
// 	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(4);
// 	// _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(5);
//     //_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.setServoOff(6);

// 	this->_servotronix_motion_control_ltd_cdhd[0].InParam.DigitalOutpus = false;
// 	this->_servotronix_motion_control_ltd_cdhd[1].InParam.DigitalOutpus = false;
// 	this->_servotronix_motion_control_ltd_cdhd[2].InParam.DigitalOutpus = false;
// 	this->_servotronix_motion_control_ltd_cdhd[3].InParam.DigitalOutpus = false;
// 	this->_servotronix_motion_control_ltd_cdhd[4].InParam.DigitalOutpus = false;
// 	this->_servotronix_motion_control_ltd_cdhd[5].InParam.DigitalOutpus = false;
// 	// this->_servotronix_motion_control_ltd_cdhd[6].InParam.DigitalOutpus = false;

// 	ethercatMaster.processRxDomain();

// 	return 1;
// }

/*****************************************************************************
 * Global variable
 ****************************************************************************/

// 이더켓 세팅을 위한 객체
EthercatMaster ethercatMaster;


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

/*****aidin*****/
// Xenomai RT tasks
// RT_TASK my_task;
// RT_TASK print_task;
// RT_TASK input_task;
// RT_TASK sensoryfeedback_task;

/*****downscale*****/
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

// Global time (beginning from zero)
double gt=0;

/// TO DO: This is user-code.
double sine_amp=50000, f=0.2, period;

int InitFlag[NUM_AXIS] = {0,0,0,0,0,0,0,0};

// EtherCAT Data (in pulse)
INT32 	ZeroPos_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT16	StatusWord_data[NUM_AXIS] =  {0,0,0,0,0,0,0,0};
UINT8	ModeOfOperationDisplay_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16 	ActualTor_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32 	ActualPos_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32 	ActualVel_data[NUM_AXIS] =  {0,0,0,0,0,0,0,0};
INT16	TorqueDemandValue_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	AnalogInput1_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	DigitalInputs_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32	ManuspecMachineHWPositionExternalcommand_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32	FollowingErrorActualValue_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};

UINT32	DataIn_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};

UINT8	DF1_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF2_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF3_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF4_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF5_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF6_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF7_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF8_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF9_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF10_data[NUM_AXIS] ={0,0,0,0,0,0,0,0};
UINT8	DF11_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF12_data[NUM_AXIS] ={0,0,0,0,0,0,0,0};
UINT8	DF13_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF14_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF15_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	DF16_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawFx_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawFy_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawFz_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawTx_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawTy_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	RawTz_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT8	OverloadStatus_data[NUM_AXIS] =  {0,0,0,0,0,0,0,0};
UINT8	ErrorFlag_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};


INT32 	TargetPos_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT32 	TargetVel_data[NUM_AXIS] ={0,0,0,0,0,0,0,0};
INT16 	TargetTor_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32 	DataOut_data[NUM_AXIS] ={0,0,0,0,0,0,0,0};
UINT8 	ModeOfOperation_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	ConfigParam1_data[NUM_AXIS] = {0,0,0,0,0,0,0,0}; //torque sensor -> 11 sensing start
UINT32	ConfigParam2_data[NUM_AXIS] = {0,0,0,0,0,0,0,0}; //torque sensor
UINT16	ControlWord_data[NUM_AXIS] ={0,0,0,0,0,0,0,0};
UINT32	ProfileVelocity_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
UINT32	DigitalOutpus_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};
INT16	TorqueOffset_data[NUM_AXIS] = {0,0,0,0,0,0,0,0};

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





/*****************************************************************************
 * Realtime task
 ****************************************************************************/

/*********************************AIDIN**************************************/
// void sensoryfeedback_run(void* arg)
// {

// 	unsigned int cycle_counter = 0;
//         RTIME now, previous;

// 	ethercatMaster.syncEthercatMaster();

// 	ros::NodeHandle nh; 

//        ros::Publisher pub_angle = nh.advertise<aidinvi::twel>("aidinvi_angle", 100);
//        ros::Publisher pub_vel = nh.advertise<aidinvi::twel>("aidinvi_vel", 100);
//        ros::Publisher pub_torques = nh.advertise<aidinvi::twel>("aidinvi_torques", 100);

// 	rt_task_set_periodic(NULL, TM_NOW, (int)ms(1));

// 	previous = rt_timer_read();

// 	while (run) {
// 		rt_task_wait_period(NULL);

// 		cycle_counter++;

// 		// receive EtherCAT frames
// 		if (!run)
// 		{
// 			break;
// 		}
// 		previous = now;
//         now = rt_timer_read();

// 		if (!(cycle_counter % 1000)) {
// 		}

//         ethercatMaster.processTxDomain();

//         ethercatMaster.readBuffer(STATUS_WORD, StatusWord);
// 		ethercatMaster.readBuffer(POSITION_ACTUAL_VALUE, ActualPos);
// 		ethercatMaster.readBuffer(VELOCITY_ACTUAL_VALUE, ActualVel);
// 		ethercatMaster.readBuffer(TORQUE_ACTUAL_VALUE, ActualTor);
// 		ethercatMaster.readBuffer(MODE_OF_OPERATION_DISPLAY, ModeOfOperationDisplay);
// 		ethercatMaster.readBuffer(POSITION_FOLLOWING_ERROR_ACTUAL_VALUE, PositionFollowingErrorActualValue);
// 		ethercatMaster.readBuffer(DC_LINK_CIRCUIT_VOLTAGE, DCLinkCircuitVoltage);
// 		ethercatMaster.readBuffer(DIGITAL_INPUTS, DigitalInputs);
// 		ethercatMaster.readBuffer(AUXILIARY_POSITION_ACTUAL_VALUE, AuxiliaryPositionActualValue);
// 		ethercatMaster.readBuffer(CURRENT_ACTUAL_VALUE, CurrentActualValue);

//         ethercatMaster.readBuffer(INPUT_BYTE0000, InputByte0000);
// 		ethercatMaster.readBuffer(INPUT_BYTE0001, InputByte0001);
// 		ethercatMaster.readBuffer(INPUT_BYTE0002, InputByte0002);
// 		ethercatMaster.readBuffer(INPUT_BYTE0003, InputByte0003);
// 		ethercatMaster.readBuffer(INPUT_BYTE0004, InputByte0004);
// 		ethercatMaster.readBuffer(INPUT_BYTE0005, InputByte0005);
// 		ethercatMaster.readBuffer(INPUT_BYTE0006, InputByte0006);
// 		ethercatMaster.readBuffer(INPUT_BYTE0007, InputByte0007);
// 		ethercatMaster.readBuffer(INPUT_BYTE0008, InputByte0008);
// 		ethercatMaster.readBuffer(INPUT_BYTE0009, InputByte0009);
// 		ethercatMaster.readBuffer(INPUT_BYTE0010, InputByte0010);
// 		ethercatMaster.readBuffer(INPUT_BYTE0011, InputByte0011);
// 		ethercatMaster.readBuffer(INPUT_BYTE0012, InputByte0012);
// 		ethercatMaster.readBuffer(INPUT_BYTE0013, InputByte0013);
// 		ethercatMaster.readBuffer(INPUT_BYTE0014, InputByte0014);
// 		ethercatMaster.readBuffer(INPUT_BYTE0015, InputByte0015);
// 		ethercatMaster.readBuffer(INPUT_BYTE0016, InputByte0016);
// 		ethercatMaster.readBuffer(INPUT_BYTE0017, InputByte0017);
// 		ethercatMaster.readBuffer(INPUT_BYTE0018, InputByte0018);
// 		ethercatMaster.readBuffer(INPUT_BYTE0019, InputByte0019);
// 		ethercatMaster.readBuffer(INPUT_BYTE0020, InputByte0020);
// 		ethercatMaster.readBuffer(INPUT_BYTE0021, InputByte0021);
// 		ethercatMaster.readBuffer(INPUT_BYTE0022, InputByte0022);
// 		ethercatMaster.readBuffer(INPUT_BYTE0023, InputByte0023);
// 		ethercatMaster.readBuffer(INPUT_BYTE0024, InputByte0024);
// 		ethercatMaster.readBuffer(INPUT_BYTE0025, InputByte0025);
// 		ethercatMaster.readBuffer(INPUT_BYTE0026, InputByte0026);
// 		ethercatMaster.readBuffer(INPUT_BYTE0027, InputByte0027);

//         compute();

//         // 변수 처리
//         curr1 = CurrentActualValue[0];
//         curr2 = CurrentActualValue[1];
//         curr3 = CurrentActualValue[2];
//         curr4 = CurrentActualValue[3];
//         curr5 = CurrentActualValue[4];
//         curr6 = CurrentActualValue[5];
//         curr7 = CurrentActualValue[6];
//         curr8 = CurrentActualValue[7];
//         curr9 = CurrentActualValue[8];
//         curr10 = CurrentActualValue[9];
//         curr11 = CurrentActualValue[10];
//         curr12 = CurrentActualValue[11];
                
//         joint1p = (double)(qIncRad[0] + init1p)*180/PI;
//         joint2p = (double)(qIncRad[1] + init2p)*180/PI;
//         joint3p = (double)(qIncRad[2] + init3p)*180/PI;
//         joint4p = (double)(qIncRad[3] + init4p)*180/PI;
//         joint5p = (double)(qIncRad[4] + init5p)*180/PI;
//         joint6p = (double)(qIncRad[5] + init6p)*180/PI;
//         joint7p = (double)(qIncRad[6] + init7p)*180/PI;
//         joint8p = (double)(qIncRad[7] + init8p)*180/PI;
//         joint9p = (double)(qIncRad[8] + init9p)*180/PI;
//         joint10p = (double)(qIncRad[9] + init10p)*180/PI;
//         joint11p = (double)(qIncRad[10] + init11p)*180/PI;
//         joint12p = (double)(qIncRad[11] + init12p)*180/PI;

//         joint1v = (double)qdotInc[0];
//         joint2v = (double)qdotInc[1];
//         joint3v = (double)qdotInc[2];
//         joint4v = (double)qdotInc[3];
//         joint5v = (double)qdotInc[4];
//         joint6v = (double)qdotInc[5];
//         joint7v = (double)qdotInc[6];
//         joint8v =  (double)qdotInc[7];
//         joint9v = (double)qdotInc[8];
//         joint10v = (double)qdotInc[9];
//         joint11v = (double)qdotInc[10];
//         joint12v = (double)qdotInc[11];

//         torques1 = -(((double)InputByte0004[0]*256+(double)InputByte0005[0])/100 - 300);
//         torques2 = (((double)InputByte0006[0]*256+(double)InputByte0007[0])/100 - 300);
//         torques3 = -(((double)InputByte0008[0]*256+(double)InputByte0009[0])/100 - 300);
//         torques4 = -(((double)InputByte0010[0]*256+(double)InputByte0011[0])/100 - 300);
//         torques5 = -(((double)InputByte0012[0]*256+(double)InputByte0013[0])/100 - 300);
//         torques6 = (((double)InputByte0014[0]*256+(double)InputByte0015[0])/100 - 300);
//         torques7 = (((double)InputByte0016[0]*256+(double)InputByte0017[0])/100 - 300);
//         torques8 = (((double)InputByte0018[0]*256+(double)InputByte0019[0])/100 - 300);
//         torques9 = -(((double)InputByte0020[0]*256+(double)InputByte0021[0])/100 - 300);
//         torques10 = (((double)InputByte0022[0]*256+(double)InputByte0023[0])/100 - 300);
//         torques11 = -(((double)InputByte0024[0]*256+(double)InputByte0025[0])/100 - 300 );
//         torques12 = (((double)InputByte0026[0]*256+(double)InputByte0027[0])/100 - 300 );

//         joint1t = torques1 - init1t;// + 7.52;
//         joint2t = torques2 - init2t;// + 6.34;
//         joint3t = torques3 - init3t;// - 48.93;

//         joint4t = torques4 - init4t;// - 3.02;
//         joint5t = torques5 - init5t;// +151.95;
//         joint6t = torques6 - init6t;// + 1.37;

//         joint7t = torques7 - init7t;// + 5.92;
//         joint8t = torques8 - init8t;// - 9.01;
//         joint9t = torques9 - init9t;// - 7.65;

//         joint10t = torques10 - init10t;// - 22.43;
//         joint11t = torques11 - init11t;// - 11.51;
//         joint12t = torques12 - init12t;// - 6.28;

//         volt = DCLinkCircuitVoltage[0];

// 	    aidinvi::twel msg; 

// 	    msg.a = (joint1p);     
// 	    msg.b = (joint2p); 
// 	    msg.c = (joint3p);     
// 	    msg.d = (joint4p);  
// 	    msg.e = (joint5p);    
// 	    msg.f = (joint6p);  
// 	    msg.g = (joint7p);     
// 	    msg.h = (joint8p);  
// 	    msg.i = (joint9p); 
// 	    msg.j = (joint10p); 
// 	    msg.k = (joint11p); 
// 	    msg.l = (joint12p); 
    
// 	    pub_angle.publish(msg);

// 	    aidinvi::twel msga;

// 	    msga.a = (joint1v);     
// 	    msga.b = (joint2v); 
// 	    msga.c = (joint3v);     
// 	    msga.d = (joint4v);  
// 	    msga.e = (joint5v);     
// 	    msga.f = (joint6v);  
// 	    msga.g = (joint7v);     
// 	    msga.h = (joint8v);  
// 	    msga.i = (joint9v); 
// 	    msga.j = (joint10v); 
// 	    msga.k = (joint11v); 
// 	    msga.l = (joint12v); 
    
// 	    pub_vel.publish(msga);

// 	    aidinvi::twel msz;

// 	    msz.a = (joint1t);     
// 	    msz.b = (joint2t); 
// 	    msz.c = (joint3t);     
// 	    msz.d = (joint4t);     
// 	    msz.e = (joint5t); 
// 	    msz.f = (joint6t);     
// 	    msz.g = (joint7t);     
// 	    msz.h = (joint8t); 
// 	    msz.i = (joint9t);     
// 	    msz.j = (joint10t);     
// 	    msz.k = (joint11t); 
// 	    msz.l = (joint12t);     

// 	    pub_torques.publish(msz);

// 		// ethercatMaster.writeBuffer(TARGET_POSITION, TargetPos);
// 		// ethercatMaster.writeBuffer(TARGET_VELOCITY, TargetVel);
// 		ethercatMaster.writeBuffer(TARGET_TORQUE, TargetTor);
// 		// ethercatMaster.writeBuffer(MODE_OF_OPERATION, ModeOfOperation);

//         ethercatMaster.processRxDomain();

//          ros::spinOnce();

// 		ethercat_time = (long) now - previous;
// 		latency = ethercat_time - cycle_ns;

// 	}
// }

// void print_run(void *arg)
// {
// 	RTIME now, previous=0;
// 	unsigned long itime=0, step;
// 	long stick=0;
// 	int count=0;
// 	unsigned int NumSlaves=0, masterState=0, slaveState=0;

// 	rt_task_set_periodic(NULL, TM_NOW, cycle_ns*100);
	
// 	while (1)
// 	{
// 		if (++count==10)
// 		{
// 			++stick;
// 			count=0;
// 		}
// 		if (1)
// 		{
// 			now = rt_timer_read();  
// 			step=(unsigned long)(now - previous) / 1000000;
// 			itime+=step;
// 			previous=now;

// 				rt_printf("\e[32;1m\t input: %d,  \e[0m\n", input);
// 				rt_printf("\e[32;1m\t Voltage: %f, curr: %f\n", volt/1000, curr);
// 				//rt_printf("\e[32;1m\t contact: %d %d %d %d  \e[0m\n", LFc, RFc, LBc, RBc);
// 				rt_printf("//////////////LF////////////\n");
// 				rt_printf("\e[32;1m\t A 1: %i 2: %i 3: %i  \e[0m\n", qDirection[0] * AuxiliaryPositionActualValue[0], qDirection[1] * AuxiliaryPositionActualValue[1], qDirection[2] * AuxiliaryPositionActualValue[2]);
// 				rt_printf("\e[32;1m\t qAbs 1: %i 2: %i 3: %i,  \e[0m\n", qAbsCnt[0], qAbsCnt[1], qAbsCnt[2]);
// 				rt_printf("\e[32;1m\t Current 1: %i 2: %i 3: %i,  \e[0m\n", CurrentActualValue[0], CurrentActualValue[1], CurrentActualValue[2]);
// 				rt_printf("\e[32;1m\t TS 1: %f 2: %f 3: %f  \e[0m\n", joint1t, joint2t, joint3t);		
// 				rt_printf("\e[32;1m\t D 1: %f 2: %f 3: %f  \e[0m\n", joint1p, joint2p, joint3p);
// 				rt_printf("\e[32;1m\t T 1: %f 2: %f 3: %f  \e[0m\n", torque1, torque2, torque3);

// 				rt_printf("//////////////RF////////////\n");
// 				rt_printf("\e[32;1m\t A 4: %i 5: %i 6: %i  \e[0m\n", qDirection[3] * AuxiliaryPositionActualValue[3], qDirection[4] * AuxiliaryPositionActualValue[4], qDirection[5] * AuxiliaryPositionActualValue[5]);
// 				rt_printf("\e[32;1m\t qAbs 4: %i 5: %i 6: %i,  \e[0m\n", qAbsCnt[3], qAbsCnt[4], qAbsCnt[5]);
// 				rt_printf("\e[32;1m\t Current 4: %i 5: %i 6: %i,  \e[0m\n", CurrentActualValue[3], CurrentActualValue[4], CurrentActualValue[5]);
// 				rt_printf("\e[32;1m\t TS 4: %f 5: %f 6: %f  \e[0m\n", joint4t, joint5t, joint6t);
// 				rt_printf("\e[32;1m\t D 4: %f 5: %f 6: %f  \e[0m\n", joint4p, joint5p, joint6p);
// 				rt_printf("\e[32;1m\t T 4: %f 5: %f 6: %f  \e[0m\n", torque4, torque5, torque6);
				
// 				rt_printf("//////////////LB////////////\n");
// 				rt_printf("\e[32;1m\t A 7: %i 8: %i 9: %i  \e[0m\n", qDirection[6] * AuxiliaryPositionActualValue[6], qDirection[7] * AuxiliaryPositionActualValue[7], qDirection[8] * AuxiliaryPositionActualValue[8]);
// 				rt_printf("\e[32;1m\t qAbs 7: %i 8: %i 9: %i,  \e[0m\n", qAbsCnt[6], qAbsCnt[7], qAbsCnt[8]);
// 				rt_printf("\e[32;1m\t Current 7: %i 8: %i 9: %i,  \e[0m\n", CurrentActualValue[6], CurrentActualValue[7], CurrentActualValue[8]);
// 				rt_printf("\e[32;1m\t TS 7: %f 8: %f 9: %f  \e[0m\n", joint7t, joint8t, joint9t);
// 				rt_printf("\e[32;1m\t D 7: %f 8: %f 9: %f  \e[0m\n", joint7p, joint8p, joint9p);
// 				rt_printf("\e[32;1m\t T 7: %f 8: %f 9: %f  \e[0m\n", torque7, torque8, torque9);

// 				rt_printf("//////////////RB////////////\n");
// 				rt_printf("\e[32;1m\t A 10: %i 11: %i 12: %i  \e[0m\n", qDirection[9] * AuxiliaryPositionActualValue[9], qDirection[10] * AuxiliaryPositionActualValue[10], qDirection[11] * AuxiliaryPositionActualValue[11]);
// 				rt_printf("\e[32;1m\t qAbs 10: %i 11: %i 12: %i,  \e[0m\n", qAbsCnt[9], qAbsCnt[10], qAbsCnt[11]);
// 				rt_printf("\e[32;1m\t Current 10: %i 11: %i 12: %i,  \e[0m\n", CurrentActualValue[9], CurrentActualValue[10], CurrentActualValue[11]);
				
// 				rt_printf("\e[32;1m\t TS 10: %f 11: %f 12: %f  \e[0m\n",joint10t, joint11t, joint12t);
// 				rt_printf("\e[32;1m\t D 10: %f 11: %f 12: %f  \e[0m\n", joint10p, joint11p, joint12p);
// 				rt_printf("\e[32;1m\t T 10: %f 11: %f 12: %f  \e[0m\n", torque10, torque11, torque12);
            
// 			rt_printf("\n");
// 		}
// 		else
// 		{
// 			if (count==0){
// 				rt_printf("%i", stick);
// 				for(i=0; i<stick; ++i)
// 					rt_printf(".");
// 				rt_printf("\n");
// 			}
// 		}
// 		rt_task_wait_period(NULL); //wait for next cycle
// 	}
// }

// void input_run(void *arg)
// {

// 	rt_task_set_periodic(NULL, TM_NOW, (int)ms(100));	// period = 1 (msec)

// 	while (1)
// 	{

// if(input == 1){
// init1p = (double)qAbsRad[0];
// init2p = (double)qAbsRad[1];
// init3p = (double)qAbsRad[2];
// init4p = (double)qAbsRad[3];
// init5p = (double)qAbsRad[4];
// init6p = (double)qAbsRad[5];
// init7p = (double)qAbsRad[6];
// init8p = (double)qAbsRad[7];
// init9p = (double)qAbsRad[8];
// init10p = (double)qAbsRad[9];
// init11p = (double)qAbsRad[10];
// init12p = (double)qAbsRad[11];
// }
// else if(input == 2){
// init1t = torques1;
// init2t = torques2;
// init3t =torques3;

// init4t = torques4;
// init5t = torques5;
// init6t = torques6;

// init7t = torques7;
// init8t= torques8;
// init9t= torques9;

// init10t= torques10;
// init11t= torques11;
// init12t= torques12;
// }
// else
// {}
// 	rt_task_wait_period(NULL);
// 	}
// }

// /****************************************************************************/
// void signal_handler(int signum = 0)
// {
// 	rt_task_delete(&sensoryfeedback_task);
// 	rt_task_delete(&print_task);
// 	rt_task_delete(&input_task);

//   	exit(1);
// }
// /****************************************************************************/







//********AUTO COMPENSATE GRAVITY and MEASURE F/T SENSOR'S DATA*****************
int compute()
{
	
	// For Command Control	-> ?????? getting feed back???
	for (int i=0; i<NUM_AXIS; ++i) // numaxis -> torque sensor included
	{
		// Axis[i].setCurrentPosInCnt(ActualPos[i]);
		// Axis[i].setCurrentVelInCnt(ActualVel[i]);
		// Axis[i].setCurrentTorInCnt(ActualTor[i]);
		// Axis[i].setDataIn(DataIn[i]);

		// Axis[i].setCurrentTime(gt);

		if(i==6) //Only 7 joint gear ratio : 120
		{
			enctodeg[i] = ActualPos_data[i]/2796202.6666; // link output deg /(pow(2,23)*120)*360
			// vel_enctodeg[i] = ActualVel[i]/2796202.6666; // link output deg/s
		}

		else if(i==0) //Only 7 joint gear ratio : 161
		{
			enctodeg[i] = ActualPos_data[i]/3751571.9111; // link output deg /(pow(2,23)*161)*360
			// vel_enctodeg[i] = ActualVel[i]/3751571.9111; // link output deg/s
		}

		else{
			enctodeg[i] = ActualPos_data[i]/3728270.2222; // link output deg /(pow(2,23)*160)*360
			// vel_enctodeg[i] = ActualVel[i]/3728270.2222; // link output deg/s
			//vel_enctodeg[i] = (enctodeg[i]-prev_enctodeg[i])/dt; // link output deg/s
		}
		disp[i] =(enctodeg[i]-prev_enctodeg[i]);
		vel_enctodeg[i] =(enctodeg[i]-prev_enctodeg[i])/dt;
		prev_enctodeg[i]=enctodeg[i];

		//vel_enctodeg[i]=ActualVel[i]/(pow(2,23)*160); // link output deg/s
	}

	//force sensing
	FTsen[0] =(double)RawFx_data[7]/df; // force x
	FTsen[1] = (double)RawFy_data[7]/df; // force y
	FTsen[2] = (double)RawFz_data[7]/df; // force z
	FTsen[3] =(double)RawTx_data[7]/dm; // moment x
	FTsen[4] = (double)RawTy_data[7]/dm; // moment y
	FTsen[5] = (double)RawTz_data[7]/dm; // moment z

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
	TargetPos_data[i] = (int) cal_position[i];
  }


	return 0;
}



/**********************************DOWNSCALE************************************/
// Servotronix_motion_control_ltd__CDHD_task
void Servotronix_CDHD__run(void *arg)
{
	int s_mode=0;
	unsigned int runcount=0;
	RTIME now, previous;

	// Synchronize EtherCAT Master (for Distributed Clock Mode)
	ethercatMaster.syncEthercatMaster();


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

		/// TO DO: read data from sensors in EtherCAT system interface
		ethercatMaster.processTxDomain();

		ethercatMaster.readBuffer(STATUS_WORD, StatusWord_data);
		ethercatMaster.readBuffer(MODE_OF_OPERATION_DISPLAY, ModeOfOperationDisplay_data);
		ethercatMaster.readBuffer(TORQUE_ACTUAL_VALUE, ActualTor_data);
		ethercatMaster.readBuffer(POSITION_ACTUAL_VALUE, ActualPos_data);
		ethercatMaster.readBuffer(TORQUE_DEMAND_VALUE, TorqueDemandValue_data);
		ethercatMaster.readBuffer(ANALOG_INPUT1, AnalogInput1_data);
		ethercatMaster.readBuffer(DIGITAL_INPUTS, DigitalInputs_data);
		ethercatMaster.readBuffer(MANUSPECS_MACHINE_HW_POSITION_EXTERNAL_COMMAND, ManuspecMachineHWPositionExternalcommand_data);
		ethercatMaster.readBuffer(POSITION_FOLLOWING_ERROR_ACTUAL_VALUE, FollowingErrorActualValue_data);
		

		// ethercatMaster.readBuffer(INPUT_DF1, DF1_data);
		// ethercatMaster.readBuffer(INPUT_DF2, DF2_data);
		// ethercatMaster.readBuffer(INPUT_DF3, DF3_data);
		// ethercatMaster.readBuffer(INPUT_DF4, DF4_data);
		// ethercatMaster.readBuffer(INPUT_DF5, DF5_data);
		// ethercatMaster.readBuffer(INPUT_DF6, DF6_data);
		// ethercatMaster.readBuffer(INPUT_DF7, DF7_data);
		// ethercatMaster.readBuffer(INPUT_DF8, DF8_data);
		// ethercatMaster.readBuffer(INPUT_DF9, DF9_data);
		// ethercatMaster.readBuffer(INPUT_DF10, DF10_data);
		// ethercatMaster.readBuffer(INPUT_DF11, DF11_data);
		// ethercatMaster.readBuffer(INPUT_DF12, DF12_data);
		// ethercatMaster.readBuffer(INPUT_DF13, DF13_data);
		// ethercatMaster.readBuffer(INPUT_DF14, DF14_data);
		// ethercatMaster.readBuffer(INPUT_DF15, DF15_data);
		// ethercatMaster.readBuffer(INPUT_DF16, DF16_data);
		// ethercatMaster.readBuffer(INPUT_RAWFX, RawFx_data);
		// ethercatMaster.readBuffer(INPUT_RAWFY, RawFy_data);
		// ethercatMaster.readBuffer(INPUT_RAWFZ, RawFz_data);
		// ethercatMaster.readBuffer(INPUT_RAWTX, RawTx_data);
		// ethercatMaster.readBuffer(INPUT_RAWTY, RawTy_data);
		// ethercatMaster.readBuffer(INPUT_RAWTZ, RawTz_data);
		// ethercatMaster.readBuffer(INPUT_OVER_LOAD_STATUS, OverloadStatus_data);
		// ethercatMaster.readBuffer(INPUT_ERROR_FLAG, ErrorFlag_data);

		/// TO DO: Main computation routine...
		compute(); // main function

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

		/// TO DO: write data to actuators in EtherCAT system interface

		ethercatMaster.writeBuffer(TARGET_POSITION, TargetPos_data);
		ethercatMaster.processRxDomain();

		ros::spinOnce();

		// saveLogData();

		// For EtherCAT performance statistics
		now = rt_timer_read();
		ethercat_time = (long) now - previous;

		// if (( (runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)) ) )//(runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns))-=>for one joint //(_systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.isSystemReady() && (runcount>WAKEUP_TIME*(NSEC_PER_SEC/cycle_ns)) )
		// {
		// 	system_ready=1;	//all drives have been done

		// 	gt+= period;

		// 	if (worst_time<ethercat_time) worst_time=ethercat_time;
		// 	if(ethercat_time > max_time)
		// 		++fault_count;
		// }

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

	// rt_printf("\e[31;1m \nPlease WAIT at least %i (s) until the system getting ready...\e[0m\n", WAKEUP_TIME);

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

		now = rt_timer_read();
		step=(unsigned long)(now - previous) / 1000000;
		itime+=step;
		previous=now;
		rt_printf("Time=%d.%d s, ", itime/1000, itime % 1000);
		rt_printf("dt= %li, worst= %li\n", ethercat_time, worst_time);


		if (ethercatMaster.getMasterStatus(NumSlaves, masterState))
		rt_printf("Master: Online - State %i - %i slave(s)\n", masterState, NumSlaves);
		else
		rt_printf("Master: Offline\n");

		if (ethercatMaster.getRxDomainStatus())
		rt_printf("RxDomain: Online\n");
		else
		rt_printf("RxDomain: Offline\n");

		if (ethercatMaster.getTxDomainStatus())
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


		//rt_printf("\e[32;1m\t StatusWord: 0x%x,  \e[0m\n",		StatusWord[i]);
		//rt_printf("\e[32;1m\t ModeOfOpDisp: 0x%x,  \e[0m\n",	ModeOfOperationDisplay[i]);
		rt_printf("\e[32;1m\t ************************************************************  \e[0m\n");
		rt_printf("\e[32;1m\t    joint[1]]		  joint[2]	       joint[3]	          joint[4]	        joint[5]	      joint[6]	        joint[7]  \e[0m\n");
		rt_printf("\e[32;1m\t StatusWor[1]: %i, StatusWor[2]: %i, StatusWor[3]: %i, StatusWor[4]: %i, StatusWor[5]: %i, StatusWor[6]: %i, StatusWor[7]: %i \e[0m\n", StatusWord_data[0], StatusWord_data[1], StatusWord_data[2], StatusWord_data[3], StatusWord_data[4], StatusWord_data[5], StatusWord_data[6]);
		rt_printf("\e[32;1m\t ContrsWor[1]: %i, ContrsWor[2]: %i, ContrsWor[3]: %i, ContrsWor[4]: %i, ContrsWor[5]: %i, ContrsWor[6]: %i, ContrsWor[7]: %i \e[0m\n", ControlWord_data[0], ControlWord_data[1], ControlWord_data[2], ControlWord_data[3], ControlWord_data[4], ControlWord_data[5], ControlWord_data[6]);
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
		rt_printf("\e[32;1m\t errorFlags: %i,  \e[0m\n", 	 	ErrorFlag_data[7]);
		rt_printf("\e[32;1m\t time taken: %.5f,  \e[0m\n", 	 	time_taken);
		//rt_printf("\e[32;1m\t Digiinput[1]: %i, Digiinput[2]: %i, Digiinput[3]: %i, Digiinput[4]: %i, Digiinput[5]: %i, Digiinput[6]: %i, Digiinput[7]: %i \e[0m\n", DigitalInputs[0], DigitalInputs[1], DigitalInputs[2], DigitalInputs[3], DigitalInputs[4], DigitalInputs[5], DigitalInputs[6]);
		//rt_printf("\e[32;1m\t Statuswod[1]: %i, Statuswod[2]: %i, Statuswod[3]: %i, Statuswod[4]: %i, Statuswod[5]: %i, Statuswod[6]: %i, Statuswod[7]: %i \e[0m\n", StatusWord[0], StatusWord[1], StatusWord[2], StatusWord[3], StatusWord[4], StatusWord[5], StatusWord[6]);


		rt_printf("\n");

		rt_task_wait_period(NULL); //wait for next cycle
	}
}


/****************************************************************************/
void signal_handler(int signum = 0)
{
	//rt_task_delete(&plot_task);
	//rt_task_delete(&gui_task);
	ConfigParam1_data[7]=12;
	//	ControlWord[0]=6; ControlWord[1]=6; ControlWord[2]=6; ControlWord[3]=6;ControlWord[4]=6; ControlWord[5]=6;ControlWord[6]=6;
	printf("Servo drives Stopped 5second!\n");


	ethercatMaster.writeBuffer(CONFIG_PARAM1, ConfigParam1_data); //torque sensor setting value
	ControlWord_data[0]=14; ControlWord_data[1]=14; ControlWord_data[2]=14; ControlWord_data[3]=14; ControlWord_data[4]=14; ControlWord_data[5]=14; ControlWord_data[6]=14;
	ethercatMaster.writeBuffer(CONTROL_WORD, ControlWord_data);

	ethercatMaster.processTxDomain();
	ethercatMaster.processRxDomain();



	//offAxes();

	//runcount>5*(NSEC_PER_SEC/cycle_ns);
	rt_task_delete(&Servotronix_motion_control_ltd__CDHD_task);
	rt_task_delete(&print_task);
    //usleep(500000);
    printf("Brake on!\n");
	//DigitalOutpus_data[0] = 1; DigitalOutpus_data[1] = 1; DigitalOutpus_data[2] = 1; DigitalOutpus_data[3] = 1; DigitalOutpus_data[4] = 1; DigitalOutpus_data[5] = 1; DigitalOutpus_data[6] = 1;

	printf("Servo drives Stopped!\n");
    // _systemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.deinit();
    exit(1);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sub_downscale");

	ros::NodeHandle nh;

	//ros::Subscriber sub_jointt = nh.subscribe("downscale_cal_torque", 100, msgCallbackt);
	ros::Subscriber sub_jointp = nh.subscribe("downscale_cal_position", 100, msgCallbackp);

    rt_print_auto_init(1);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mlockall(MCL_CURRENT | MCL_FUTURE);


	// // Set the demo mode for EtherCAT application
	// demo_mode = DEMO_MODE_POSITION;

	// if (demo_mode == DEMO_MODE_TORQUE)
	// {
	// 	// For CST (cyclic synchronous torque) control
	// 	if(ethercatMaster.init(OP_MODE_CYCLIC_SYNC_TORQUE) == -1)
	// 	{
	// 		printf("System Initialization Failed\n");
	// 		return 0;
	// 	}
	// }
	// else if (demo_mode == DEMO_MODE_POSITION)
	// {
	// 	// For CSP (cyclic synchronous position) control
	// 	if(ethercatMaster.init(OP_MODE_CYCLIC_SYNC_POSITION) == -1)
	// 	{
	// 		printf("System Initialization Failed\n");
	// 		return 0;
	// 	}
	// }
	// else // Default: cyclic synchronous position MODE
	// {
	// 	// For CSP (cyclic synchronous position) control
	// 	if(ethercatMaster.init(OP_MODE_CYCLIC_SYNC_POSITION) == -1)
	// 	{
	// 		printf("System Initialization Failed\n");
	// 		return 0;
	// 	}
	// }
	if(ethercatMaster.init(OP_MODE_CYCLIC_SYNC_POSITION) == -1)
    {
        printf("System Initialization Failed\n");
        return 0;
    }


	/*******Init Motor Breaks (ON/OFF)********/
	ethercatMaster.initAxes(); 
	ethercatMaster.processRxDomain();

	// FT SENSOR BIAS START
	// ConfigParam1_data[7]=11;
	// ethercatMaster.writeBuffer(CONFIG_PARAM1, ConfigParam1_data);
	// ethercatMaster.writeBuffer(CONFIG_PARAM2, ConfigParam2_data);
	// ethercatMaster.processRxDomain();

	// // FT SENSOR GETTING VALUE START
	// ConfigParam1_data[7]=273;
	// ethercatMaster.writeBuffer(CONFIG_PARAM1, ConfigParam1_data);
	// ethercatMaster.writeBuffer(CONFIG_PARAM2, ConfigParam2_data);
	// ethercatMaster.processRxDomain();

	/******aidin*****/
	// rt_task_create(&input_task, "input_task", 0, 50, 0);
	// rt_task_start(&input_task, &input_run, NULL);

	// rt_task_create(&sensoryfeedback_task, "sensoryfeedback", 0, 99, 0);
	// rt_task_start(&sensoryfeedback_task, &sensoryfeedback_run, NULL);

	// rt_task_create(&print_task, "printing", 0, 80, 0);
	// rt_task_start(&print_task, &print_run, NULL);

	/******downscale*****/
	rt_task_create(&Servotronix_motion_control_ltd__CDHD_task, "Servotronix_motion_control_ltd__CDHD_task", 0, 99, 0);
	rt_task_start(&Servotronix_motion_control_ltd__CDHD_task, &Servotronix_CDHD__run, NULL);

	// printing: create and start
	rt_task_create(&print_task, "printing", 0, 80, 0);
	rt_task_start(&print_task, &print_run, NULL);

	while (1)
	{



		//scanf("%d", &input);
		usleep(1e5);
	}

	// Finalize
	signal_handler();

    return 0;
}