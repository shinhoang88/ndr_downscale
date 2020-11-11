//! \file SystemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD.h
//!
//! \brief Automatically generated header file for the EtherCAT system interface
//!
// This file is part of NRMKPlatform SDK, Windows-based development tool and SDK
// for Real-time Linux Embedded EtherCAT master controller (STEP)
//
// Copyright (C) 2013-2016 Neuromeka <http://www.neuromeka.com>

#ifndef SYSTEMINTERFACE_ETHERCAT_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_H_
#define SYSTEMINTERFACE_ETHERCAT_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_H_

#pragma once 

//EtherCAT Library ******************************************************************
#include "ecrt.h"

#include "CoE.h"

#include "PDOConfig.h"
#include <stdio.h>
#include <memory>

class SystemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD
{ 
	public:
		enum
		{
			NUM_ROBOTOUS_RFT_EC02_AXES = 1,
				NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES = 7,
		};
		
		enum
		{
			NUM_MOTION_AXIS = NUM_ROBOTOUS_RFT_EC02_AXES + NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES,
			MAX_TORQUE = 2000,
		};
								
		struct ROBOTOUS_RFT_EC02_IN
			{
				UINT32		ConfigParam1; 	// 0x7000
				UINT32		ConfigParam2; 	// 0x7000				
			};
			struct ROBOTOUS_RFT_EC02_OUT
			{
				UINT8		DF1; 	// 0x6000
				UINT8		DF2; 	// 0x6000
				UINT8		DF3; 	// 0x6000
				UINT8		DF4; 	// 0x6000
				UINT8		DF5; 	// 0x6000
				UINT8		DF6; 	// 0x6000
				UINT8		DF7; 	// 0x6000
				UINT8		DF8; 	// 0x6000
				UINT8		DF9; 	// 0x6000
				UINT8		DF10; 	// 0x6000
				UINT8		DF11; 	// 0x6000
				UINT8		DF12; 	// 0x6000
				UINT8		DF13; 	// 0x6000
				UINT8		DF14; 	// 0x6000
				UINT8		DF15; 	// 0x6000
				UINT8		DF16; 	// 0x6000
				INT16		RawFx; 	// 0x6000
				INT16		RawFy; 	// 0x6000
				INT16		RawFz; 	// 0x6000
				INT16		RawTx; 	// 0x6000
				INT16		RawTy; 	// 0x6000
				INT16		RawTz; 	// 0x6000
				UINT8		OverloadStatus; 	// 0x6000
				UINT8		ErrorFlag; 	// 0x6000				
			};
			struct ROBOTOUS_RFT_EC02
			{
				UINT32 Index;
				UINT32 Alias;
				UINT32 Position;
				SLAVE_CONFIG Config;
				
				ROBOTOUS_RFT_EC02_IN 	InParam;
				ROBOTOUS_RFT_EC02_OUT 	OutParam;
				
				UINT32 offConfigParam1;
				UINT32 offConfigParam2;
				UINT32 offDF1;
				UINT32 offDF2;
				UINT32 offDF3;
				UINT32 offDF4;
				UINT32 offDF5;
				UINT32 offDF6;
				UINT32 offDF7;
				UINT32 offDF8;
				UINT32 offDF9;
				UINT32 offDF10;
				UINT32 offDF11;
				UINT32 offDF12;
				UINT32 offDF13;
				UINT32 offDF14;
				UINT32 offDF15;
				UINT32 offDF16;
				UINT32 offRawFx;
				UINT32 offRawFy;
				UINT32 offRawFz;
				UINT32 offRawTx;
				UINT32 offRawTy;
				UINT32 offRawTz;
				UINT32 offOverloadStatus;
				UINT32 offErrorFlag;
				UINT32 bitoffConfigParam1;
				UINT32 bitoffConfigParam2;
				UINT32 bitoffDF1;
				UINT32 bitoffDF2;
				UINT32 bitoffDF3;
				UINT32 bitoffDF4;
				UINT32 bitoffDF5;
				UINT32 bitoffDF6;
				UINT32 bitoffDF7;
				UINT32 bitoffDF8;
				UINT32 bitoffDF9;
				UINT32 bitoffDF10;
				UINT32 bitoffDF11;
				UINT32 bitoffDF12;
				UINT32 bitoffDF13;
				UINT32 bitoffDF14;
				UINT32 bitoffDF15;
				UINT32 bitoffDF16;
				UINT32 bitoffRawFx;
				UINT32 bitoffRawFy;
				UINT32 bitoffRawFz;
				UINT32 bitoffRawTx;
				UINT32 bitoffRawTy;
				UINT32 bitoffRawTz;
				UINT32 bitoffOverloadStatus;
				UINT32 bitoffErrorFlag;								
				
			};

			struct SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_IN
			{
				UINT16		ControlWord; 	// 0x6040
				INT8		ModesOfOperation; 	// 0x6060
				INT32		TargetPosition; 	// 0x607a
				UINT32		ProfileVelocity; 	// 0x6081
				INT32		TargetVelocity; 	// 0x60ff
				INT16		TargetTorque; 	// 0x6071
				UINT32		DigitalOutpus; 	// 0x60fe
				INT16		TorqueOffset; 	// 0x60b2				
			};
			struct SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_OUT
			{
				UINT16		StatusWord; 	// 0x6041
				INT8		ModesOfOperationDisplay; 	// 0x6061
				INT16		TorqueActualValue; 	// 0x6077
				INT32		PositionActualValue; 	// 0x6064
				INT32		VelocityActualValue; 	// 0x606C
				INT16		TorqueDemandValue; 	// 0x6074
				INT16		AnalogInput1; 	// 0x20f2
				UINT32		DigitalInputs; 	// 0x60fd
				INT32		ManuspecMachineHWPositionExternalcommand; 	// 0x20b6
				INT32		FollowingErrorActualValue; 	// 0x60f4				
			};
			struct SERVOTRONIX_MOTION_CONTROL_LTD__CDHD
			{
				UINT32 Index;
				UINT32 Alias;
				UINT32 Position;
				SLAVE_CONFIG Config;
				
				SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_IN 	InParam;
				SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_OUT 	OutParam;
				
				UINT32 offControlWord;
				UINT32 offModesOfOperation;
				UINT32 offTargetPosition;
				UINT32 offProfileVelocity;
				UINT32 offTargetVelocity;
				UINT32 offTargetTorque;
				UINT32 offDigitalOutpus;
				UINT32 offTorqueOffset;
				UINT32 offStatusWord;
				UINT32 offModesOfOperationDisplay;
				UINT32 offTorqueActualValue;
				UINT32 offPositionActualValue;
				UINT32 offVelocityActualValue;
				UINT32 offTorqueDemandValue;
				UINT32 offAnalogInput1;
				UINT32 offDigitalInputs;
				UINT32 offManuspecMachineHWPositionExternalcommand;
				UINT32 offFollowingErrorActualValue;
				UINT32 bitoffControlWord;
				UINT32 bitoffModesOfOperation;
				UINT32 bitoffTargetPosition;
				UINT32 bitoffProfileVelocity;
				UINT32 bitoffTargetVelocity;
				UINT32 bitoffTargetTorque;
				UINT32 bitoffDigitalOutpus;
				UINT32 bitoffTorqueOffset;
				UINT32 bitoffStatusWord;
				UINT32 bitoffModesOfOperationDisplay;
				UINT32 bitoffTorqueActualValue;
				UINT32 bitoffPositionActualValue;
				UINT32 bitoffVelocityActualValue;
				UINT32 bitoffTorqueDemandValue;
				UINT32 bitoffAnalogInput1;
				UINT32 bitoffDigitalInputs;
				UINT32 bitoffManuspecMachineHWPositionExternalcommand;
				UINT32 bitoffFollowingErrorActualValue;								
				
			};
			

		typedef std::auto_ptr<struct ecat_variables> EcatSystemVars;
		
	public: 
		SystemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD();
		~SystemInterface_EtherCAT_Servotronix_motion_control_ltd__CDHD();
		
		int init(INT8 ModeOp, UINT32 CycleNano)
		{
			_setMasterCycle(CycleNano);
				
			// TODO: Initiate Axes' parameters here
			for (int i=0; i<NUM_ROBOTOUS_RFT_EC02_AXES; i++)
			{
				_systemReady[_robotous_RFT_EC02[i].Index]=0;
				_servoOn[_robotous_RFT_EC02[i].Index] = false;
				// TODO: Init params here. 														
			}

			for (int i=0; i<NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES; i++)
			{
				_systemReady[_servotronix_motion_control_ltd__CDHD[i].Index]=0;
				_servoOn[_servotronix_motion_control_ltd__CDHD[i].Index] = false;
				// TODO: Init params here. 
				_servotronix_motion_control_ltd__CDHD[i].InParam.ModesOfOperation = ModeOp;															
			}
			
			
			if (_initMaster() < 0)
			{
				printf("Init Master Failed.\n");
				return -1;
			}					

			if (_initSlaves() == -1)
			{
				printf("Init Slaves Failed.\n");
				deinit();
				return -1;
			}

			if (_initDomains() == -1)
			{
				printf("Init Domains Failed.\n");
				deinit();
				return -1;
			}

			if (_activateMaster() == -1)
			{
				deinit();
				return -1;
			}

			return 0;
		}

		int deinit();
	
		void processTxDomain();
		void processRxDomain();
		
		void readBuffer(int EntryID, void * const data);
		void writeBuffer(int EntryID, void * const data);
		
		void syncEcatMaster();
		
		void readSDO(int EntryID, void * const data);
		void writeSDO(int EntryID, void * const data);
		
		int getRxDomainStatus();
		int getTxDomainStatus();
		int getMasterStatus(unsigned int & NumSlaves, unsigned int & State);
		int getAxisEcatStatus(unsigned int AxisIdx, unsigned int & State);
		
		
		void setServoOn(unsigned int AxisIdx)
		{
			_servoOn[AxisIdx] = true;
		}
		void setServoOff(unsigned int AxisIdx)
		{
			_servoOn[AxisIdx] = false;
		}
		bool isServoOn(unsigned int AxisIdx)
		{
			return _servoOn[AxisIdx];
		}
	
		bool isSystemReady()
		{
			for (int i=0; i<NUM_MOTION_AXIS; ++i)
				if (!_systemReady[i])
					return false;

			return true;
		}
	
	private:
		void _setMasterCycle(UINT32 DCCycle);
		int	_initMaster();
		int _initSlaves();
		int _initDomains();		
		int _activateMaster();
		
	private: 
		EcatSystemVars _systemVars;
		
		bool _servoOn[NUM_MOTION_AXIS];
		unsigned int _systemReady[NUM_MOTION_AXIS];	
		
		/* EtherCAT Slaves */
		ROBOTOUS_RFT_EC02 _robotous_RFT_EC02[NUM_ROBOTOUS_RFT_EC02_AXES];

		SERVOTRONIX_MOTION_CONTROL_LTD__CDHD _servotronix_motion_control_ltd__CDHD[NUM_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_AXES];
						
}; 

#endif /* SYSTEMINTERFACE_ETHERCAT_SERVOTRONIX_MOTION_CONTROL_LTD__CDHD_H_ */
