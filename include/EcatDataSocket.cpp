/*
 * EcatDataSocket.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: Thach Do
 */

#include "EcatDataSocket.h"

/// TO DO: Fill in the number of data and the data channel index array
const unsigned int EcatDataSocket::_dataChannelArray[]
// The following code generates two (or three) channels charts (all of time chart style),
// where each chart plots the signals (as many as the joint dof).
//
// This is the example to plot the joint angles in the first channel, the joint velocities in the second channel,
// the joint torque (in controlled simulation) in the third channel for a seven dof manipulator.
	= {
		NRMK_SCOKET_PACKET_DATA_CH(0), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, NUM_JOINT, 0, 1, 2, 3, 4, 5, 6, 7, 

		NRMK_SCOKET_PACKET_DATA_CH(1), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, NUM_JOINT, 8, 9, 10, 11, 12, 13, 14, 15, 
		
	};

// TO DO: Set the actual size of the data channel format array defined above
const unsigned int EcatDataSocket::_sizeDataChannelArray = sizeof(_dataChannelArray)/sizeof(unsigned int);



