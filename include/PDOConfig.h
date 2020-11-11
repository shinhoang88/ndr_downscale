#pragma once 

// Ethercat Master---- 
#include "ecrt.h" 

// Vendor ID & Product Code 
#define Robotous 0x8EE 
#define Robotous_RFT_EC02 0x00000002 

extern ec_pdo_entry_info_t Robotous_RFT_EC02_pdo_entries[]; 

// Vendor ID & Product Code 
#define Servotronix 0x000002E1 
#define Servotronix_motion_control_ltd__CDHD 0x0 

extern ec_pdo_entry_info_t Servotronix_motion_control_ltd__CDHD_pdo_entries[]; 

