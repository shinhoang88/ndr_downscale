#include "PDOConfig.h" 
ec_pdo_entry_info_t Robotous_RFT_EC02_pdo_entries[] = 
{ 
	{0x7000,	1,	32},	/* ConfigParam1 */
	{0x7000,	2,	32},	/* ConfigParam2 */
	{0x6000,	1,	8},	/* DF1 */
	{0x6000,	2,	8},	/* DF2 */
	{0x6000,	3,	8},	/* DF3 */
	{0x6000,	4,	8},	/* DF4 */
	{0x6000,	5,	8},	/* DF5 */
	{0x6000,	6,	8},	/* DF6 */
	{0x6000,	7,	8},	/* DF7 */
	{0x6000,	8,	8},	/* DF8 */
	{0x6000,	9,	8},	/* DF9 */
	{0x6000,	10,	8},	/* DF10 */
	{0x6000,	11,	8},	/* DF11 */
	{0x6000,	12,	8},	/* DF12 */
	{0x6000,	13,	8},	/* DF13 */
	{0x6000,	14,	8},	/* DF14 */
	{0x6000,	15,	8},	/* DF15 */
	{0x6000,	16,	8},	/* DF16 */
	{0x6000,	17,	16},	/* RawFx */
	{0x6000,	18,	16},	/* RawFy */
	{0x6000,	19,	16},	/* RawFz */
	{0x6000,	20,	16},	/* RawTx */
	{0x6000,	21,	16},	/* RawTy */
	{0x6000,	22,	16},	/* RawTz */
	{0x6000,	23,	8},	/* OverloadStatus */
	{0x6000,	24,	8},	/* ErrorFlag */
};

ec_pdo_entry_info_t Servotronix_motion_control_ltd__CDHD_pdo_entries[] = 
{ 
	{0x6040,	0,	16},	/* ControlWord */
	{0x6060,	0,	8},	/* ModesOfOperation */
	{0x607a,	0,	32},	/* TargetPosition */
	{0x6081,	0,	32},	/* ProfileVelocity */
	{0x60ff,	0,	32},	/* TargetVelocity */
	{0x6071,	0,	16},	/* TargetTorque */
	{0x60fe,	1,	32},	/* DigitalOutpus */
	{0x60b2,	0,	16},	/* TorqueOffset */
	{0x6041,	0,	16},	/* StatusWord */
	{0x6061,	0,	8},	/* ModesOfOperationDisplay */
	{0x6077,	0,	16},	/* TorqueActualValue */
	{0x6064,	0,	32},	/* PositionActualValue */
	{0x606C,	0,	32},	/* VelocityActualValue */
	{0x6074,	0,	16},	/* TorqueDemandValue */
	{0x20f2,	0,	16},	/* AnalogInput1 */
	{0x60fd,	0,	32},	/* DigitalInputs */
	{0x20b6,	0,	32},	/* ManuspecMachineHWPositionExternalcommand */
	{0x60f4,	0,	32},	/* FollowingErrorActualValue */
};

