/*
 * struct.h
 *
 *  Created on: 29 May 2015
 *      Author: Stuff
 */

#ifndef STRUCT_H_
#define STRUCT_H_

#ifndef FLAG
#define FLAG(x) unsigned int x :1;
#endif // FLAG(x)

/// MPPT
#define _MPPT_FLAGS 1
#define _MPPT_POWER 1
#define _MPPT_PEAKS 1

/// MTR CONTROLLER
#define _MC_ERR 1
#define _MC_LIM 0
#define _MC_PHASE 0
#define _MC_VECTORS 0
#define _MC_RAILS 0
#define _MC_TMP 0
#define _MC_AMPHRS 0
#define _MC_ODO 1
#define _MC_SLIP 0
#define _MC_VELOCITY 1		// 1 = KMH, 2 = RPM, 3 = BOTH
#define _MC_POWER 1
#define _MC_PEAKS 1

/// BMU
#define _BMU_SOC 2			// 1 = SOC, 2 = SOC%, 3 = BOTH
#define _BMU_BAL_SOC 0		// 1 = Balance AmpHrs, 2 = Balance as %, 3 = BOTH
#define _BMU_THRES 0
#define _BMU_CAP 0
#define _BMU_PRECHARGE 0
#define _BMU_CELL_V 1
#define _BMU_CMU_TMP 1
#define _BMU_BAL_THRES 0
#define _BMU_CMU_CNT 1
#define _BMU_VER 0
#define _BMU_FAN 0			// 1 = Fan0, 2 = Fan1, 3 = BOTH
#define _BMU_12V_CONSUM 0
#define _BMU_POWER 1
#define _BMU_PEAKS 1

typedef struct // MPPT
{
	/// Predetermined Values
	uint32_t CAN_ID;			// CAN Bus identifier

	/// From CAN Bus
	uint32_t VIn;				// Input Voltage
	uint32_t VOut;				// Output Voltage
	uint32_t IIn;				// Input Current
	uint32_t Tmp;				// Temperature in degrees
#if _MPPT_FLAGS
	FLAG(BVLR)					// Battery voltage level reached flag
	FLAG(OVT)					// Over Temperature Flag
	FLAG(NOC)					// No Battery Connected
	FLAG(UNDV)					// Vin Under-voltage Flag
#endif // _MPPT_FLAGS

	/// Calculated Values & Peaks
	unsigned int Connected :2;	// 2 bit field for 0-2 count range
#if _MPPT_POWER
	uint32_t Watts;				// Watts into MPPT
	float WattHrs;				// Watt Hours
#if _MPPT_PEAKS
	uint32_t MAX_Watts;			// Peak Watts into MPPT
#endif // _MPPT_PEAKS
#endif // _MPPT_POWER
#if _MPPT_PEAKS
	uint32_t MAX_VIn;			// Peak Input Voltage
	uint32_t MAX_VOut;			// Peak Output Voltage
	uint32_t MAX_IIn;			// Peak Input Current
	uint32_t MAX_Tmp;			// Peak Temperature in degrees
#endif // _MPPT_PEAKS
}MPPT;

typedef struct // fakeMPPT
{
	uint32_t DataA;
	uint32_t DataB;
}fakeMPPTFRAME;

#if _MC_VECTORS
typedef struct // VECTORS_MTRCONT
{
	float V_Real;
	float V_Imag;
	float I_Real;
	float I_Imag;
	float BEMF_Real;
	float BEMF_Imag;
}VECTORS_MTRCONT;
#endif // _MC_VECTORS

typedef struct // MOTORCONTROLLER
{
	/// Predetermined Values
	uint32_t CAN_ID;			// CAN Bus identifier

	/// From CAN Bus
	uint32_t SERIAL_NO;			// Motor Controller Serial
	float Bus_I;				// Bus Current
	float Bus_V;				// Bus Voltage
#if _MC_ERR
	uint16_t ERROR;				// Error Flags
#endif // _MC_ERR
#if _MC_LIM
	uint16_t LIMIT;				// Limit Flags
#endif // _MC_LIM
#if _MC_PHASE
	float PhaseC_I;				// Motor Phase C Current
	float PhaseB_I;				// Motor Phase B Current
#endif // _MC_PHASE
#if _MC_VECTORS
	VECTORS_MTRCONT *VECTORS;	// Motor Vectors
#endif // _MC_VECTORS
#if _MC_RAILS
	float Rail_15V;				// 15V Rail Actual Voltage
	float Rail_3300mV;			// 3.3V Rail Actual Voltage
	float Rail_1900mV;			// 1.9V Rail Actual Voltage
#endif // _MC_RAILS
#if _MC_TMP
	float Heatsink_Tmp;			// Heatsink Temperature
	float Motor_Tmp;			// Motor Temperature
	float Board_Tmp;			// Board Temperature
#if _MC_PEAKS // && _MC_TMP
	float MAX_Heathsink_Tmp;
	float MAX_Motor_Tmp;
	float MAX_Board_Tmp;
#endif // _MC_PEAKS
#endif // _MC_TMP
#if _MC_AMPHRS
	float DC_AmpHrs;			// DC Bus AmpHrs
#endif // _MC_AMPHRS
#if _MC_ODO
	float Odometer;				// Distance traveled since reset
#endif // _MC_ODO
#if _MC_SLIP
	float Slip_Speed;			// Motor Slip Speed (Hz)
#endif // _MC_SLIP
#if _MC_VELOCITY == 1
	float Velocity_KMH;
#elif _MC_VELOCITY == 2
	float Velocity_RPM;
#elif _MC_VELOCITY == 3
	float Velocity_KMH;
	float Velocity_RPM;
#endif // _MC_VELOCITY

	/// Calculated Values & Peaks
#if _MC_POWER
	float Watts;				// Bus_I * Bus_V
	float WattHrs;				// Calculated every 10mS
#if _MC_PEAKS // && _MC_POWER
	float MAX_Watts;
#endif // _MC_PEAKS
#endif // _MC_POWER
#if _MC_PEAKS
	float MAX_Bus_I;
	float MAX_Bus_V;
#endif // MC_PEAKS
}MOTORCONTROLLER;

struct // BMU
{
	/// Predetermined Values
	uint32_t CAN_ID;				// CAN Bus Identifier

	/// From CAN Bus
	uint32_t SERIAL_NO;				// BMU Serial Number
	uint32_t Battery_V;				// Battery Voltage
	int32_t Battery_I;				// Battery Output Current
	uint32_t Status;				// Status Flags
#if _BMU_SOC == 1
	float SOC;						// Battery State of Charge (0 = Full)
#elif _BMU_SOC == 2
	float SOC_PER;					// Battery State of Charge as % (1 = Full)
#elif _BMU_SOC ==3
	float SOC;
	float SOC_PER;
#endif // _BMU_SOC
#if _BMU_BAL_SOC == 1
	float BAL_SOC;					// Balance State of Charge. Ah supplied to pack since first cell began balancing
#elif _BMU_BAL_SOC == 2
	float BAL_SOC_PER;				// Balance State of Charge as %
#elif _BMU_BAL_SOC == 3
	float BAL_SOC;
	float BAL_SOC_PER;
#endif // _BMU_BAL_SOC
#if _BMU_THRES
	int16_t Charge_Cell_V_Err;		//
	int16_t Cell_Tmp_Margin;		//
	int16_t Discharge_Cell_V_Err;	//
#endif // _BMU_THRES
#if _BMU_CAP
	uint16_t Pack_Capacity;
#endif // _BMU_CAP
#if _BMU_PRECHARGE
	char Driver_Status;
	char Precharge_State;
	FLAG(Precharge_Time_Elapsed)
	uint8_t Precharge_Timer;
#endif // _BMU_PRECHARGE
#if _BMU_CELL_V
	uint16_t Min_Cell_V; 	// Minimum Cell Voltage
	uint16_t Max_Cell_V; 	// Maximum Cell Voltage
	uint8_t CMU_Min_V;		// CMU number with minimum cell voltage
	uint8_t CMU_Max_V;		// CMU number with maximum cell voltage
	uint8_t Cell_Min_V;		// Cell number with minimum cell voltage
	uint8_t Cell_Max_V;		// Cell number with maximum cell voltage
#endif // _BMU_CELL_V
#if _BMU_CMU_TMP
	uint16_t Min_Cell_Tmp;	// Minimum Cell Temperature
	uint16_t Max_Cell_Tmp;	// Maximum Cell Temperature
	uint8_t CMU_Min_Tmp;	// CMU number with minimum cell temperature
	uint8_t CMU_Max_Tmp;	// CMU number with maximum cell temperature
#endif // _BMU_CMU_TMP
#if _BMU_BAL_THRES
	uint16_t Bal_Thres_Rising;
	uint16_t Bal_Thres_Falling;
#endif // _BMU_BAL_THRES
#if _BMU_CMU_CNT
	uint8_t CMU_Count;
#endif // _BMU_CMU_CNT
#if _BMU_VER
	uint16_t BMU_FW_Ver;
	uint8_t BMU_HW_Ver;
	uint8_t BMU_Model_ID;
#endif // _BMU_VER
#if _BMU_FAN == 1
	uint16_t Fan0_Spd;
#elif _BMU_FAN == 2
	uint16_t Fan1_Spd;
#elif _BMU_FAN == 3
	uint16_t Fan0_Spd;
	uint16_t Fan1_Spd;
#endif // _BMU_FAN
#if _BMU_12V_CONSUM
	uint16_t Fan_Contactor_12V_mA;
	uint16_t CMU_12V_mA;
#endif // _BMU_12V_CONSUM

	/// Calculated & Peaks
#if _BMU_POWER
	uint32_t Watts;
	float WattHrs;
#if _BMU_PEAKS // && _BMU_POWER
	uint32_t MAX_Watts;
#endif // _BMU_PEAKS
#endif // _BMU_POWER
#if _BMU_PEAKS
	uint32_t MAX_Battery_V;
	int32_t MAX_Battery_I;
#endif // _BMU_PEKAS
}BMU;

struct // STATS
{
	int RAMP_SPEED;
	uint8_t IGNITION;
	float ODOMETER;
	float MAX_SPEED;
	float CRUISE_SPEED;
	FLAG(DRIVE_MODE)
	FLAG(BUZZER)
	FLAG(MPPT_POLL_COUNT)
	FLAG(SWOC_ACK)
	FLAG(HWOC_ACK)
	FLAG(COMMS)
	FLAG(CR_ACT)
	FLAG(CR_STS)
	unsigned int FAULT :2;
}STATS;

struct // DRIVE
{
	float Speed_RPM;
	float Current;
}DRIVE;



struct CLOCK_STRUCT
{
	uint8_t		T_mS;
	uint8_t		T_S;
	uint8_t		T_M;
	uint8_t		T_H;
	uint32_t	T_D;
}CLOCK;

#endif /* STRUCT_H_ */
