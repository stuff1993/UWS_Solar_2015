/*
 * struct.h
 *
 *  Created on: 29 May 2015
 *      Author: Stuart G
 */

#ifndef STRUCT_H_
#define STRUCT_H_

/// MPPT
#define _MPPT_POWER 1
#define _MPPT_PEAKS 1

/// MTR CONTROLLER
#define _MC_ERR 1
#define _MC_LIM 0
#define _MC_PHASE 0
#define _MC_VECTORS 0
#define _MC_RAILS 0
#define _MC_TMP 1
#define _MC_AMPHRS 0
#define _MC_ODO 1
#define _MC_SLIP 0
#define _MC_VELOCITY 1    // 1 = KMH, 2 = RPM, 3 = BOTH
#define _MC_POWER 1
#define _MC_PEAKS 1

/// BMU
#define _BMU_SOC 2        // 1 = SOC, 2 = SOC%, 3 = BOTH
#define _BMU_BAL_SOC 0    // 1 = Balance AmpHrs, 2 = Balance as %, 3 = BOTH
#define _BMU_THRES 0
#define _BMU_CAP 0
#define _BMU_PRECHARGE 0
#define _BMU_CELL_V 1
#define _BMU_CMU_TMP 1
#define _BMU_BAL_THRES 0
#define _BMU_CMU_CNT 0
#define _BMU_VER 0
#define _BMU_FAN 0        // 1 = Fan0, 2 = Fan1, 3 = BOTH
#define _BMU_12V_CONSUM 0
#define _BMU_POWER 1
#define _BMU_PEAKS 1

typedef struct MPPT_STRUCT
{
  /// From CAN Bus
  uint32_t v_in;        // Input Voltage
  uint32_t v_out;       // Output Voltage
  uint32_t i_in;        // Input Current
  uint32_t tmp;         // Temperature in degrees
  uint8_t flags;

#if _MPPT_POWER
  uint32_t watts;       // Watts into MPPT
  float watt_hrs;       // Watt Hours
#if _MPPT_PEAKS
  uint32_t max_watts;   // Peak watts into MPPT
#endif // _MPPT_PEAKS
#endif // _MPPT_POWER
#if _MPPT_PEAKS
  uint32_t max_v_in;     // Peak Input Voltage
  uint32_t max_v_out;    // Peak Output Voltage
  uint32_t max_i_in;     // Peak Input Current
  uint32_t max_tmp;     // Peak Temperature in degrees
#endif // _MPPT_PEAKS
}MPPT;

typedef struct MPPT_RELAY_STRUCT
{
  uint32_t data_a;
  uint32_t data_b;
}MPPT_RELAY;

#if _MC_VECTORS
typedef struct VECTORS_MTRCONT_STRUCT
{
  float v_real;
  float v_imag;
  float i_real;
  float i_imag;
  float bemf_real;
  float bemf_imag;
}VECTORS_MTRCONT;
#endif // _MC_VECTORS

typedef struct MOTORCONTROLLER_STRUCT
{
  /// From CAN Bus
  float bus_i;          // Bus Current
  float bus_v;          // Bus Voltage
#if _MC_ERR
  uint16_t error;       // Error Flags
#endif // _MC_ERR
#if _MC_LIM
  uint16_t limit;       // Limit Flags
#endif // _MC_LIM
#if _MC_PHASE
  float phase_c_i;       // Motor Phase C Current
  float phase_b_i;       // Motor Phase B Current
#endif // _MC_PHASE
#if _MC_VECTORS
  VECTORS_MTRCONT *vectors; // Motor Vectors
#endif // _MC_VECTORS
#if _MC_RAILS
  float rail_15v;       // 15V Rail Actual Voltage
  float rail_3300mv;    // 3.3V Rail Actual Voltage
  float rail_1900mv;    // 1.9V Rail Actual Voltage
#endif // _MC_RAILS
#if _MC_TMP
  float heatsink_tmp;   // Heatsink Temperature
  float motor_tmp;      // Motor Temperature
  float board_tmp;      // Board Temperature
#if _MC_PEAKS // && _MC_TMP
  float max_heathsink_tmp;
  float max_motor_tmp;
  float max_board_tmp;
#endif // _MC_PEAKS
#endif // _MC_TMP
#if _MC_AMPHRS
  float dc_amp_hrs;      // DC Bus AmpHrs
#endif // _MC_AMPHRS
#if _MC_ODO
  float odometer;       // Distance traveled since reset (m)
#endif // _MC_ODO
#if _MC_SLIP
  float slip_speed;     // Motor Slip Speed (Hz)
#endif // _MC_SLIP
#if _MC_VELOCITY == 1
  float velocity_kmh;
#elif _MC_VELOCITY == 2
  float velocity_rpm;
#elif _MC_VELOCITY == 3
  float velocity_kmh;
  float velocity_rpm;
#endif // _MC_VELOCITY

  /// Calculated Values & Peaks
#if _MC_POWER
  float watts;          // Bus_I * Bus_V
  float watt_hrs;        // Calculated every 10mS
#if _MC_PEAKS // && _MC_POWER
  float max_watts;
#endif // _MC_PEAKS
#endif // _MC_POWER
#if _MC_PEAKS
  float max_bus_i;
  float max_bus_v;
#endif // MC_PEAKS
}MOTORCONTROLLER;

struct BMU_STRUCT
{
  /// From CAN Bus
  uint32_t bus_v;   // Battery Voltage
  int32_t bus_i;    // Battery Output Current
  uint32_t status;      // status Flags
#if _BMU_SOC == 1
  float soc;            // Battery State of Charge (0 = Full)
#elif _BMU_SOC == 2
  float soc_per;        // Battery State of Charge as % (1 = Full)
#elif _BMU_SOC ==3
  float soc;
  float soc_per;
#endif // _BMU_SOC
#if _BMU_BAL_SOC == 1
  float bal_soc;        // Balance State of Charge. Ah supplied to pack since first cell began balancing
#elif _BMU_BAL_SOC == 2
  float bal_soc_per;    // Balance State of Charge as %
#elif _BMU_BAL_SOC == 3
  float bal_soc;
  float bal_soc_per;
#endif // _BMU_BAL_SOC
#if _BMU_THRES
  int16_t charge_cell_v_err;    //
  int16_t cell_tmp_margin;      //
  int16_t discharge_cell_v_err; //
#endif // _BMU_THRES
#if _BMU_CAP
  uint16_t pack_capacity;
#endif // _BMU_CAP
#if _BMU_PRECHARGE
  char driver_status;
  char precharge_state;
  uint8_t precharge_time_elapsed;
  uint8_t precharge_timer;
#endif // _BMU_PRECHARGE
#if _BMU_CELL_V
  uint16_t min_cell_v;    // Minimum Cell Voltage
  uint16_t max_cell_v;    // Maximum Cell Voltage
  uint8_t cmu_min_v;      // CMU number with minimum cell voltage
  uint8_t cmu_max_v;      // CMU number with maximum cell voltage
  uint8_t cell_min_v;     // Cell number with minimum cell voltage
  uint8_t cell_max_v;     // Cell number with maximum cell voltage
#endif // _BMU_CELL_V
#if _BMU_CMU_TMP
  uint16_t min_cell_tmp;  // Minimum Cell Temperature
  uint16_t max_cell_tmp;  // Maximum Cell Temperature
  uint8_t cmu_min_tmp;    // CMU number with minimum cell temperature
  uint8_t cmu_max_tmp;    // CMU number with maximum cell temperature
#endif // _BMU_CMU_TMP
#if _BMU_BAL_THRES
  uint16_t bal_thres_rising;
  uint16_t bal_thres_falling;
#endif // _BMU_BAL_THRES
#if _BMU_CMU_CNT
  uint8_t cmu_count;
#endif // _BMU_CMU_CNT
#if _BMU_VER
  uint16_t bmu_fw_ver;
  uint8_t bmu_hw_ver;
  uint8_t bmu_model_id;
#endif // _BMU_VER
#if _BMU_FAN == 1
  uint16_t fan0_spd;
#elif _BMU_FAN == 2
  uint16_t fan1_spd;
#elif _BMU_FAN == 3
  uint16_t fan0_spd;
  uint16_t fan1_spd;
#endif // _BMU_FAN
#if _BMU_12V_CONSUM
  uint16_t fan_contactor_12v_ma;
  uint16_t cmu_12v_ma;
#endif // _BMU_12V_CONSUM

  /// Calculated & Peaks
#if _BMU_POWER
  uint32_t watts;
  float watt_hrs;
#if _BMU_PEAKS // && _BMU_POWER
  uint32_t max_watts;
#endif // _BMU_PEAKS
#endif // _BMU_POWER
#if _BMU_PEAKS
  uint32_t max_bus_v;
  int32_t max_bus_i;
#endif // _BMU_PEKAS
}bmu;

struct STATS_STRUCT
{
  unsigned int ramp_speed;  // .1%/cycle
  float odometer;           // km
  float odometer_tr;        // km
  float max_speed;          // kmh
  float cruise_speed;       // kmh
  uint16_t hv_counter;      //
  uint8_t buz_tim;          // 10mS ticks to sound buzzer
  uint8_t strobe_tim;
  uint8_t paddle_mode;
  volatile uint8_t flags;
  volatile uint8_t errors;
}stats;

/// stats.flags
#define STATS_DRV_MODE       ((stats.flags & 0x01) >> 0)
#define STATS_BUZZER         ((stats.flags & 0x02) >> 1)
#define STATS_MPPT_POLL      ((stats.flags & 0x04) >> 2)
#define STATS_ARMED          ((stats.flags & 0x08) >> 3)
#define STATS_CR_ACT         ((stats.flags & 0x10) >> 4)
#define STATS_CR_STS         ((stats.flags & 0x20) >> 5)
#define STATS_HAZARDS        ((stats.flags & 0x40) >> 6)
#define STATS_STROBE         ((stats.flags & 0x80) >> 7)

#define SET_STATS_DRV_MODE   stats.flags |= 0x01;	// Sports Flagged
#define SET_STATS_BUZZER     stats.flags |= 0x02;
#define SET_STATS_MPPT_POLL  stats.flags |= 0x04;
#define SET_STATS_ARMED      stats.flags |= 0x08;
#define SET_STATS_CR_ACT     stats.flags |= 0x10;
#define SET_STATS_CR_STS     stats.flags |= 0x20;
#define SET_STATS_HAZARDS    stats.flags |= 0x40;
#define SET_STATS_STROBE     stats.flags |= 0x80;

#define CLR_STATS_DRV_MODE   stats.flags &= 0xFE;	// Economy Flagged
#define CLR_STATS_BUZZER     stats.flags &= 0xFD;
#define CLR_STATS_MPPT_POLL  stats.flags &= 0xFB;
#define CLR_STATS_ARMED      stats.flags &= 0xF7;
#define CLR_STATS_CR_ACT     stats.flags &= 0xEF;
#define CLR_STATS_CR_STS     stats.flags &= 0xDF;
#define CLR_STATS_HAZARDS    stats.flags &= 0xBF;
#define CLR_STATS_STROBE     stats.flags &= 0x7F;

#define TOG_STATS_DRV_MODE   stats.flags ^= 0x01;
#define TOG_STATS_BUZZER     stats.flags ^= 0x02;
#define TOG_STATS_MPPT_POLL  stats.flags ^= 0x04;
#define TOG_STATS_ARMED      stats.flags ^= 0x08;
#define TOG_STATS_CR_ACT     stats.flags ^= 0x10;
#define TOG_STATS_CR_STS     stats.flags ^= 0x20;
#define TOG_STATS_HAZARDS    stats.flags ^= 0x40;
#define TOG_STATS_STROBE     stats.flags ^= 0x80;

/// stats.errors
#define STATS_SWOC_ACK       ((stats.errors & 0x01) >> 0)
#define STATS_HWOC_ACK       ((stats.errors & 0x02) >> 1)
#define STATS_COMMS          ((stats.errors & 0x04) >> 2)
#define STATS_FAULT          ((stats.errors & 0x18) >> 3)
#define STATS_EUNUSED_1      ((stats.errors & 0x20) >> 5)
#define STATS_EUNUSED_2      ((stats.errors & 0x40) >> 6)
#define STATS_EUNUSED_3      ((stats.errors & 0x80) >> 7)

#define SET_STATS_SWOC_ACK   stats.errors |= 0x01;
#define SET_STATS_HWOC_ACK   stats.errors |= 0x02;
#define SET_STATS_COMMS      stats.errors |= 0x04;
#define SET_STATS_EUNUSED_1  stats.errors |= 0x20;
#define SET_STATS_EUNUSED_2  stats.errors |= 0x40;
#define SET_STATS_EUNUSED_3  stats.errors |= 0x80;

#define CLR_STATS_SWOC_ACK   stats.errors &= 0xFE;
#define CLR_STATS_HWOC_ACK   stats.errors &= 0xFD;
#define CLR_STATS_COMMS      stats.errors &= 0xFB;
#define CLR_STATS_EUNUSED_1  stats.errors &= 0xDF;
#define CLR_STATS_EUNUSED_2  stats.errors &= 0xBF;
#define CLR_STATS_EUNUSED_3  stats.errors &= 0x7F;

#define TOG_STATS_SWOC_ACK   stats.errors ^= 0xFE;
#define TOG_STATS_HWOC_ACK   stats.errors ^= 0xFD;
#define TOG_STATS_COMMS      stats.errors ^= 0xFB;
#define TOG_STATS_EUNUSED_1  stats.errors ^= 0xDF;
#define TOG_STATS_EUNUSED_2  stats.errors ^= 0xBF;
#define TOG_STATS_EUNUSED_3  stats.errors ^= 0x7F;


struct DRIVE_STRUCT
{
  float speed_rpm;
  float current;
}drive;


struct CLOCK_STRUCT
{
  uint8_t   T_mS;   // (mS / 10)
  uint8_t   T_S;
  uint8_t   T_M;
  uint8_t   T_H;
  uint32_t  T_D;
  uint8_t   blink;  // half second toggle bit
}clock;

struct SHUNT_STRUCT
{
	float bus_v;
	float bus_i;
	float watt_hrs_in;
	float watt_hrs_out;
	float watt_hrs;
	float watts;
	float max_bus_v;
	float max_bus_i;
	float max_watts;
	uint8_t con_tim;
}shunt;

#endif /* STRUCT_H_ */
