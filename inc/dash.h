/*
 * dash.h
 *
 *  Created on: Jun 9, 2015
 *      Author: Stuff
 */

#ifndef DASH_H_
#define DASH_H_

#include "struct.h"

#ifndef AUTO_SWOC
#define AUTO_SWOC 1 // auto reset SWOCs
#endif

#define WHEEL_D_M (0.557) // metres

#define MAX_RGN_DZ        0.05  // V at pin
#define MIN_RGN_DZ        0.05
#define MAX_THR_DZ        0.05
#define MIN_THR_DZ        0.05
#define LOW_PAD_V         0.25
#define MID_PAD_V         1.25
#define HGH_PAD_V         2.25
#define ADC_POINTS_PER_V  1240.909091

#define ECONOMY_RAMP_SPEED  5
#define SPORTS_RAMP_SPEED   30
#define REGEN_RAMP_SPEED    30

#define MAX_ESC_CUR         65.0  // Amps

#define MAX_REGEN           350.0 // Maximum available regen - 0 to 1000 (0% to 100%)
#define MAX_THR_DISP        600.0 // Maximum available throttle in display mode - 0 - 1000 (0% - 100%)
#define MAX_THR_LOWSPD      750.0 // Maximum available throttle under LOWSPD_THRES - 0 - 1000 (0% - 100%)
#define LOWSPD_THRES        20.0  // Threshold speed for low speed throttle cap

#define IIR_GAIN_ELECTRICAL 1000
#define IIR_GAIN_THERMAL    10

/// OUTPUTS
#define BUZZER_ON       LPC_GPIO0->FIOSET |= (1<<3);
#define BUZZER_OFF      LPC_GPIO0->FIOCLR |= (1<<3);

#define FAULT_ON        LPC_GPIO0->FIOSET |= (1<<27);
#define FAULT_OFF       LPC_GPIO0->FIOCLR |= (1<<27);

#define HV_ON           LPC_GPIO1->FIOSET |= (1<<8);
#define HV_OFF          LPC_GPIO1->FIOCLR |= (1<<8);

#define BLINKER_R_ON    LPC_GPIO1->FIOSET |= (1<<19);LPC_GPIO3->FIOSET |= (1<<26);
#define BLINKER_R_OFF   LPC_GPIO1->FIOCLR |= (1<<19);LPC_GPIO3->FIOCLR |= (1<<26);

#define BLINKER_L_ON    LPC_GPIO1->FIOSET |= (1<<20);LPC_GPIO3->FIOSET |= (1<<25);
#define BLINKER_L_OFF   LPC_GPIO1->FIOCLR |= (1<<20);LPC_GPIO3->FIOCLR |= (1<<25);

#define BRAKELIGHT_ON   LPC_GPIO1->FIOSET |= (1<<21);
#define BRAKELIGHT_OFF  LPC_GPIO1->FIOCLR |= (1<<21);

#define REVERSE_ON      LPC_GPIO1->FIOSET |= (1<<23);
#define REVERSE_OFF     LPC_GPIO1->FIOCLR |= (1<<23);

#define REGEN_ON        LPC_GPIO1->FIOSET |= (1<<24);
#define REGEN_OFF       LPC_GPIO1->FIOCLR |= (1<<24);

#define NEUTRAL_ON      LPC_GPIO1->FIOSET |= (1<<25);
#define NEUTRAL_OFF     LPC_GPIO1->FIOCLR |= (1<<25);

#define DRIVE_ON        LPC_GPIO1->FIOSET |= (1<<26);
#define DRIVE_OFF       LPC_GPIO1->FIOCLR |= (1<<26);

#define ECO_ON          LPC_GPIO1->FIOSET |= (1<<30);
#define ECO_OFF         LPC_GPIO1->FIOCLR |= (1<<30);

#define SPORTS_ON       LPC_GPIO1->FIOSET |= (1<<31);
#define SPORTS_OFF      LPC_GPIO1->FIOCLR |= (1<<31);

#define HAZARDS_ON      BLINKER_R_ON;BLINKER_L_ON;
#define HAZARDS_OFF     BLINKER_R_OFF;BLINKER_L_OFF;

/// INPUTS
#define MECH_BRAKE      !(LPC_GPIO0->FIOPIN & (1<<25))

#define LEFT            !(LPC_GPIO1->FIOPIN & (1<<27))
#define INCREMENT       !(LPC_GPIO0->FIOPIN & (1<<1))
#define DECREMENT       !(LPC_GPIO1->FIOPIN & (1<<28))
#define RIGHT           !(LPC_GPIO0->FIOPIN & (1<<0))
#define SELECT          !(LPC_GPIO1->FIOPIN & (1<<29))

#define FORWARD         !(LPC_GPIO0->FIOPIN & (1<<10))
#define REVERSE         !(LPC_GPIO0->FIOPIN & (1<<11))

#define SPORTS_MODE     !(LPC_GPIO2->FIOPIN & (1<<10)) // Aux_ON
#define ECONOMY_MODE    !(LPC_GPIO2->FIOPIN & (1<<11)) // Aux_OFF

#define CC_ON           !(LPC_GPIO2->FIOPIN & (1<<12))
#define CC_OFF          !(LPC_GPIO2->FIOPIN & (1<<13))

#define LEFT_ON         !(LPC_GPIO1->FIOPIN & (1<<1))
#define RIGHT_ON        !(LPC_GPIO1->FIOPIN & (1<<0))


#define PORT_USED 1 // I2C port

#define ECONOMY (0)
#define SPORTS  (1)

#define ON      (1)
#define OFF     (0)

/// EEPROM Addresses ///
#define ADD_BUZZ     0
#define ADD_ODO      4
#define ADD_ODOTR    8
#define ADD_MPPT1WHR 12
#define ADD_BMUWHR   16
#define ADD_MPPT2WHR 20

/// Function Prototypes ///
void      BOD_IRQHandler      (void);
void      main_mppt_poll      (void);
void      mppt_data_extract   (MPPT *_MPPT, MPPT_RELAY *_fkMPPT);
void      main_input_check    (void);
int       main_fault_check    (void);
void      main_drive          (void);
void      main_paddles        (uint32_t _pad1, uint32_t _pad2, uint16_t *_thr, uint16_t *_rgn);
void      main_lights         (void);
void      main_can_handler    (void);
void      main_calc           (void);
void      main_HV             (void);
void      esc_reset           (void);
uint32_t  EE_read             (uint16_t _EEadd);
uint32_t  EE_seq_read         (uint16_t _EEadd, int _len);
void      EE_write            (uint16_t _EEadd, uint32_t data);
uint32_t  I2C_read            (uint16_t _EEadd);
void      I2C_seq_read        (uint16_t _EEadd, int read_len);
void      I2C_write           (uint16_t _EEadd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3);
uint32_t  iir_filter_uint     (uint32_t _data_in, uint32_t _cur_data, uint16_t _gain);
int32_t   iir_filter_int      (int32_t _data_in, int32_t _cur_data, uint16_t _gain);
float     iir_filter_float    (float _data_in, float _cur_data, uint16_t _gain);
void      nonpersistent_load  (void);
void      persistent_load     (void);
void      persistent_store    (void);
void      GPIO_init           (void);
void      buzzer              (uint8_t val);
void      BOD_init            (void);

#endif /* DASH_H_ */
