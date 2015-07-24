/*
 * dash.h
 *
 *  Created on: Jun 9, 2015
 *      Author: Stuff
 */

#ifndef DASH_H_
#define DASH_H_

#include "struct.h"

#define WHEEL_D_M (0.557) // metres

/// OUTPUTS
#define BUZZER_ON		LPC_GPIO0->FIOSET = (1<<3);
#define BUZZER_OFF		LPC_GPIO0->FIOCLR = (1<<3);

#define FAULT_ON		LPC_GPIO0->FIOSET = (1<<27);
#define FAULT_OFF		LPC_GPIO0->FIOCLR = (1<<27);

#define BLINKER_R_ON	LPC_GPIO1->FIOSET = (1<<19);LPC_GPIO3->FIOSET = (1<<26);
#define BLINKER_R_OFF	LPC_GPIO1->FIOCLR = (1<<19);LPC_GPIO3->FIOCLR = (1<<26);

#define BLINKER_L_ON	LPC_GPIO1->FIOSET = (1<<20);LPC_GPIO3->FIOSET = (1<<25);
#define BLINKER_L_OFF	LPC_GPIO1->FIOCLR = (1<<20);LPC_GPIO3->FIOCLR = (1<<25);

#define BRAKELIGHT_ON	LPC_GPIO1->FIOSET = (1<<21);
#define BRAKELIGHT_OFF	LPC_GPIO1->FIOCLR = (1<<21);

#define REVERSE_ON		LPC_GPIO1->FIOSET = (1<<23);
#define REVERSE_OFF		LPC_GPIO1->FIOCLR = (1<<23);

#define NEUTRAL_ON		LPC_GPIO1->FIOSET = (1<<24);
#define NEUTRAL_OFF		LPC_GPIO1->FIOCLR = (1<<24);

#define REGEN_ON		LPC_GPIO1->FIOSET = (1<<25);
#define REGEN_OFF		LPC_GPIO1->FIOCLR = (1<<25);

#define DRIVE_ON		LPC_GPIO1->FIOSET = (1<<26);
#define DRIVE_OFF		LPC_GPIO1->FIOCLR = (1<<26);

#define ECO_ON			LPC_GPIO1->FIOSET = (1<<30);
#define ECO_OFF			LPC_GPIO1->FIOCLR = (1<<30);

#define SPORTS_ON		LPC_GPIO1->FIOSET = (1<<31);
#define SPORTS_OFF		LPC_GPIO1->FIOCLR = (1<<31);

#define DUTYBL			LPC_PWM1->MR6

#define HAZARDS_ON		BLINKER_R_ON;BLINKER_L_ON;
#define HAZARDS_OFF		BLINKER_R_OFF;BLINKER_L_OFF;

/// INPUTS
#define MECH_BRAKE		!(LPC_GPIO0->FIOPIN & (1<<25))

#define RIGHT 			!(LPC_GPIO0->FIOPIN & (1<<0))
#define INCREMENT 		!(LPC_GPIO0->FIOPIN & (1<<1))
#define LEFT 			!(LPC_GPIO1->FIOPIN & (1<<27))
#define DECREMENT		!(LPC_GPIO1->FIOPIN & (1<<28))
#define SELECT 			!(LPC_GPIO1->FIOPIN & (1<<29))

#define FORWARD 		!(LPC_GPIO0->FIOPIN & (1<<11))
#define REVERSE 		!(LPC_GPIO0->FIOPIN & (1<<10))

#define SPORTS_MODE		!(LPC_GPIO2->FIOPIN & (1<<10)) // Aux_ON
#define ECONOMY_MODE	!(LPC_GPIO2->FIOPIN & (1<<11)) // Aux_OFF

#define CC_ON			!(LPC_GPIO2->FIOPIN & (1<<12))
#define CC_OFF			!(LPC_GPIO2->FIOPIN & (1<<13))

#define ARMED			!(LPC_GPIO1->FIOPIN & (1<<8))

#define LEFT_ON			!(LPC_GPIO1->FIOPIN & (1<<1))
#define RIGHT_ON		!(LPC_GPIO1->FIOPIN & (1<<2))


#define IIR_FILTER_GAIN	16

#define PORT_USED	1	// I2C port

#define ECONOMY		(0)
#define SPORTS 		(1)

#define ECONOMY_RAMP_SPEED	5
#define SPORTS_RAMP_SPEED	30
#define REGEN_RAMP_SPEED	30

#define MAX_REGEN			200

#define ON		(1)
#define OFF		(0)

/// EEPROM Addresses ///
#define AddressBUZZ 	0
#define AddressBL 		4 // Trip ODO for kieran
#define AddressODO		8
#define AddressMPPT1WHR	12
#define AddressBMUWHR	16
#define AddressMPPT2WHR	20

// BMU Ignition switches

#define SW_MODE_R		0x0001
#define SW_MODE_N		0x0002
#define SW_MODE_B		0x0004
#define SW_MODE_D		0x0008
#define SW_IGN_ACC		0x0010
#define SW_IGN_ON		0x0020
#define SW_IGN_START	0x0040


void BOD_IRQHandler(void);
void menu_mppt_poll(void);
void mppt_data_extract(MPPT *_MPPT, fakeMPPTFRAME *_fkMPPT);
void menu_input_check(void);
int menu_fault_check(void);
void menu_drive(void);
void menu_lights(void);
void menu_can_handler(void);
void menu_calc(void);
void recallVariables(void);
void storeVariables(void);
uint32_t EE_Read(uint32_t _EEadd);
void EE_Write(uint32_t _EEadd, uint32_t data);
uint32_t I2C_Read(uint32_t _EEadd);
void I2C_Write(uint32_t _EEadd, uint32_t data0, uint32_t data1, uint32_t data2, uint32_t data3);
uint32_t iirFILTER(uint32_t _data_in, uint32_t _cur_data, uint8_t _gain);
void init_GPIO(void);
void buzzer(uint32_t val);
void BOD_Init(void);

#endif /* DASH_H_ */
