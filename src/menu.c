/*
 * menu.c
 *
 *  Created on: 29 May 2015
 *      Author: Stuff
 *
 *
 *  lcd_display_<menu>
 */
// TODO: Redo menu nav. Modulus? -ves get wierd
/*
 * Sequential menu loop logic
 * Next item: SELECTOR++; SELECTOR %= NUM_ITEMS
 * Previous item: SELECTOR += (NUM_ITEMS - 1); SELECTOR %= NUM_ITEMS;
 */
/*
 * int mod (int a, int b)
 * {
 * 	int r = a % b;
 * 	return r < 0 ? r + b: r;
 * }
 */
#include "lpc17xx.h"

#include <stdio.h>
#include <stdint.h>
#include "type.h"
#include "can.h"
#include "timer.h"
#include "lcd.h"
#include "menu.h"
#include "dash.h"

extern MOTORCONTROLLER ESC;
extern MPPT MPPT1, MPPT2;
extern CAN_MSG MsgBuf_TX1;
extern uint16_t THR_POS, RGN_POS;


//////////////////////////////////////////////
/// Not in array, reference manually

/******************************************************************************
** Function name:		lcd_display_errOnStart
**
** Description:			Error screen on boot
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_errOnStart (void)
{
	lcd_putstring(0,0, "--    CAUTION!    --");
	lcd_putstring(1,0, "                    ");
	lcd_putstring(2,0, "   GEARS ENGAGED!   ");
	lcd_putstring(3,0, "                    ");
}

/******************************************************************************
** Function name:		lcd_display_driver
**
** Description:			Select driver type. Determines menus available.
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_driver (void)
{
	char buffer[20], blank[2];

	MENU.DRIVER = 255;
	MENU.ITEM_SELECTOR = 0;
	MENU.SELECTED = 0;

	_lcd_putTitle("-DRIVER-");
	lcd_putstring(1,0, "   DISPLAY    TEST  ");
	lcd_putstring(2,0, "   RACE 1     RACE 2");


	sprintf(buffer, ">>");
	sprintf(blank, "  ");

	while(MENU.DRIVER == 255)
	{
		switch (MENU.ITEM_SELECTOR)
		{
		default:
		case 0:
			lcd_putstring(1,0, buffer);
			lcd_putstring(1,10, blank);
			lcd_putstring(2,0, blank);
			lcd_putstring(2,10, blank);
			break;
		case 1:
			lcd_putstring(1,0, blank);
			lcd_putstring(1,10, buffer);
			lcd_putstring(2,0, blank);
			lcd_putstring(2,10, blank);
			break;
		case 2:
			lcd_putstring(1,0, blank);
			lcd_putstring(1,10, blank);
			lcd_putstring(2,0, buffer);
			lcd_putstring(2,10, blank);
			break;
		case 3:
			lcd_putstring(1,0, blank);
			lcd_putstring(1,10, blank);
			lcd_putstring(2,0, blank);
			lcd_putstring(2,10, buffer);
			break;
		}

		if(SELECT)
		{
			MENU.DRIVER = MENU.ITEM_SELECTOR;
		}
		else if (INCREMENT || DECREMENT) // TODO: new selection logic
		{
			MENU.ITEM_SELECTOR += 2;
			MENU.ITEM_SELECTOR %= 4;
		}
		else if (LEFT || RIGHT)
		{
			MENU.ITEM_SELECTOR += 1;
			MENU.ITEM_SELECTOR %= 4;
		}
	}
}

/******************************************************************************
** Function name:		lcd_display_intro
**
** Description:			Boot intro screen
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_intro (void)
{
	lcd_putstring(0,0, "**  UWS WSC 2015  **");
	lcd_putstring(1,0, "                    ");
	lcd_putstring(2,0, "   NEWCAR Driver    ");
	lcd_putstring(3,0, "   Interface v2.0   ");
	delayMs(1,1600);

	lcd_putstring(0,0, "**  UWS WSC 2015  **");
	lcd_putstring(1,0, "                    ");
	lcd_putstring(2,0, "    BUZZER Test..   ");
	lcd_putstring(3,0, "                    ");
	BUZZER_ON
	delayMs(1,100);
	BUZZER_OFF

	lcd_clear();
}
///////////////////////////////////////////////

///////////////////////////////////////////////
/// menus array

/******************************************************************************
** Function name:		lcd_display_info
**
** Description:			Car information screen
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_info (void)
{
	char buffer[20];

	sprintf(buffer, "-INFO-");
	_lcd_putTitle(buffer);

	lcd_putstring(1,0, "NEWCAR DashBOARD2.0 ");
	lcd_putstring(2,0, "HW Version: 2.0a    ");
	lcd_putstring(3,0, "SW Version: 2.0a    ");
}

/******************************************************************************
** Function name:		lcd_display_escBus
**
** Description:			Data screen for precharge
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_escBus (void)
{
	char buffer[20];

	_lcd_putTitle("-ESC BUS-");

	sprintf(buffer, "BUS VOLTAGE: %03.0f V  ", ESC.Bus_V);
	lcd_putstring(1,0, buffer);

	sprintf(buffer, "BAT VOLTAGE: %03lu V  ", BMU.Battery_I);
	lcd_putstring(2,0, buffer);
	
	lcd_putstring(3,0, "                    ");
}

/******************************************************************************
** Function name:		lcd_display_home
**
** Description:			Speed, drive, array power, basic errors
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_home (void)
{
	char buffer[20];

	_lcd_putTitle("-HOME-");

	if(STATS.DRIVE_MODE == SPORTS){lcd_putstring(1,0, "DRIVE MODE:  SPORTS ");}
	else{lcd_putstring(1,0, "DRIVE MODE: ECONOMY ");}

	// TODO: Unlikely to get throttle % from controller, can get absolute current.
	// May be able to hard code current cap for controller and divide for %
	/*if(STATS.CR_ACT)
	{
		if (CRUISE.CO >= 1000){sprintf(buffer, "CRUISE FW: %3d.%d%%   ", PWMA/10,PWMA%10);}
		else{sprintf(buffer, "CRUISE RG: %3d.%d%%   ", PWMC/10,PWMC%10);}

		lcd_putstring(2,0, buffer);
	}
	else */if(FORWARD && !RGN_POS){
		sprintf(buffer, "DRIVE:     %3d.%d%%   ", THR_POS/10,THR_POS%10);
		lcd_putstring(2,0, buffer);}

	else if(REVERSE && !RGN_POS){
		sprintf(buffer, "REVERSE:   %3d.%d%%   ", THR_POS/10,THR_POS%10);
		lcd_putstring(2,0, buffer);}

	else if(RGN_POS){
		sprintf(buffer, "REGEN:     %3d.%d%%   ", RGN_POS/10,RGN_POS%10);
		lcd_putstring(2,0, buffer);}

	else{
		sprintf(buffer, "NEUTRAL:   %3d.%d%%   ", THR_POS/10,THR_POS%10);
		lcd_putstring(2,0, buffer);}

	// If no ERRORS then display MPPT Watts
	if		(ESC.ERROR)								{sprintf(buffer, "ESC  FAULT          ");}
	else if	(MPPT1.UNDV || MPPT1.OVT)				{sprintf(buffer, "MPPT1  FAULT        ");}
	else if	(MPPT2.UNDV || MPPT2.OVT)				{sprintf(buffer, "MPPT2  FAULT        ");}
	else if	(BMU.Status & 0x00001FBF)				{sprintf(buffer, "BMU  FAULT          ");}
	else if (!(MPPT1.Connected && MPPT2.Connected))	{sprintf(buffer, "NO ARRAY            ");}
	else											{sprintf(buffer, "ARRAY:    %4lu W  ", MPPT2.Watts+MPPT1.Watts);}
	lcd_putstring(3,0, buffer);
}

/******************************************************************************
** Function name:		lcd_display_drive
**
** Description:			Drive details screen
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_drive (void)
{
	char buffer[20];

	_lcd_putTitle("-CONTROLS-");

	if(FORWARD){
	lcd_putstring(1,0, "MODE:          DRIVE");}

	else if(REVERSE){
	lcd_putstring(1,0, "MODE:        REVERSE");}

	else{
	lcd_putstring(1,0, "MODE:        NEUTRAL");}

	if(!RGN_POS)
	{
		sprintf(buffer, "OUTPUT:       %3.1f%%", DRIVE.Current*100);
		lcd_putstring(2,0, buffer);

		sprintf(buffer, "THROTTLE:     %3d.%d%%", THR_POS/10,THR_POS%10);
		lcd_putstring(3,0, buffer);
	}

	else
	{
		sprintf(buffer, "OUTPUT:       %3.1f%%", DRIVE.Current*100);
		lcd_putstring(2,0, buffer);

		sprintf(buffer, "REGEN:        %3d.%d%%", RGN_POS/10,RGN_POS%10);
		lcd_putstring(3,0, buffer);
	}
}

/******************************************************************************
** Function name:		lcd_display_cruise
**
** Description:			Cruise control screen
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_cruise (void)
{
	char buffer[20];
	
	_lcd_putTitle("-CRUISE-");

	if (!REVERSE)
	{
		if (STATS.CR_STS && STATS.CR_ACT)
		{
			lcd_putstring(1,0, " STS:  ON  ACT:  ON ");

			sprintf(buffer, " SET: %3.0f  SPD: %3.0f ", STATS.CRUISE_SPEED, ESC.Velocity_KMH);
			lcd_putstring(2,0, buffer);

			lcd_putstring(3,0, "                    ");

			// Button presses
			if (SELECT){STATS.CR_ACT = OFF;}
			else if (INCREMENT){STATS.CRUISE_SPEED += 1;}
			else if (DECREMENT && STATS.CRUISE_SPEED > 1){STATS.CRUISE_SPEED -= 1;}
		}
		else if (STATS.CR_STS && !STATS.CR_ACT)
		{
			lcd_putstring(1,0, " STS:  ON  ACT: OFF ");

			sprintf(buffer, " SET: %3.0f  SPD: %3.0f ", STATS.CRUISE_SPEED, ESC.Velocity_KMH);
			lcd_putstring(2,0, buffer);

			lcd_putstring(3,0, "                    ");

			// Button presses
			if (SELECT){STATS.CR_STS = OFF;STATS.CRUISE_SPEED = 0;}
			else if (INCREMENT && (STATS.CRUISE_SPEED > 1)){STATS.CR_ACT = ON;}
			else if (DECREMENT){STATS.CRUISE_SPEED = ESC.Velocity_KMH;STATS.CR_ACT = ON;}
		}
		else if (STATS.CR_ACT && !STATS.CR_STS) // Should never trip, but just in case
		{
			lcd_putstring(1,0, " STS: OFF  ACT:  ON ");
			lcd_putstring(2,0, "    CRUISE ERROR    ");
			lcd_putstring(3,0, "     RESETTING      ");

			STATS.CR_ACT = OFF;
			STATS.CR_STS = OFF;
			STATS.CRUISE_SPEED = 0;
		}
		else
		{
			lcd_putstring(1,0, " STS: OFF  ACT: OFF ");

			sprintf(buffer, " SET:      SPD: %3.0f ", ESC.Velocity_KMH);
			lcd_putstring(2,0, buffer);

			lcd_putstring(3,0, "                    ");

			// Button presses
			if (SELECT){STATS.CRUISE_SPEED = 0;STATS.CR_STS = ON;}
		}
	}
	else // no cruise in reverse
	{
		lcd_putstring(1,0, " STS: OFF  ACT: OFF ");
		lcd_putstring(2,0, "  REVERSE ENGAGED!  ");
		lcd_putstring(3,0, "                    ");

		STATS.CR_STS = OFF;
		STATS.CR_ACT = OFF;
		STATS.CRUISE_SPEED = 0;
	}
}

/******************************************************************************
** Function name:		lcd_display_MPPT1
**
** Description:			MPPT1 information screen
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_MPPT1(void)
{
	char buffer[20];

	_lcd_putTitle("-MPPT 1-");

	if(MPPT1.Connected)
	{
		sprintf(buffer, "IN: %3lu.%luV @ %2lu.%02luA ", MPPT1.VIn/10, MPPT1.VIn%10, MPPT1.IIn/100, MPPT1.IIn%100);
		lcd_putstring(1,0, buffer);

		sprintf(buffer, "OUT:%3lu.%luV @ %4luW ", MPPT1.VOut/10, MPPT1.VOut%10, MPPT1.Watts);
		lcd_putstring(2,0, buffer);

		sprintf(buffer, "%2lu%cC", MPPT1.Tmp, 0xB2);
		lcd_putstring(3,16, buffer);

		if(CLOCK.T_mS > 0 && CLOCK.T_mS < 50)
		{
			if(MPPT1.OVT)		{sprintf(buffer, "OVER TEMP       ");}
			else if(MPPT1.UNDV)	{sprintf(buffer, "LOW IN VOLTAGE  ");}
			else if(MPPT1.BVLR)	{sprintf(buffer, "BATTERY FULL    ");}
			else if(MPPT1.NOC)	{sprintf(buffer, "NO BATTERY      ");}
			else				{sprintf(buffer, "                ");}
			lcd_putstring(3,0, buffer);
		}
		else{lcd_putstring(3,0, "                    ");}
	}
	else // No connection
	{
		lcd_putstring(1,0, "                    ");
		lcd_putstring(3,0, "                    ");

		if(CLOCK.T_mS > 25 && CLOCK.T_mS < 75){lcd_putstring(2,0, "**CONNECTION ERROR**");}
		else{lcd_putstring(2,0, "                    ");}
	}
}


/******************************************************************************
** Function name:		lcd_display_MPPT2
**
** Description:			MPPT2 information screen
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_MPPT2(void)
{
	char buffer[20];

	_lcd_putTitle("-MPPT 2-");

	if(MPPT2.Connected)
	{
		sprintf(buffer, "IN: %3lu.%luV @ %2lu.%02luA ", MPPT2.VIn/10, MPPT2.VIn%10, MPPT2.IIn/100, MPPT2.IIn%100);
		lcd_putstring(1,0, buffer);

		sprintf(buffer, "OUT:%3lu.%luV @ %4luW ", MPPT2.VOut/10, MPPT2.VOut%10, MPPT2.Watts);
		lcd_putstring(2,0, buffer);

		sprintf(buffer, "%2lu%cC", MPPT2.Tmp, 0xB2);
		lcd_putstring(3,16, buffer);

		if(CLOCK.T_mS > 0 && CLOCK.T_mS < 50)
		{
			if(MPPT2.OVT)		{sprintf(buffer, "OVER TEMP       ");}
			else if(MPPT2.UNDV)	{sprintf(buffer, "LOW IN VOLTAGE  ");}
			else if(MPPT2.BVLR)	{sprintf(buffer, "BATTERY FULL    ");}
			else if(MPPT2.NOC)	{sprintf(buffer, "NO BATTERY      ");}
			else				{sprintf(buffer, "                ");}
			lcd_putstring(3,0, buffer);
		}
		else{lcd_putstring(3,0, "                ");}
	}
	else // No connection
	{
		lcd_putstring(1,0, "                ");
		lcd_putstring(3,0, "                ");

		if(CLOCK.T_mS > 25 && CLOCK.T_mS < 75){lcd_putstring(2,0, "**CONNECTION ERROR**");}
		else{lcd_putstring(2,0, "                ");}
	}
}


/******************************************************************************
** Function name:		lcd_display_MPPTPower
**
** Description:			Total power from MPPTs
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_MPPTPower (void)
{
	char buffer[20];
	int len;

	_lcd_putTitle("-POWER IN-");

	sprintf(buffer, "MPPT1: %.2f Whrs%n", MPPT1.WattHrs, &len);
	lcd_putstring(1,0, buffer);

	if (len < 20){_lcd_padding(1, len, 20 - len);}

	sprintf(buffer, "MPPT2: %.2f Whrs%n", MPPT2.WattHrs, &len);
	lcd_putstring(2,0, buffer);

	if (len < 20){_lcd_padding(2, len, 20 - len);}

	sprintf(buffer, "TOTAL: %.2f Whrs%n", MPPT1.WattHrs + MPPT2.WattHrs, &len);
	lcd_putstring(3,0, buffer);

	if (len < 20){_lcd_padding(3, len, 20 - len);}

	if(SELECT && INCREMENT)
	{
		MPPT1.WattHrs = 0;
		MPPT2.WattHrs = 0;
		buzzer(300);
	}
}


/******************************************************************************
** Function name:		lcd_display_motor
**
** Description:			Motor stats screens
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_motor (void)
{
	char buffer[20];

	switch(MENU.SUBMENU_POS)
	{
	default:
		MENU.SUBMENU_POS = 0;
		/* no break */
	case 0:
		_lcd_putTitle("-MTR PWR-");

		sprintf(buffer, "ESC: %5.1fV @ %5.1fA ", ESC.Bus_V, ESC.Bus_I);
		lcd_putstring(1,0, buffer);

		sprintf(buffer, "TOTAL: %.2f",  ESC.Watts);
		lcd_putstring(2,0, buffer);
		break;
	case 1:
		_lcd_putTitle("-PWR USED-");

		sprintf(buffer, "ESC: %.2f W/hrs", ESC.WattHrs);
		lcd_putstring(1,0, buffer);

		lcd_putstring(2,0, "                ");

		break;
	case 2:
		_lcd_putTitle("-MTR PKS-");

		sprintf(buffer, "%.1fAp @ %.1fWp ", ESC.MAX_Bus_I, ESC.MAX_Watts);
		lcd_putstring(1,0, buffer);

		sprintf(buffer, "%.1fVp", ESC.MAX_Bus_V);
		lcd_putstring(2,0, buffer);

		if (SELECT)
		{
			ESC.MAX_Bus_I = 0;
			ESC.MAX_Bus_V = 0;
			ESC.MAX_Watts = 0;

			buzzer(600);
		}
		break;
	}
	lcd_putstring(3,0, "                    ");

	if(INCREMENT){MENU.SUBMENU_POS++;MENU.SUBMENU_POS %= 3;}
	else if(DECREMENT){MENU.SUBMENU_POS += 2;MENU.SUBMENU_POS %= 3;}
}


/******************************************************************************
** Function name:		lcd_display_debug
**
** Description:			Bus debug screen
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_debug (void) // menus[9]
{// TODO: Update field names
	char buffer[20];

	_lcd_putTitle("-DEBUG-");

	sprintf(buffer, "BUS E: %4.1f Whrs  ", BMU.WattHrs);
	lcd_putstring(1,0, buffer);

	sprintf(buffer, "BUS I: %3lu Amps     ", BMU.Battery_I);
	lcd_putstring(2,0, buffer);

	sprintf(buffer, "BUS P: %4lu Watts   ", BMU.Watts);
	lcd_putstring(3,0, buffer);

	if(SELECT && INCREMENT)
	{
		BMU.WattHrs = 0;
		buzzer(300);
	}
}


/******************************************************************************
** Function name:		lcd_display_errors
**
** Description:			Error display screen
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_errors (void) // menus[10]
{
	char buffer[20];
	int len;

	_lcd_putTitle("-ESC FALT-");

	// If no ERRORS then display MPPT Watts
	if(ESC.ERROR)
	{
		lcd_putstring(2,0, "ESC FAULT           ");

		sprintf(buffer, "CODE: %d%n", ESC.ERROR, &len);
		lcd_putstring(3,0, buffer);
		
		if (len < 20){_lcd_padding(3, len, 20 - len);}
	}
	else
	{
		lcd_putstring(2,0, "NO FAULTS           ");
		lcd_putstring(3,0, "                    ");
	}

	if(SELECT && ESC.ERROR)	// MOTOR CONTROLLER ERROR RESET	GOES BELOW	--	NOT YET TESTED
	{
		sprintf(buffer, "RESET MOTOR CONTROLS");
		lcd_putstring(1,0, buffer);
		lcd_putstring(2,0, buffer);
		lcd_putstring(3,0, buffer);
		buzzer(500);

		// RESET MOTOR CONTROLLERS
		// see WS22 user manual and Tritium CAN network specs
		// try MC + 25 (0x19) + msg "RESETWS" (TRI88.004 ver3 doc, July 2013)

		if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
		{
			esc_reset();
			buzzer(20);
		}
	}
}


/******************************************************************************
** Function name:		lcd_display_options
**
** Description:			Other options on this screen.
** 						Buzzer and driver settings
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_options (void) // menus[11]
{ // TODO: Redo for no backlight, add driver mode
	char buffer[20];

	_lcd_putTitle("-OPTIONS-");

	if (MENU.SELECTED || (CLOCK.T_S%2))
	{
		switch(MENU.ITEM_SELECTOR)
		{
		default:
			MENU.ITEM_SELECTOR = 0;
			/* no break */
		case 0:
			if(STATS.BUZZER){lcd_putstring(2,0, ">> BUZZER:     ON   ");}
			else{lcd_putstring(2,0, ">> BUZZER:    OFF   ");}
			break;
		case 1:
			if(STATS.BUZZER){lcd_putstring(2,0, "   BUZZER:     ON   ");}
			else{lcd_putstring(2,0, "   BUZZER:    OFF   ");}
			break;
		}
	}
	else
	{
		if(STATS.BUZZER){lcd_putstring(2,0, "   BUZZER:     ON   ");}
		else{lcd_putstring(2,0, "   BUZZER:    OFF   ");}
	}

	/////////////////////////////   ACTIONS   //////////////////////////////
	if(MENU.SELECTED)
	{
		if(SELECT)
		{
			MENU.SELECTED = 0;
			switch(MENU.ITEM_SELECTOR)
			{
			case 0:
				EE_Write(AddressBUZZ, STATS.BUZZER);
				break;
			case 1:
				break;
			}

		}
		else if(INCREMENT)
		{
			switch(MENU.ITEM_SELECTOR)
			{
			case 0:
				if(STATS.BUZZER){BUZZER_OFF}
				else{BUZZER_ON}
				break;
			case 1:
				break;
			default:
				break;
			}
		}
		else if(DECREMENT)
		{
			switch(MENU.ITEM_SELECTOR)
			{
			case 0:
				if(STATS.BUZZER){BUZZER_OFF}
				else{BUZZER_ON}
				break;
			case 1:
				break;
			default:
				break;
			}
		}
	}
	else
	{
		if(SELECT){MENU.SELECTED = 1;}
		else if(INCREMENT)
		{
			MENU.ITEM_SELECTOR++;
			MENU.ITEM_SELECTOR %= 2;
		}
		else if(DECREMENT)
		{
			MENU.ITEM_SELECTOR--;
			MENU.ITEM_SELECTOR %= 2;
		}
	}
}


/******************************************************************************
** Function name:		lcd_display_peaks
**
** Description:			Car peaks screen
** 						1. Array power
** 						2. Top speed
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_peaks (void) // menus[12]
{
	char buffer[20];
	
	_lcd_putTitle("-DATA PKS-");

	sprintf(buffer, "ARRAY: %4lu Watts  ", MPPT1.MAX_Watts + MPPT2.MAX_Watts);
	lcd_putstring(2,0, buffer);

	sprintf(buffer, "TOP SPD: %3.1f kmh  ", STATS.MAX_SPEED);
	lcd_putstring(3,0, buffer);

	if(SELECT)
	{
		MPPT1.MAX_Watts = 0;
		MPPT2.MAX_Watts = 0;
		STATS.MAX_SPEED = 0;
		buzzer(300);
	}
}


/******************************************************************************
** Function name:		lcd_display_runtime
**
** Description:			Displays car's current runtime
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_runtime (void) // menus[13]
{
	char buffer[20];

	_lcd_putTitle("-RUNTIME-");

	lcd_putstring(1,0, "                    ")

	sprintf(buffer, "%luD%02dhr", CLOCK.T_D, CLOCK.T_H);
	lcd_putstring(2,14, buffer);

	sprintf(buffer, "%02dm%02d.%01ds", CLOCK.T_M, CLOCK.T_S, CLOCK.T_mS/10);
	lcd_putstring(3,12, buffer);
}

/******************************************************************************
** Function name:		lcd_display_odometer
**
** Description:			Displays odometer
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_odometer (void) // menus[14]
{
	char buffer[20];

	_lcd_putTitle("-ODOMETER-");

	sprintf(buffer, "CAR: %.5f KM   ", STATS.ODOMETER); // TODO: Check string formats for float
	lcd_putstring(2,0, buffer);

	sprintf(buffer, "ESC: %.5f KM   ", ESC.Odometer);
	lcd_putstring(3,0, buffer);

	if(SELECT && INCREMENT)
	{
		STATS.ODOMETER = 0;
		buzzer(300);
	}
}
///////////////////////////////////////////////

///////////////////////////////////////////////
/// errors array

/******************************************************************************
** Function name:		lcd_display_SWOC
**
** Description:			Display screen for SWOC error
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_SWOC (void) // errors[0]
{
	char buffer[20];

	sprintf(buffer, "-SWOC ERR-");
	_lcd_putTitle(buffer);

	sprintf(buffer, "*******ERROR!*******");
	lcd_putstring(1,0, buffer);

	sprintf(buffer, "PRESS SELECT 2 RESET");
	lcd_putstring(2,0, buffer);

	sprintf(buffer, "PRESS OTHER 2 CANCEL");
	lcd_putstring(3,0, buffer);

	// BUTTONS
	if(SELECT)
	{
		if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
		{
			esc_reset();
			buzzer(20);
		}
	}
	else if(INCREMENT || DECREMENT){STATS.SWOC_ACK = TRUE;}// mark error acknowledged
}

/******************************************************************************
** Function name:		lcd_display_HWOC
**
** Description:			Display screen for HWOC error
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_HWOC (void) // errors[1]
{
	char buffer[20];

	sprintf(buffer, "-HWOC ERR-");
	_lcd_putTitle(buffer);

	sprintf(buffer, "*******ERROR!*******");
	lcd_putstring(1,0, buffer);

	sprintf(buffer, "PRESS SELECT 2 RESET");
	lcd_putstring(2,0, buffer);

	sprintf(buffer, "PRESS OTHER 2 CANCEL");
	lcd_putstring(3,0, buffer);

	BUZZER_ON

	// BUTTONS
	if(SELECT)
	{
		if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
		{
			esc_reset();
		}
	}
	else if(INCREMENT || DECREMENT){STATS.HWOC_ACK = TRUE;}// mark error acknowledged
}

/******************************************************************************
** Function name:		lcd_display_COMMS
**
** Description:			Display screen for COMMs check
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_COMMS (void) // errors[2]
{
	char buffer[20];
	sprintf(buffer, "-COMMS-");
	_lcd_putTitle(buffer);

	sprintf(buffer, "    CHECK  COMMS    ");
	lcd_putstring(1,0, buffer);

	sprintf(buffer, "SELECT: RADIO WORKS ");
	lcd_putstring(2,0, buffer);

	sprintf(buffer, "OTHER:  NO RESPONSE ");
	lcd_putstring(3,0, buffer);

	if(SELECT)
		{
			if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
			{
				MsgBuf_TX1.Frame = 0x00010000; 			/* 11-bit, no RTR, DLC is 1 byte */
				MsgBuf_TX1.MsgID = DASH_RPLY + 1; 		/* Explicit Standard ID */
				MsgBuf_TX1.DataA = 0xFF;
				MsgBuf_TX1.DataB = 0x0;
				CAN1_SendMessage( &MsgBuf_TX1 );
			}
			STATS.COMMS = 0;
		}
	else if(INCREMENT || DECREMENT)
	{
		if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
		{
			MsgBuf_TX1.Frame = 0x00010000; 			/* 11-bit, no RTR, DLC is 1 byte */
			MsgBuf_TX1.MsgID = DASH_RPLY + 1; 		/* Explicit Standard ID */
			MsgBuf_TX1.DataA = 0x0;
			MsgBuf_TX1.DataB = 0x0;
			CAN1_SendMessage( &MsgBuf_TX1 );
		}
		STATS.COMMS = 0;
	}
}
///////////////////////////////////////////////

/******************************************************************************
** Function name:		_lcd_putTitle
**
** Description:			Used to place the screen title and current car speed on
** 						top line of LCD. Will truncate titles with more than
** 						10 characters.
**
** Parameters:			1. Address of char array with title string (10 character max)
** Returned value:		None
**
******************************************************************************/
void _lcd_putTitle (char *_title)
{
	// displays first 10 chars of _title and current speed at (0,0) on lcd screen
	char buffer[20];
	char spd[11];
	char *bufadd;
	char *spdadd;

	bufadd = buffer;
	spdadd = spd;

	sprintf(buffer, _title);
	while ((*(++bufadd) != '\0') && (bufadd < buffer + 10))
	{;}

	for (;bufadd != buffer + 10; bufadd++)
	{*bufadd = ' ';}

	sprintf(spd, " %5.1fkmh ", ESC.Velocity_KMH);

	for (;bufadd != buffer + 20; bufadd++)
	{
		*bufadd = *spdadd;
		spdadd++;
	}

	lcd_putstring(0, 0, buffer);
}

/******************************************************************************
** Function name:		_lcd_padding
**
** Description:			Places blank chars at a location
**
** Parameters:			1. Row (0-3)
** 						2. Position (0-19)
** 						3. Length to clear
** Returned value:		None
**
******************************************************************************/
void _lcd_padding (int row, int pos, int len)
{
	sprintf(buffer, "%*s", len, "");
	lcd_putstring(row,pos, buffer);
}

/******************************************************************************
** Function name:		menuInit
**
** Description:			Initialize menu arrays
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void menuInit (void)
{
	MENU.errors[0] = lcd_display_SWOC;
	MENU.errors[1] = lcd_display_HWOC;
	MENU.errors[2] = lcd_display_COMMS;

	switch (MENU.DRIVER)
	{
	default:
	case 0: // DISPLAY
		MENU.ACTUAL_ITEMS = 11;
		MENU.menus[0] = lcd_display_info;
		MENU.menus[1] = lcd_display_home;
		MENU.menus[2] = lcd_display_drive;
		MENU.menus[3] = lcd_display_MPPT1;
		MENU.menus[4] = lcd_display_MPPT2;
		MENU.menus[5] = lcd_display_MPPTPower;
		MENU.menus[6] = lcd_display_motor;
		MENU.menus[7] = lcd_display_options;
		MENU.menus[8] = lcd_display_peaks;
		MENU.menus[9] = lcd_display_runtime;
		MENU.menus[10] = lcd_display_odometer;
		MENU.MENU_POS = 1; // Initial menu screen
		break;
	case 1: // TEST
		MENU.ACTUAL_ITEMS = 15;
		MENU.menus[0] = lcd_display_info;
		MENU.menus[1] = lcd_display_escBus;
		MENU.menus[2] = lcd_display_home;
		MENU.menus[3] = lcd_display_drive;
		MENU.menus[4] = lcd_display_cruise;
		MENU.menus[5] = lcd_display_MPPT1;
		MENU.menus[6] = lcd_display_MPPT2;
		MENU.menus[7] = lcd_display_MPPTPower;
		MENU.menus[8] = lcd_display_motor;
		MENU.menus[9] = lcd_display_debug;
		MENU.menus[10] = lcd_display_errors;
		MENU.menus[11] = lcd_display_options;
		MENU.menus[12] = lcd_display_peaks;
		MENU.menus[13] = lcd_display_runtime;
		MENU.menus[14] = lcd_display_odometer;
		MENU.MENU_POS = 2; // Initial menu screen
		break;
	case 2: // RACE 1
		MENU.ACTUAL_ITEMS = 7;
		MENU.menus[0] = lcd_display_home;
		MENU.menus[1] = lcd_display_drive;
		MENU.menus[2] = lcd_display_cruise;
		MENU.menus[3] = lcd_display_errors;
		MENU.menus[4] = lcd_display_options;
		MENU.menus[5] = lcd_display_runtime;
		MENU.menus[6] = lcd_display_odometer;
		MENU.MENU_POS = 0; // Initial menu screen
		break;
	case 3: // RACE 2
		MENU.ACTUAL_ITEMS = 7;
		MENU.menus[0] = lcd_display_home;
		MENU.menus[1] = lcd_display_drive;
		MENU.menus[2] = lcd_display_cruise;
		MENU.menus[3] = lcd_display_errors;
		MENU.menus[4] = lcd_display_options;
		MENU.menus[5] = lcd_display_runtime;
		MENU.menus[6] = lcd_display_odometer;
		MENU.MENU_POS = 0; // Initial menu screen
		break;
	}
}
