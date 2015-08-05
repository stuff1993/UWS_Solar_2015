/*
 * menu.c
 *
 *  Created on: 29 May 2015
 *      Author: Stuff
 *
 *
 *  lcd_display_<menu>
 */
/*
 * Sequential menu loop logic
 * Next item: SELECTOR++; SELECTOR %= NUM_ITEMS
 * Previous item: SELECTOR += (NUM_ITEMS - 1); SELECTOR %= NUM_ITEMS;
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
extern uint16_t thr_pos, rgn_pos;


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
	lcd_putstring(1,0, EROW);
	lcd_putstring(2,0, "   GEARS ENGAGED!   ");
	lcd_putstring(3,0, EROW);
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
	char sel[2], blank[2];

	menu.driver = 255;
	menu.submenu_pos = 0;
	menu.selected = 0;

	_lcd_putTitle("-DRIVER-");
	lcd_putstring(1,0, "   DISPLAY    TEST  ");
	lcd_putstring(2,0, "   RACE 1     RACE 2");

	sprintf(sel, ">>");
	sprintf(blank, "  ");

	while(menu.driver == 255)
	{
		switch(menu.submenu_pos)
		{
		default:
		case 0:
			lcd_putstring(1,0, sel);
			lcd_putstring(1,11, blank);
			lcd_putstring(2,0, blank);
			lcd_putstring(2,11, blank);
			break;
		case 1:
			lcd_putstring(1,0, blank);
			lcd_putstring(1,11, sel);
			lcd_putstring(2,0, blank);
			lcd_putstring(2,11, blank);
			break;
		case 2:
			lcd_putstring(1,0, blank);
			lcd_putstring(1,11, blank);
			lcd_putstring(2,0, sel);
			lcd_putstring(2,11, blank);
			break;
		case 3:
			lcd_putstring(1,0, blank);
			lcd_putstring(1,11, blank);
			lcd_putstring(2,0, blank);
			lcd_putstring(2,11, sel);
			break;
		}

		if(SELECT || menu.sel_dwn)
		{
			if(!SELECT && menu.sel_dwn)
			{
				menu.sel_dwn = 0;
				menu.driver = menu.submenu_pos;
			}
			else{menu.sel_dwn = 1;}
		}
		else if(INCREMENT || DECREMENT || menu.inc_dwn || menu.dec_dwn)
		{
			if((!INCREMENT && menu.inc_dwn) || (!DECREMENT && menu.dec_dwn))
			{
				menu.inc_dwn = 0;
				menu.dec_dwn = 0;
				menu.submenu_pos = (menu.submenu_pos + 2) % 4;	// (pos + width) % total
			}
			else if(INCREMENT){menu.inc_dwn = 1;}
			else{menu.dec_dwn = 1;}
		}
		else if(LEFT || RIGHT || menu.left_dwn || menu.right_dwn)
		{
			if((!LEFT && menu.left_dwn) || (!RIGHT && menu.right_dwn))
			{
				menu.left_dwn = 0;
				menu.right_dwn = 0;
				menu.submenu_pos = ((menu.submenu_pos / 2) * 2) + ((menu.submenu_pos + 1) % 2);	// ((pos / width) * width) + ((pos + 1) % width) -- Get row number, get item at start of row number, get next width looping on width
			}
			else if(LEFT){menu.left_dwn = 1;}
			else{menu.right_dwn = 1;}
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
	lcd_putstring(1,0, EROW);
	lcd_putstring(2,0, "   NEWCAR Driver    ");
	lcd_putstring(3,0, "   Interface v2.0   ");
	delayMs(1,3500);

	lcd_putstring(0,0, "**  UWS WSC 2015  **");
	lcd_putstring(1,0, EROW);
	lcd_putstring(2,0, "    BUZZER Test..   ");
	lcd_putstring(3,0, EROW);
	BUZZER_ON
	delayMs(1,1000);
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
	_lcd_putTitle("-INFO-");
	lcd_putstring(1,0, "NEWCAR DashBOARD2.0 ");
	lcd_putstring(2,0, "HW Version: 2.0     ");
	lcd_putstring(3,0, "SW Version: 2.0.1   ");
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
void lcd_display_escBus (void) // likely to remove
{
	char buffer[20];
	int len;

	_lcd_putTitle("-ESC BUS-");

	len = sprintf(buffer, "BUS VOLTAGE: %03.0f V", ESC.Bus_V);
	lcd_putstring(1,0, buffer);
	if(len<20){_lcd_padding(1, len, 20 - len);}

	len = sprintf(buffer, "BAT VOLTAGE: %03lu V", BMU.Battery_I);
	lcd_putstring(2,0, buffer);
	if(len<20){_lcd_padding(2, len, 20 - len);}
	
	lcd_putstring(3,0, EROW);
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

	if(STATS.DRIVE_MODE == SPORTS)	{lcd_putstring(1,0, "DRIVE MODE:  SPORTS ");}
	else							{lcd_putstring(1,0, "DRIVE MODE: ECONOMY ");}

	if(STATS.CR_ACT)				{sprintf(buffer, "CRUISE:    %3.1f%%   ", ESC.Bus_I * (100 / MAX_ESC_CUR));} // hard coded to 65A = 100% (100 / 65)
	else if(FORWARD && !rgn_pos)	{sprintf(buffer, "DRIVE:     %3d.%d%%   ", thr_pos/10,thr_pos%10);}
	else if(REVERSE && !rgn_pos)	{sprintf(buffer, "REVERSE:   %3d.%d%%   ", thr_pos/10,thr_pos%10);}
	else if(rgn_pos)				{sprintf(buffer, "REGEN:     %3d.%d%%   ", rgn_pos/10,rgn_pos%10);}
	else							{sprintf(buffer, "NEUTRAL:   %3d.%d%%   ", thr_pos/10,thr_pos%10);}
	lcd_putstring(2,0, buffer);

	// If no ERRORS then display MPPT Watts
	if(ESC.ERROR)									{sprintf(buffer, "ESC  FAULT          ");}
	else if(MPPT1.UNDV || MPPT1.OVT)				{sprintf(buffer, "MPPT1  FAULT        ");}
	else if(MPPT2.UNDV || MPPT2.OVT)				{sprintf(buffer, "MPPT2  FAULT        ");}
	else if(BMU.Status & 0x00001FBF)				{sprintf(buffer, "BMU  FAULT          ");}
	else if(!(MPPT1.Connected && MPPT2.Connected))	{sprintf(buffer, "NO ARRAY            ");}
	else											{int len = sprintf(buffer, "POWER: %4.1f W", ESC.Watts);_lcd_padding(3, len, 20 - len);}
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

	if(FORWARD)		{lcd_putstring(1,0, "MODE:          DRIVE");}
	else if(REVERSE){lcd_putstring(1,0, "MODE:        REVERSE");}
	else			{lcd_putstring(1,0, "MODE:        NEUTRAL");}

	if(!rgn_pos)
	{
		sprintf(buffer, "OUTPUT:       %5.1f%%", DRIVE.Current*100);
		lcd_putstring(2,0, buffer);

		sprintf(buffer, "THROTTLE:     %3d.%d%%", thr_pos/10,thr_pos%10);
		lcd_putstring(3,0, buffer);
	}
	else
	{
		sprintf(buffer, "OUTPUT:       %5.1f%%", DRIVE.Current*100);
		lcd_putstring(2,0, buffer);

		sprintf(buffer, "REGEN:        %3d.%d%%", rgn_pos/10,rgn_pos%10);
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
	_lcd_putTitle("-CRUISE-");

	if(!REVERSE)
	{
		char buffer[20];

		if(STATS.CR_STS && STATS.CR_ACT)
		{
			lcd_putstring(1,0, " STS:  ON  ACT:  ON ");

			sprintf(buffer, " SET: %3.0f  SPD: %3.0f ", STATS.CRUISE_SPEED, ESC.Velocity_KMH);
			lcd_putstring(2,0, buffer);

			sprintf(buffer, " THR: %3.0f%% ABS: %3.0fA", ESC.Bus_I * (100 / MAX_ESC_CUR), ESC.Bus_I);
			lcd_putstring(3,0, buffer);

			// Button presses
			if (SELECT)										{STATS.CR_ACT = OFF;delayMs(1,500);}
			else if (INCREMENT)								{STATS.CRUISE_SPEED += 1;delayMs(1,500);}
			else if (DECREMENT && STATS.CRUISE_SPEED > 1)	{STATS.CRUISE_SPEED -= 1;delayMs(1,500);}
		}
		else if(STATS.CR_STS && !STATS.CR_ACT)
		{
			lcd_putstring(1,0, " STS:  ON  ACT: OFF ");

			sprintf(buffer, " SET: %3.0f  SPD: %3.0f ", STATS.CRUISE_SPEED, ESC.Velocity_KMH);
			lcd_putstring(2,0, buffer);

			lcd_putstring(3,0, EROW);

			// Button presses
			if(SELECT)										{STATS.CR_STS = OFF;STATS.CRUISE_SPEED = 0;delayMs(1,500);}
			else if(INCREMENT && (STATS.CRUISE_SPEED > 1))	{STATS.CR_ACT = ON;delayMs(1,500);}
			else if(DECREMENT)								{STATS.CRUISE_SPEED = ESC.Velocity_KMH;STATS.CR_ACT = ON;delayMs(1,500);}
		}
		else if(STATS.CR_ACT && !STATS.CR_STS) // Should never trip, but just in case
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

			lcd_putstring(3,0, EROW);

			// Button presses
			if(SELECT){STATS.CRUISE_SPEED = 0;STATS.CR_STS = ON;delayMs(1,500);}
		}
	}
	else // no cruise in reverse
	{
		lcd_putstring(1,0, " STS: OFF  ACT: OFF ");
		lcd_putstring(2,0, "  REVERSE ENGAGED!  ");
		lcd_putstring(3,0, EROW);

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
	_lcd_putTitle("-MPPT 1-");

	if(MPPT1.Connected)
	{
		char buffer[20];
		int len;

		len = sprintf(buffer, "IN: %3lu.%luV @ %2lu.%02luA", MPPT1.VIn/10, MPPT1.VIn%10, MPPT1.IIn/100, MPPT1.IIn%100);
		lcd_putstring(1,0, buffer);
		if(len<20){_lcd_padding(1, len, 20 - len);}

		len = sprintf(buffer, "OUT:%3lu.%luV @ %4luW", MPPT1.VOut/10, MPPT1.VOut%10, MPPT1.Watts);
		lcd_putstring(2,0, buffer);
		if(len<20){_lcd_padding(2, len, 20 - len);}

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
		else{lcd_putstring(3,0, "                ");}
	}
	else // No connection
	{
		lcd_putstring(1,0, EROW);
		lcd_putstring(3,0, EROW);

		if(CLOCK.T_mS > 25 && CLOCK.T_mS < 75){lcd_putstring(2,0, "**CONNECTION ERROR**");}
		else{lcd_putstring(2,0, EROW);}
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
	_lcd_putTitle("-MPPT 2-");

	if(MPPT2.Connected)
	{
		char buffer[20];
		int len;

		len = sprintf(buffer, "IN: %3lu.%luV @ %2lu.%02luA", MPPT2.VIn/10, MPPT2.VIn%10, MPPT2.IIn/100, MPPT2.IIn%100);
		lcd_putstring(1,0, buffer);
		if(len<20){_lcd_padding(1, len, 20 - len);}

		len = sprintf(buffer, "OUT:%3lu.%luV @ %4luW", MPPT2.VOut/10, MPPT2.VOut%10, MPPT2.Watts);
		lcd_putstring(2,0, buffer);
		if(len<20){_lcd_padding(2, len, 20 - len);}

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
		lcd_putstring(1,0, EROW);
		lcd_putstring(3,0, EROW);

		if(CLOCK.T_mS > 25 && CLOCK.T_mS < 75){lcd_putstring(2,0, "**CONNECTION ERROR**");}
		else{lcd_putstring(2,0, EROW);}
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

	len = sprintf(buffer, "MPPT1: %.2f Whrs", MPPT1.WattHrs);
	lcd_putstring(1,0, buffer);
	if(len<20){_lcd_padding(1, len, 20 - len);}

	len = sprintf(buffer, "MPPT2: %.2f Whrs", MPPT2.WattHrs);
	lcd_putstring(2,0, buffer);
	if(len<20){_lcd_padding(2, len, 20 - len);}

	len = sprintf(buffer, "TOTAL: %.2f Whrs", MPPT1.WattHrs + MPPT2.WattHrs);
	lcd_putstring(3,0, buffer);
	if(len<20){_lcd_padding(3, len, 20 - len);}

	if(SELECT && INCREMENT)
	{
		MPPT1.WattHrs = 0;
		MPPT2.WattHrs = 0;
		buzzer(50);
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
	int len;
	menu.submenu_items = 3;

	switch(menu.submenu_pos)
	{
	default:
		menu.submenu_pos = 0;
		/* no break */
	case 0:
		_lcd_putTitle("-MTR PWR-");

		len = sprintf(buffer, "%5.1fV @ %5.1fA", ESC.Bus_V, ESC.Bus_I);
		lcd_putstring(1,0, buffer);
		if(len<20){_lcd_padding(1, len, 20 - len);}

		len = sprintf(buffer, "TOTAL: %.2fW",  ESC.Watts);
		lcd_putstring(2,0, buffer);
		if(len<20){_lcd_padding(2, len, 20 - len);}
		break;
	case 1:
		_lcd_putTitle("-PWR USED-");

		len = sprintf(buffer, "ESC: %.2f W/hrs", ESC.WattHrs);
		lcd_putstring(1,0, buffer);
		if(len<20){_lcd_padding(1, len, 20 - len);}

		lcd_putstring(2,0, EROW);
		break;
	case 2:
		_lcd_putTitle("-MTR PKS-");

		len = sprintf(buffer, "%5.1fV @ %5.1fA", ESC.MAX_Bus_V, ESC.MAX_Bus_I);
		lcd_putstring(1,0, buffer);
		if(len<20){_lcd_padding(1, len, 20 - len);}

		len = sprintf(buffer, "TOTAL: %.2fW",  ESC.MAX_Watts);
		lcd_putstring(2,0, buffer);
		if(len<20){_lcd_padding(2, len, 20 - len);}

		if(SELECT)
		{
			ESC.MAX_Bus_I = 0;
			ESC.MAX_Bus_V = 0;
			ESC.MAX_Watts = 0;

			buzzer(50);
		}
		break;
	}
	len = sprintf(buffer, "ERROR: %d", ESC.ERROR);
	lcd_putstring(3,0, buffer);
	if(len<20){_lcd_padding(3, len, 20 - len);}

	if(INCREMENT)		{menu_inc(&menu.submenu_pos, menu.submenu_items);delayMs(1,500);}
	else if(DECREMENT)	{menu_dec(&menu.submenu_pos, menu.submenu_items);delayMs(1,500);}
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
void lcd_display_debug (void)
{// TODO: TEAM - Update field names
	char buffer[20];

	_lcd_putTitle("-DEBUG-");

	sprintf(buffer, "BUS E: %4.1f Whrs  ", BMU.WattHrs);
	lcd_putstring(1,0, buffer);

	sprintf(buffer, "BUS I: %3lu Amps  ", BMU.Battery_I);
	lcd_putstring(2,0, buffer);

	sprintf(buffer, "BUS P: %4lu Watts  ", BMU.Watts);
	lcd_putstring(3,0, buffer);

	if(SELECT && INCREMENT)
	{
		BMU.WattHrs = 0;
		buzzer(50);
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
void lcd_display_errors (void)
{
	char buffer[20];

	_lcd_putTitle("-FAULTS-");

	sprintf(buffer, "ESC: %d", ESC.ERROR);
	lcd_putstring(1,0, buffer);

	sprintf(buffer, "MPPT: %#5x", ((MPPT2.Connected ? 1 : 0) << 9)|((MPPT1.Connected ? 1 : 0) << 8)|(MPPT1.BVLR << 7)|(MPPT1.OVT << 6)|(MPPT1.NOC << 5)|(MPPT1.UNDV << 4)|(MPPT2.BVLR << 3)|(MPPT2.OVT << 2)|(MPPT2.NOC << 1)|(MPPT2.UNDV));
	lcd_putstring(2,0, buffer);

	sprintf(buffer, "BMU: %lu", BMU.Status);
	lcd_putstring(3,0, buffer);

	if(SELECT && ESC.ERROR)	// MOTOR CONTROLLER ERROR RESET	GOES BELOW	--	NOT YET TESTED
	{
		sprintf(buffer, "RESET MOTOR CONTROLS");
		lcd_putstring(1,0, buffer);
		lcd_putstring(2,0, buffer);
		lcd_putstring(3,0, buffer);

		if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
		{
			esc_reset();
			buzzer(50);
		}
		delayMs(1,1000);
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
void lcd_display_options (void)
{
	char buffer[20];
	int len;
	menu.submenu_items = 2;

	_lcd_putTitle("-OPTIONS-");
	lcd_putstring(1,0, EROW);

	if (menu.selected || (CLOCK.T_mS > 25 && CLOCK.T_mS < 75))
	{
		switch(menu.submenu_pos)
		{
		default:
			menu.submenu_pos = 0;
			/* no break */
		case 0:
			if(STATS.BUZZER){lcd_putstring(2,0, ">> BUZZER: ON       ");}
			else{lcd_putstring(2,0, ">> BUZZER: OFF      ");}
			len = sprintf(buffer, "   DRIVER: %d", menu.driver);
			break;
		case 1:
			if(STATS.BUZZER){lcd_putstring(2,0, "   BUZZER: ON       ");}
			else{lcd_putstring(2,0, "   BUZZER: OFF      ");}
			len = sprintf(buffer, ">> DRIVER: %d", menu.driver);
			break;
		}
	}
	else
	{
		if(STATS.BUZZER){lcd_putstring(2,0, "   BUZZER: ON       ");}
		else{lcd_putstring(2,0, "   BUZZER: OFF      ");}
		len = sprintf(buffer, "   DRIVER: %d", menu.driver);
	}
	lcd_putstring(3,0, buffer);
	_lcd_padding(3,len, 20 - len);

	/////////////////////////////   ACTIONS   //////////////////////////////
	if(menu.selected & 0x1) // retry basic check with new board
	{
		if(SELECT)
		{
			menu.selected = 0;
			switch(menu.submenu_pos)
			{
			case 0:
				EE_Write(AddressBUZZ, STATS.BUZZER);
				delayMs(1,500);
				break;
			case 1:
				menu_init();
				delayMs(1,500);
				break;
			}
		}
		else if(INCREMENT)
		{
			switch(menu.submenu_pos)
			{
			case 0:
				STATS.BUZZER ^= 0b1;
				delayMs(1,500);
				break;
			case 1:
				menu.driver = (menu.driver + 1) % 4;
				delayMs(1,500);
				break;
			default:
				break;
			}
		}
		else if(DECREMENT)
		{
			switch(menu.submenu_pos)
			{
			case 0:
				STATS.BUZZER ^= 0b1;
				delayMs(1,500);
				break;
			case 1:
				menu.driver = (menu.driver + 3) % 4;
				delayMs(1,500);
				break;
			default:
				break;
			}
		}
	}
	else
	{
		if(SELECT)			{menu.selected = 1;delayMs(1,500);}
		else if(INCREMENT)	{menu_inc(&menu.submenu_pos, menu.submenu_items);delayMs(1,500);}
		else if(DECREMENT)	{menu_dec(&menu.submenu_pos, menu.submenu_items);delayMs(1,500);}
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
void lcd_display_peaks (void)
{
	char buffer[20];
	int len;
	
	_lcd_putTitle("-DATA PKS-");

	len = sprintf(buffer, "ARRAY: %4lu Watts", MPPT1.MAX_Watts + MPPT2.MAX_Watts);
	lcd_putstring(2,0, buffer);
	if(len<20){_lcd_padding(2, len, 20 - len);}

	len = sprintf(buffer, "TOP SPD: %3.1f kmh", STATS.MAX_SPEED);
	lcd_putstring(3,0, buffer);
	if(len<20){_lcd_padding(3, len, 20 - len);}

	if(SELECT)
	{
		MPPT1.MAX_Watts = 0;
		MPPT2.MAX_Watts = 0;
		STATS.MAX_SPEED = 0;
		buzzer(50);
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
void lcd_display_runtime (void)
{
	char buffer[20];
	int len;

	_lcd_putTitle("-RUNTIME-");
	lcd_putstring(1,0, EROW);

	len = sprintf(buffer, "%luD %02dhr", CLOCK.T_D, CLOCK.T_H);
	lcd_putstring(2,0, buffer);
	if(len<20){_lcd_padding(2, len, 20 - len);}

	len = sprintf(buffer, "%02dm %02d.%01ds", CLOCK.T_M, CLOCK.T_S, CLOCK.T_mS/10);
	lcd_putstring(3,0, buffer);
	if(len<20){_lcd_padding(3, len, 20 - len);}
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
void lcd_display_odometer (void)
{
	char buffer[20];
	int len;

	_lcd_putTitle("-ODOMETER-");

	len = sprintf(buffer, "CAR: %.3f KM", STATS.ODOMETER);
	lcd_putstring(1,0, buffer);
	if(len<20){_lcd_padding(1, len, 20 - len);}

	len = sprintf(buffer, "ESC: %.3f KM", ESC.Odometer/1000);
	lcd_putstring(2,0, buffer);
	if(len<20){_lcd_padding(2, len, 20 - len);}

	len = sprintf(buffer, "TRP: %.3f KM", STATS.TR_ODOMETER);
	lcd_putstring(3,0, buffer);
	if(len<20){_lcd_padding(3, len, 20 - len);}

	if(SELECT)
	{
		STATS.TR_ODOMETER = 0;
		buzzer(50);
	}
	else if (INCREMENT && DECREMENT)
	{
		delayMs(1,1000);
		if (INCREMENT && DECREMENT)
		{
			STATS.ODOMETER = 0;
			STATS.TR_ODOMETER = 0;
			buzzer(50);
		}
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
	_lcd_putTitle("-SWOC ERR-");
	lcd_putstring(1,0, "*******ERROR!*******");
	lcd_putstring(2,0, "PRESS SELECT 2 RESET");
	lcd_putstring(3,0, "PRESS OTHER 2 CANCEL");

	// BUTTONS
	if(SELECT)
	{
		if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
		{
			esc_reset();
			buzzer(20);
		}
		delayMs(1,500);
	}
	else if(INCREMENT || DECREMENT){STATS.SWOC_ACK = TRUE;}	// mark error acknowledged
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
	_lcd_putTitle("-HWOC ERR-");
	lcd_putstring(1,0, "*******ERROR!*******");
	lcd_putstring(2,0, "PRESS SELECT 2 RESET");
	lcd_putstring(3,0, "PRESS OTHER 2 CANCEL");

	BUZZER_ON

	// BUTTONS
	if(SELECT)
	{
		if((LPC_CAN1->GSR & (1 << 3))){esc_reset();}
		delayMs(1,500);
	}
	else if(INCREMENT || DECREMENT){STATS.HWOC_ACK = TRUE;}	// mark error acknowledged
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
	_lcd_putTitle("-COMMS-");
	lcd_putstring(1,0, "    CHECK  COMMS    ");
	lcd_putstring(2,0, "SELECT: RADIO WORKS ");
	lcd_putstring(3,0, "OTHER:  NO RESPONSE ");

	if(SELECT)
	{
		if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
		{
			MsgBuf_TX1.Frame = 0x00010000; 			// 11-bit, no RTR, DLC is 1 byte
			MsgBuf_TX1.MsgID = DASH_RPLY + 1;
			MsgBuf_TX1.DataA = 0xFF;
			MsgBuf_TX1.DataB = 0x0;
			CAN1_SendMessage( &MsgBuf_TX1 );
		}
		STATS.COMMS = 0;
		delayMs(1,500);
	}
	else if(INCREMENT || DECREMENT)
	{
		if((LPC_CAN1->GSR & (1 << 3)))				// If previous transmission is complete, send message;
		{
			MsgBuf_TX1.Frame = 0x00010000; 			// 11-bit, no RTR, DLC is 1 byte
			MsgBuf_TX1.MsgID = DASH_RPLY + 1;
			MsgBuf_TX1.DataA = 0x0;
			MsgBuf_TX1.DataB = 0x0;
			CAN1_SendMessage( &MsgBuf_TX1 );
		}
		STATS.COMMS = 0;
		delayMs(1,500);
	}
}

/******************************************************************************
** Function name:		lcd_display_CAN_BUS
**
** Description:			Display when 12v bus low
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void lcd_display_CAN_BUS (void) // errors[3]
{
	_lcd_putTitle("-12v LOW-");
	lcd_putstring(1,0, "    ** ERROR! **    ");
	lcd_putstring(2,0, "    12v Rail low    ");
	lcd_putstring(3,0, "    ** ERROR! **    ");
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
	char buffer[20];
	char spd[11];
	char *bufadd;
	char *spdadd;

	bufadd = buffer;
	spdadd = spd;

	sprintf(buffer, _title);
	while ((*(++bufadd) != '\0') && (bufadd < buffer + 10)){;}

	for (;bufadd != buffer + 10; bufadd++){*bufadd = ' ';}

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
	char buffer[len];
	sprintf(buffer, "%*s", len, "");
	lcd_putstring(row,pos, buffer);
}

/******************************************************************************
** Function name:		_buffer_rotate_right
**
** Description:			Rotates characters in buffer to the right
**
** Parameters:			1. Address of buffer/string
** 						2. Length of buffer
** Returned value:		None
**
******************************************************************************/
void _buffer_rotate_right (char *_buf, int _len)
{
	char _last = *(_buf + _len - 1);
	char* _cur = (_buf + _len - 1);
	while(_cur != _buf){*_cur = *(_cur - 1);_cur--;}
	*_buf = _last;
}

/******************************************************************************
** Function name:		_buffer_rotate_left
**
** Description:			Rotates characters in buffer to the left
**
** Parameters:			1. Address of buffer/string
** 						2. Length of buffer
** Returned value:		None
**
******************************************************************************/
void _buffer_rotate_left (char *_buf, int _len)
{
	char _first = *_buf;
	char* _cur = _buf;
	while(_cur != (_buf + _len - 1)){*_cur = *(_cur + 1);_cur++;}
	*(_buf + _len - 1) = _first;
}

/******************************************************************************
** Function name:		menu_inc
**
** Description:			Increments menu selection by 1. Will loop to first item
**
** Parameters:			1. Address of menu to increment
** 						2. Total items in menu
** Returned value:		None
**
******************************************************************************/
void menu_inc (uint8_t *_pos, uint8_t _total)
{*_pos = (*_pos + 1) % _total;}

/******************************************************************************
** Function name:		menu_dec
**
** Description:			Decrements menu selection by 1. Will loop to last item
**
** Parameters:			1. Address of menu to decrement
** 						2. Total items in menu
** Returned value:		None
**
******************************************************************************/
void menu_dec (uint8_t *_pos, uint8_t _total)
{*_pos = (*_pos + _total - 1) % _total;}

/******************************************************************************
** Function name:		menu_init
**
** Description:			Initialize menu arrays
**
** Parameters:			None
** Returned value:		None
**
******************************************************************************/
void menu_init (void)
{
	menu.errors[0] = lcd_display_SWOC;
	menu.errors[1] = lcd_display_HWOC;
	menu.errors[2] = lcd_display_COMMS;
	menu.errors[3] = lcd_display_CAN_BUS;

	switch (menu.driver)
	{
	default:
	case 0: // DISPLAY
		menu.menu_items = 11;
		menu.menus[0] = lcd_display_home;
		menu.menus[1] = lcd_display_drive;
		menu.menus[2] = lcd_display_MPPT1;
		menu.menus[3] = lcd_display_MPPT2;
		menu.menus[4] = lcd_display_MPPTPower;
		menu.menus[5] = lcd_display_motor;
		menu.menus[6] = lcd_display_options;
		menu.menus[7] = lcd_display_peaks;
		menu.menus[8] = lcd_display_runtime;
		menu.menus[9] = lcd_display_odometer;
		menu.menus[10] = lcd_display_info;
		break;
	case 1: // TEST
		menu.menu_items = 15;
		menu.menus[0] = lcd_display_home;
		menu.menus[1] = lcd_display_drive;
		menu.menus[2] = lcd_display_cruise;
		menu.menus[3] = lcd_display_MPPT1;
		menu.menus[4] = lcd_display_MPPT2;
		menu.menus[5] = lcd_display_MPPTPower;
		menu.menus[6] = lcd_display_motor;
		menu.menus[7] = lcd_display_debug;
		menu.menus[8] = lcd_display_errors;
		menu.menus[9] = lcd_display_options;
		menu.menus[10] = lcd_display_peaks;
		menu.menus[11] = lcd_display_runtime;
		menu.menus[12] = lcd_display_odometer;
		menu.menus[13] = lcd_display_info;
		menu.menus[14] = lcd_display_escBus;
		break;
	case 2: // RACE 1
		menu.menu_items = 7;
		menu.menus[0] = lcd_display_home;
		menu.menus[1] = lcd_display_drive;
		menu.menus[2] = lcd_display_cruise;
		menu.menus[3] = lcd_display_errors;
		menu.menus[4] = lcd_display_options;
		menu.menus[5] = lcd_display_runtime;
		menu.menus[6] = lcd_display_odometer;
		break;
	case 3: // RACE 2
		menu.menu_items = 7;
		menu.menus[0] = lcd_display_home;
		menu.menus[1] = lcd_display_drive;
		menu.menus[2] = lcd_display_cruise;
		menu.menus[3] = lcd_display_errors;
		menu.menus[4] = lcd_display_options;
		menu.menus[5] = lcd_display_runtime;
		menu.menus[6] = lcd_display_odometer;
		break;
	}

	menu.selected = 0;
	menu.submenu_pos = 0;
	menu.menu_pos = 0; // Initial menu screen
}
