/*
 * menu.c
 *
 *  Created on: 29 May 2015
 *      Author: Stuart G
 */
 
/*
 * Sequential menu loop logic
 * Next item:     SELECTOR = (SELECTOR + 1) % NUM_ITEMS;
 * Previous item: SELECTOR = (SELECTOR + NUM_ITEMS - 1) % NUM_ITEMS;
 */

#include "lpc17xx.h"

#include <stdio.h>
#include <stdint.h>
#include "struct.h"
#include "inputs.h"
#include "can.h"
#include "lcd.h"
#include "menu.h"
#include "dash.h"
#include "timer.h"

extern MOTORCONTROLLER esc;
extern MPPT mppt1, mppt2;
extern CAN_MSG MsgBuf_TX1;
extern uint16_t thr_pos, rgn_pos;


//////////////////////////////////////////////
/// Not in array, reference manually

/******************************************************************************
 ** Function name:  menu_errOnStart
 **
 ** Description:    Error screen on boot
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_errOnStart (void)
{
  lcd_putstring(0,0, "--    CAUTION!    --");
  lcd_putstring(1,0, EROW);
  lcd_putstring(2,0, "   GEARS ENGAGED!   ");
  lcd_putstring(3,0, EROW);
}

/******************************************************************************
 ** Function name:  menu_driver
 **
 ** Description:    Select driver type. Determines menus available.
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_driver (void)
{
  menu.driver = 255;
  menu.submenu_pos = 1;
  CLR_MENU_SELECTED;

  _lcd_putTitle("-DRIVER-");
  lcd_putstring(1,0, "   DISPLAY    TEST  ");
  lcd_putstring(2,0, "   RACE 1     RACE 2");

  while(menu.driver == 255)
  {
    _lcd_putTitle("-DRIVER-");
    switch(menu.submenu_pos)
    {
      default:
      case 0:
        lcd_putstring(1,0, SELECTOR);
        lcd_putstring(1,11, DESELECTOR);
        lcd_putstring(2,0, DESELECTOR);
        lcd_putstring(2,11, DESELECTOR);
        break;
      case 1:
        lcd_putstring(1,0, DESELECTOR);
        lcd_putstring(1,11, SELECTOR);
        lcd_putstring(2,0, DESELECTOR);
        lcd_putstring(2,11, DESELECTOR);
        break;
      case 2:
        lcd_putstring(1,0, DESELECTOR);
        lcd_putstring(1,11, DESELECTOR);
        lcd_putstring(2,0, SELECTOR);
        lcd_putstring(2,11, DESELECTOR);
        break;
      case 3:
        lcd_putstring(1,0, DESELECTOR);
        lcd_putstring(1,11, DESELECTOR);
        lcd_putstring(2,0, DESELECTOR);
        lcd_putstring(2,11, SELECTOR);
        break;
    }

    if(btn_release_select()){menu.driver = menu.submenu_pos;}

    if(btn_release_increment()){menu.submenu_pos = (menu.submenu_pos + 2) % 4;} // (pos + width) % total

    if(btn_release_decrement()){menu.submenu_pos = (menu.submenu_pos + 2) % 4;} // (pos + total - width) % total

    if(btn_release_left()){menu.submenu_pos = ((menu.submenu_pos / 2) * 2) + ((menu.submenu_pos + 1) % 2);}   // ((pos / width) * width) + ((pos + 1) % width) -- Get row number, get item at start of row number, get next width looping on width

    if(btn_release_right()){menu.submenu_pos = ((menu.submenu_pos / 2) * 2) + ((menu.submenu_pos + 1) % 2);}  // ((pos / width) * width) + ((pos + width - 1) % width) -- Get row number, get item at start of row number, get next width looping on width
  }
}

/******************************************************************************
 ** Function name:  menu_intro
 **
 ** Description:    Boot intro screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_intro (void)
{
  lcd_putstring(0,0, "**  UWS WSC 2015  **");
  lcd_putstring(1,0, EROW);
  lcd_putstring(2,0, "  UNLIMITED Driver  ");
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
 ** Function name:  menu_info
 **
 ** Description:    Car information screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_info (void)
{
  _lcd_putTitle("-INFO-");
  lcd_putstring(1,0, "UNLIMITED Dash2.0   ");
  lcd_putstring(2,0, "HW Version: 2.0     ");
  lcd_putstring(3,0, "SW Version: 2.0.2   ");
}

/******************************************************************************
 ** Function name:  menu_escBus
 **
 ** Description:    Data screen for precharge
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_escBus (void) // likely to remove
{
  char buffer[20];
  int len;

  _lcd_putTitle("-ESC BUS-");

  len = sprintf(buffer, "BUS VOLTAGE: %03.0f V", esc.Bus_V);
  lcd_putstring(1,0, buffer);
  if(len<20){_lcd_padding(1, len, 20 - len);}

  len = sprintf(buffer, "BAT VOLTAGE: %03lu V", BMU.Battery_I);
  lcd_putstring(2,0, buffer);
  if(len<20){_lcd_padding(2, len, 20 - len);}

  lcd_putstring(3,0, EROW);
}

/******************************************************************************
 ** Function name:  menu_home
 **
 ** Description:    Speed, drive, array power, basic errors
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_home (void)
{
  char buffer[20];

  _lcd_putTitle("-HOME-");

  sprintf(buffer, "MPPT: %3luW Drv: ", mppt1.Watts + mppt2.Watts);
  if(STATS_DRV_MODE == SPORTS){sprintf(buffer + 16, "S");}
  else{sprintf(buffer + 16, "E");}

  if(STATS_CR_ACT){sprintf(buffer + 17, "C  ");}
  else if(FORWARD){sprintf(buffer + 17, "D  ");}
  else if(REVERSE){sprintf(buffer + 17, "R  ");}
  else{sprintf(buffer + 17, "N  ");}

  if(rgn_pos){sprintf(buffer + 18, "B");}

  lcd_putstring(1,0, buffer);

  sprintf(buffer, "Bat:  %3luW Thr:", BMU.Watts);
  if(STATS_CR_ACT){sprintf(buffer + 15, "%3.0f%% ", esc.Bus_I * (100 / MAX_ESC_CUR));}
  else if(rgn_pos){sprintf(buffer + 11, "Brk:%3d%% ", rgn_pos/10);}
  else if(FORWARD){sprintf(buffer + 15, "%3d%% ", thr_pos/10);}
  else if(REVERSE){sprintf(buffer + 15, "%3d%% ", thr_pos/10);}
  else{sprintf(buffer + 15, "---%% ");}

  lcd_putstring(2,0, buffer);

  sprintf(buffer, "Motor:%3.0fW Err:", esc.Watts);

  if(esc.ERROR)                                       {sprintf(buffer + 15, "ESC  ");}
  else if(mppt1.flags & 0x28)                         {sprintf(buffer + 15, "MPPT1");}
  else if(mppt2.flags & 0x28)                         {sprintf(buffer + 15, "MPPT2");}
  else if(BMU.Status & 0x00001FBF)                    {sprintf(buffer + 15, "BMU  ");}
  else if(!(mppt1.flags & 0x03 && mppt2.flags & 0x03)){sprintf(buffer + 15, "NoARR");}
  else                                                {sprintf(buffer + 15, " --- ");}
  lcd_putstring(3,0, buffer);
}

/******************************************************************************
 ** Function name:  menu_controls
 **
 ** Description:    Drive details screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_controls (void)
{
  char buffer[20];

  _lcd_putTitle("-CONTROLS-");

  if(FORWARD)     {lcd_putstring(1,0, "MODE:          DRIVE");}
  else if(REVERSE){lcd_putstring(1,0, "MODE:        REVERSE");}
  else            {lcd_putstring(1,0, "MODE:        NEUTRAL");}

  if(!rgn_pos)
  {
    sprintf(buffer, "OUTPUT:       %5.1f%%", drive.current*100);
    lcd_putstring(2,0, buffer);

    sprintf(buffer, "THROTTLE:     %3d.%d%%", thr_pos/10,thr_pos%10);
    lcd_putstring(3,0, buffer);
  }
  else
  {
    sprintf(buffer, "OUTPUT:       %5.1f%%", drive.current*100);
    lcd_putstring(2,0, buffer);

    sprintf(buffer, "REGEN:        %3d.%d%%", rgn_pos/10,rgn_pos%10);
    lcd_putstring(3,0, buffer);
  }
}

/******************************************************************************
 ** Function name:  menu_cruise
 **
 ** Description:    Cruise control screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_cruise (void)
{
  _lcd_putTitle("-CRUISE-");

  if(!REVERSE)
  {
    char buffer[20];

    if(STATS_CR_STS && STATS_CR_ACT)
    {
      lcd_putstring(1,0, " STS:  ON  ACT:  ON ");
      sprintf(buffer, " SET: %3.0f  SPD: %3.0f ", stats.cruise_speed, esc.Velocity_KMH);
      lcd_putstring(2,0, buffer);
      sprintf(buffer, " THR: %3.0f%% ABS: %3.0fA", esc.Bus_I * (100 / MAX_ESC_CUR), esc.Bus_I);
      lcd_putstring(3,0, buffer);

      // Button presses
      if(btn_release_select()){CLR_STATS_CR_ACT;}
      if(btn_release_increment()){stats.cruise_speed += 1;}
      if((stats.cruise_speed > 1) && btn_release_decrement()){stats.cruise_speed -= 1;}
    }
    else if(STATS_CR_STS && !STATS_CR_ACT)
    {
      lcd_putstring(1,0, " STS:  ON  ACT: OFF ");
      sprintf(buffer, " SET: %3.0f  SPD: %3.0f ", stats.cruise_speed, esc.Velocity_KMH);
      lcd_putstring(2,0, buffer);
      lcd_putstring(3,0, EROW);

      // Button presses
      if(btn_release_select()){CLR_STATS_CR_STS;stats.cruise_speed = 0;}
      if((stats.cruise_speed > 1) && btn_release_increment()){SET_STATS_CR_ACT;}
      if(btn_release_decrement()){stats.cruise_speed = esc.Velocity_KMH;SET_STATS_CR_ACT;}

    }
    else if(STATS_CR_ACT && !STATS_CR_STS) // Should never trip, but just in case
    {
      lcd_putstring(1,0, " STS: OFF  ACT:  ON ");
      lcd_putstring(2,0, "    CRUISE ERROR    ");
      lcd_putstring(3,0, "     RESETTING      ");

      CLR_STATS_CR_ACT;
      CLR_STATS_CR_STS;
      stats.cruise_speed = 0;
    }
    else
    {
      lcd_putstring(1,0, " STS: OFF  ACT: OFF ");
      sprintf(buffer, " SET:      SPD: %3.0f ", esc.Velocity_KMH);
      lcd_putstring(2,0, buffer);
      lcd_putstring(3,0, EROW);

      // Button presses
      if(btn_release_select()){stats.cruise_speed = 0;SET_STATS_CR_STS;}
    }
  }
  else // no cruise in reverse
  {
    lcd_putstring(1,0, " STS: OFF  ACT: OFF ");
    lcd_putstring(2,0, "  REVERSE ENGAGED!  ");
    lcd_putstring(3,0, EROW);

    CLR_STATS_CR_ACT;
    CLR_STATS_CR_STS;
    stats.cruise_speed = 0;
  }
}

/******************************************************************************
 ** Function name:  menu_MPPT1
 **
 ** Description:    MPPT1 information screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_MPPT1(void)
{
  _lcd_putTitle("-MPPT 1-");

  if(mppt1.flags & 0x03)
  {
    char buffer[20];
    int len;

    len = sprintf(buffer, "IN: %3lu.%luV @ %2lu.%02luA", mppt1.VIn/10, mppt1.VIn%10, mppt1.IIn/100, mppt1.IIn%100);
    lcd_putstring(1,0, buffer);
    if(len<20){_lcd_padding(1, len, 20 - len);}

    len = sprintf(buffer, "OUT:%3lu.%luV @ %4luW", mppt1.VOut/10, mppt1.VOut%10, mppt1.Watts);
    lcd_putstring(2,0, buffer);
    if(len<20){_lcd_padding(2, len, 20 - len);}

    sprintf(buffer, "%2lu%cC", mppt1.Tmp, 0xB2);
    lcd_putstring(3,16, buffer);

    if(clock.blink)
    {
      if(mppt1.flags & 0x08)      {sprintf(buffer, "OVER TEMP       ");}
      else if(mppt1.flags & 0x20) {sprintf(buffer, "LOW IN VOLTAGE  ");}
      else if(mppt1.flags & 0x04) {sprintf(buffer, "BATTERY FULL    ");}
      else if(mppt1.flags & 0x10) {sprintf(buffer, "NO BATTERY      ");}
      else                        {sprintf(buffer, "                ");}
      lcd_putstring(3,0, buffer);
    }
    else{lcd_putstring(3,0, "                ");}
  }
  else // No connection
  {
    lcd_putstring(1,0, EROW);
    lcd_putstring(3,0, EROW);

    if(clock.blink){lcd_putstring(2,0, "**CONNECTION ERROR**");}
    else{lcd_putstring(2,0, EROW);}
  }
}

/******************************************************************************
 ** Function name:  menu_MPPT2
 **
 ** Description:    MPPT2 information screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_MPPT2(void)
{
  _lcd_putTitle("-MPPT 2-");

  if(mppt2.flags & 0x03)
  {
    char buffer[20];
    int len;

    len = sprintf(buffer, "IN: %3lu.%luV @ %2lu.%02luA", mppt2.VIn/10, mppt2.VIn%10, mppt2.IIn/100, mppt2.IIn%100);
    lcd_putstring(1,0, buffer);
    if(len<20){_lcd_padding(1, len, 20 - len);}

    len = sprintf(buffer, "OUT:%3lu.%luV @ %4luW", mppt2.VOut/10, mppt2.VOut%10, mppt2.Watts);
    lcd_putstring(2,0, buffer);
    if(len<20){_lcd_padding(2, len, 20 - len);}

    sprintf(buffer, "%2lu%cC", mppt2.Tmp, 0xD2);
    lcd_putstring(3,16, buffer);

    if(clock.blink)
    {
      if(mppt2.flags & 0x08)      {sprintf(buffer, "OVER TEMP       ");}
      else if(mppt2.flags & 0x20) {sprintf(buffer, "LOW IN VOLTAGE  ");}
      else if(mppt2.flags & 0x04) {sprintf(buffer, "BATTERY FULL    ");}
      else if(mppt2.flags & 0x10) {sprintf(buffer, "NO BATTERY      ");}
      else                        {sprintf(buffer, "                ");}
      lcd_putstring(3,0, buffer);
    }
    else{lcd_putstring(3,0, "                ");}
  }
  else // No connection
  {
    lcd_putstring(1,0, EROW);
    lcd_putstring(3,0, EROW);

    if(clock.blink){lcd_putstring(2,0, "**CONNECTION ERROR**");}
    else{lcd_putstring(2,0, EROW);}
  }
}

/******************************************************************************
 ** Function name:  menu_MPPTPower
 **
 ** Description:    Total power from MPPTs
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_MPPTPower (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-POWER IN-");

  len = sprintf(buffer, "MPPT1: %.2f Whrs", mppt1.WattHrs);
  lcd_putstring(1,0, buffer);
  if(len<20){_lcd_padding(1, len, 20 - len);}

  len = sprintf(buffer, "MPPT2: %.2f Whrs", mppt2.WattHrs);
  lcd_putstring(2,0, buffer);
  if(len<20){_lcd_padding(2, len, 20 - len);}

  len = sprintf(buffer, "TOTAL: %.2f Whrs", mppt1.WattHrs + mppt2.WattHrs);
  lcd_putstring(3,0, buffer);
  if(len<20){_lcd_padding(3, len, 20 - len);}

  if(btn_release_inc_sel() == 3)
  {
      mppt1.WattHrs = 0;
      mppt2.WattHrs = 0;
      buzzer(50);
  }
}

/******************************************************************************
 ** Function name:  menu_motor
 **
 ** Description:    Motor stats screens
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_motor (void)
{
  char buffer[20];
  int len;
  menu.submenu_items = 3;

  switch(menu.submenu_pos)
  {
    default:
      menu.submenu_pos = 0;
    case 0:
      _lcd_putTitle("-MTR PWR-");

      len = sprintf(buffer, "%5.1fV @ %5.1fA", esc.Bus_V, esc.Bus_I);
      lcd_putstring(1,0, buffer);
      if(len<20){_lcd_padding(1, len, 20 - len);}

      len = sprintf(buffer, "TOTAL: %.2fW",  esc.Watts);
      lcd_putstring(2,0, buffer);
      if(len<20){_lcd_padding(2, len, 20 - len);}
      break;
    case 1:
      _lcd_putTitle("-PWR USED-");

      len = sprintf(buffer, "ESC: %.2f W/hrs", esc.WattHrs);
      lcd_putstring(1,0, buffer);
      if(len<20){_lcd_padding(1, len, 20 - len);}

      lcd_putstring(2,0, EROW);
      break;
    case 2:
      _lcd_putTitle("-MTR PKS-");

      len = sprintf(buffer, "%5.1fV @ %5.1fA", esc.MAX_Bus_V, esc.MAX_Bus_I);
      lcd_putstring(1,0, buffer);
      if(len<20){_lcd_padding(1, len, 20 - len);}

      len = sprintf(buffer, "TOTAL: %.2fW",  esc.MAX_Watts);
      lcd_putstring(2,0, buffer);
      if(len<20){_lcd_padding(2, len, 20 - len);}

      if(btn_release_select())
      {
        esc.MAX_Bus_I = 0;
        esc.MAX_Bus_V = 0;
        esc.MAX_Watts = 0;
        buzzer(50);
      }
      break;
  }
  len = sprintf(buffer, "ERROR: %d", esc.ERROR);
  lcd_putstring(3,0, buffer);
  if(len<20){_lcd_padding(3, len, 20 - len);}

  /// BUTTONS
  if(btn_release_increment()){menu_inc(&menu.submenu_pos, menu.submenu_items);}

  if(btn_release_decrement()){menu_dec(&menu.submenu_pos, menu.submenu_items);}
}

/******************************************************************************
 ** Function name:  menu_debug
 **
 ** Description:    Bus debug screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_debug (void)
{// TODO: TEAM - Update field names
  char buffer[20];
  int len;

  _lcd_putTitle("-DEBUG-");

  len = sprintf(buffer, "%5.1fWh %5.1fWh", shunt.WattHrs, BMU.WattHrs);
  lcd_putstring(1,0, buffer);
  if(len<20){_lcd_padding(1,len, 20 - len);}

  len = sprintf(buffer, "%5.1fA  %5.1fA", shunt.BusI, BMU.Battery_I);
  lcd_putstring(2,0, buffer);
  if(len<20){_lcd_padding(1,len, 20 - len);}

  len = sprintf(buffer, "%5.1fV  %5.1fV", shunt.BusV, BMU.Battery_V);
  lcd_putstring(3,0, buffer);
  if(len<20){_lcd_padding(1,len, 20 - len);}
/*
  if(btn_release_inc_sel() == 3)
  {
      BMU.WattHrs = 0;
      buzzer(50);
  }*/
}

/******************************************************************************
 ** Function name:  menu_config
 **
 ** Description:    Compiler configurations screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_config (void)
{
  char buffer[20];
  static uint8_t scroll = 0;
  menu.submenu_items = 4;

  if(!MENU_SELECTED)
  {
    _lcd_putTitle("-CONFIG-");
    lcd_putstring(1,0, "   CAN      PADDLES ");
    lcd_putstring(2,0, "   OTHER    LIMITS  ");
    lcd_putstring(3,0, EROW);

    switch(menu.submenu_pos)
    {
      default:
      case 0:
        lcd_putstring(1,0, SELECTOR);
        lcd_putstring(1,9, DESELECTOR);
        lcd_putstring(2,0, DESELECTOR);
        lcd_putstring(2,9, DESELECTOR);
        break;
      case 1:
        lcd_putstring(1,0, DESELECTOR);
        lcd_putstring(1,9, DESELECTOR);
        lcd_putstring(2,0, SELECTOR);
        lcd_putstring(2,9, DESELECTOR);
        break;
      case 2:
        lcd_putstring(1,0, DESELECTOR);
        lcd_putstring(1,9, SELECTOR);
        lcd_putstring(2,0, DESELECTOR);
        lcd_putstring(2,9, DESELECTOR);
        break;
      case 3:
        lcd_putstring(1,0, DESELECTOR);
        lcd_putstring(1,9, DESELECTOR);
        lcd_putstring(2,0, DESELECTOR);
        lcd_putstring(2,9, SELECTOR);
        break;
    }
  }
  else
  {
    switch(menu.submenu_pos)
    {
      default:
      case 0:
        _lcd_putTitle("-CAN-");
        switch(scroll)
        {
          default:
          case 0:
            sprintf(buffer, "ESC_BASE      %#05x ", ESC_BASE);
            lcd_putstring(1,0, buffer);
            sprintf(buffer, "ESC_CONTROL   %#05x ", ESC_CONTROL);
            lcd_putstring(2,0, buffer);
            sprintf(buffer, "BMU_BASE      %#05x ", BMU_BASE);
            lcd_putstring(3,0, buffer);
            break;
          case 1:
            sprintf(buffer, "DASH_RPLY     %#05x ", DASH_RPLY);
            lcd_putstring(1,0, buffer);
            sprintf(buffer, "DASH_RQST     %#05x ", DASH_RQST);
            lcd_putstring(2,0, buffer);
            sprintf(buffer, "MPPT1_BASE    %#05x ", MPPT1_BASE);
            lcd_putstring(3,0, buffer);
            break;
          case 2:
            sprintf(buffer, "MPPT1_RPLY    %#05x ", MPPT1_RPLY);
            lcd_putstring(1,0, buffer);
            sprintf(buffer, "MPPT2_BASE    %#05x ", MPPT2_BASE);
            lcd_putstring(2,0, buffer);
            sprintf(buffer, "MPPT2_RPLY    %#05x ", MPPT2_RPLY);
            lcd_putstring(3,0, buffer);
            break;
          case 3:
        	sprintf(buffer, "BMU_SHUNT     %#05x ", BMU_SHUNT);
        	lcd_putstring(1,0, buffer);
        	lcd_putstring(2,0, EROW);
        	lcd_putstring(3,0, EROW);
        }
        break;
      case 1:
        _lcd_putTitle("-OTHER-");
        sprintf(buffer, "AUTO_SWOC       %1d   ", AUTO_SWOC);
        lcd_putstring(1,0, buffer);
        sprintf(buffer, "WHEEL D (M)   %5.3f ", WHEEL_D_M);
        lcd_putstring(2,0, buffer);
        lcd_putstring(3,0, EROW);
        break;
        break;
      case 2:
        _lcd_putTitle("-PADDLES-");
        switch(scroll)
        {
          default:
          case 0:
            sprintf(buffer, "MAX_RGN_DZ    %5.2f ", MAX_RGN_DZ);
            lcd_putstring(1,0, buffer);
            sprintf(buffer, "MIN_RGN_DZ    %5.2f ", MIN_RGN_DZ);
            lcd_putstring(2,0, buffer);
            sprintf(buffer, "MAX_THR_DZ    %5.2f ", MAX_THR_DZ);
            lcd_putstring(3,0, buffer);
            break;
          case 1:
            sprintf(buffer, "MIN_THR_DZ    %5.2f ", MIN_THR_DZ);
            lcd_putstring(1,0, buffer);
            sprintf(buffer, "LOW_PAD_V     %5.2f ", LOW_PAD_V);
            lcd_putstring(2,0, buffer);
            sprintf(buffer, "MID_PAD_V     %5.2f ", MID_PAD_V);
            lcd_putstring(3,0, buffer);
            break;
          case 2:
            sprintf(buffer, "HGH_PAD_V     %5.2f ", HGH_PAD_V);
            lcd_putstring(1,0, buffer);
            sprintf(buffer, "ADC/V         %5.0f ", ADC_POINTS_PER_V);
            lcd_putstring(2,0, buffer);
            lcd_putstring(3,0, EROW);
            break;
        }
        break;
          case 3:
        _lcd_putTitle("-LIMITS-");
        switch(scroll)
        {
          default:
          case 0:
            sprintf(buffer, "MAX_ESC_CUR   %5.1f ", MAX_ESC_CUR);
            lcd_putstring(1,0, buffer);
            sprintf(buffer, "MAX_REGEN     %5.1f ", MAX_REGEN);
            lcd_putstring(2,0, buffer);
            sprintf(buffer, "MAX_THR_DISP  %5.1f ", MAX_THR_DISP);
            lcd_putstring(3,0, buffer);
            break;
          case 1:
            sprintf(buffer, "MAX_THR_LOW   %5.1f ", MAX_THR_LOWSPD);
            lcd_putstring(1,0, buffer);
            sprintf(buffer, "LOWSPD_THRES  %5.1f ", LOWSPD_THRES);
            lcd_putstring(2,0, buffer);
            lcd_putstring(3,0, EROW);
            break;
        }
        break;
    }
  }

  if(btn_release_select())
  {
    if(MENU_SELECTED){CLR_MENU_SELECTED;}
    else{scroll = 0;SET_MENU_SELECTED;}
  }

  if(btn_release_increment())
  {
    if(MENU_SELECTED)
    {
      switch(menu.submenu_pos)
      {
        default:
        case 0:
          menu_dec(&scroll, 4);
          break;
        case 1:
          break;
        case 2:
          menu_dec(&scroll, 3);
          break;
        case 3:
          menu_dec(&scroll, 2);
          break;
      }
    }
    else{menu_dec(&menu.submenu_pos, menu.submenu_items);}
  }

  if(btn_release_decrement())
  {
    if(MENU_SELECTED)
    {
      switch(menu.submenu_pos)
      {
        default:
        case 0:
          menu_inc(&scroll, 4);
          break;
        case 1:
          break;
        case 2:
          menu_inc(&scroll, 3);
          break;
        case 3:
          menu_inc(&scroll, 2);
          break;
      }
    }
    else{menu_inc(&menu.submenu_pos, menu.submenu_items);}
  }
}


/******************************************************************************
 ** Function name:  menu_errors
 **
 ** Description:    Error display screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_errors (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-FAULTS-");

  len = sprintf(buffer, "ESC: %d", esc.ERROR);
  lcd_putstring(1,0, buffer);
  if(len>20){_lcd_padding(1,len, 20 - len);}

  sprintf(buffer, "MPPT: %#05x", ((mppt2.flags & 0x03 ? 1 : 0) << 9)|((mppt1.flags & 0x03 ? 1 : 0) << 8)|(mppt1.flags & 0x3C << 2)|((mppt2.flags & 0x3C) >> 2));
  lcd_putstring(2,0, buffer);

  sprintf(buffer, "BMU: %lu", BMU.Status);
  lcd_putstring(3,0, buffer);

  if(btn_release_select() && esc.ERROR)
  {
    sprintf(buffer, "RESET MOTOR CONTROLS");
    lcd_putstring(1,0, buffer);
    lcd_putstring(2,0, buffer);
    lcd_putstring(3,0, buffer);

    if((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      esc_reset();
      buzzer(50);
    }
  }
}

/******************************************************************************
 ** Function name:  menu_options
 **
 ** Description:    Other options on this screen.
 **                 Buzzer and driver settings
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_options (void)
{
  char buffer[20];
  int len;
  menu.submenu_items = 3;

  _lcd_putTitle("-OPTIONS-");

  if (MENU_SELECTED || (clock.blink))
  {
    switch(menu.submenu_pos)
    {
      default:
        menu.submenu_pos = 0;
      case 0:
        if(STATS_BUZZER){lcd_putstring(1,0, ">> BUZZER: ON       ");}
        else{lcd_putstring(1,0, ">> BUZZER: OFF      ");}
        len = sprintf(buffer, "   DRIVER: %d", menu.driver);
        lcd_putstring(2,0, buffer);
        if(len<20){_lcd_padding(2,len, 20 - len);}
        len = sprintf(buffer, "   PADDLES: %d", stats.paddle_mode);
        lcd_putstring(3,0, buffer);
        if(len<20){_lcd_padding(3,len, 20 - len);}
        break;
      case 1:
        if(STATS_BUZZER){lcd_putstring(1,0, "   BUZZER: ON       ");}
        else{lcd_putstring(1,0, "   BUZZER: OFF      ");}
        len = sprintf(buffer, ">> DRIVER: %d", menu.driver);
        lcd_putstring(2,0, buffer);
        if(len<20){_lcd_padding(2,len, 20 - len);}
        len = sprintf(buffer, "   PADDLES: %d", stats.paddle_mode);
        lcd_putstring(3,0, buffer);
        if(len<20){_lcd_padding(3,len, 20 - len);}
        break;
      case 2:
        if(STATS_BUZZER){lcd_putstring(1,0, "   BUZZER: ON       ");}
        else{lcd_putstring(1,0, "   BUZZER: OFF      ");}
        len = sprintf(buffer, "   DRIVER: %d", menu.driver);
        lcd_putstring(2,0, buffer);
        if(len<20){_lcd_padding(2,len, 20 - len);}
        len = sprintf(buffer, ">> PADDLES: %d", stats.paddle_mode);
        lcd_putstring(3,0, buffer);
        if(len<20){_lcd_padding(3,len, 20 - len);}
        break;
    }
  }
  else
  {
    if(STATS_BUZZER){lcd_putstring(1,0, "   BUZZER: ON       ");}
    else{lcd_putstring(1,0, "   BUZZER: OFF      ");}
    len = sprintf(buffer, "   DRIVER: %d", menu.driver);
    lcd_putstring(2,0, buffer);
    if(len<20){_lcd_padding(2,len, 20 - len);}
    len = sprintf(buffer, "   PADDLES: %d", stats.paddle_mode);
    lcd_putstring(3,0, buffer);
    if(len<20){_lcd_padding(3,len, 20 - len);}
  }


  /////////////////////////////   ACTIONS   //////////////////////////////
  if(btn_release_select())
  {
    if(MENU_SELECTED)
    {
      switch(menu.submenu_pos)
      {
        case 0:
          EE_write(ADD_BUZZ, STATS_BUZZER);
          break;
        case 1:
          menu_init();
          break;
      }
      CLR_MENU_SELECTED;
    }
    else{SET_MENU_SELECTED;}
  }

  if(btn_release_increment())
  {
    if(MENU_SELECTED)
    {
      switch(menu.submenu_pos)
      {
        case 0:
          stats.flags ^= 0x02;
          break;
        case 1:
          menu.driver = (menu.driver + 1) % 4;
          break;
        case 2:
          stats.paddle_mode = (stats.paddle_mode + 1) % 3;
          break;
        default:
          break;
      }
    }
    else{menu_inc(&menu.submenu_pos, menu.submenu_items);}
  }

  if(btn_release_decrement())
  {
    if(MENU_SELECTED)
    {
      switch(menu.submenu_pos)
      {
        case 0:
          stats.flags ^= 0x02;
          break;
        case 1:
          menu.driver = (menu.driver + 3) % 4;
          break;
        case 2:
          stats.paddle_mode = (stats.paddle_mode + 2) % 3;
          break;
        default:
          break;
      }
    }
    else{menu_dec(&menu.submenu_pos, menu.submenu_items);}
  }
}


/******************************************************************************
 ** Function name:  menu_peaks
 **
 ** Description:    Car peaks screen
 **                 1. Array power
 **                 2. Top speed
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_peaks (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-DATA PKS-");

  len = sprintf(buffer, "ARRAY: %4lu Watts", mppt1.MAX_Watts + mppt2.MAX_Watts);
  lcd_putstring(2,0, buffer);
  if(len<20){_lcd_padding(2, len, 20 - len);}

  len = sprintf(buffer, "TOP SPD: %3.1f kmh", stats.max_speed);
  lcd_putstring(3,0, buffer);
  if(len<20){_lcd_padding(3, len, 20 - len);}

  if(btn_release_select())
  {
    mppt1.MAX_Watts = 0;
    mppt2.MAX_Watts = 0;
    stats.max_speed = 0;
    buzzer(50);
  }
}


/******************************************************************************
 ** Function name:  menu_runtime
 **
 ** Description:    Displays car's current runtime
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_runtime (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-RUNTIME-");

  len = sprintf(buffer, "Counter: %lu", menu.counter);
  lcd_putstring(1,0, buffer);
  if(len<20){_lcd_padding(1, len, 20 - len);}

  len = sprintf(buffer, "%luD %02dhr", clock.T_D, clock.T_H);
  lcd_putstring(2,0, buffer);
  if(len<20){_lcd_padding(2, len, 20 - len);}

  len = sprintf(buffer, "%02dm %02d.%01ds", clock.T_M, clock.T_S, clock.T_mS/10);
  lcd_putstring(3,0, buffer);
  if(len<20){_lcd_padding(3, len, 20 - len);}
}

/******************************************************************************
 ** Function name:  menu_odometer
 **
 ** Description:    Displays odometer
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_odometer (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-ODOMETER-");

  len = sprintf(buffer, "CAR: %.3f KM", stats.odometer);
  lcd_putstring(1,0, buffer);
  if(len<20){_lcd_padding(1, len, 20 - len);}

  len = sprintf(buffer, "ESC: %.3f KM", esc.Odometer/1000);
  lcd_putstring(2,0, buffer);
  if(len<20){_lcd_padding(2, len, 20 - len);}

  len = sprintf(buffer, "TRP: %.3f KM", stats.odometer_tr);
  lcd_putstring(3,0, buffer);
  if(len<20){_lcd_padding(3, len, 20 - len);}

  if(btn_release_select()){stats.odometer_tr = 0;buzzer(50);}

  if(btn_release_inc_dec() == 3)
  {
      stats.odometer = 0;
      stats.odometer_tr = 0;
      buzzer(50);
  }
}
///////////////////////////////////////////////

///////////////////////////////////////////////
/// errors array

/******************************************************************************
 ** Function name:  menu_SWOC
 **
 ** Description:    Display screen for SWOC error
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_swoc (void) // errors[0]
{
  _lcd_putTitle("-SWOC ERR-");
  lcd_putstring(1,0, "*******ERROR!*******");
  lcd_putstring(2,0, "PRESS SELECT 2 RESET");
  lcd_putstring(3,0, "PRESS OTHER 2 CANCEL");

  // BUTTONS
  if(btn_release_select())
  {
    if((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      esc_reset();
      buzzer(20);
    }
  }
  if(btn_release_inc_dec()){SET_STATS_SWOC_ACK;}
}

/******************************************************************************
 ** Function name:  menu_HWOC
 **
 ** Description:    Display screen for HWOC error
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_hwoc (void) // errors[1]
{
  _lcd_putTitle("-HWOC ERR-");
  lcd_putstring(1,0, "*******ERROR!*******");
  lcd_putstring(2,0, "PRESS SELECT 2 RESET");
  lcd_putstring(3,0, "PRESS OTHER 2 CANCEL");

  BUZZER_ON;

  // BUTTONS
  if(btn_release_select())
  {
    if((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      esc_reset();
      BUZZER_OFF;
    }
  }
  if(btn_release_inc_dec()){SET_STATS_HWOC_ACK;}
}

/******************************************************************************
 ** Function name:  menu_COMMS
 **
 ** Description:    Display screen for COMMs check
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_comms (void) // errors[2]
{
  _lcd_putTitle("-COMMS-");
  lcd_putstring(1,0, "    CHECK  COMMS    ");
  lcd_putstring(2,0, "SELECT: RADIO WORKS ");
  lcd_putstring(3,0, "OTHER:  NO RESPONSE ");

  if(btn_release_select())
  {
    if((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      MsgBuf_TX1.Frame = 0x00010000;  // 11-bit, no RTR, DLC is 1 byte
      MsgBuf_TX1.MsgID = DASH_RPLY + 1;
      MsgBuf_TX1.DataA = 0xFF;
      MsgBuf_TX1.DataB = 0x0;
      CAN1_SendMessage( &MsgBuf_TX1 );
    }
    CLR_STATS_COMMS;
  }

  if(btn_release_inc_dec())
  {
    if((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      MsgBuf_TX1.Frame = 0x00010000;  // 11-bit, no RTR, DLC is 1 byte
      MsgBuf_TX1.MsgID = DASH_RPLY + 1;
      MsgBuf_TX1.DataA = 0x0;
      MsgBuf_TX1.DataB = 0x0;
      CAN1_SendMessage( &MsgBuf_TX1 );
    }
    CLR_STATS_COMMS;
  }
}

/******************************************************************************
 ** Function name:  menu_battery_error
 **
 ** Description:    Display when battery overheating
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_battery_error (void) // errors[3]
{
  _lcd_putTitle("-BAT OVT-");
  lcd_putstring(1,0, "    **WARNING!**    ");
  lcd_putstring(2,0, "  BATTERY OVERTEMP  ");
  lcd_putstring(3,0, "    **WARNING!**    ");
}
///////////////////////////////////////////////

/******************************************************************************
 ** Function name:  _lcd_putTitle
 **
 ** Description:    Used to place the screen title and current car speed on
 **                 top line of LCD. Will truncate titles with more than
 **                 10 characters.
 **
 ** Parameters:     Address of char array with title string (10 character max)
 ** Returned value: None
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

  sprintf(spd, " %5.1fkmh ", esc.Velocity_KMH);

  for (;bufadd != buffer + 20; bufadd++)
  {
    *bufadd = *spdadd;
    spdadd++;
  }
  lcd_putstring(0, 0, buffer);
}

/******************************************************************************
 ** Function name:  _lcd_padding
 **
 ** Description:    Places blank chars at a location
 **
 ** Parameters:     1. Row (0-3)
 **                 2. Position (0-19)
 **                 3. Length to clear
 ** Returned value: None
 **
 ******************************************************************************/
void _lcd_padding (int row, int pos, int len)
{
  char buffer[len];
  sprintf(buffer, "%*s", len, "");
  lcd_putstring(row,pos, buffer);
}


/******************************************************************************
 ** Function name:  _buffer_rotate_right
 **
 ** Description:    Rotates characters in buffer to the right
 **
 ** Parameters:     1. Address of buffer/string
 **                 2. Length of buffer
 ** Returned value: None
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
 ** Function name:  _buffer_rotate_left
 **
 ** Description:    Rotates characters in buffer to the left
 **
 ** Parameters:     1. Address of buffer/string
 **                 2. Length of buffer
 ** Returned value: None
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
 ** Function name:  menu_inc
 **
 ** Description:    Increments menu selection by 1. Will loop to first item
 **
 ** Parameters:     1. Address of menu to increment
 **                 2. Total items in menu
 ** Returned value: None
 **
 ******************************************************************************/
void menu_inc (uint8_t *_pos, uint8_t _total)
{*_pos = (*_pos + 1) % _total;}

/******************************************************************************
 ** Function name:  menu_dec
 **
 ** Description:    Decrements menu selection by 1. Will loop to last item
 **
 ** Parameters:     1. Address of menu to decrement
 **                 2. Total items in menu
 ** Returned value: None
 **
 ******************************************************************************/
void menu_dec (uint8_t *_pos, uint8_t _total)
{*_pos = (*_pos + _total - 1) % _total;}


/******************************************************************************
 ** Function name:  menu_init
 **
 ** Description:    Initialize menu arrays
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void menu_init (void)
{
  menu.errors[0] = menu_swoc;
  menu.errors[1] = menu_hwoc;
  menu.errors[2] = menu_comms;
  menu.errors[3] = menu_battery_error;

  switch (menu.driver)
  {
    case 0: // DISPLAY
      menu.menu_items = 11;
      menu.menus[0] = menu_home;
      menu.menus[1] = menu_controls;
      menu.menus[2] = menu_MPPT1;
      menu.menus[3] = menu_MPPT2;
      menu.menus[4] = menu_MPPTPower;
      menu.menus[5] = menu_motor;
      menu.menus[6] = menu_options;
      menu.menus[7] = menu_peaks;
      menu.menus[8] = menu_runtime;
      menu.menus[9] = menu_odometer;
      menu.menus[10] = menu_info;
      break;
    default:
    case 1: // TEST
      menu.menu_items = 16;
      menu.menus[0] = menu_home;
      menu.menus[1] = menu_controls;
      menu.menus[2] = menu_cruise;
      menu.menus[3] = menu_MPPT1;
      menu.menus[4] = menu_MPPT2;
      menu.menus[5] = menu_MPPTPower;
      menu.menus[6] = menu_motor;
      menu.menus[7] = menu_debug;
      menu.menus[8] = menu_errors;
      menu.menus[9] = menu_options;
      menu.menus[10] = menu_config;
      menu.menus[11] = menu_peaks;
      menu.menus[12] = menu_runtime;
      menu.menus[13] = menu_odometer;
      menu.menus[14] = menu_info;
      menu.menus[15] = menu_escBus;
      break;
    case 2: // RACE 1
      menu.menu_items = 7;
      menu.menus[0] = menu_home;
      menu.menus[1] = menu_controls;
      menu.menus[2] = menu_cruise;
      menu.menus[3] = menu_errors;
      menu.menus[4] = menu_options;
      menu.menus[5] = menu_runtime;
      menu.menus[6] = menu_odometer;
      break;
    case 3: // RACE 2
      menu.menu_items = 7;
      menu.menus[0] = menu_home;
      menu.menus[1] = menu_controls;
      menu.menus[2] = menu_cruise;
      menu.menus[3] = menu_errors;
      menu.menus[4] = menu_options;
      menu.menus[5] = menu_runtime;
      menu.menus[6] = menu_odometer;
      break;
  }

  CLR_MENU_SELECTED;
  menu.submenu_pos = 0;
  menu.menu_pos = 0; // Initial menu screen
}
