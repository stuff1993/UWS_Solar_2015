/*
 * buttons.c
 *
 *  Created on: Sep 6, 2015
 *      Author: Stuff
 */

#include "buttons.h"
#include "menu.h"

/******************************************************************************
 ** Function name:    btn_release_select
 **
 ** Description:      Checks for button release (SELECT)
 **
 ** Parameters:     None
 ** Returned value:   If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_select(void)
{
  if(SELECT || MENU_SEL_DWN){
    if(!SELECT && MENU_SEL_DWN){CLR_MENU_SEL_DWN;return 1;}
    else{SET_MENU_SEL_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:    btn_release_increment
 **
 ** Description:      Checks for button release (INCREMENT)
 **
 ** Parameters:     None
 ** Returned value:   If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_increment(void)
{
  if(INCREMENT || MENU_INC_DWN){
    if(!INCREMENT && MENU_INC_DWN){CLR_MENU_INC_DWN;return 1;}
    else{SET_MENU_INC_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:    btn_release_decrement
 **
 ** Description:      Checks for button release (DECREMENT)
 **
 ** Parameters:     None
 ** Returned value:   If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_decrement(void)
{
  if(DECREMENT || MENU_DEC_DWN){
    if(!DECREMENT && MENU_DEC_DWN){CLR_MENU_DEC_DWN;return 1;}
    else{SET_MENU_DEC_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:    btn_release_left
 **
 ** Description:      Checks for button release (LEFT)
 **
 ** Parameters:     None
 ** Returned value:   If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_left(void)
{
  if(LEFT || MENU_LEFT_DWN){
    if(!SELECT && MENU_LEFT_DWN){CLR_MENU_LEFT_DWN;return 1;}
    else{SET_MENU_LEFT_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:    btn_release_right
 **
 ** Description:      Checks for button release (RIGHT)
 **
 ** Parameters:     None
 ** Returned value:   If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_right(void)
{
  if(RIGHT || MENU_RIGHT_DWN){
    if(!SELECT && MENU_RIGHT_DWN){CLR_MENU_RIGHT_DWN;return 1;}
    else{SET_MENU_RIGHT_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:    btn_release_inc_dec
 **
 ** Description:      Checks for button release (INCREMENT & DECREMENT)
 **
 ** Parameters:     None
 ** Returned value:   Button Status
 **                   Value:
 **                     1 - Both buttons pressed and released
 **                     2 - INCREMENT pressed and released
 **                     4 - DECREMENT pressed and released
 **
 ******************************************************************************/
uint8_t btn_release_inc_dec(void)
{
  if(INCREMENT || DECREMENT || MENU_INC_DWN || MENU_DEC_DWN)
  {
    if(!(INCREMENT || DECREMENT) && (MENU_INC_DWN && MENU_DEC_DWN))
    {
      CLR_MENU_INC_DWN;
      CLR_MENU_DEC_DWN;
      return 1;
    }
    else if(INCREMENT && !DECREMENT)
    {
      SET_MENU_INC_DWN;
    }
    else if(!INCREMENT && DECREMENT)
    {
      SET_MENU_DEC_DWN;
    }
    else if(INCREMENT && DECREMENT)
    {
      SET_MENU_INC_DWN;
      SET_MENU_DEC_DWN;
    }
    else if(!INCREMENT && MENU_INC_DWN)
    {
      CLR_MENU_INC_DWN;
      return 2;
    }
    else if(!DECREMENT && MENU_DEC_DWN)
    {
      CLR_MENU_DEC_DWN;
      return 4;
    }
  }
  return 0;
}

/******************************************************************************
 ** Function name:    btn_release_inc_sel
 **
 ** Description:      Checks for button release (INCREMENT & SELECT)
 **
 ** Parameters:       None
 ** Returned value:   Button Status
 **                   Value:
 **                     1 - Both buttons pressed and released
 **                     2 - INCREMENT pressed and released
 **                     4 - SELECT pressed and released
 **
 ******************************************************************************/
uint8_t btn_release_inc_sel(void)
{
  if(INCREMENT || SELECT || MENU_INC_DWN || MENU_SEL_DWN)
  {
    if(!(INCREMENT || SELECT) && (MENU_INC_DWN && MENU_SEL_DWN))
    {
      CLR_MENU_SEL_DWN;
      CLR_MENU_INC_DWN;
      return 1;
    }
    else if(INCREMENT && !SELECT)
    {
      SET_MENU_INC_DWN;
    }
    else if(!INCREMENT && SELECT)
    {
      SET_MENU_SEL_DWN;
    }
    else if(INCREMENT && SELECT)
    {
      SET_MENU_SEL_DWN;
      SET_MENU_INC_DWN;
    }
    else if(!INCREMENT && MENU_INC_DWN)
    {
      CLR_MENU_INC_DWN;
      return 2;
    }
    else if(!SELECT && MENU_SEL_DWN)
    {
      CLR_MENU_SEL_DWN;
      return 4;
    }
  }
  return 0;
}

/******************************************************************************
 ** Function name:    btn_release_dec_sel
 **
 ** Description:      Checks for button release (DECREMENT & SELECT)
 **
 ** Parameters:       None
 ** Returned value:   Button Status
 **                   Value:
 **                     1 - Both buttons pressed and released
 **                     2 - DECREMENT pressed and released
 **                     4 - SELECT pressed and released
 **
 ******************************************************************************/
uint8_t btn_release_dec_sel(void)
{
  if(DECREMENT || SELECT || MENU_DEC_DWN || MENU_SEL_DWN)
  {
    if(!(DECREMENT || SELECT) && (MENU_DEC_DWN && MENU_SEL_DWN))
    {
      CLR_MENU_SEL_DWN;
      CLR_MENU_DEC_DWN;
      return 1;
    }
    else if(DECREMENT && !SELECT)
    {
      SET_MENU_DEC_DWN;
    }
    else if(!DECREMENT && SELECT)
    {
      SET_MENU_SEL_DWN;
    }
    else if(DECREMENT && SELECT)
    {
      SET_MENU_SEL_DWN;
      SET_MENU_DEC_DWN;
    }
    else if(!DECREMENT && MENU_DEC_DWN)
    {
      CLR_MENU_DEC_DWN;
      return 2;
    }
    else if(!SELECT && MENU_SEL_DWN)
    {
      CLR_MENU_SEL_DWN;
      return 4;
    }
  }
  return 0;
}

/******************************************************************************
 ** Function name:    btn_release_left_right
 **
 ** Description:      Checks for button release (LEFT & RIGHT)
 **
 ** Parameters:       None
 ** Returned value:   Button Status
 **                   Value:
 **                     1 - Both buttons pressed and released
 **                     2 - LEFT pressed and released
 **                     4 - RIGHT pressed and released
 **
 ******************************************************************************/
uint8_t btn_release_left_right(void)
{
  if(LEFT || RIGHT || MENU_LEFT_DWN || MENU_RIGHT_DWN)
  {
    if(!(LEFT || RIGHT) && (MENU_LEFT_DWN && MENU_RIGHT_DWN))
    {
      CLR_MENU_RIGHT_DWN;
      CLR_MENU_LEFT_DWN;
      return 1;
    }
    else if(LEFT && !RIGHT)
    {
      SET_MENU_LEFT_DWN;
    }
    else if(!LEFT && RIGHT)
    {
      SET_MENU_RIGHT_DWN;
    }
    else if(LEFT && RIGHT)
    {
      SET_MENU_RIGHT_DWN;
      SET_MENU_LEFT_DWN;
    }
    else if(!LEFT && MENU_DEC_DWN)
    {
      CLR_MENU_LEFT_DWN;
      return 2;
    }
    else if(!RIGHT && MENU_SEL_DWN)
    {
      CLR_MENU_RIGHT_DWN;
      return 4;
    }
  }
  return 0;
}

/******************************************************************************
 ** Function name:    swt_cruise
 **
 ** Description:      Checks for switch status (CRUISE)
 **
 ** Parameters:       None
 ** Returned value:   Switch Status
 **                   Value:
 **                     0 - Switch centred
 **                     1 - Held up
 **                     2 - Held down
 **                     4 - Released from up
 **                     8 - Released from down
 **
 ******************************************************************************/
uint8_t swt_cruise(void)
{
  // TODO: Implement
  return 0;
}
