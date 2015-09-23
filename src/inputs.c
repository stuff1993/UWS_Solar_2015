/*
 * inputs.c
 *
 *  Created on: Sep 6, 2015
 *      Author: Stuart G
 */

#include "inputs.h"
#include "dash.h"

/******************************************************************************
 ** Function name:  btn_release_select
 **
 ** Description:    Checks for button release (SELECT)
 **
 ** Parameters:     None
 ** Returned value: If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_select(void)
{
  if(SELECT || INPUTS_SEL_DWN){
    if(!SELECT && INPUTS_SEL_DWN){CLR_INPUTS_SEL_DWN;return 1;}
    else{SET_INPUTS_SEL_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:  btn_release_increment
 **
 ** Description:    Checks for button release (INCREMENT)
 **
 ** Parameters:     None
 ** Returned value: If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_increment(void)
{
  if(INCREMENT || INPUTS_INC_DWN){
    if(!INCREMENT && INPUTS_INC_DWN){CLR_INPUTS_INC_DWN;return 1;}
    else{SET_INPUTS_INC_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:  btn_release_decrement
 **
 ** Description:    Checks for button release (DECREMENT)
 **
 ** Parameters:     None
 ** Returned value: If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_decrement(void)
{
  if(DECREMENT || INPUTS_DEC_DWN){
    if(!DECREMENT && INPUTS_DEC_DWN){CLR_INPUTS_DEC_DWN;return 1;}
    else{SET_INPUTS_DEC_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:  btn_release_left
 **
 ** Description:    Checks for button release (LEFT)
 **
 ** Parameters:     None
 ** Returned value: If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_left(void)
{
  if(LEFT || INPUTS_LEFT_DWN){
    if(!LEFT && INPUTS_LEFT_DWN){CLR_INPUTS_LEFT_DWN;return 1;}
    else{SET_INPUTS_LEFT_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:  btn_release_right
 **
 ** Description:    Checks for button release (RIGHT)
 **
 ** Parameters:     None
 ** Returned value: If button has been released since last function call
 **
 ******************************************************************************/
uint8_t btn_release_right(void)
{
  if(RIGHT || INPUTS_RIGHT_DWN){
    if(!RIGHT && INPUTS_RIGHT_DWN){CLR_INPUTS_RIGHT_DWN;return 1;}
    else{SET_INPUTS_RIGHT_DWN;}}
  return 0;
}

/******************************************************************************
 ** Function name:  btn_release_inc_dec
 **
 ** Description:    Checks for button release (INCREMENT & DECREMENT)
 **
 ** Parameters:     None
 ** Returned value: Button Status
 **                 Value:
 **                   1 - INCREMENT pressed and released
 **                   2 - DECREMENT pressed and released
 **                   3 - Both buttons pressed and released
 **
 ******************************************************************************/
uint8_t btn_release_inc_dec(void)
{
  if(INCREMENT || DECREMENT || INPUTS_INC_DWN || INPUTS_DEC_DWN)
  {
    if(!(INCREMENT || DECREMENT) && (INPUTS_INC_DWN && INPUTS_DEC_DWN))
    {
      CLR_INPUTS_INC_DWN;
      CLR_INPUTS_DEC_DWN;
      return 3;
    }
    else if(INCREMENT && !DECREMENT)
    {
      SET_INPUTS_INC_DWN;
    }
    else if(!INCREMENT && DECREMENT)
    {
      SET_INPUTS_DEC_DWN;
    }
    else if(INCREMENT && DECREMENT)
    {
      SET_INPUTS_INC_DWN;
      SET_INPUTS_DEC_DWN;
    }
    else if(!INCREMENT && INPUTS_INC_DWN)
    {
      CLR_INPUTS_INC_DWN;
      return 1;
    }
    else if(!DECREMENT && INPUTS_DEC_DWN)
    {
      CLR_INPUTS_DEC_DWN;
      return 2;
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
 **                     1 - INCREMENT pressed and released
 **                     2 - SELECT pressed and released
 **                     3 - Both buttons pressed and released
 **
 ******************************************************************************/
uint8_t btn_release_inc_sel(void)
{
  if(INCREMENT || SELECT || INPUTS_INC_DWN || INPUTS_SEL_DWN)
  {
    if(!(INCREMENT || SELECT) && (INPUTS_INC_DWN && INPUTS_SEL_DWN))
    {
      CLR_INPUTS_SEL_DWN;
      CLR_INPUTS_INC_DWN;
      return 3;
    }
    else if(INCREMENT && !SELECT)
    {
      SET_INPUTS_INC_DWN;
    }
    else if(!INCREMENT && SELECT)
    {
      SET_INPUTS_SEL_DWN;
    }
    else if(INCREMENT && SELECT)
    {
      SET_INPUTS_SEL_DWN;
      SET_INPUTS_INC_DWN;
    }
    else if(!INCREMENT && INPUTS_INC_DWN)
    {
      CLR_INPUTS_INC_DWN;
      return 1;
    }
    else if(!SELECT && INPUTS_SEL_DWN)
    {
      CLR_INPUTS_SEL_DWN;
      return 2;
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
 **                     1 - DECREMENT pressed and released
 **                     2 - SELECT pressed and released
 **                     3 - Both buttons pressed and released
 **
 ******************************************************************************/
uint8_t btn_release_dec_sel(void)
{
  if(DECREMENT || SELECT || INPUTS_DEC_DWN || INPUTS_SEL_DWN)
  {
    if(!(DECREMENT || SELECT) && (INPUTS_DEC_DWN && INPUTS_SEL_DWN))
    {
      CLR_INPUTS_SEL_DWN;
      CLR_INPUTS_DEC_DWN;
      return 3;
    }
    else if(DECREMENT && !SELECT)
    {
      SET_INPUTS_DEC_DWN;
    }
    else if(!DECREMENT && SELECT)
    {
      SET_INPUTS_SEL_DWN;
    }
    else if(DECREMENT && SELECT)
    {
      SET_INPUTS_SEL_DWN;
      SET_INPUTS_DEC_DWN;
    }
    else if(!DECREMENT && INPUTS_DEC_DWN)
    {
      CLR_INPUTS_DEC_DWN;
      return 1;
    }
    else if(!SELECT && INPUTS_SEL_DWN)
    {
      CLR_INPUTS_SEL_DWN;
      return 2;
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
 **                     1 - LEFT pressed and released
 **                     2 - RIGHT pressed and released
 **                     3 - Both buttons pressed and released
 **
 ******************************************************************************/
uint8_t btn_release_left_right(void)
{
  if(LEFT || RIGHT || INPUTS_LEFT_DWN || INPUTS_RIGHT_DWN)
  {
    if(!(LEFT || RIGHT) && (INPUTS_LEFT_DWN && INPUTS_RIGHT_DWN))
    {
      CLR_INPUTS_RIGHT_DWN;
      CLR_INPUTS_LEFT_DWN;
      return 3;
    }
    else if(LEFT && !RIGHT)
    {
      SET_INPUTS_LEFT_DWN;
    }
    else if(!LEFT && RIGHT)
    {
      SET_INPUTS_RIGHT_DWN;
    }
    else if(LEFT && RIGHT)
    {
      SET_INPUTS_RIGHT_DWN;
      SET_INPUTS_LEFT_DWN;
    }
    else if(!LEFT && INPUTS_LEFT_DWN)
    {
      CLR_INPUTS_LEFT_DWN;
      return 1;
    }
    else if(!RIGHT && INPUTS_RIGHT_DWN)
    {
      CLR_INPUTS_RIGHT_DWN;
      return 2;
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
  if(CC_ON || INPUTS_CRU_UP)
  {
    if(!CC_ON && INPUTS_CRU_UP){CLR_INPUTS_CRU_UP;return 4;}
    else{SET_INPUTS_CRU_UP;return 1;}
  }
  else if(CC_OFF || INPUTS_CRU_DWN)
  {
    if(!CC_OFF && INPUTS_CRU_DWN){CLR_INPUTS_CRU_DWN;return 8;}
    else{SET_INPUTS_CRU_DWN;return 2;}
  }
  return 0;
}
