/*
===============================================================================
 Name        : SPOFS_1.c
 Author      : Stuart Gales
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdint.h>
#include <stdio.h>

#include "type.h"
#include "inttofloat.h"
#include "timer.h"
#include "lcd.h"
#include "adc.h"
#include "i2c.h"
#include "struct.h"
#include "dash.h"
#include "inputs.h"
#include "can.h"
#include "menu.h"

// TODO: MAJOR - naming consistency
// TODO: MAJOR - change I2C to handle multiple words in one write

/////////////////////////////   CAN    ////////////////////////////////
CAN_MSG MsgBuf_TX1, MsgBuf_TX2; /* TX Buffers for CAN messages */
CAN_MSG MsgBuf_RX1, MsgBuf_RX2; /* RX Buffers for CAN messages */
volatile uint32_t CAN1RxDone = FALSE, CAN2RxDone = FALSE;
///////////////////////////////////////////////////////////////////////

/////////////////////////////   I2C    ////////////////////////////////
extern volatile uint8_t I2CMasterBuffer[I2C_PORT_NUM][BUFSIZE];
extern volatile uint32_t I2CWriteLength[I2C_PORT_NUM];
extern volatile uint32_t I2CReadLength[I2C_PORT_NUM];
extern volatile uint8_t I2CSlaveBuffer[I2C_PORT_NUM][BUFSIZE];
extern volatile uint32_t I2CMasterState[I2C_PORT_NUM]; // TODO: For Timeout test
///////////////////////////////////////////////////////////////////////

volatile unsigned char SWITCH_IO  = 0;

uint16_t thr_pos = 0;
uint16_t rgn_pos = 0;

/// MPPTs
MPPT mppt1;
MPPT mppt2;

/// Relayed MPPTs
MPPT_RELAY mppt_relay1;
MPPT_RELAY mppt_relay2;

/// Motor Controller
MOTORCONTROLLER esc;

/******************************************************************************
 ** Function name:  BOD_IRQHandler
 **
 ** Description:    Brown-out detection handler
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void BOD_IRQHandler (void)
{HV_OFF;REVERSE_ON;NEUTRAL_ON;REGEN_ON;DRIVE_ON;FAULT_ON;ECO_ON;SPORTS_ON;}

/******************************************************************************
 ** Function name:  SysTick_Handler
 **
 ** Description:    System clock event handler. Fires every 10mS
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void SysTick_Handler (void)
{
  clock.T_mS++;

  // MinorSec: DIU CAN Heart Beat
  if((!(clock.T_mS % 10)) && STATS_ARMED) // Every 100 mS send heart beat CAN packets
  {
    MsgBuf_TX1.Frame = 0x00080000;
    MsgBuf_TX1.MsgID = ESC_CONTROL + 1;
    MsgBuf_TX1.DataA = conv_float_uint(drive.speed_rpm);
    if(drive.current < 0){MsgBuf_TX1.DataB = conv_float_uint(drive.current * -1.0);}
    else{MsgBuf_TX1.DataB = conv_float_uint(drive.current);}
    CAN1_SendMessage( &MsgBuf_TX1 );

    MsgBuf_TX1.Frame = 0x00080000;
    MsgBuf_TX1.MsgID = ESC_CONTROL + 2;
    MsgBuf_TX1.DataA = 0x0;
    MsgBuf_TX1.DataB = conv_float_uint(1);
    CAN1_SendMessage( &MsgBuf_TX1 );
  }

  if(clock.T_mS / 50){clock.blink = 1;}
  else{clock.blink = 0;}

  if(stats.buz_tim)
  {
    if(!(--stats.buz_tim)){BUZZER_OFF;}
  }

  // MinorSec:  Time sensitive Calculations
  esc.WattHrs += (esc.Watts/360000.0);

  mppt1.WattHrs += (mppt1.Watts/360000.0);
  mppt2.WattHrs += (mppt2.Watts/360000.0);

  BMU.WattHrs += (BMU.Watts/360000.0);

  stats.odometer += esc.Velocity_KMH/360000.0;
  stats.odometer_tr += esc.Velocity_KMH/360000.0;

  if(clock.T_mS >= 100) // Calculate time
  {
    clock.T_mS = 0;clock.T_S++;

    if((mppt1.flags & 0x03)>0){mppt1.flags |= (((mppt1.flags & 0x03) >> 0) - 1) & 0x03;} // if disconnected for 2 seconds. Then FLAG disconnect.
    if((mppt2.flags & 0x03)>0){mppt2.flags |= (((mppt2.flags & 0x03) >> 0) - 1) & 0x03;} // if disconnected for 2 seconds. Then FLAG disconnect.

    persistent_store(); // Store data in eeprom every second

    if(clock.T_S >= 60){clock.T_S = 0;clock.T_M++;
    if(clock.T_M >= 60){clock.T_M = 0;clock.T_H++;
    if(clock.T_H >= 24){clock.T_H = 0;clock.T_D++;}}}
  }
}

/******************************************************************************
 ** Function name:  main_mppt_poll
 **
 ** Description:    1. Sends request packet to MPPT (125K CAN Bus)
 **                 2. Sends previous MPPT packet to car (500K CAN Bus)
 **                 3. Receives new packet and extracts data (125K CAN Bus)
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_mppt_poll (void)
{
  if(STATS_MPPT_POLL){CLR_STATS_MPPT_POLL;} // Toggle bit. Selects which MPPT to poll this round
  else{SET_STATS_MPPT_POLL;}

  // 1. Sends request packet to MPPT (125K CAN Bus)
  if((LPC_CAN2->GSR & (1 << 3)) && STATS_MPPT_POLL) // Check Global Status Reg
  {
    MsgBuf_TX2.MsgID = MPPT2_BASE;
    CAN2_SendMessage( &MsgBuf_TX2 );
  }

  else if(LPC_CAN2->GSR & (1 << 3)) // Check Global Status Reg
  {
    MsgBuf_TX2.MsgID = MPPT1_BASE;
    CAN2_SendMessage( &MsgBuf_TX2 );
  }

  // 2. Sends previous MPPT packet to car (500K CAN Bus)
  if((LPC_CAN1->GSR & (1 << 3)) && STATS_MPPT_POLL) // Check Global Status Reg
  {
    MsgBuf_TX1.Frame = 0x00070000;  // 11-bit, no RTR, DLC is 7 bytes
    MsgBuf_TX1.MsgID = MPPT2_RPLY;
    if (mppt2.flags & 0x03)
    {
      MsgBuf_TX1.DataA = mppt_relay2.DataA;
      MsgBuf_TX1.DataB = mppt_relay2.DataB;
    }
    else
    {
      MsgBuf_TX1.DataA = 0x0;
      MsgBuf_TX1.DataB = 0x0;
    }
    CAN1_SendMessage( &MsgBuf_TX1 );
  }
  else if(LPC_CAN1->GSR & (1 << 3)) // Check Global Status Reg
  {
    MsgBuf_TX1.Frame = 0x00070000;  // 11-bit, no RTR, DLC is 7 bytes
    MsgBuf_TX1.MsgID = MPPT1_RPLY;
    if (mppt1.flags & 0x03)
    {
      MsgBuf_TX1.DataA = mppt_relay1.DataA;
      MsgBuf_TX1.DataB = mppt_relay1.DataB;
    }
    else
    {
      MsgBuf_TX1.DataA = 0x0;
      MsgBuf_TX1.DataB = 0x0;
    }
    CAN1_SendMessage( &MsgBuf_TX1 );
  }

  // 3. Receives new packet and extracts data (125K CAN Bus)
  if ( CAN2RxDone == TRUE )
  {
    CAN2RxDone = FALSE;
    if      (MsgBuf_RX2.MsgID == MPPT1_RPLY){mppt_data_extract(&mppt1, &mppt_relay1);}
    else if (MsgBuf_RX2.MsgID == MPPT2_RPLY){mppt_data_extract(&mppt2, &mppt_relay2);}

    // Reset buffer to prevent packets being received multiple times
    MsgBuf_RX2.Frame = 0x0;
    MsgBuf_RX2.MsgID = 0x0;
    MsgBuf_RX2.DataA = 0x0;
    MsgBuf_RX2.DataB = 0x0;
  }

  // Check mppt connection timeouts - clear instantaneous data
  if(!(mppt1.flags & 0x03))
  {
    mppt1.VIn = 0;
    mppt1.IIn = 0;
    mppt1.VOut = 0;
    mppt1.Watts = 0;

    mppt_relay1.DataA = 0;
    mppt_relay1.DataB = 0;
  }

  if(!(mppt2.flags & 0x03))
  {
    mppt2.VIn = 0;
    mppt2.IIn = 0;
    mppt2.VOut = 0;
    mppt2.Watts = 0;

    mppt_relay2.DataA = 0;
    mppt_relay2.DataB = 0;
  }
}

/******************************************************************************
 ** Function name:  mppt_data_extract
 **
 ** Description:    Extracts data from CAN 2 Rx buffer into MPPT structure.
 **
 ** Parameters:     1. Address of MPPT to extract to
 **                 2. Address of MPPT_RELAY to use for retransmit
 ** Returned value: None
 **
 ******************************************************************************/
void mppt_data_extract (MPPT *_MPPT, MPPT_RELAY *_fkMPPT)
{
  uint32_t _VIn = 0;
  uint32_t _IIn = 0;
  uint32_t _VOut = 0;

  uint32_t _Data_A = MsgBuf_RX2.DataA;
  uint32_t _Data_B = MsgBuf_RX2.DataB;

  _fkMPPT->DataA = _Data_A;
  _fkMPPT->DataB = _Data_B;

  // Status Flags
  _MPPT->flags |= ((_Data_A & 0xF0) >> 2);

  // Power Variables
  _VIn = ((_Data_A & 0b11) << 8);             // Masking and shifting the upper 2 MSB
  _VIn = _VIn + ((_Data_A & 0xFF00) >> 8);    // Masking and shifting the lower 8 LSB
  _VIn = _VIn * 1.50;                         // Scaling

  _Data_A = (_Data_A >> 16);
  _IIn = ((_Data_A & 0b11) << 8);             // Masking and shifting the lower 8 LSB
  _IIn = _IIn + ((_Data_A & 0xFF00) >> 8);    // Masking and shifting the upper 2 MSB
  _IIn = _IIn * 0.87;                         // Scaling

  _VOut = ((_Data_B & 0b11) << 8);            // Masking and shifting the upper 2 MSB
  _VOut = _VOut + ((_Data_B & 0xFF00) >> 8);  // Masking and shifting the lower 8 LSB
  _VOut = _VOut * 2.10;                       // Scaling

  // Update the global variables after IIR filtering
  _MPPT->Tmp = iir_filter_int(((_Data_B & 0xFF0000) >> 16), _MPPT->Tmp, IIR_GAIN_THERMAL);
  _MPPT->VIn = iir_filter_int(_VIn, _MPPT->VIn, IIR_GAIN_ELECTRICAL);
  _MPPT->IIn = iir_filter_int(_IIn, _MPPT->IIn, IIR_GAIN_ELECTRICAL);
  _MPPT->VOut = iir_filter_int(_VOut, _MPPT->VOut, IIR_GAIN_ELECTRICAL);
  _MPPT->flags |= 0x03; // Connection timing bits
}

/******************************************************************************
 ** Function name:  main_input_check
 **
 ** Description:    Checks majority of inputs (Switches, Left, Right)
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_input_check (void)
{
  unsigned char OLD_IO = SWITCH_IO;
  uint8_t btn_ret = 0;

  SWITCH_IO = 0;
  SWITCH_IO |= (FORWARD << 0);
  SWITCH_IO |= (REVERSE << 1);
  SWITCH_IO |= (SPORTS_MODE << 2);
  SWITCH_IO |= (LEFT_ON << 3);
  SWITCH_IO |= (RIGHT_ON << 4);

  if(OLD_IO != SWITCH_IO){buzzer(50);}  // BEEP if toggle position has changed.

  if((btn_ret = btn_release_left_right()))
  {
    buzzer(2);

    if(btn_ret == 3)     {menu.menu_pos = 0;}
    else if(btn_ret == 1){menu_dec(&menu.menu_pos, menu.menu_items);}
    else if(btn_ret == 2){menu_inc(&menu.menu_pos, menu.menu_items);}

    if(menu.menu_pos==0){buzzer(10);}
    if((esc.ERROR & 0x2) && !STATS_SWOC_ACK){SET_STATS_SWOC_ACK;}
    if((esc.ERROR & 0x1) && !STATS_HWOC_ACK){SET_STATS_HWOC_ACK;BUZZER_OFF}
    if(STATS_COMMS == 1)  // send NO RESPONSE packet
    {
      if((LPC_CAN1->GSR & (1 << 3)))  // Check Global Status Reg
      {
        MsgBuf_TX1.Frame = 0x00010000;  // 11-bit, no RTR, DLC is 1 byte
        MsgBuf_TX1.MsgID = DASH_RPLY + 1;
        MsgBuf_TX1.DataA = 0x0;
        MsgBuf_TX1.DataB = 0x0;
        CAN1_SendMessage( &MsgBuf_TX1 );
      }
      CLR_STATS_COMMS;
    }

    lcd_clear();
    inputs.input_dwn = 0;
    menu.submenu_pos = 0;
  }

  if(SWITCH_IO & 0x4) {SET_STATS_DRV_MODE;stats.ramp_speed = SPORTS_RAMP_SPEED;}
  else                {CLR_STATS_DRV_MODE;stats.ramp_speed = ECONOMY_RAMP_SPEED;}
}

/******************************************************************************
 ** Function name:  main_fault_check
 **
 ** Description:    Checks for faults in car components
 **
 ** Parameters:     None
 ** Returned value: Fault status
 **                   0 - No fault
 **                   1 - Non critical fault
 **                   2 - Critical fault - cancel drive
 **
 ******************************************************************************/
int main_fault_check (void)
{
  if(esc.ERROR || (BMU.Status & 0xD37)){drive.current = 0;drive.speed_rpm = 0;return 2;}
  if(mppt1.flags & 0x28 || mppt2.flags & 0x28 || (BMU.Status & 0x1288)){return 1;}
  return 0;
}

/******************************************************************************
 ** Function name:  main_drive
 **
 ** Description:    Reads drive inputs and configures drive packet
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_drive (void)
{
  uint32_t ADC_A;
  uint32_t ADC_B;

  /// THROTTLE
  ADC_A = (ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0))/8;

  /// REGEN
  ADC_B = (ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1))/8;

  main_paddles(ADC_A, ADC_B, &thr_pos, &rgn_pos);

  if(!FORWARD || !menu.driver){buzzer(10);CLR_STATS_CR_ACT;CLR_STATS_CR_STS; stats.cruise_speed = 0;} // Must be in forward or not in display mode to use cruise
  if(rgn_pos || thr_pos){CLR_STATS_CR_ACT;}


  // MinorSec: DRIVE LOGIC
  if(!MECH_BRAKE && (FORWARD || REVERSE)){
    if(STATS_CR_ACT)                                                                                                    {drive.current = 1.0;    drive.speed_rpm = stats.cruise_speed / ((60 * 3.14 * WHEEL_D_M) / 1000.0);}
    else if(!thr_pos && !rgn_pos)                                                                                       {drive.speed_rpm = 0;    drive.current = 0;}
    else if(rgn_pos && drive.current > 0)                                                                               {                        drive.current = 0;}
    else if(rgn_pos && (((drive.current * 1000) + REGEN_RAMP_SPEED) < rgn_pos))                                         {drive.speed_rpm = 0;    drive.current -= (REGEN_RAMP_SPEED / 1000.0);}
    else if(rgn_pos)                                                                                                    {drive.speed_rpm = 0;    drive.current = (rgn_pos / 2);}
    else if(thr_pos && drive.current < 0)                                                                               {                        drive.current = 0;}
    else if(FORWARD && esc.Velocity_KMH > -5.0 && !rgn_pos && (((drive.current * 1000) + stats.ramp_speed) < thr_pos))  {drive.speed_rpm = 1500; drive.current += (stats.ramp_speed / 1000.0);}
    else if(FORWARD && esc.Velocity_KMH > -5.0 && !rgn_pos)                                                             {drive.speed_rpm = 1500; drive.current = (thr_pos / 1000.0);}
    else if(REVERSE && esc.Velocity_KMH <  1.0 && !rgn_pos && (((drive.current * 1000) + stats.ramp_speed) < thr_pos))  {drive.speed_rpm = -200; drive.current += (stats.ramp_speed / 1000.0);}
    else if(REVERSE && esc.Velocity_KMH <  1.0 && !rgn_pos)                                                             {drive.speed_rpm = -200; drive.current = (thr_pos / 1000.0);}
    else{drive.speed_rpm = 0; drive.current = 0;}}
  else{drive.speed_rpm = 0; drive.current = 0;CLR_STATS_CR_ACT;}
}

/******************************************************************************
 ** Function name:  main_paddles
 **
 ** Description:    Takes paddle inputs and returns regen and throttle values
 **                 Mode count here must be reflected in lcd_display_options
 **
 ** Parameters:     1. Paddle 1 ADC Read (Right)
 **                 2. Paddle 2 ADC Read (Left)
 **                 3. Address of throttle variable (Output)
 **                 4. Address of regen variable (Output)
 ** Returned value: None
 **
 ******************************************************************************/
void main_paddles (uint32_t _pad1, uint32_t _pad2, uint16_t *_thr, uint16_t *_rgn)
{
  if(_pad1 > ((HGH_PAD_V + 0.1) * ADC_POINTS_PER_V)){_pad1 = 0;}
  if(_pad2 > ((HGH_PAD_V + 0.1) * ADC_POINTS_PER_V)){_pad2 = 0;}
  switch(stats.paddle_mode)
  {
    default:
    case 0:
      // Throttle - Paddle 1
      _pad1 = (_pad1 < ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) ? 0 : _pad1 - ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
      _pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);
      if(esc.Velocity_KMH < LOWSPD_THRES && _pad1 > MAX_THR_LOWSPD){_pad1 = MAX_THR_LOWSPD;}
      if(!menu.driver && _pad1 > MAX_THR_DISP){_pad1 = MAX_THR_DISP;}
      if(_pad1>1000){_pad1=1000;}
      // Regen - Paddle 2
      _pad2 = (_pad2 < ((MID_PAD_V + MIN_RGN_DZ) * ADC_POINTS_PER_V)) ? 0 : _pad2 - ((MID_PAD_V + MIN_RGN_DZ) * ADC_POINTS_PER_V);
      _pad2 = (_pad2 * 1000) / (((HGH_PAD_V - MAX_RGN_DZ) - (MID_PAD_V + MIN_RGN_DZ)) * ADC_POINTS_PER_V);
      if(_pad2>1000){_pad2=1000;}
      _pad2 *= MAX_REGEN / 1000.0;

      *_thr = _pad1;
      *_rgn = _pad2;
      break;
    case 1:
      if(_pad1 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 1 Forward
      {
        _pad1 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
        _pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);
        if(esc.Velocity_KMH < LOWSPD_THRES && _pad1 > MAX_THR_LOWSPD){_pad1 = MAX_THR_LOWSPD;}
        if(!menu.driver && _pad1 > MAX_THR_DISP){_pad1 = MAX_THR_DISP;}
        if(_pad1>1000){_pad1=1000;}

        *_thr = _pad1;
        *_rgn = 0;
      }
      else if(_pad1 < ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V)) // Regen - Paddle 1 Back
      {
        _pad1 = ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V) - _pad1;
        _pad1 = (_pad1 * 1000) / (((MID_PAD_V - MIN_RGN_DZ) - (LOW_PAD_V + MAX_THR_DZ)) * ADC_POINTS_PER_V);
        if(_pad1>1000){_pad1=1000;}
        _pad1 *= MAX_REGEN / 1000.0;

        *_thr = 0;
        *_rgn = _pad1;
      }
      else{*_thr = 0;*_rgn = 0;} // Centred Paddle
      break;
    case 2:
      if(_pad1 < ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V)) // Regen - Paddle 1 Back
      {
        _pad1 = ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V) - _pad1;
        _pad1 = (_pad1 * 1000) / (((MID_PAD_V - MIN_RGN_DZ) - (LOW_PAD_V + MAX_THR_DZ)) * ADC_POINTS_PER_V);
        if(_pad1>1000){_pad1=1000;}
        _pad1 *= MAX_REGEN / 1000.0;

        *_thr = 0;
        *_rgn = _pad1;
      }
      else if(_pad2 < ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V)) // Regen - Paddle 2 Back
      {
        _pad2 = ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V) - _pad2;
        _pad2 = (_pad2 * 1000) / (((MID_PAD_V - MIN_RGN_DZ) - (LOW_PAD_V + MAX_THR_DZ)) * ADC_POINTS_PER_V);
        if(_pad2>1000){_pad2=1000;}
        _pad2 *= MAX_REGEN / 1000.0;

        *_thr = 0;
        *_rgn = _pad2;
      }
      else if(_pad1 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 1 Forward
      {
        _pad1 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
        _pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);

        if(esc.Velocity_KMH < LOWSPD_THRES && _pad1 > MAX_THR_LOWSPD){_pad1 = MAX_THR_LOWSPD;}
        if(!menu.driver && _pad1 > MAX_THR_DISP){_pad1 = MAX_THR_DISP;}
        if(_pad1>1000){_pad1=1000;}

        *_thr = _pad1;
        *_rgn = 0;
      }
      else if(_pad2 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 2 Forward
      {
        _pad2 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
        _pad2 = (_pad2 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);

        if(esc.Velocity_KMH < LOWSPD_THRES && _pad2 > MAX_THR_LOWSPD){_pad2 = MAX_THR_LOWSPD;}
        if(!menu.driver && _pad2 > MAX_THR_DISP){_pad2 = MAX_THR_DISP;}
        if(_pad2>1000){_pad2=1000;}

        *_thr = _pad2;
        *_rgn = 0;
      }
      else{*_thr = 0;*_rgn = 0;} // Centred Paddle
      break;
  }
}


/******************************************************************************
 ** Function name:  main_lights
 **
 ** Description:    Controls status of lights and LEDs
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_lights (void)
{
  if(MECH_BRAKE || rgn_pos) {BRAKELIGHT_ON;}
  else                      {BRAKELIGHT_OFF;}

  if(!rgn_pos)
  {
    if(REVERSE)     {REVERSE_ON;NEUTRAL_OFF;REGEN_OFF;DRIVE_OFF;}
    else if(FORWARD){REVERSE_OFF;NEUTRAL_OFF;REGEN_OFF;DRIVE_ON;}
    else            {REVERSE_OFF;NEUTRAL_ON;REGEN_OFF;DRIVE_OFF;}
  }
  else
  {
    if(REVERSE)     {REVERSE_ON;NEUTRAL_OFF;REGEN_ON;DRIVE_OFF;}
    else if(FORWARD){REVERSE_OFF;NEUTRAL_OFF;REGEN_ON;DRIVE_ON;}
    else            {REVERSE_OFF;NEUTRAL_ON;REGEN_OFF;DRIVE_OFF;}
  }

  if((SWITCH_IO & 0x8) && (clock.blink)){BLINKER_L_ON}
  else{BLINKER_L_OFF}
  if((SWITCH_IO & 0x10) && (clock.blink)){BLINKER_R_ON}
  else{BLINKER_R_OFF}

  if(SWITCH_IO & 0x4) {SPORTS_ON;ECO_OFF;}
  else                {SPORTS_OFF;ECO_ON;}

  if(STATS_FAULT == 1){FAULT_ON}
  else if(STATS_FAULT == 2 && (clock.blink)){FAULT_ON}
  else{FAULT_OFF}
}

/******************************************************************************
 ** Function name:  main_can_handler
 **
 ** Description:    Handles custom CAN packets that aren't handled in can.c
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_can_handler (void)
{
  /*
   * List of packets handled:
   * Kill drive - 0x520: use loop to prevent drive logic running. Not done in can.c to release Rx buffer
   * SWOC Error - 0x401: Send MC reset packet
   */
  if(CAN1RxDone == TRUE)
  {
    CAN1RxDone = FALSE;

    if((MsgBuf_RX1.MsgID == DASH_RQST) && (MsgBuf_RX1.DataA == 0x4C4C494B) && (MsgBuf_RX1.DataB == 0x45565244)) // Data = KILLDRVE
    {
      lcd_clear();
      char rot1[20], rot2[20];

      drive.current = 0;
      drive.speed_rpm = 0;

      if(menu.driver == 3)
      {
        _lcd_putTitle("-GOT DICK-");
        lcd_putstring(1,0, EROW);
        sprintf(rot1, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", 0x00, 0x02, 0x04, 0x04, 0x05, 0x00, 0x02, 0x04, 0x04, 0x05, 0x00, 0x02, 0x04, 0x04, 0x05, 0x00, 0x02, 0x04, 0x04, 0x05);
        lcd_putstring_custom(2,0, rot1, 20); // does not catch nulls
        sprintf(rot2, "%c%c   %c%c   %c%c   %c%c   ", 0x01, 0x03, 0x01, 0x03, 0x01, 0x03, 0x01, 0x03);
        lcd_putstring(3,0, rot2);
      }
      else
      {
        _lcd_putTitle("-KILLDRVE-");
        lcd_putstring(1,0, "--   KILL DRIVE   --");
        lcd_putstring(2,0, "--   KILL DRIVE   --");
        lcd_putstring(3,0, "--   KILL DRIVE   --");
      }

      while(FORWARD || REVERSE)
      {
        if(menu.driver == 3)
        {
          _lcd_putTitle("-GOT DICK-");
          _buffer_rotate_right(rot1, 20);
          _buffer_rotate_right(rot2, 20);
          lcd_putstring_custom(2,0, rot1, 20);
          lcd_putstring(3,0, rot2);
        }
        buzzer(5);
        delayMs(1,250);
      }
      lcd_clear();
    }
    else if(MsgBuf_RX1.MsgID == ESC_BASE + 1 && esc.ERROR == 0x2 && (AUTO_SWOC || menu.driver == 0))
    {
      if(esc.ERROR == 0x2)
      {
        esc_reset();
        buzzer(50);
        NEUTRAL_OFF;REVERSE_OFF;DRIVE_OFF;REGEN_OFF;
      }
    }

    // Clear Rx Buffer
    MsgBuf_RX1.Frame = 0x0;
    MsgBuf_RX1.MsgID = 0x0;
    MsgBuf_RX1.DataA = 0x0;
    MsgBuf_RX1.DataB = 0x0;
  }
}

/******************************************************************************
 ** Function name:  main_calc
 **
 ** Description:    Calculates instantaneous values and peaks
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_calc (void)
{
  // Calculate Power of components
  esc.Watts = esc.Bus_V * esc.Bus_I;

  BMU.Watts = BMU.Battery_I * BMU.Battery_V;

  mppt1.Watts = (mppt1.VIn * mppt1.IIn) / 1000.0;
  mppt2.Watts = (mppt2.VIn * mppt2.IIn) / 1000.0;

  // Check peaks
  if(esc.Watts > esc.MAX_Watts){esc.MAX_Watts = esc.Watts;}
  if(esc.Bus_I > esc.MAX_Bus_I){esc.MAX_Bus_I = esc.Bus_I;}
  if(esc.Bus_V > esc.MAX_Bus_V){esc.MAX_Bus_V = esc.Bus_V;}
  if(esc.Velocity_KMH > stats.max_speed){stats.max_speed = esc.Velocity_KMH;}

  if(mppt1.Tmp > mppt1.MAX_Tmp){mppt1.MAX_Tmp = mppt1.Tmp;}
  if(mppt1.VIn > mppt1.MAX_VIn){mppt1.MAX_VIn = mppt1.VIn;}
  if(mppt1.IIn > mppt1.MAX_IIn){mppt1.MAX_IIn = mppt1.IIn;}
  if(mppt1.Watts > mppt1.MAX_Watts){mppt1.MAX_Watts = mppt1.Watts;}

  if(mppt2.Tmp > mppt2.MAX_Tmp){mppt2.MAX_Tmp = mppt2.Tmp;}
  if(mppt2.VIn > mppt2.MAX_VIn){mppt2.MAX_VIn = mppt2.VIn;}
  if(mppt2.IIn > mppt2.MAX_IIn){mppt2.MAX_IIn = mppt2.IIn;}
  if(mppt2.Watts > mppt2.MAX_Watts){mppt2.MAX_Watts = mppt2.Watts;}

  if(BMU.Watts > BMU.MAX_Watts){BMU.MAX_Watts = BMU.Watts;}
  if(BMU.Battery_I > BMU.MAX_Battery_I){BMU.MAX_Battery_I = BMU.Battery_I;}
  if(BMU.Battery_V > BMU.MAX_Battery_V){BMU.MAX_Battery_V = BMU.Battery_V;}
}

/******************************************************************************
 ** Function name:  main_HV
 **
 ** Description:    Controls HV contactor
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_HV (void)
{
  if(esc.Bus_V > 90 && esc.Bus_V > 0.9 * BMU.Battery_V && esc.Bus_V < 1.1 * BMU.Battery_V && stats.hv_counter<1100){stats.hv_counter++;}
  else if(stats.hv_counter){stats.hv_counter--;}

  if(stats.hv_counter>1000 && !STATS_ARMED){buzzer(50);SET_STATS_ARMED;HV_ON;}
  else if(stats.hv_counter<100){CLR_STATS_ARMED;HV_OFF;}
}

/******************************************************************************
 ** Function name:  esc_reset
 **
 ** Description:    Resets motorcontroller(s) with CAN packet
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void esc_reset (void)
{
  // RESET MOTOR CONTROLLERS
  // see WS22 user manual and Tritium CAN network specs
  // try MC + 25 (0x19) + msg "RESETWS" (TRI88.004 ver3 doc, July 2013)
  MsgBuf_TX1.Frame = 0x00080000;  // 11-bit, no RTR, DLC is 8 bytes
  MsgBuf_TX1.MsgID = ESC_CONTROL + 3;
  MsgBuf_TX1.DataB = 0x0;
  MsgBuf_TX1.DataA = 0x0;
  CAN1_SendMessage( &MsgBuf_TX1 );
}

/******************************************************************************
 ** Function name:  EE_Read
 **
 ** Description:    Reads a word from EEPROM (Uses I2CRead)
 **
 ** Parameters:     Address to read from
 ** Returned value: Data at address
 **
 ******************************************************************************/
uint32_t EE_read (uint16_t _EEadd)
{
  uint32_t retDATA = 0;

  retDATA = I2C_read(_EEadd+3);
  retDATA = (retDATA << 8) + I2C_read(_EEadd+2);
  retDATA = (retDATA << 8) + I2C_read(_EEadd+1);
  retDATA = (retDATA << 8) + I2C_read(_EEadd+0);

  return retDATA;
}

/******************************************************************************
 ** Function name:  EE_Seq_Read
 **
 ** Description:    Reads a word from EEPROM (Uses I2CRead)
 **
 ** Parameters:     1. Address to read from
 **                 2. Byte length of read
 ** Returned value: Data at address
 **
 ******************************************************************************/
uint32_t EE_seq_read (uint16_t _EEadd, int _len)
{
  uint32_t _ret = 0;

  I2C_seq_read(_EEadd, _len);
  while(_len--)
  {
    _ret += I2CSlaveBuffer[PORT_USED][_len] << (_len * 8);
  }

  return _ret;
}

/******************************************************************************
 ** Function name:  EE_Write
 **
 ** Description:    Saves a word to EEPROM (Uses I2CWrite)
 **
 ** Parameters:     1. Address to save to
 **                 2. Data to save (convert to uint with converter first)
 ** Returned value: None
 **
 ******************************************************************************/
void EE_write (uint16_t _EEadd, uint32_t _EEdata)
{
  uint8_t temp0 = (_EEdata & 0x000000FF);
  uint8_t temp1 = (_EEdata & 0x0000FF00) >> 8;
  uint8_t temp2 = (_EEdata & 0x00FF0000) >> 16;
  uint8_t temp3 = (_EEdata & 0xFF000000) >> 24;

  I2C_write(_EEadd, temp0,temp1,temp2,temp3);
}

/******************************************************************************
 ** Function name:  I2C_Read
 **
 ** Description:    Reads a byte from EEPROM
 **
 ** Parameters:     Address to read from
 ** Returned value: Data at address
 **
 ******************************************************************************/
uint32_t I2C_read (uint16_t _EEadd)
{
  int i;

  for ( i = 0; i < BUFSIZE; i++ ) // clear buffer
  {
    I2CMasterBuffer[PORT_USED][i] = 0;
  }

  I2CWriteLength[PORT_USED] = 3;
  I2CReadLength[PORT_USED] = 1;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = (_EEadd & 0x0f00) >> 8; // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd & 0x00ff;        // address
  I2CMasterBuffer[PORT_USED][3] = _24LC256_ADDR | RD_BIT;
  I2CEngine( PORT_USED );
  I2CStop(PORT_USED);

  // TODO: Test I2C timeouts
  if(I2CMasterState[PORT_USED] == I2C_TIME_OUT){BLINKER_L_ON;}

  return (uint32_t)I2CSlaveBuffer[PORT_USED][0];
}

/******************************************************************************
 ** Function name:  I2C_Seq_Read
 **
 ** Description:    Reads a byte from EEPROM
 **
 ** Parameters:     1. Address to read from
 **                 2. Byte length of read
 ** Returned value: None
 **
 ******************************************************************************/
void I2C_seq_read (uint16_t _EEadd, int read_len)
{
  int i;
  for ( i = 0; i < BUFSIZE; i++ ) // clear buffer
  {
    I2CSlaveBuffer[PORT_USED][i] = 0;
  }

  I2CWriteLength[PORT_USED] = 3;
  I2CReadLength[PORT_USED] = read_len;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = (_EEadd & 0x0f00) >> 8; // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd & 0x00ff;        // address
  I2CMasterBuffer[PORT_USED][3] = _24LC256_ADDR | RD_BIT;
  I2CEngine( PORT_USED );
  I2CStop(PORT_USED);
}

/******************************************************************************
 ** Function name:  I2C_Write
 **
 ** Description:    Saves a word to EEPROM
 **
 ** Parameters:     1. Address to save to
 **                 2. Data to save
 ** Returned value: None
 **
 ******************************************************************************/
void I2C_write (uint16_t _EEadd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
  I2CWriteLength[PORT_USED] = 7;
  I2CReadLength[PORT_USED] = 0;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = (_EEadd & 0x0f00) >> 8; // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd & 0x00ff;        // address
  I2CMasterBuffer[PORT_USED][3] = data0;
  I2CMasterBuffer[PORT_USED][4] = data1;
  I2CMasterBuffer[PORT_USED][5] = data2;
  I2CMasterBuffer[PORT_USED][6] = data3;
  I2CEngine( PORT_USED );

  // TODO: Test I2C timeouts
  if(I2CMasterState[PORT_USED] == I2C_TIME_OUT){BLINKER_R_ON;}

  delayMs(1,2);
}

/******************************************************************************
 ** Function name:  iir_filter_uint
 **
 ** Description:    Filter to flatten out erratic data reads
 **
 ** Parameters:     1. Input data
 **                 2. Existing data
 **                 3. Gain factor
 ** Returned value: Smoothed value
 **
 ******************************************************************************/
uint32_t iir_filter_uint (uint32_t _data_in, uint32_t _cur_data, uint16_t _gain)
{return (((_gain-1)*_cur_data)+_data_in)/_gain;}

/******************************************************************************
 ** Function name:  iir_filter_int
 **
 ** Description:    Filter to flatten out erratic data reads
 **
 ** Parameters:     1. Input data
 **                 2. Existing data
 **                 3. Gain factor
 ** Returned value: Smoothed value
 **
 ******************************************************************************/
int32_t iir_filter_int (int32_t _data_in, int32_t _cur_data, uint16_t _gain)
{return (((_gain-1)*_cur_data)+_data_in)/_gain;}

/******************************************************************************
 ** Function name:  iir_filter_float
 **
 ** Description:    Filter to flatten out erratic data reads
 **
 ** Parameters:     1. Input data
 **                 2. Existing data
 **                 3. Gain factor
 ** Returned value: Smoothed value
 **
 ******************************************************************************/
float iir_filter_float (float _data_in, float _cur_data, uint16_t _gain)
{return (((_gain-1)*_cur_data)+_data_in)/_gain;}

/******************************************************************************
 ** Function name:  load_persistent
 **
 ** Description:    Restores persistent variables from EEPROM
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void persistent_load (void)
{
  stats.flags |= (EE_read(ADD_BUZZ) & 0b1) << 1;
  stats.odometer = conv_uint_float(EE_read(ADD_ODO));
  stats.odometer_tr = conv_uint_float(EE_read(ADD_ODOTR));

  BMU.WattHrs = conv_uint_float(EE_read(ADD_BMUWHR));
  mppt1.WattHrs = conv_uint_float(EE_read(ADD_MPPT1WHR));
  mppt2.WattHrs = conv_uint_float(EE_read(ADD_MPPT2WHR));
}

/******************************************************************************
 ** Function name:  store_persistent
 **
 ** Description:    Saves persistent variables to EEPROM
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void persistent_store (void)
{
  if(clock.T_S % 2)
  {
    EE_write(ADD_ODO, conv_float_uint(stats.odometer));
    EE_write(ADD_ODOTR, conv_float_uint(stats.odometer_tr));
  }
  else
  {
    EE_write(ADD_BMUWHR, conv_float_uint(BMU.WattHrs));
    EE_write(ADD_MPPT1WHR, conv_float_uint(mppt1.WattHrs));
    EE_write(ADD_MPPT2WHR, conv_float_uint(mppt2.WattHrs));
  }
}

/******************************************************************************
 ** Function name:  load_nonpersistent
 **
 ** Description:    Loads non-persistent default values
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void nonpersistent_load(void)
{
  menu.counter = 0;
  menu.driver = 255;
  menu.menu_pos = 0;
  menu.submenu_pos = 0;
  menu.submenu_items = 0;
  menu.flags = 0;

  clock.T_D = 0;
  clock.T_H = 0;
  clock.T_M = 0;
  clock.T_S = 0;
  clock.T_mS = 0;
  clock.blink = 0;

  drive.current = 0;
  drive.speed_rpm = 0;

  stats.flags = 0;
  stats.errors = 0;
  stats.max_speed = 0;
  stats.cruise_speed = 0;
  stats.ramp_speed = 5;

  BMU.Battery_I = 0;
  BMU.Battery_V = 0;
  BMU.CAN_ID = BMU_BASE;
  BMU.Status = 0;
  BMU.MAX_Battery_I = 0;
  BMU.MAX_Battery_V = 0;
  BMU.MAX_Watts = 0;
  BMU.Max_Cell_Tmp = 0;
  BMU.Max_Cell_V = 0;
  BMU.Min_Cell_Tmp = 0;
  BMU.Min_Cell_V = 0;

  esc.Bus_I = 0;
  esc.Bus_V = 0;
  esc.CAN_ID = ESC_BASE;
  esc.ERROR = 0;
  esc.MAX_Bus_I = 0;
  esc.MAX_Bus_V = 0;
  esc.MAX_Watts = 0;
  esc.Velocity_KMH = 0;
  esc.WattHrs = 0;
  esc.Watts = 0;

  mppt1.CAN_ID = MPPT1_BASE;
  mppt1.IIn = 0;
  mppt1.MAX_IIn = 0;
  mppt1.MAX_Tmp = 0;
  mppt1.MAX_VIn = 0;
  mppt1.MAX_VOut = 0;
  mppt1.MAX_Watts = 0;
  mppt1.Tmp = 0;
  mppt1.VIn = 0;
  mppt1.VOut = 0;
  mppt1.Watts = 0;
  mppt1.flags = 0;

  mppt2.CAN_ID = MPPT2_BASE;
  mppt2.IIn = 0;
  mppt2.MAX_IIn = 0;
  mppt2.MAX_Tmp = 0;
  mppt2.MAX_VIn = 0;
  mppt2.MAX_VOut = 0;
  mppt2.MAX_Watts = 0;
  mppt2.Tmp = 0;
  mppt2.VIn = 0;
  mppt2.VOut = 0;
  mppt2.Watts = 0;
  mppt2.flags = 0;

  MsgBuf_TX1.Frame = 0x00080000;
  MsgBuf_TX1.MsgID = 0x0;
  MsgBuf_TX1.DataA = 0x0;
  MsgBuf_TX1.DataB = 0x0;

  MsgBuf_RX1.Frame = 0x0;
  MsgBuf_RX1.MsgID = 0x0;
  MsgBuf_RX1.DataA = 0x0;
  MsgBuf_RX1.DataB = 0x0;

  MsgBuf_TX2.Frame = 0x00080000;
  MsgBuf_TX2.MsgID = MPPT1_BASE;
  MsgBuf_TX2.DataA = 0x00000000;
  MsgBuf_TX2.DataB = 0x00000000;

  MsgBuf_RX2.Frame = 0x0;
  MsgBuf_RX2.MsgID = 0x0;
  MsgBuf_RX2.DataA = 0x0;
  MsgBuf_RX2.DataB = 0x0;
}

/******************************************************************************
 ** Function name:  init_GPIO
 **
 ** Description:    Configures pins to be used for GPIO
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void GPIO_init (void)
{
  /* GPIO0:
   *  PINSEL0:
   *    0 - IN - RIGHT
   *    1 - IN - INC/UP
   *    3 - OUT - Buzzer
   *    10 - IN - REVERSE
   *    11 - IN - FORWARD
   *    15 - OUT - LCD Reset
   *  PINSEL1:
   *    16 - OUT - LCD Enable
   *    25 - IN - Mech Brake
   *    27 - OUT - Fault LED
   */
  LPC_GPIO0->FIODIR = (1<<3)|(1<<15)|(1<<16)|(1<<27);
  LPC_GPIO0->FIOCLR = (1<<3)|(1<<15)|(1<<16)|(1<<27);

  /*
   * GPIO1:
   *  PINSEL2:
   *    0 - IN - RIGHT_ON
   *    1 - IN - LEFT_ON
   *    4 - IN - Power Status
   *    8 - OUT - Armed Status
   *  PINSEL3:
   *    19 - OUT - Blinker R
   *    20 - OUT - Blinker L
   *    21 - OUT - Brake Light
   *    23 - OUT - Reverse LED
   *    24 - OUT - Neutral LED
   *    25 - OUT - Regen LED
   *    26 - OUT - Drive LED
   *    27 - IN - LEFT
   *    28 - IN - DEC/DOWN
   *    29 - IN - SELECT
   *    30 - OUT - ECO LED
   *    31 - OUT - SPORTS LED
   */
  LPC_GPIO1->FIODIR = (1<<8)|(1<<19)|(1<<20)|(1<<21)|(1<<23)|(1<<24)|(1<<25)|(1<<26)|(1<<30)|(1<<31);
  LPC_GPIO1->FIOCLR = (1<<8)|(1<<19)|(1<<20)|(1<<21)|(1<<23)|(1<<24)|(1<<25)|(1<<26)|(1<<30)|(1<<31);

  /*
   * GPIO2:
   *  PINSEL4:
   *    6 - OUT - LCD D7
   *    7 - OUT - LCD D6
   *    8 - OUT - LCD D5
   *    9 - OUT - LCD D4
   *    10 - IN - Aux ON (SPORTS MODE)
   *    11 - IN - Aux OFF
   *    12 - IN - Spare switch
   *    13 - IN - Spare switch
   */
  LPC_GPIO2->FIODIR = (1<<6)|(1<<7)|(1<<8)|(1<<9);
  LPC_GPIO2->FIOCLR = (1<<6)|(1<<7)|(1<<8)|(1<<9);

  /*
   * GPIO3:
   *  PINSEL7:
   *    25 - OUT - Left LED
   *    26 - OUT - Right LED
   */
  LPC_GPIO3->FIODIR = (1<<25)|(1<<26);
  LPC_GPIO3->FIOCLR = (1<<25)|(1<<26);
}

/******************************************************************************
 ** Function name:  buzzer
 **
 ** Description:    Turns buzzer on for set amount of time
 **
 ** Parameters:     Number of 10mS ticks to sound buzzer
 ** Returned value: None
 **
 ******************************************************************************/
void buzzer (uint8_t val)
{
  if(STATS_BUZZER)
  {
    stats.buz_tim = val;
    BUZZER_ON;
  }
}

/******************************************************************************
 ** Function name:  BOD_Init
 **
 ** Description:    Configures BOD Handler
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void BOD_init ( void )
{
  // Turn on BOD.
  LPC_SC->PCON &= ~(0x1<<3);

  // Enable the BOD Interrupt
  NVIC_EnableIRQ(BOD_IRQn);
}

/******************************************************************************
 ** Function name:  main
 **
 ** Description:    Program entry point. Contains initializations and menu loop
 **
 ** Parameters:     None
 ** Returned value: Program exit value
 **
 ******************************************************************************/
int main (void)
{
  SystemInit();
  SystemCoreClockUpdate();

  CAN1_Init( BITRATE500K30MHZ );
  CAN2_Init( BITRATE125K30MHZ );
  CAN_SetACCF( ACCF_BYPASS );

  I2C1Init();

  nonpersistent_load();
  persistent_load();

  SysTick_Config(SystemCoreClock / 100);  // 10mS Systicker.

  ADCInit(ADC_CLK);

  GPIO_init();

  lcd_init();
  lcd_clear();

  BOD_init();

  menu_driver();

  menu_intro();

  menu_init();

  if(FORWARD || REVERSE){menu_errOnStart();}
  while(FORWARD || REVERSE){buzzer(60);delayMs(1, 1000);}

  while(1){ // Exiting this loop ends the program
    if((esc.ERROR & 0x1) && !STATS_HWOC_ACK) // on unacknowledged HWOC error, show error screen
    {menu.errors[1]();}
    else if((esc.ERROR & 0x2) && !STATS_SWOC_ACK && !AUTO_SWOC && menu.driver) // show SWOC screen when auto reset off and not on display mode and error not acknowledged
    {menu.errors[0]();}
    else if(STATS_COMMS)
    {menu.errors[2]();}
    else
    {
      if(STATS_SWOC_ACK && !(esc.ERROR & 0x2)) // if acknowledged previous error is reset
      {CLR_STATS_SWOC_ACK;}
      if(STATS_HWOC_ACK && !(esc.ERROR & 0x1)) // if acknowledged previous error is reset
      {CLR_STATS_HWOC_ACK;}

      menu.counter++;

      menu.menus[menu.menu_pos]();
    }
    main_mppt_poll();
    main_input_check();
    if((stats.errors = ((0b11100111 & stats.errors) | (main_fault_check() << 3))) & (0x2 << 3)){main_drive();} // no drive on fault code 2
    main_lights();
    main_can_handler();
    main_calc();
    main_HV();
  }

  return 0; // For compilers sanity
}

