/*
 * can.c
 *
 *  Created on: 28 Apr 2015
 *      Author: Stuart G
 */

#include "lpc17xx.h"
#include "type.h"
#include "can.h"
#include "dash.h"
#include "struct.h"
#include "inttofloat.h"

extern CAN_MSG MsgBuf_RX1, MsgBuf_RX2;
extern volatile uint8_t CAN1RxDone, CAN2RxDone;
extern MOTORCONTROLLER esc;

volatile uint32_t CANStatus;
volatile uint32_t CAN1RxCount = 0, CAN2RxCount = 0;
volatile uint32_t CAN1TxCount = 0, CAN2TxCount = 0;
uint16_t CAN1ErrCount = 0, CAN2ErrCount = 0;

#if CAN_WAKEUP
volatile uint32_t CANActivityInterruptFlag = 0;
#endif

/******************************************************************************
** Function name: CAN_ISR_Rx1
**
** Description:   CAN Rx1 interrupt handler
**
** Parameters:      None
** Returned value:  None
**
******************************************************************************/
void CAN_ISR_Rx1( void )
{
  uint32_t * pDest = (uint32_t *)&MsgBuf_RX1;

  *pDest = LPC_CAN1->RFS; // Frame
  pDest++;

  *pDest = LPC_CAN1->RID; // ID
  pDest++;

  *pDest = LPC_CAN1->RDA; // DataA
  pDest++;

  *pDest = LPC_CAN1->RDB; // DataB
  pDest++;

  switch (MsgBuf_RX1.MsgID)
  {
  case ESC_BASE:
    break;
  case ESC_BASE + 1:
#if _MC_ERR
  esc.error = (MsgBuf_RX1.DataA >> 16);
  if(esc.error == 0x2){CAN1RxDone = TRUE;NEUTRAL_ON;REVERSE_ON;DRIVE_ON;REGEN_ON;}
#endif
#if _MC_LIM
  esc.limit = (MsgBuf_RX1.DataA & 0xFFFF);
#endif
  break;
  case ESC_BASE + 2:
  esc.bus_v = iir_filter_float(esc.bus_v, conv_uint_float(MsgBuf_RX1.DataA), IIR_GAIN_ELECTRICAL);
  esc.bus_i = iir_filter_float(esc.bus_i, conv_uint_float(MsgBuf_RX1.DataB), IIR_GAIN_ELECTRICAL);
  break;
  case ESC_BASE + 3:
#if _MC_VELOCITY == 1
  esc.velocity_kmh = conv_uint_float(MsgBuf_RX1.DataB)*3.6;
#elif _MC_VELOCITY == 2
  esc.velocity_rpm = conv_uint_float(MsgBuf_RX1.DataA);
#elif _MC_VELOCITY == 3
  esc.velocity_kmh = conv_uint_float(MsgBuf_RX1.DataB)*3.6;
  esc.velocity_rpm = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 4:
#if _MC_PHASE
  esc.phase_c_i = conv_uint_float(MsgBuf_RX1.DataB);
  esc.phase_b_i = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 5:
#if _MC_VECTORS
  esc.vectors->v_real = conv_uint_float(MsgBuf_RX1.DataB);
  esc.vectors->v_imag = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 6:
#if _MC_VECTORS
  esc.vectors->i_real = conv_uint_float(MsgBuf_RX1.DataB);
  esc.vectors->i_imag = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 7:
#if _MC_VECTORS
  esc.vectors->bemf_real = conv_uint_float(MsgBuf_RX1.DataB);
  esc.vectors->bemf_imag = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 8:
#if _MC_RAILS
  esc.rail_15v = conv_uint_float(MsgBuf_RX1.DataB);
#endif
  break;
  case ESC_BASE + 9:
#if _MC_RAILS
  esc.rail_3300mv = conv_uint_float(MsgBuf_RX1.DataB);
  esc.rail_1900mv = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 10:
  // Reserved Address
  break;
  case ESC_BASE + 11:
#if _MC_TMP
  esc.heatsink_tmp = conv_uint_float(MsgBuf_RX1.DataB);
  esc.motor_tmp = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 12:
#if _MC_TMP
  esc.board_tmp = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 13:
  // Reserved Address
  break;
  case ESC_BASE + 14:
#if _MC_AMPHRS
  esc.dc_amp_hrs = conv_uint_float(MsgBuf_RX1.DataB);
#endif
#if _MC_ODO
  esc.odometer = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case ESC_BASE + 23:
#if _MC_SLIP
  esc.slip_speed = conv_uint_float(MsgBuf_RX1.DataB);
#endif
  break;

  case DASH_RQST:
    CAN1RxDone = TRUE;
    break;
  case DASH_RQST + 1:
    SET_STATS_COMMS;
  break;
  case DASH_RQST + 2:
  break;
  case DASH_RQST + 3:
  break;
  case DASH_RQST + 4:
  break;
  case DASH_RQST + 5:
  break;
  case DASH_RQST + 6:
  break;
  case DASH_RQST + 7:
  break;
  case DASH_RQST + 8:
  break;
  case DASH_RQST + 9:
  break;

  case BMU_SHUNT:
	  shunt.bus_v = conv_uint_float(MsgBuf_RX1.DataA);
	  shunt.bus_i = conv_uint_float(MsgBuf_RX1.DataB);
	  shunt.watts = shunt.bus_i * shunt.bus_v;
	  shunt.con_tim = 3;
	  break;
  case BMU_SHUNT + 1:
  shunt.watt_hrs_out = conv_uint_float(MsgBuf_RX1.DataA);
  shunt.watt_hrs_in = conv_uint_float(MsgBuf_RX1.DataB);
  break;
  case BMU_SHUNT + 2:
  shunt.watt_hrs = conv_uint_float(MsgBuf_RX1.DataA);
  break;

  case BMU_BASE:
    break;

  case BMU_BASE + BMU_INFO:
#if _BMU_SOC == 1
  bmu.soc = conv_uint_float(MsgBuf_RX1.DataB);
#elif _BMU_SOC == 2
  bmu.soc_per = conv_uint_float(MsgBuf_RX1.DataA);
#elif _BMU_SOC == 3
  bmu.soc = conv_uint_float(MsgBuf_RX1.DataB);
  bmu.soc_per = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case BMU_BASE + BMU_INFO + 1:
#if _BMU_BAL_SOC == 1
  bmu.bal_soc = conv_uint_float(MsgBuf_RX1.DataB);
#elif _BMU_BAL_SOC == 2
  bmu.bal_soc_per = conv_uint_float(MsgBuf_RX1.DataA);
#elif _BMU_BAL_SOC == 3
  bmu.bal_soc = conv_uint_float(MsgBuf_RX1.DataB);
  bmu.bal_soc_per = conv_uint_float(MsgBuf_RX1.DataA);
#endif
  break;
  case BMU_BASE + BMU_INFO + 2:
#if _BMU_THRES
  bmu.charge_cell_v_err = MsgBuf_RX1.DataB >> 16;
  bmu.cell_tmp_margin = MsgBuf_RX1.DataB & 0xFFFF;
  bmu.discharge_cell_v_err = MsgBuf_RX1.DataA >> 16;
#endif
#if _BMU_CAP
  bmu.pack_capacity = MsgBuf_RX1.DataA & 0xFFFF;
#endif
  break;
  case BMU_BASE + BMU_INFO + 3:
#if _BMU_PRECHARGE
  bmu.driver_status = MsgBuf_RX1.DataB >> 24;
  bmu.precharge_state = (MsgBuf_RX1.DataB >> 16) & 0xFF;
  if ((MsgBuf_RX1.DataA >> 8) & 0xFF) {bmu.precharge_time_elapsed = TRUE;}
  else                                {bmu.precharge_time_elapsed = FALSE;}
  bmu.precharge_timer = MsgBuf_RX1.DataA & 0xFF;
#endif
  break;
  case BMU_BASE + BMU_INFO + 4:
#if _BMU_CELL_V
  bmu.min_cell_v = (MsgBuf_RX1.DataB >> 16) & 0xFFFF;
  bmu.max_cell_v = MsgBuf_RX1.DataB & 0xFFFF;
  bmu.cmu_min_v = (MsgBuf_RX1.DataA >> 24) & 0xFF;
  bmu.cmu_max_v = (MsgBuf_RX1.DataA >> 8) & 0xFF;
  bmu.cell_min_v = (MsgBuf_RX1.DataA >> 16) & 0xFF;
  bmu.cell_max_v = MsgBuf_RX1.DataA & 0xFF;
#endif
  break;
  case BMU_BASE + BMU_INFO + 5:
#if _BMU_CMU_TMP
  bmu.max_cell_tmp = (MsgBuf_RX1.DataB >> 16) & 0xFFFF;
  bmu.max_cell_tmp = MsgBuf_RX1.DataB & 0xFFFF;
  bmu.cmu_min_tmp = (MsgBuf_RX1.DataA >> 24) & 0xFF;
  bmu.cmu_max_tmp = (MsgBuf_RX1.DataA >> 8) & 0xFF;
#endif
  break;
  case BMU_BASE + BMU_INFO + 6:
  bmu.bus_v = iir_filter_int(MsgBuf_RX1.DataB / 1000, bmu.bus_v, IIR_GAIN_ELECTRICAL); // Packet is in mV and mA
  bmu.bus_i = iir_filter_int(MsgBuf_RX1.DataA / 1000, bmu.bus_i, IIR_GAIN_ELECTRICAL);
  break;
  case BMU_BASE + BMU_INFO + 7:
  // Can extract 8 Status flags here but they are also contained with others in BASE + 9
#if _BMU_BAL_THRES
  bmu.bal_thres_rising = (MsgBuf_RX1.DataB >> 16) & 0xFFFF;
  bmu.bal_thres_falling = MsgBuf_RX1.DataB & 0xFFFF;
#endif
#if _BMU_CMU_CNT
  bmu.cmu_count = (MsgBuf_RX1.DataA >> 16) & 0xFF;
#endif
#if _BMU_VER
  bmu.bmu_fw_ver = MsgBuf_RX1.DataA & 0xFFFF;
#endif
  break;
  case BMU_BASE + BMU_INFO + 8:
#if _BMU_FAN == 1
  bmu.fan0_spd = (MsgBuf_RX1.DataB >> 16) & 0xFFFF;
#elif _BMU_FAN == 2
  bmu.fan1_spd = MsgBuf_RX1.DataB & 0xFFFF;
#elif _BMU_FAN == 3
  bmu.fan0_spd = (MsgBuf_RX1.DataB >> 16) & 0xFFFF;
  bmu.fan1_spd = MsgBuf_RX1.DataB & 0xFFFF;
#endif
#if _BMU_12V_CONSUM
  bmu.fan_contactor_12v_ma = (MsgBuf_RX1.DataA >> 16) & 0xFFFF;
  bmu.cmu_12v_ma = MsgBuf_RX1.DataA & 0xFFFF;
#endif
  break;
  case BMU_BASE + BMU_INFO + 9:
  bmu.status = MsgBuf_RX1.DataB;
#if _BMU_VER
  bmu.bmu_hw_ver = (MsgBuf_RX1.DataA >> 24) & 0xFF;
  bmu.bmu_model_id = (MsgBuf_RX1.DataA >> 16) & 0xFF;
#endif
  break;

  default:
    break;
  }
  LPC_CAN1->CMR = 0x4; // Release Receive Buffer
}

/******************************************************************************
** Function name:   CAN_ISR_Rx2
**
** Description:     CAN Rx2 interrupt handler
**
** Parameters:      None
** Returned value:  None
**
******************************************************************************/
void CAN_ISR_Rx2( void )
{
  uint32_t *pDest;

  /* initialize destination pointer	*/
  pDest = (uint32_t *)&MsgBuf_RX2;
  *pDest = LPC_CAN2->RFS;  /* Frame	*/

  pDest++;
  *pDest = LPC_CAN2->RID; /* ID	*/

  pDest++;
  *pDest = LPC_CAN2->RDA; /* Data A	*/

  pDest++;
  *pDest = LPC_CAN2->RDB; /* Data B	*/

  CAN2RxDone = TRUE;
  LPC_CAN2->CMR = 0x4; /* release receive buffer */
  return;
}

/*****************************************************************************
** Function name:   CAN_Handler
**
** Descriptions:    CAN interrupt handler
**
** parameters:      None
** Returned value:  None
**
*****************************************************************************/
void CAN_IRQHandler(void)
{
  CANStatus = LPC_CANCR->CANRxSR;
  if ( CANStatus & (1 << 8) )
  {
  CAN1RxCount++;
  CAN_ISR_Rx1();
  }
  if ( CANStatus & (1 << 9) )
  {
  CAN2RxCount++;
  CAN_ISR_Rx2();
  }
  if ( LPC_CAN1->GSR & (1 << 6 ) )
  {
  /* The error count includes both TX and RX */
  CAN1ErrCount = LPC_CAN1->GSR >> 16;
  }
  if ( LPC_CAN2->GSR & (1 << 6 ) )
  {
  /* The error count includes both TX and RX */
  CAN2ErrCount = LPC_CAN2->GSR >> 16;
  }
  return;
}

#if CAN_WAKEUP
/******************************************************************************
** Function name:   CANActivity_IRQHandler
**
** Descriptions:    Wake up from CAN handler
**
** parameters:      None
** Returned value:  None
**
******************************************************************************/
void CANActivity_IRQHandler (void)
{
  CAN2RxDone = TRUE;
  CANActivityInterruptFlag = 1;

  LPC_SC->CANSLEEPCLR = (0x1<<1)|(0x1<<2);
  LPC_CAN1->MOD = LPC_CAN2->MOD &= ~(0x1<<4);
  LPC_SC->CANWAKEFLAGS = (0x1<<1)|(0x1<<2);
  return;
}
#endif

/******************************************************************************
** Function name:   CAN_Init
**
** Descriptions:    Initialize CAN, install CAN interrupt handler
**
** parameters:      bitrate
** Returned value:  true or false, false if initialization failed.
**
******************************************************************************/
uint32_t CAN1_Init( uint32_t can_btr )
{
  CAN1RxDone = FALSE;

  LPC_SC->PCONP |= (1<<13);  /* Enable CAN1 clock */

  LPC_PINCON->PINSEL1 |= (1<<13)|(1<<12)|(1<<11)|(1<<10);

  LPC_CAN1->MOD = 1;    /* Reset CAN */
  LPC_CAN1->IER = 0;    /* Disable Receive Interrupt */
  LPC_CAN1->GSR = 0;    /* Reset error counter when CANxMOD is in reset	*/

  LPC_CAN1->BTR = can_btr;
  LPC_CAN1->MOD = 0x0;  /* CAN in normal operation mode */

  NVIC_EnableIRQ(CAN_IRQn);

  LPC_CAN1->IER = 0x01; /* Enable receive interrupts */
  return( TRUE );
}

/******************************************************************************
** Function name:   CAN_Init
**
** Descriptions:    Initialize CAN, install CAN interrupt handler
**
** parameters:      bitrate
** Returned value:  true or false, false if initialization failed.
**
******************************************************************************/
uint32_t CAN2_Init( uint32_t can_btr )
{
  CAN2RxDone = FALSE;

  LPC_SC->PCONP |= (1<<14);  /* Enable CAN2 clock */

  LPC_PINCON->PINSEL0 |= (1<<9)|(1<<11);

  LPC_CAN2->MOD = 1;    /* Reset CAN */
  LPC_CAN2->IER = 0;    /* Disable Receive Interrupt */
  LPC_CAN2->GSR = 0;    /* Reset error counter when CANxMOD is in reset	*/

  LPC_CAN2->BTR = can_btr;
  LPC_CAN2->MOD = 0x0;  /* CAN in normal operation mode */

  NVIC_EnableIRQ(CAN_IRQn);

  LPC_CAN2->IER = 0x01; /* Enable receive interrupts */
  return( TRUE );
}

/******************************************************************************
** Function name:   CAN_SetACCF_Lookup
**
** Descriptions:    Initialize CAN, install CAN interrupt handler
**
** parameters:      bitrate
** Returned value:  true or false, false if initialization failed.
**
******************************************************************************/
void CAN_SetACCF_Lookup( void )
{
  uint32_t address = 0;
  uint32_t i;
  uint32_t ID_high, ID_low;

  /* Set explicit standard Frame */
  LPC_CANAF->SFF_sa = address;
  for ( i = 0; i < ACCF_IDEN_NUM; i += 2 )
  {
  ID_low = (i << 29) | (MPPT1_BASE << 16);
  ID_high = ((i+1) << 13) | (MPPT1_BASE << 0);
  *((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low | ID_high;
  address += 4;
  }

  /* Set group standard Frame */
  LPC_CANAF->SFF_GRP_sa = address;
  for ( i = 0; i < ACCF_IDEN_NUM; i += 2 )
  {
  ID_low = (i << 29) | (GRP_STD_ID << 16);
  ID_high = ((i+1) << 13) | (GRP_STD_ID << 0);
  *((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low | ID_high;
  address += 4;
  }

  /* Set explicit extended Frame */
  LPC_CANAF->EFF_sa = address;
  for ( i = 0; i < ACCF_IDEN_NUM; i++  )
  {
  ID_low = (i << 29) | (EXP_EXT_ID << 0);
  *((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low;
  address += 4;
  }

  /* Set group extended Frame */
  LPC_CANAF->EFF_GRP_sa = address;
  for ( i = 0; i < ACCF_IDEN_NUM; i++  )
  {
  ID_low = (i << 29) | (GRP_EXT_ID << 0);
  *((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low;
  address += 4;
  }

  /* Set End of Table */
  LPC_CANAF->ENDofTable = address;
  return;
}

/******************************************************************************
** Function name:   CAN_SetACCF
**
** Descriptions:    Set acceptance filter and SRAM associated with
**
** parameters:      ACMF mode
** Returned value:  None
**
**
******************************************************************************/
void CAN_SetACCF( uint32_t ACCFMode )
{
  switch ( ACCFMode )
  {
  case ACCF_OFF:
    LPC_CANAF->AFMR = ACCFMode;
    LPC_CAN1->MOD = LPC_CAN2->MOD = 1;  // Reset CAN
    LPC_CAN1->IER = LPC_CAN2->IER = 0;  // Disable Receive Interrupt
    LPC_CAN1->GSR = LPC_CAN2->GSR = 0;  // Reset error counter when CANxMOD is in reset
  break;

  case ACCF_BYPASS:
    LPC_CANAF->AFMR = ACCFMode;
  break;

  case ACCF_ON:
  case ACCF_FULLCAN:
    LPC_CANAF->AFMR = ACCF_OFF;
    CAN_SetACCF_Lookup();
    LPC_CANAF->AFMR = ACCFMode;
  break;

  default:
  break;
  }
  return;
}

/******************************************************************************
** Function name:   CAN1_SendMessage
**
** Descriptions:    Send message block to CAN1
**
** parameters:      pointer to the CAN message
** Returned value:  true or false, if message buffer is available,
**                  message can be sent successfully, return TRUE,
**                  otherwise, return FALSE.
**
******************************************************************************/
uint32_t CAN1_SendMessage( CAN_MSG *pTxBuf )
{
  uint32_t CANStatus;

  CAN1TxCount++;
  CANStatus = LPC_CAN1->SR;
  if ( CANStatus & 0x00000004 )
  {
  LPC_CAN1->TFI1 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN1->TID1 = pTxBuf->MsgID;
  LPC_CAN1->TDA1 = pTxBuf->DataA;
  LPC_CAN1->TDB1 = pTxBuf->DataB;
  LPC_CAN1->CMR |= 0x21;
  return ( TRUE );
  }
  else if ( CANStatus & 0x00000400 )
  {
  LPC_CAN1->TFI2 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN1->TID2 = pTxBuf->MsgID;
  LPC_CAN1->TDA2 = pTxBuf->DataA;
  LPC_CAN1->TDB2 = pTxBuf->DataB;
  LPC_CAN1->CMR |= 0x41;
  return ( TRUE );
  }
  else if ( CANStatus & 0x00040000 )
  {
  LPC_CAN1->TFI3 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN1->TID3 = pTxBuf->MsgID;
  LPC_CAN1->TDA3 = pTxBuf->DataA;
  LPC_CAN1->TDB3 = pTxBuf->DataB;
  LPC_CAN1->CMR |= 0x81;
  return ( TRUE );
  }
  return ( FALSE );
}

/******************************************************************************
** Function name:   CAN2_SendMessage
**
** Descriptions:    Send message block to CAN2
**
** parameters:      pointer to the CAN message
** Returned value:  true or false, if message buffer is available,
**                  message can be sent successfully, return TRUE,
**                  otherwise, return FALSE.
**
******************************************************************************/
uint32_t CAN2_SendMessage( CAN_MSG *pTxBuf )
{
  uint32_t CANStatus;

  CAN2TxCount++;
  CANStatus = LPC_CAN2->SR;
  if ( CANStatus & 0x00000004 )
  {
  LPC_CAN2->TFI1 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN2->TID1 = pTxBuf->MsgID;
  LPC_CAN2->TDA1 = pTxBuf->DataA;
  LPC_CAN2->TDB1 = pTxBuf->DataB;
  LPC_CAN2->CMR |= 0x21;
  return ( TRUE );
  }
  else if ( CANStatus & 0x00000400 )
  {
  LPC_CAN2->TFI2 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN2->TID2 = pTxBuf->MsgID;
  LPC_CAN2->TDA2 = pTxBuf->DataA;
  LPC_CAN2->TDB2 = pTxBuf->DataB;
  LPC_CAN2->CMR |= 0x41;
  return ( TRUE );
  }
  else if ( CANStatus & 0x00040000 )
  {
  LPC_CAN2->TFI3 = pTxBuf->Frame & 0xC00F0000;
  LPC_CAN2->TID3 = pTxBuf->MsgID;
  LPC_CAN2->TDA3 = pTxBuf->DataA;
  LPC_CAN2->TDB3 = pTxBuf->DataB;
  LPC_CAN2->CMR |= 0x81;
  return ( TRUE );
  }
  return ( FALSE );
}
