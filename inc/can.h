/****************************************************************************
 *   $Id:: can.h 5666 2010-11-19 01:02:14Z usb00423                         $
 *   Project: NXP LPC17xx CAN example
 *
 *   Description:
 *     This file contains CAN code header definition.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#ifndef __CAN_H 
#define __CAN_H

#define CAN_WAKEUP					0
#define ACCEPTANCE_FILTER_ENABLED	1

#define MAX_PORTS	2		/* Number of CAN port on the chip */		

/* BRP+1 = Fpclk/(CANBitRate * QUANTAValue)
   QUANTAValue = 1 + (Tseg1+1) + (Tseg2+1)
   QUANTA value varies based on the Fpclk and sample point
   e.g. (1) sample point is 87.5%, Fpclk is 48Mhz
   the QUANTA should be 16
        (2) sample point is 90%, Fpclk is 12.5Mhz
   the QUANTA should be 10 
   Fpclk = Fclk /APBDIV
   or
   BitRate = Fcclk/(APBDIV * (BRP+1) * ((Tseg1+1)+(Tseg2+1)+1))
*/ 	
/* Here are some popular bit timing settings for LPC23xx, google on "SJA1000"
CAN bit timing, the same IP used inside LPC2000 CAN controller. There are several 
bit timing calculators on the internet. 
http://www.port.de/engl/canprod/sv_req_form.html
http://www.kvaser.com/can/index.htm
*/

/**
 * Bit Timing Values for 16MHz clk frequency
 */
#define BITRATE100K16MHZ          0x001C0009
#define BITRATE125K16MHZ          0x001C0007
#define BITRATE250K16MHZ          0x001C0003
#define BITRATE500K16MHZ          0x001C0001
#define BITRATE1000K16MHZ         0x001C0000

/*
 * Bit Timing Values for 24MHz clk frequency
 */
#define BITRATE100K24MHZ          0x001C000E
#define BITRATE125K24MHZ          0x001C000B
#define BITRATE250K24MHZ          0x001C0005
#define BITRATE500K24MHZ          0x001C0002
#define BITRATE1000K24MHZ         0x00090001

/*
 * Bit Timing Values for 48MHz clk frequency
 */
#define BITRATE100K48MHZ          0x001C001D
#define BITRATE125K48MHZ          0x001C0017
#define BITRATE250K48MHZ          0x001C000B
#define BITRATE500K48MHZ          0x001C0005
#define BITRATE1000K48MHZ         0x001C0002

/*
 * Bit Timing Values for 60MHz clk frequency
 */
#define BITRATE100K60MHZ          0x00090031
#define BITRATE125K60MHZ          0x00090027
#define BITRATE250K60MHZ          0x00090013
#define BITRATE500K60MHZ          0x00090009
#define BITRATE1000K60MHZ         0x00090004

/*
 * Bit Timing Values for 28.8MHz pclk frequency, 1/2 of 57.6Mhz CCLK
 */
#define BITRATE100K28_8MHZ        0x00090017

/*
 * Bit Timing Values for 18MHz pclk frequency, 1/4 of 72Mhz CCLK
 */
#define BITRATE125K18MHZ        0x001C0008

/* When Fcclk is 50Mhz and 60Mhz and APBDIV is 4,
so Fpclk is 12.5Mhz and 15Mhz respectively. 
when Fpclk is 12.5Mhz, QUANTA is 10 and sample point is 90% 
when Fpclk is 15Mhz, QUANTA is 10 and sample point is 90% */

/* Common CAN bit rates for 12.5Mhz(50Mhz CCLK) clock frequency */
#define BITRATE125K12_5MHZ		0x00070009
#define BITRATE250K12_5MHZ		0x00070004

/**
 * Bit Timing Values for 15MHz(60Mhz CCLK) clk frequency
 */
#define BITRATE62_5K15MHZ		0x00070017
#define BITRATE100K15MHZ		0x0007000E
#define BITRATE125K15MHZ		0x0007000B
#define BITRATE250K15MHZ		0x00070005
#define BITRATE500K15MHZ		0x00070002

/**
 * Bit Timing Values for 30MHz(120Mhz CCLK) clk frequency
 */
#define BITRATE125K30MHZ		0x00070017
#define BITRATE200K30MHZ		0x0007000E
#define BITRATE250K30MHZ		0x0007000B
#define BITRATE500K30MHZ		0x00070005
#define BITRATE1000K30MHZ		0x00070002

/* Acceptance filter mode in AFMR register */
#define ACCF_OFF				0x01
#define ACCF_BYPASS				0x02
#define ACCF_ON					0x00
#define ACCF_FULLCAN			0x04

/* This number applies to all FULLCAN IDs, explicit STD IDs, group STD IDs, 
explicit EXT IDs, and group EXT IDs. */ 
#define ACCF_IDEN_NUM			4

/* Identifiers for FULLCAN, EXP STD, GRP STD, EXP EXT, GRP EXT */
//#define MPPT_RPLY				0x060		// Distance to MPPT reply packet 0b000001100000


/// CAN BASE ADDRESSES
#ifndef ESC_BASE
#define ESC_BASE				0x400		// Base address of motor controller
#endif

#ifndef ESC_CONTROL
#define ESC_CONTROL				0x500		// Base for control packets for motor controller(s)
#endif

#ifndef DASH_RPLY
#define DASH_RPLY				0x510		// Messages from dash
#endif

#ifndef DASH_RQST
#define DASH_RQST				0x520		// Messages to dash
#endif

#ifndef BMU_BASE
#define BMU_BASE 				0x600
#endif

#ifndef MPP1_BASE
#define MPPT1_BASE				0x716		// DriveTek Master Request 0b011100010110
#endif

#ifndef MPPT2_BASE
#define MPPT2_BASE				0x719		// DriveTek Master Request 0b011100011001
#endif

#ifndef MPPT1_RPLY
#define MPPT1_RPLY 				0x776		// DriveTek Slave Answer Frame -- 0110 0b011101110110
#endif

#ifndef MPPT2_RPLY
#define MPPT2_RPLY 				0x779		// DriveTek Slave Answer Frame -- 1001 0b011101111001
#endif


#define BMU_INFO 				0xF4		// Add to base to skip CMU packets

#define GRP_STD_ID				0x200
#define EXP_EXT_ID				0x100000
#define GRP_EXT_ID				0x200000


// Type definition to hold a CAN message
typedef struct
{
	uint32_t Frame; // Bits 16..19: DLC - Data Length Counter
					// Bit 30: Set if this is a RTR message
					// Bit 31: Set if this is a 29-bit ID message
	uint32_t MsgID;	// CAN Message ID (11-bit or 29-bit)
	uint32_t DataA;	// CAN Message Data Bytes 0-3
	uint32_t DataB;	// CAN Message Data Bytes 4-7
} CAN_MSG;


/**************************************************************************
PUBLIC FUNCTIONS
***************************************************************************/
uint32_t CAN1_Init( uint32_t can_btr );
uint32_t CAN2_Init( uint32_t can_btr );
void CAN_SetACCF_Lookup( void );
void CAN_SetACCF( uint32_t ACCFMode );
uint32_t CAN1_SendMessage( CAN_MSG* pTXBuf );
uint32_t CAN2_SendMessage( CAN_MSG* pTXBuf );
void setCANBUS1(void);
void setCANBUS2(void);

#endif	// __CAN_H

/******************************************************************************
**                            End Of File
******************************************************************************/

