/*
 * inputs.h
 *
 *  Created on: Sep 6, 2015
 *      Author: Stuff
 */

#ifndef INPUTS_H_
#define INPUTS_H_

#include "LPC17xx.h"

struct INPUT_STRUCT // in structure for easy logical expansion to other events (on press, timed hold)
{
  uint8_t input_dwn;
}inputs;

#define INPUTS_DEC_DWN        ((inputs.input_dwn & 0x01) >> 0)
#define INPUTS_INC_DWN        ((inputs.input_dwn & 0x02) >> 1)
#define INPUTS_LEFT_DWN       ((inputs.input_dwn & 0x04) >> 2)
#define INPUTS_RIGHT_DWN      ((inputs.input_dwn & 0x08) >> 3)
#define INPUTS_SEL_DWN        ((inputs.input_dwn & 0x10) >> 4)
#define INPUTS_CRU_UP         ((inputs.input_dwn & 0x20) >> 5)
#define INPUTS_CRU_DWN        ((inputs.input_dwn & 0x40) >> 6)
#define INPUTS_UNUSED_1       ((inputs.input_dwn & 0x80) >> 7)

#define SET_INPUTS_DEC_DWN    inputs.input_dwn |= 0x01;
#define SET_INPUTS_INC_DWN    inputs.input_dwn |= 0x02;
#define SET_INPUTS_LEFT_DWN   inputs.input_dwn |= 0x04;
#define SET_INPUTS_RIGHT_DWN  inputs.input_dwn |= 0x08;
#define SET_INPUTS_SEL_DWN    inputs.input_dwn |= 0x10;
#define SET_INPUTS_CRU_UP     inputs.input_dwn |= 0x20;
#define SET_INPUTS_CRU_DWN    inputs.input_dwn |= 0x40;
#define SET_INPUTS_UNUSED_1   inputs.input_dwn |= 0x80;

#define CLR_INPUTS_DEC_DWN    inputs.input_dwn &= 0xFE;
#define CLR_INPUTS_INC_DWN    inputs.input_dwn &= 0xFD;
#define CLR_INPUTS_LEFT_DWN   inputs.input_dwn &= 0xFB;
#define CLR_INPUTS_RIGHT_DWN  inputs.input_dwn &= 0xF7;
#define CLR_INPUTS_SEL_DWN    inputs.input_dwn &= 0xEF;
#define CLR_INPUTS_CRU_UP     inputs.input_dwn &= 0xDF;
#define CLR_INPUTS_CRU_DWN    inputs.input_dwn &= 0xBF;
#define CLR_INPUTS_UNUSED_1   inputs.input_dwn &= 0x7F;

uint8_t btn_release_select(void);
uint8_t btn_release_increment(void);
uint8_t btn_release_decrement(void);
uint8_t btn_release_left(void);
uint8_t btn_release_right(void);

uint8_t btn_release_inc_dec(void);
uint8_t btn_release_inc_sel(void);
uint8_t btn_release_dec_sel(void);
uint8_t btn_release_left_right(void);

uint8_t swt_cruise(void);

#endif /* INPUTS_H_ */
