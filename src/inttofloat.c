/*
 * inttofloat.c
 *
 *  Created on: 21 Jul 2015
 *      Author: Stuff
 */

#include "inttofloat.h"

float conv_uint_float(uint32_t _inVal){_converter._uint = _inVal;return _converter._float;}
float conv_int_float(int32_t _inVal){_converter._int = _inVal;return _converter._float;}

uint32_t conv_float_uint(float _inVal){_converter._float = _inVal;return _converter._uint;}
uint32_t conv_int_uint(int32_t _inVal){_converter._int = _inVal;return _converter._uint;}

int32_t conv_float_int(float _inVal){_converter._float = _inVal;return _converter._int;}
int32_t conv_uint_int(uint32_t _inVal){_converter._uint = _inVal;return _converter._int;}
