/*
 * inttofloat.h
 *
 *  Created on: Jun 9, 2015
 *      Author: Stuff
 */

#ifndef INTTOFLOAT_H_
#define INTTOFLOAT_H_

#include <stdint.h>

union
{
	uint32_t 	_uint;
	int32_t		_int;
	float		_float;
}_converter;

float conv_uint_float(uint32_t _inVal);
float conv_int_float(int32_t _inVal);

uint32_t conv_float_uint(float _inVal);
uint32_t conv_int_uint(int32_t _inVal);

int32_t conv_float_int(float _inVal);
int32_t conv_uint_int(uint32_t _inVal);

#endif /* INTTOFLOAT_H_ */
