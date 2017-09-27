/**
 * @file    svlib_stm32f10x.h
 * @author  SirVolta
 * @ide     Emacs
 * @date    Mar 1, 2016
 * @brief   SirVolta's library
 * @note    Function descriptions are in source file
 */
/*
 Copyright (C) 2015 Pelle Sepp Florens Jansen

 This file is part of SirVolta's Library

 SirVolta's Library is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 SirVolta's Library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with SirVolta's Library.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef SVLIB_STM32F10X_H_
#define SVLIB_STM32F10X_H_
#include <stdint.h>
#include "stm32f10x_conf.h"
#include "diag/Trace.h"


/*!
 @brief DWT clock cycles. Useful for timing execution.
 @section Time timer Example
 @code
 uint32_t start, end;
 // DWT_CYCLES = 0; // use if you worried about overflow during measurement
 ...
 start = DWT_CYCLES;
 // code to be measured
 end = DWT_CYCLES;
 trace_printf("%d ms\r\n",((end-start)/72000)); // at 72MHz
 @endcode
 */
#define DWT_CYCLES DWT->CYCCNT

#define GPIO_ToggleBits(GPIOx, GPIO_Pin) GPIO_WriteBit(GPIOx, GPIO_Pin, !GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin));
#define delayMs(ms) delayUs (ms * 1000);
#define print_size(str, object) trace_printf("%s: %i bytes %i bits\n", str, sizeof(object), sizeof(object) * 8)

struct time_types
{
 uint32_t days;
 uint32_t hours;
 uint32_t minutes;
 uint32_t seconds;
};

void
secondsToTime (struct time_types * tim);

void
initDWT (void);

void
delayUs (uint32_t us);

#endif /* SVLIB_STM32F10X_H_ */
