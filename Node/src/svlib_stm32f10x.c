/**
 * @file    svlib_stm32f10x.c
 * @author  SirVolta
 * @ide     Emacs
 * @date    Mar 1, 2016
 * @brief   SirVolta's library
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

#include "svlib_stm32f10x.h"

/* Global constants */
//const int8_t a;

/* Global variables */
//int8_t a;

/* Variables local to svlib */
//static int8_t a;

/* constants local to svlib */
//const static int8_t a;

/* local defines */
// #define CONVERT_THING(x) x ? init_thing(x) : error_thing(x)

/* local function prototypes */
// static void svlib_thing(void);

/*!
 @brief  converts seconds to days, hours, minutes and seconds
 @param[in,out] time_types pointer with seconds initialized
 @note 86400 is amount of seconds in one day
 */
void
secondsToTime(struct time_types * tim)
{
 tim->days = (uint32_t)(tim->seconds / 86400);
 tim->seconds -= tim->days * 86400;
 tim->hours = (uint32_t)(tim->seconds / 3600);
 tim->seconds -= tim->hours * 3600;
 tim->minutes = (uint32_t)(tim->seconds / 60);
 tim->seconds -= tim->minutes * 60;
}

/*!
 @brief  Initializes Data Watchpoint and Trace Registers (DWT) processor cycle counter.
 Usually, this is already done by uOS clock init and is not required.
 */
void
initDWT (void)
{
   CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
   DWT->CYCCNT = 0;
   DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*!
 @brief  Holds up execution by n microseconds to maximum 52 seconds.
 Reqiures DWT to be initialized.
 @param[in] us
 Microseconds to stall execution
 */
void
delayUs (uint32_t us)
{
 int32_t tp = DWT_CYCLES + us * (SystemCoreClock / 1000000);
 tp += 2;
 assert_param (us < (52000000)); // max 52 second delay, check after storing initial cycle count
 while (((int32_t) DWT_CYCLES - tp) < 0)
  ;
}

