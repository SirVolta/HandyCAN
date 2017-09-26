/*
 Copyright (C) 2017 Pelle Sepp Florens Jansen

 This file is part of HandyCAN Node

 HandyCAN is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 HandyCAN is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with HandyCAN.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32f10x_conf.h"
#include "svlib_stm32f10x.h"
#include "HandyCAN.h"
//#include <stdio.h>
//#include <stdlib.h>

/*!
 @brief Called when package addressed to us is received
 @note needs changing if peripheral is not CAN1
 */
extern void
USB_LP_CAN1_RX0_IRQHandler (void)
{
  CanRxMsg rx_message;
  struct HandyCAN_package package;

  CAN_Receive (CAN1, CAN_FIFO0, &rx_message);
  HandyCAN_decodeCanRxMsg (&rx_message, &package);
  trace_printf ("Received message from %#x: %u\n", package.source_adress,
		package.data[0]);

  for (uint8_t i = 1; i < package.len; i++)
    trace_printf ("Data[%u]: %#x %u\n", i, package.data[i], package.data[i]);

  GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
}

/*!
 @brief Called when broadcast package is received
 @note needs changing if peripheral is not CAN1
 */
extern void
CAN1_RX1_IRQHandler (void)
{
  CanRxMsg rx_message;
  struct HandyCAN_package package;

  CAN_Receive (CAN1, CAN_FIFO1, &rx_message);
  HandyCAN_decodeCanRxMsg (&rx_message, &package);
  trace_printf ("Received BROADCAST from %#x: %u\n", package.source_adress,
		package.data[0]);

  for (uint8_t i = 1; i < package.len; i++)
    trace_printf ("Data[%u]: %#x %u\n", i, package.data[i], package.data[i]);
}

int
main (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t data[8];

  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd (RCC_APB1Periph_CAN1, ENABLE);

  // Configure CAN pin: A11: RX
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (GPIOA, &GPIO_InitStruct);

  // Configure CAN pin: A12: TX
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (GPIOA, &GPIO_InitStruct);

  // onboard led
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init (GPIOC, &GPIO_InitStruct);

  initDWT ();
  //Init handycan: CAN1, our address is 0x0F, loopback mode for testing, CAN1 interrupts.
  HandyCAN_init (CAN1, 0x0F, CAN_Mode_LoopBack, USB_LP_CAN1_RX0_IRQn,
		 CAN1_RX1_IRQn);

  // test data
  data[0] = 128; //intent byte
  data[1] = 0x00;
  data[2] = 0x22;
  data[3] = 0x32;
  data[4] = 0x42;
  data[5] = 0x52;
  data[6] = 0x62;
  data[7] = 0x72;

  trace_puts ("HandyCAN Ready");

  while (1)
    {
      trace_printf ("Available: %u\n", HandyCAN_remainingMailboxes ());

      data[1]++;
      HandyCAN_transmit (0x0F, data, 4);

      data[1]++;
      HandyCAN_transmit (HC_BROADCAST_ADDR, data, 2);
      trace_printf ("Available: %u\n", HandyCAN_remainingMailboxes ());

      delayUs (3000 * 1000);
    }
}
