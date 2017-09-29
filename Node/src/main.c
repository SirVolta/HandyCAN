/**
 * @file    main.c
 * @author  SirVolta
 * @date    Sep 18, 2017
 * @brief   HandyCAN node library test
 * @note    Test and basic loopback demo of HandyCAN
 */
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

/*! \mainpage HandyCAN Node library and loopback demo
 *
 * \section intro_sec Introduction
 *
 * This is the HandyCAN node library.\n
 * To build a handyCAN node, include the HandyCAN.c and HandyCAN.h files into your project.\n
 *
 * HandyCAN is Copyright (C) 2017 Pelle Sepp Florens Jansen
 * and is free software under the terms of the GNU General Public License v3
 *
 *
 * \section nav_sec Navigation
 * If you are building a node, see HandyCAN.h for the library reference.\n
 * A loopback example is in main.c. For more comprehensive examples please see the github examples page.\n
 * To check for still unfinished components, please check the todo list under related pages.\n
 *
 * \see https://github.com/SirVolta/HandyCAN/tree/master/examples
 *
 *
 */

#include "stm32f10x_conf.h"
#include "svlib_stm32f10x.h"
#include "HandyCAN.h"
//#include <stdio.h>
//#include <stdlib.h>

#define INTENT_NODE_UPTIME 111

uint64_t systick_ms = 0;

/*!
 @brief Called when package addressed to us is received
 @note needs changing if peripheral is not CAN1
 */
extern void
USB_LP_CAN1_RX0_IRQHandler (void)
{
  struct HandyCAN_package package;

  HandyCAN_recievePackage(CAN_FIFO0, &package);
  //HandyCAN_dumpRxPackage(&package);
  if (package.data[0] == 2)
    GPIO_WriteBit(GPIOC, GPIO_Pin_13, package.data[1]);
}

/*!
 @brief Called when broadcast package is received
 @note needs changing if peripheral is not CAN1
 */
extern void
CAN1_RX1_IRQHandler (void)
{
  struct HandyCAN_package package;

  trace_puts("BCast");
  HandyCAN_recievePackage(CAN_FIFO1, &package);
  HandyCAN_dumpRxPackage(&package);
}

extern void
SysTick_Handler (void)
{
  systick_ms++;
}

int
main (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  uint8_t data[8];
  struct time_types uptime;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  // Configure CAN pin: A11: RX
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure CAN pin: A12: TX
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  // onboard led
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  //SysTick timer
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SysTick_Config(SystemCoreClock / 1000);
  SystemCoreClockUpdate();
  systick_ms = 0;

  initDWT();
  //Init handycan: CAN1, our address is 0x0F, loopback mode for testing, CAN1 interrupts.
  HandyCAN_init(CAN1, 0x0A, CAN_Mode_Normal, USB_LP_CAN1_RX0_IRQn,
		CAN1_RX1_IRQn);

  // test data
  data[5] = 0;   // counter byte

  trace_puts("HandyCAN Ready");

  while (1)
    {
      while (HandyCAN_remainingMailboxes())
	{
	  /// Place the system uptime in the data bytes as a example
	  uptime.seconds = (uint32_t) (systick_ms / 1000);
	  secondsToTime(&uptime);
	  data[0] = INTENT_NODE_UPTIME;
	  data[1] = uptime.seconds;
	  data[2] = uptime.minutes;
	  data[3] = uptime.hours;
	  data[4]++;
	  data[5] = 0xE0;
	  data[6] = 0xEF;

	  HandyCAN_transmit(0x0A, data, 7);
	}
    }

  while (1)
    {
      /// Place the system uptime in the data bytes as a example
      uptime.seconds = (uint32_t) (systick_ms / 1000);
      secondsToTime(&uptime);
      data[0] = INTENT_NODE_UPTIME;
      data[1] = uptime.seconds;
      data[2] = uptime.minutes;
      data[3] = uptime.hours;
      data[4]++;
      data[5] = 0xE0;
      data[6] = 0xEF;

      /// There should be 3 mailboxes avaliable
      trace_printf("Available: %u\n", HandyCAN_remainingMailboxes());

      /// Transmit data to ourself, then increment the counter,
      /// And broadcast everything but the days.
      HandyCAN_transmit(0x0A, data, 7);

      delayUs(2500 * 1000);

      uptime.seconds = (uint32_t) (systick_ms / 1000);
      secondsToTime(&uptime);
      data[0] = INTENT_NODE_UPTIME;
      data[1] = uptime.seconds;
      data[2] = uptime.minutes;
      data[3] = uptime.hours;
      data[4]++;
      data[5] = 0xF0;
      data[6] = 0xFA;

      HandyCAN_transmit(HC_BROADCAST_ADDR, data, 7);

      trace_printf("Available: %u\n", HandyCAN_remainingMailboxes());

      delayUs(2500 * 1000);
    }
}
