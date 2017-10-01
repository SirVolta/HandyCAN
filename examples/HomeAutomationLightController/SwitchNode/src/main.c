/**
 * @file    main.c
 * @author  SirVolta
 * @date    Sep 18, 2017
 * @brief   Home automation switch node
 * @note    Interfaces a switch to the CAN bus
 */
/*
 Copyright (C) 2017 Pelle Sepp Florens Jansen

 This file is part of HandyCAN Home Automation Demo

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

/*! \mainpage HandyCAN Switch node
 *
 * \section intro_sec Introduction
 *
 * This is a switchnode. It reads three switches, and sends their state if they change. \n
 * The switch intent is as following: [0x02, switch_idx, switch_state]\n
 * The switchnode also responds to the universal discover intent 0xFF:
 * [0x0E]
 *
 * HandyCAN is Copyright (C) 2017 Pelle Sepp Florens Jansen
 * and is free software under the terms of the GNU General Public License v3
 *
 *
 *
 *
 */

#include "stm32f10x_conf.h"
#include "svlib_stm32f10x.h"
#include "HandyCAN.h"
//#include <stdio.h>
//#include <stdlib.h>

#define SWITCHNODE_ID 0x01
#define DEBOUNCETIME 50 //ms

#define INTENT_NODE_UPTIME 0x01
#define INTENT_INTERLINK_SWITCH_CHANGED 0x02
#define INTENT_NODE_GET_SWITCH_STATE 0x03
#define INTENT_INVALID_INTENT 0xFD
#define INTENT_NODE_DISCOVER 0xFF
#define INTENT_NODE_DISCOVER_RESPONSE 0xFE

uint64_t systick_ms = 0;

/*!
 @brief Called when package addressed to us is received
 @note needs changing if peripheral is not CAN1
 */
extern void
USB_LP_CAN1_RX0_IRQHandler (void)
{
  struct HandyCAN_package package;
  struct time_types uptime;
  uint8_t data[8];
  HandyCAN_recievePackage(CAN_FIFO0, &package);
  //HandyCAN_dumpRxPackage(&package);
  switch (package.data[0])
    {
    case INTENT_NODE_UPTIME:
      uptime.seconds = (uint32_t) (systick_ms / 1000);
      secondsToTime(&uptime);
      data[0] = INTENT_NODE_UPTIME;
      data[1] = uptime.seconds;
      data[2] = uptime.minutes;
      data[3] = uptime.hours;
      data[4] = uptime.days;
      HandyCAN_transmit(package.source_adress, data, 5);
      break;
    case INTENT_NODE_GET_SWITCH_STATE:
      data[0] = INTENT_NODE_GET_SWITCH_STATE;
      data[1] = package.data[1];
      data[2] = GPIO_ReadInputDataBit(GPIOA, 1 << package.data[1]);
      HandyCAN_transmit(package.source_adress, data, 3);
      break;

    default:
      data[0] = INTENT_INVALID_INTENT;
      HandyCAN_transmit(package.source_adress, data, 0);
      break;
    }
}

/*!
 @brief Called when broadcast package is received
 @note needs changing if peripheral is not CAN1
 */
extern void
CAN1_RX1_IRQHandler (void)
{
  struct HandyCAN_package package;
  uint8_t data[8];

  trace_puts("BCast");
  HandyCAN_recievePackage(CAN_FIFO1, &package);
  HandyCAN_dumpRxPackage(&package);
  if (package.data[0] == INTENT_NODE_DISCOVER)
    {
      data[0] = INTENT_NODE_DISCOVER_RESPONSE;
      data[1] = SWITCHNODE_ID;
      HandyCAN_transmit(package.source_adress, data, 2);
    }

}

/**
 * @brief Handles interrupt of brake, start and reverse switch
 */
extern void
EXTI15_10_IRQHandler (void)
{
  //last time switch was pressed to debounce switch
  static uint64_t lastsw1 = 0, lastsw2 = 0, lastsw3 = 0;
  uint8_t data[8] =
    {INTENT_INTERLINK_SWITCH_CHANGED};

  if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
      if (systick_ms - lastsw1 > DEBOUNCETIME)
	{
	  data[1] = 11;
	  data[2] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
	  HandyCAN_transmit(0, data, 3);
	  lastsw1 = systick_ms;
	}
      EXTI_ClearITPendingBit(EXTI_Line11);
    }

  //start switch
  if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
      if (systick_ms - lastsw2 > DEBOUNCETIME)
	{
	  data[1] = 12;
	  data[2] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
	  HandyCAN_transmit(0, data, 3);
	  lastsw2 = systick_ms;
	}
      EXTI_ClearITPendingBit(EXTI_Line12);
    }

  //sw1 reverse switch
  if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
      if (systick_ms - lastsw3 > DEBOUNCETIME)
	{
	  data[1] = 15;
	  data[2] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15);
	  HandyCAN_transmit(0, data, 3);
	  lastsw3 = systick_ms;
	}
      EXTI_ClearITPendingBit(EXTI_Line15);
    }
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
  NVIC_InitTypeDef NVIC_InitStruct;
  EXTI_InitTypeDef EXTI_InitStruct;
  uint8_t data[8];
  struct time_types uptime;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

  // Configure CAN pin: B8: RX
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure CAN pin: B9: TX
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);

  // onboard led
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);

  /// Set switches to input with weak pullups
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  /// enable interrupts on the switches
  NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  /// Configure the switches to create an interrupt on both edges
  EXTI_InitStruct.EXTI_Line = EXTI_Line11 | EXTI_Line12 | EXTI_Line15;
  EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStruct.EXTI_LineCmd = ENABLE;
  /// Connect the IO pins to the interrupt controller
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,
  GPIO_PinSource11 | GPIO_PinSource12 | GPIO_PinSource15);
  EXTI_Init(&EXTI_InitStruct);
  EXTI_ClearITPendingBit(EXTI_Line11 | EXTI_Line12 | EXTI_Line15);

  //SysTick timer
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SysTick_Config(SystemCoreClock / 1000);
  SystemCoreClockUpdate();
  systick_ms = 0;

  initDWT();
  HandyCAN_init(CAN1, 0x01, CAN_Mode_Normal, USB_LP_CAN1_RX0_IRQn,
		CAN1_RX1_IRQn);

  trace_puts("HandyCAN Switchnode Ready");

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

//HandyCAN_transmit(HC_BROADCAST_ADDR, data, 7);

      delayUs(5000 * 1000);
    }
}
