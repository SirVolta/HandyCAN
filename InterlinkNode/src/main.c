/**
 * @file    main.c
 * @author  SirVolta
 * @date    Sep 18, 2017
 * @brief   HandyCAN interlink node
 * @note    Connects the CAN bus to the PC
 */
/*
 Copyright (C) 2017 Pelle Sepp Florens Jansen

 This file is part of HandyCAN Interlink Node

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
 * This is the HandyCAN interlink node firmware\n
 * An interlink node serves as a bridge between a PC and the HandyCAN network\n
 * There is no limit to the amount of interlink nodes in a setup\n
 * They are not required.\n
 * \n
 * For simplicity and because the interlink node firmware is not very complex,\n
 * it is entirely implemented in main.c.\n
 *
 * HandyCAN is Copyright (C) 2017 Pelle Sepp Florens Jansen
 * and is free software under the terms of the GNU General Public License v3
 *
 *
 * \section nav_sec Usage
 * One or more interlink nodes are connected to the CAN bus and,
 * via a fast UART, to the PC.
 *
 * \section proto_sec Protocol
 * The handyCAN protocol is described here: https://github.com/SirVolta/HandyCAN/tree/master/doc/protocol \n
 * Following is a description of the UART protocol between the PC and the interlink node:\n
 *
 *
 *
 * \see https://github.com/SirVolta/HandyCAN/tree/master/examples
 *
 *
 */

//#include <stdio.h>
//#include <stdlib.h>
#include "diag/Trace.h"
#include "stm32f10x_conf.h"

///Toggles a IO pin.
#define GPIO_ToggleBits(GPIOx, GPIO_Pin) GPIO_WriteBit(GPIOx, GPIO_Pin, !GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin));

///Buffer. Way oversized. Can never be more than 15 large.
uint8_t buf[80];

static struct CANMessageToSend
{
  ///Indicates if a message is ready
  uint8_t messageReady;
  /// The message to be sent to the PC
  CanRxMsg message;
} messageToSend;

/*!
 @brief Called when there is a package on the CAN bus
 @note Buffers the data to keep processing out of the ISR
 */
extern void
USB_LP_CAN1_RX0_IRQHandler (void)
{
  CanRxMsg rx_message;
  CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
  if (!messageToSend.messageReady)
    {
      messageToSend.message = rx_message;
      messageToSend.messageReady = 1;
    }
  else
    trace_puts("Overrun!");

  GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
}

static void
sendCANMessage (void)
{
  uint8_t len = messageToSend.message.DLC;
  uint8_t i;
  // 0 and 1 indicates start of frame
  buf[0] = 0xAA;
  buf[1] = 0xAB;
  // 2 and 3 is StdId
  buf[2] = (uint8_t) (messageToSend.message.StdId & 0xFF);
  buf[3] = (uint8_t) ((messageToSend.message.StdId & 0xFF00) >> 8);
  // 4 is data length
  buf[4] = len;
  // next comes the data
  for (i = 5; i < len + 5; i++)
    buf[i] = messageToSend.message.Data[i - 5];
  // and the last two to indicate end
  buf[i++] = 0xBA;
  buf[i++] = 0xBB;

  for (uint8_t j = 0; j < i; j++)
    {
      while (USART_GetFlagStatus (USART1, USART_FLAG_TC) == RESET);
      USART_SendData(USART1, buf[j]);
      while (USART_GetFlagStatus (USART1, USART_FLAG_TC) == RESET);
      //trace_printf("buf[%u]: %#x, %u\n", j, buf[j], buf[j]);
    }

}

static void
CAN_init (void)
{
  CAN_InitTypeDef CAN_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  CAN_FilterInitTypeDef CAN_FilterInitStruct;

  CAN_DeInit(CAN1);
  CAN_StructInit(&CAN_InitStruct);
  CAN_InitStruct.CAN_TTCM = DISABLE;
  CAN_InitStruct.CAN_ABOM = DISABLE;
  CAN_InitStruct.CAN_AWUM = DISABLE;
  CAN_InitStruct.CAN_NART = ENABLE;
  CAN_InitStruct.CAN_RFLM = DISABLE;
  CAN_InitStruct.CAN_TXFP = DISABLE;
  CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStruct.CAN_Prescaler = 4; //4: 1M, 8: 500k
  CAN_Init(CAN1, &CAN_InitStruct);

  // Set the filter to receive everything into FIFO0
  CAN_FilterInitStruct.CAN_FilterNumber = 0;
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStruct.CAN_FilterIdHigh = 0;
  CAN_FilterInitStruct.CAN_FilterIdLow = 0;
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStruct);

  //Setup for FIFO0 data
  NVIC_InitStruct.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

static void
UART_init (void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
  USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  // Setup the uart interrupt
  //USART_DeInit(USART1);
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable the uart and the uart interrupt
  USART_Cmd(USART1, ENABLE);
  //Enable recieve interrupt on the CLI uart
  NVIC_EnableIRQ(USART1_IRQn);
  //USART_ClearFlag(USART1, USART_FLAG_TC);
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

static void
GPIO_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /// disable JTAG to free up RB3 and RB4
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

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

  /// uart pins, alternate function
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

int
main (void)
{
  messageToSend.messageReady = 0;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  GPIO_init();
  UART_init();
  CAN_init();

  //Setup the watchdog
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  IWDG_SetReload(0xFF);
  IWDG_ReloadCounter();
  IWDG_Enable();

  trace_puts("Interlink node ready!");

  while (1)
    {
      if (messageToSend.messageReady)
	{
	  sendCANMessage();
	  messageToSend.messageReady = 0;
	}
      ///Kick the dog
      IWDG_ReloadCounter();
    }
}

