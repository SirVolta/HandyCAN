/**
 * @file    main.c
 * @author  SirVolta
 * @date    Sep 18, 2017
 * @brief   Home automation sensor node
 * @note    Interfaces multiple sensors to the CAN bus
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
 * This is a sensor node. It reads a light, motion temperature and humidity sensor. \n
 * It will send their data on request, or if they change more then a set amount \n
 * The sensor node also responds to the universal discover intent 0xFF:
 * [0x0E]
 *
 * HandyCAN is Copyright (C) 2017 Pelle Sepp Florens Jansen
 * and is free software under the terms of the GNU General Public License v3
 *
 */

#include "stm32f10x_conf.h"
#include "svlib_stm32f10x.h"
#include "HandyCAN.h"
#include "DHT1122.h"
//#include <stdio.h>
//#include <stdlib.h>

#define SENSORNODE_ID 0x02

#define INTENT_NODE_UPTIME 0x01
#define INTENT_GET_TEMP 0x02
#define INTENT_MOTION_DETECTED 0x03
#define INTENT_GET_LIGHT 0x04
#define INTENT_TEMP_ALARM 0x05

#define INTENT_INVALID_INTENT 0xFD
#define INTENT_NODE_DISCOVER 0xFF
#define INTENT_NODE_DISCOVER_RESPONSE 0xFE

#define DEBOUNCETIME 50

uint64_t systick_ms = 0;
static DHT1122struct dht;

/**
 * @brief reads from ADC periperal periperhal, blocks until conversion complete.
 * @note Initialize ADC and set channel to read first
 * @param ADCx: Which ADC to read from
 * @return 16-bit read value from ADC.
 */
uint16_t
adc_read (ADC_TypeDef* ADCx)
{
  uint16_t ad_value;
  ADC_SoftwareStartConvCmd(ADCx, ENABLE);
  //wait for conversion complete
  while (!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC))
    ;
  ad_value = ADC_GetConversionValue(ADCx);
  //clear end of conversion flag
  ADC_ClearFlag(ADCx, ADC_FLAG_EOC);
  return ad_value;
}

/**
 * @brief Select an ADC channel to read from
 * @param ADCx: Which ADC to read from
 * @param ADC_Channel: ADC channel to read from
 * @param ADC_SampleTime: time to take for sampling
 * @note check the doc on ADC_RegularChannelConfig() for possible parameters
 */
void
adc_set_channel (ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t ADC_SampleTime)
{
  //Disable the ADC before changing the channel
  ADC_Cmd(ADC1, DISABLE);
  ///Rank is one, as it doesn't matter for single channel conversion mode
  ADC_RegularChannelConfig(ADCx, ADC_Channel, 1, ADC_SampleTime);
  //reenable the adc
  ADC_Cmd(ADC1, ENABLE);
}

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
  uint8_t error;
  uint16_t analog_value;

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

    case INTENT_GET_TEMP:
      error = dht1122_read(&dht);
      data[0] = INTENT_GET_TEMP;
      data[1] = error;
      data[2] = dht.temp_decimal;
      data[3] = dht.temp_intergral;
      data[4] = dht.hum_decimal;
      data[5] = dht.hum_intergral;
      HandyCAN_transmit(package.source_adress, data, 6);
      break;

    case INTENT_MOTION_DETECTED:
      data[0] = INTENT_MOTION_DETECTED;
      data[1] = 11;
      data[2] = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
      HandyCAN_transmit(package.source_adress, data, 3);
      break;

    case INTENT_GET_LIGHT:
      adc_set_channel(ADC1, ADC_Channel_6, ADC_SampleTime_41Cycles5);
      analog_value = adc_read(ADC1);
      data[0] = INTENT_GET_LIGHT;
      data[1] = (uint8_t) (analog_value & 0xFF);
      data[2] = (uint8_t) ((analog_value & 0xFF00) >> 8);
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
      data[1] = SENSORNODE_ID;
      HandyCAN_transmit(package.source_adress, data, 2);
    }

}

/**
 * @brief Handles interrupt of the motion sensor
 */
extern void
EXTI15_10_IRQHandler (void)
{
  //last time switch was pressed to debounce switch
  static uint64_t lastsw1 = 0, lastsw2 = 0, lastsw3 = 0;
  uint8_t data[8] =
    {INTENT_MOTION_DETECTED};

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

  if (EXTI_GetITStatus(EXTI_Line12) != RESET)
    {
      if (systick_ms - lastsw2 > DEBOUNCETIME)
	{
	  lastsw2 = systick_ms;
	}
      EXTI_ClearITPendingBit(EXTI_Line12);
    }

  if (EXTI_GetITStatus(EXTI_Line15) != RESET)
    {
      if (systick_ms - lastsw3 > DEBOUNCETIME)
	{
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

uint8_t
dht1122_readint (DHT1122struct* dht, int16_t* temp, uint16_t* hum)
{
  uint8_t error;

  if (!(error = dht1122_read(dht)))
    {
      if (dht->temp_decimal)
	{
	  *temp = (dht->temp_intergral & 0x7Fu) * 256u + dht->temp_decimal;
	  if (dht->temp_intergral & 0x80u)
	    *temp *= -1;
	}
      else
	*temp = dht->temp_intergral * 10u;
      if (dht->hum_decimal)
	*hum = dht->hum_intergral * 256u + dht->hum_decimal;
      else
	*hum = dht->hum_intergral * 10u;
    }

  return error;
}

int
main (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  EXTI_InitTypeDef EXTI_InitStruct;
  ADC_InitTypeDef ADC_InitStruct;
  uint8_t data[8];
  struct time_types uptime;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

  /// disable JTAG to free up RB3 and RB4
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

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
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
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

  /// ADC pins, Analog input
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  ///Setup ADC1
  ADC_StructInit(&ADC_InitStruct);
  //Enable the clock for ADC1
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  //select independent conversion mode (single)
  ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
  //We will convert single channel only
  ADC_InitStruct.ADC_ScanConvMode = DISABLE;
  //we will convert one time
  ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
  //select no external triggering
  ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  //right 12-bit data alignment in ADC data register
  ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
  //single channel conversion
  ADC_InitStruct.ADC_NbrOfChannel = 1;
  //load structure values to control and status registers
  ADC_Init(ADC1, &ADC_InitStruct);

  dht.IO_Pin = GPIO_Pin_3;
  dht.IO_Port = GPIOB;

  //SysTick timer
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
  SysTick_Config(SystemCoreClock / 1000);
  SystemCoreClockUpdate();
  systick_ms = 0;

  initDWT();
  HandyCAN_init(CAN1, 0x02, CAN_Mode_Normal, USB_LP_CAN1_RX0_IRQn,
		CAN1_RX1_IRQn);

  trace_puts("HandyCAN Switchnode Ready");

  while (1)
    {

      delayUs(5000 * 1000);
    }
}
