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

/*! \mainpage HandyCAN Interlink Node firmware
 *
 * \section intro_sec Introduction
 *
 * This is the HandyCAN interlink node firmware for STM32F10x\n
 * A interlink node serves as a bridge between a PC and the HandyCAN network\n
 * There is no limit to the amount of interlink nodes in a setup
 * and they are not required if PC comms is not needed\n
 * If the CAN bus is run at over 125kbaud, the STM32F10 is not powerful
 * enough to recieve and transmit all packages in realtime.\n
 * But, it can receive OR transmit at a maximum of 500kbaud.\n
 * So if a speed lower then 500k but higher then 125k is required
 * you can use two Interlink nodes: one for receive, one for transmit.\n
 * Connect the UART RX of the pc to the UART TX of the first interlink node connected to the CAN bus\n
 * Connect the UART TX of the PC to the UART RX of the second interlink node connected to the CAN bus\n
 * Also connect A0 of the second interlink node to ground.
 * This indicates that it is PC->CAN only, and it should not decode incoming can messages\n
 *
 * If a speed higher then 500k is required, a more powerfull microcontroller like the STM32F4 is needed\n
 *
 * In the future i might build a STMF4 or better based interlink node
 * that is fast enough to send AND recieve at 1Mbaud.\n
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
 * If a faster connection (higher then 125kbaud, is needed, use two (2) interlink nodes as described in the introduction)
 *
 *
 * \section proto_sec Protocol
 * The handyCAN and interlink protocol is described here: https://github.com/SirVolta/HandyCAN/tree/master/doc/protocol \n
 *
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

/// Indicates start of the UART protocol
#define STARTSYNCBYTE1 0xF0
/// Must come right after #STARTSYNCBYTE1 to indicate start
#define STARTSYNCBYTE2 0xFA
/// Indicates end of the UART protocol
#define ENDSYNCBYTE1  0xE0
/// Must come right after #ENDSYNCBYTE1 to indicate end
#define ENDSYNCBYTE2  0xEE
/// Index of length in a UART message
#define LENIDX 2
/// Index of amount of check bytes in UART message
#define CHECKLENIDX 3
/// Index of stdid low byte in UART message
#define STDIDLIDX 4
/// Index of stdid high byte in UART message
#define STDIDHIDX 5
/// Index of start of the data bytes in uart message
#define DATAIDX 6

/// for debug
uint32_t overruns = 0;
/// Indicates if we are receiving a package
static uint8_t recieving;

/// incoming CAN message to send over the UART
static struct CANMessageToSend
{
  ///Indicates if a message is ready
  uint8_t messageReady;

  /// The message to be sent to the PC
  CanRxMsg message;
} messageToSend;

/*!
 *
 @brief Called when there is a package on the CAN bus
 @note Places it into message to send, and will be sent by main()
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
    {
      trace_puts("overrun");
      //overruns++;
      GPIO_ToggleBits(GPIOC, GPIO_Pin_13);
    }
}

/*!
 *
 @brief Sends a CAN message over UART to the PC
 @note Does all of the transmit UART protocol handling
 */
static inline void
sendCANMessage (void)
{
  // DMA timeout
  uint32_t dma_timeout = 1000;
  // if we're still transmitting ,wait
  // the chances of this happening should be zero,
  // but better safe than sorry.
  while ((DMA1_Channel4->CCR & DMA_CCR1_EN) && dma_timeout)
    {
      trace_puts("wait");
      if (--dma_timeout == 0)
	{
	  trace_puts("DMA TX TIMEOUT");
	  return;
	}
    }

  // Length of the data to send
  uint8_t len = messageToSend.message.DLC;
  // holds the index of shifted bytes
  uint8_t shift[16];
  // amount of bytes that have been shifted
  uint8_t shiftloc = 0;
  // current location in the buffer
  uint8_t i;
  // Buffer. Way oversized. Can never be more than 15 large.
  uint8_t buf[80];


  // 0 and 1 indicates start of frame
  buf[0] = STARTSYNCBYTE1;
  buf[1] = STARTSYNCBYTE2;
  // 2 is data length
  // As it will never exceed 8, let alone 0xE0 (224) in size,
  // it will never come close to being a sync byte and we can skip checking this
  buf[LENIDX] = len;

  // 3 is check length
  // it also will never exceed 0xE0. Sync will not have to touch this
  buf[CHECKLENIDX] = 0;

  // 2 and 3 is StdId
  buf[STDIDLIDX] = (uint8_t) (messageToSend.message.StdId & 0xFF);
  buf[STDIDHIDX] = (uint8_t) ((messageToSend.message.StdId & 0xFF00) >> 8);

  // next comes the data, [6 .. len]
  for (i = DATAIDX; i < len + DATAIDX; i++)
    buf[i] = messageToSend.message.Data[i - DATAIDX];

  /// To prevent issues with the sync bytes, we must now check
  /// there are any bytes with 0xF0, 0xFA, 0xE0, or 0xEE in them.\n
  /// if so, increment them and store their position.\n
  /// These positions will be appended to the message so the
  /// Receiver will know to decrement them.\n
  /// The increment indexes themselves will never be large enough
  /// to come anywhere close to the start and end bytes, so
  /// we can safely append this to the output without them becoming
  /// sync bytes.
  for (uint8_t check = 4; check < i; check++)
    {
      if ((buf[check - 1] == STARTSYNCBYTE1 && buf[check] == STARTSYNCBYTE2)
	  || (buf[check - 1] == ENDSYNCBYTE1 && buf[check] == ENDSYNCBYTE2))
	{
	  //trace_printf("Injected check for elem %u %#x now %#x\n", check,
	  //      	 buf[check], buf[check] + 1);
	  buf[check]++;
	  shift[shiftloc++] = check;
	  buf[CHECKLENIDX]++;
	}
    }

  for (uint8_t checkloc = 0; checkloc < shiftloc; checkloc++)
    buf[i++] = shift[checkloc];

  // and the last two to indicate end
  buf[i++] = ENDSYNCBYTE1;
  buf[i++] = ENDSYNCBYTE2;


  // now send using DMA
  //should already be disabled at this point
  DMA_Cmd(DMA1_Channel4, DISABLE);
  DMA1_Channel4->CMAR = (uint32_t) buf;
  // Set the amount of data to be transmitted
  DMA_SetCurrDataCounter(DMA1_Channel4, i);
  DMA_Cmd(DMA1_Channel4, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);


  /*
   for (uint8_t j = 0; j < i; j++)
   {
   /// TODO: Use DMA for transmit
   while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
   ;
   USART_SendData(USART1, buf[j]);
   //trace_printf("buf[%u]: %#x, %u\n", j, buf[j], buf[j]);
   }
   */
}

/*!
 *
 @brief Recieves and stores bytes from PC. When all are present, sends them as a CAN message.
 @note Does all of the recieve UART protocol handling
 */
extern void
USART1_IRQHandler (void)
{
  // recieved byte from UART
  uint8_t incoming;
  // data length
  uint8_t len;
  // amount of checkbytes
  uint8_t checklen;
  // start of checkbytes
  uint8_t checkbytes_start;
  // CAN message to send
  CanTxMsg TxMessage;
  // uart buffer
  static uint8_t uartbuf[80];
  // previously recieved byte
  static uint8_t previous = '\0';
  // current position in the buffer
  static uint8_t bufloc = 0;

  if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
      USART_ClearITPendingBit(USART1, USART_IT_TXE);
    }

  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
      incoming = (char) USART_ReceiveData(USART1);
      USART_ClearITPendingBit(USART1, USART_IT_RXNE);

      if (incoming == STARTSYNCBYTE1)
	recieving = 1;

      // beginning of the message
      if (incoming == STARTSYNCBYTE2)
	{
	  if (previous == STARTSYNCBYTE1)
	    {
	      bufloc = 1;
	      uartbuf[0] = previous;
	      recieving = 1;
	    }
	}
      // end of the message
      // beginning of the message
      if (incoming == ENDSYNCBYTE2)
	{
	  if (previous == ENDSYNCBYTE1)
	    {
	      uartbuf[bufloc] = incoming;
	      // message is ready
	      // first, verify
	      if (uartbuf[0] != STARTSYNCBYTE1 || uartbuf[1] != STARTSYNCBYTE2)
		{
		  trace_printf("not a valid handycan message: invalid header: "
			       "%#x %#x\n",
			       uartbuf[0], uartbuf[1]);
		  recieving = 0;
		  return;
		}
	      if (uartbuf[bufloc - 1] != ENDSYNCBYTE1
		  || uartbuf[bufloc] != ENDSYNCBYTE2)
		{
		  trace_printf("not a valid handycan message: invalid footer: "
			       "%#x %#x\n",
			       uartbuf[bufloc - 1], uartbuf[bufloc]);
		  recieving = 0;
		  return;
		}

	      // get the data length
	      len = uartbuf[LENIDX];
	      if ((8 + len + uartbuf[CHECKLENIDX]) != bufloc + 1)
		{
		  trace_printf("Message size invalid!\n"
			       "expected %u got %u\n",
			       8 + len + uartbuf[CHECKLENIDX], bufloc + 1);
		  recieving = 0;
		  return;
		}

	      // if there are checkbytes, decrement them
	      checklen = uartbuf[CHECKLENIDX];
	      if (checklen)
		{
		  checkbytes_start = 6 + len;
		  // decrement them
		  for (uint8_t i = 0; i < checklen; i++)
		    {
		      uartbuf[uartbuf[checkbytes_start + i]]--;
		    }
		}
	      // now the message has been reconstructed, get the data out of it
	      // get the StdId
	      TxMessage.StdId = uartbuf[STDIDLIDX] | (uartbuf[STDIDHIDX] << 8);
	      TxMessage.ExtId = 0;
	      TxMessage.RTR = CAN_RTR_DATA;
	      TxMessage.IDE = CAN_ID_STD;
	      // the data length
	      TxMessage.DLC = len;
	      // and finally the data
	      for (uint8_t i = 0; i < len; i++)
		{
		  TxMessage.Data[i] = uartbuf[DATAIDX + i];
		}

	      // now transmit this
	      // note: CAN_Transmit is non-blocking.
	      // it will queue a CAN message in a mailbox.
	      // the peripheral will send it as soon as the bus is ready
	      CAN_Transmit(CAN1, &TxMessage);
	      recieving = 0;

	      return;
	    }
	}
      // Store the byte in the buffer, then increment the buffer position
      uartbuf[bufloc++] = incoming;
      // and store current byte for next time
      previous = incoming;
    }
}

/*!
 * There is a chance that the recieve flag does not get reset.
 * most likely due to a message not being completely sent or other kinds of corruption.
 * In this case, the interlink node will become inert. To counter that, we reset
 * the line every few minutes so a deadlock will never last long.
 @brief Reset the receiving flag if something goes wrong
 @note Even if this happens during receive, the CTS line will restore itself when the CAN message is sent
 */
extern void
TIM4_IRQHandler (void)
{
  trace_printf("ResetRecieve!\n");
  recieving = 0;
  TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
}

/*!
 @brief Called by NVIC when UART DMA transmit is completed
 @note  Stop DMA, clear interrupt and transfer flags
 */
extern void
DMA1_Channel4_IRQHandler (void)
{
  if (DMA_GetFlagStatus(DMA1_IT_TC4))
    {
      DMA_ClearITPendingBit(DMA1_IT_TC4);
      USART_DMACmd(USART1, USART_DMAReq_Tx, DISABLE);
      DMA_Cmd(DMA1_Channel4, DISABLE);
    }
}

/*!
 *
 @brief Initializes the CAN peripheral and recieve interupt
 @note The filter recieves EVERY CAN message into FIFO0
 */
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
  CAN_InitStruct.CAN_Prescaler = 8; //4: 1M, 8: 500k
  CAN_Init(CAN1, &CAN_InitStruct);

  // Set the filter to receive everything into FIFO0
  // but only if A0 is set.
  if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
    {
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

      //Setup for FIFO0 data, higher priority than UART
      NVIC_InitStruct.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
      NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
      NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStruct);
      CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
      NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }
}

/*!
 @brief Initializes the UART peripheral and recieve interupt
 @note The baudrate must be set very high (at least 1M)
 */
static void
UART_init (void)
{
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  //USART_DeInit(USART1);
  USART_StructInit(&USART_InitStructure);
  USART_InitStructure.USART_BaudRate = 2000000;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
  USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  // Setup the uart interrupt
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  // Enable the uart and the uart interrupt
  USART_Cmd(USART1, ENABLE);
  //Enable recieve interrupt
  NVIC_EnableIRQ(USART1_IRQn);
  //USART_ClearFlag(USART1, USART_FLAG_TC);
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

  DMA_DeInit(DMA1_Channel4);
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &USART1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) NULL;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  /// Enable DMA Stream Transfer Complete interrupt
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
  DMA_ClearFlag(DMA1_IT_TC4);

  // set interupt controller for DMA1C4
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn; //I2C1 connect to channel 7 of DMA1
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x05;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x05;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
/*!
 @brief Initializes the GPIO pins
 @note JTAG is disabled
 */
static void
GPIO_init (void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /// disable JTAG to free up RB3 and RB4
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

  // pin A0: UART transmit and CAN receive enable
  // checked only once during startup.
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  // pin A1: CAN Clear To Send
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

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

  // uart pins, alternate function
  GPIO_StructInit(&GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void
timer_init (void)
{
  NVIC_InitTypeDef nvicStructure;
  TIM_TimeBaseInitTypeDef timerInitStructure;

  //Setup for interrupt roughy every minute
  timerInitStructure.TIM_Prescaler = 0xFFFF;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  timerInitStructure.TIM_Period = 0xFFFF;
  timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM4, &timerInitStructure);
  TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);

  // setup the timer interrupt
  nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 1;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

}

int
main (void)
{
  messageToSend.messageReady = 0;

  // enable all of the clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  // initialize the peripherals
  GPIO_init();
  UART_init();
  CAN_init();
  timer_init();

  //Setup the watchdog
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler(IWDG_Prescaler_256);
  IWDG_SetReload(0xFF);
  IWDG_ReloadCounter();
  IWDG_Enable();

  trace_puts("Interlink node ready!");

  // check if there is a message to send, if so send it.
  while (1)
    {
      if (recieving || !((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0))
	//if (!((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0))
	GPIOA->BSRR = GPIO_Pin_1;
      else
	GPIOA->BRR = GPIO_Pin_1;

      if (messageToSend.messageReady)
	{
	  sendCANMessage();
	  messageToSend.messageReady = 0;
	}

      if (recieving || !((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0))
	//if (!((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0))
	GPIOA->BSRR = GPIO_Pin_1;
      else
	GPIOA->BRR = GPIO_Pin_1;

      ///Kick the dog
      IWDG_ReloadCounter();
    }
}

