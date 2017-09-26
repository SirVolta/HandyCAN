/**
 * @file    HandyCAN.c
 * @author  SirVolta
 * @ide     Emacs
 * @date    Sep 18, 2017
 * @brief   HandyCAN node library
 * @note    ...
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

/*
 * First some coding practices:
 * All formatting is to be done in GNU style
 * All functions have to be prefixed with HandyCAN_
 * Constants, macros, enums and the like have to be prefixed with HC_
 *  This keeps longish macro's clean
 * Global variables are prohibited. There is a static config struct for this.
 * All functions that could possibly have something go wrong need to return int8_t
 * with an error condition. Variable IO has to be done using pointers.
 *
 */
#include <string.h>
#include "stm32f10x_conf.h"
#include "svlib_stm32f10x.h"
#include "HandyCAN.h"

/// @brief local state and configuration info
static struct HandyCAN_config
{
  uint8_t local_address;
  CAN_TypeDef* CANx;
} handycan;

/*!
 @brief  Initializes handyCAN to a specific CAN peripheral
 @param[in,out] CANx the can peripheral to use
 @param[in] CAN_Mode mode t initialize the CAN peripheral into
 @note Takes over full access of the CAN peripheral. GPIO setup is to be done in user application. If using CAN2, change the interrupt function names accordingly.
 @return 0: ok,
 @todo Error handling, check can init
 */
int8_t
HandyCAN_init (CAN_TypeDef* CANx, uint8_t local_addr, uint8_t CAN_Mode,
	       IRQn_Type FIFO0_IRQ, IRQn_Type FIFO1_IRQ)
{
  CAN_InitTypeDef CAN_InitStruct;
  NVIC_InitTypeDef NVIC_InitStruct;
  CAN_FilterInitTypeDef CAN_FilterInitStruct;

  handycan.CANx = CANx;
  handycan.local_address = local_addr;

  //Configure the CAN
  CAN_DeInit (handycan.CANx);
  CAN_StructInit (&CAN_InitStruct);
  CAN_InitStruct.CAN_TTCM = DISABLE;
  CAN_InitStruct.CAN_ABOM = DISABLE;
  CAN_InitStruct.CAN_AWUM = DISABLE;
  CAN_InitStruct.CAN_NART = DISABLE;
  CAN_InitStruct.CAN_RFLM = DISABLE;
  CAN_InitStruct.CAN_TXFP = DISABLE;
  CAN_InitStruct.CAN_Mode = CAN_Mode;
  CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
  CAN_InitStruct.CAN_BS1 = CAN_BS1_3tq;
  CAN_InitStruct.CAN_BS2 = CAN_BS2_5tq;
  CAN_InitStruct.CAN_Prescaler = 4; //4: 1M, 8: 500k
  CAN_Init (handycan.CANx, &CAN_InitStruct);

  // Set the filter to receive everything addressed to us into FIFO0
  CAN_FilterInitStruct.CAN_FilterNumber = 0;
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  // match our address into bits 0..4 of the 11 id bits
  // shift 5 to the left, as this is a 32bit register
  CAN_FilterInitStruct.CAN_FilterIdHigh = handycan.local_address << 5;
  CAN_FilterInitStruct.CAN_FilterIdLow = 0;
  // only look at bit 0..4 of the id bits
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x1F << 5;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = 0;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInit (&CAN_FilterInitStruct);

  // Set the filter to receive broadcast into FIFO1
  CAN_FilterInitStruct.CAN_FilterNumber = 1;
  CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
  // match broadcast address in bits 0..4 of the id bits
  // shift 5 to the left as it is a 32 bit register
  CAN_FilterInitStruct.CAN_FilterIdHigh = HC_BROADCAST_ADDR << 5; // broadcast address
  CAN_FilterInitStruct.CAN_FilterIdLow = 0;
  // match only bit 0 to 4 in the id section
  CAN_FilterInitStruct.CAN_FilterMaskIdHigh = 0x1F << 5;
  CAN_FilterInitStruct.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStruct.CAN_FilterFIFOAssignment = 1;
  CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
  CAN_FilterInit (&CAN_FilterInitStruct);

  //Setup receive interrupt for FIFO0(own address) data
  NVIC_InitStruct.NVIC_IRQChannel = FIFO0_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStruct);
  CAN_ITConfig (handycan.CANx, CAN_IT_FMP0, ENABLE);
  NVIC_EnableIRQ (FIFO0_IRQ);

  //Setup for FIFO1(broadcast) data
  NVIC_InitStruct.NVIC_IRQChannel = FIFO1_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init (&NVIC_InitStruct);
  CAN_ITConfig (handycan.CANx, CAN_IT_FMP1, ENABLE);
  NVIC_EnableIRQ (FIFO1_IRQ);

  return 0;
}

/*!
 @brief Takes a CAN Message and translates it into a HandyCAN packet
 @param[in] rx_msg incoming can message
 @param[out] package output HandyCAN package
 @note Copies the data so the next interrupt can't overwrite it.
 @return 0: ok,
 @todo Error handling, check if memcopy really is required. Passing pointers may be prefered.
 */
inline int8_t
HandyCAN_decodeCanRxMsg (CanRxMsg* rx_msg, struct HandyCAN_package* package)
{
  package->source_adress = (rx_msg->StdId & HC_SRC_MASK) >> HC_SRC_OFFSET;
  package->dest_adress = rx_msg->StdId & HC_DEST_MASK;
  package->len = rx_msg->DLC;
  memcpy (package->data, rx_msg->Data, rx_msg->DLC);
  return 0;
}

/*!
 @brief Check how many mailboxes there are available
 @note Tests if it is possible to transmit
 @return 0: no, 1, one remaining, 2: 2 remaining, 3: 3 remaining
 */
int8_t
HandyCAN_remainingMailboxes (void)
{
  uint8_t transmit_mailbox_remaining;

  if ((handycan.CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
    transmit_mailbox_remaining = 3;
  else if ((handycan.CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1)
    transmit_mailbox_remaining = 2;
  else if ((handycan.CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    transmit_mailbox_remaining = 1;
  else
    transmit_mailbox_remaining = 0;

  return transmit_mailbox_remaining;

}

/*!
 @brief Transmit a handican package
 @note Queues message for transmit. Transmits as soon as older packages are sent.
 @return 0: transmit queued. -1: no mailboxes available, -2:
 @todo Error handling
 */
int8_t
HandyCAN_transmit (uint8_t destination, uint8_t data[], uint8_t len)
{
  CanTxMsg TxMessage;
  uint8_t mailbox;

  // Construct the message to be sent
  // The ID is
  TxMessage.StdId = destination | (handycan.local_address << HC_SRC_OFFSET);
  TxMessage.ExtId = 0;
  TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.IDE = CAN_ID_STD;
  TxMessage.Data = data;
  TxMessage.DLC = len;

  mailbox = CAN_Transmit (CAN1, &TxMessage);

  if (mailbox == CAN_TxStatus_NoMailBox)
    return -1;

  return 0;
}

/*!
 @brief Check if there are packets currently being transmitted
 @note checks if there is a mailbox in use
 @return The amount of messages queued. (0 to 3)
 */
int8_t
HandyCAN_isTransmitting (void)
{
  if (HandyCAN_remainingMailboxes () < 3)
    return 1;
  else
    return 0;
}

/*!
 @brief For debug. Dumps a received can message to STDOUT
 @param [in] rx_msg: incoming can message to dump
 */
void
HandyCAN_dumpCanRxMessage (CanRxMsg* rx_msg)
{
  trace_printf ("StdId: %#x\n"
		"ExtID: %#x\n"
		"IDE: %#x\n"
		"RTR: %#x\n"
		"DLC: %u\n"
		"FMI: %#x\n",
		rx_msg->StdId, rx_msg->ExtId, rx_msg->IDE, rx_msg->RTR,
		rx_msg->DLC, rx_msg->FMI);

  for (uint8_t i = 0; i < 8; i++)
    trace_printf ("Data[%u]: %#x %u\n", i, rx_msg->Data[i], rx_msg->Data[i]);
}

