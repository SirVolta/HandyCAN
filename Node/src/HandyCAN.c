/**
 * @file    HandyCAN.c
 * @author  SirVolta
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
 */

#include <string.h>
#include "stm32f10x_conf.h"
#include "svlib_stm32f10x.h"
#include "HandyCAN.h"

///CAN Mailbox Transmit Request
#define TMIDxR_TXRQ  ((uint32_t)0x00000001)

///@brief local state and configuration info
static struct HandyCAN_config
{
  ///Address of this node
  uint8_t local_address;
  ///CAN peripheral to use
  CAN_TypeDef* CANx;
#if HC_KEEP_STATISTICS
  uint64_t tx_packets, rx_packets, failures;
#endif
} handycan;

/*!
 @brief  Initializes handyCAN to a specific CAN peripheral
 @param[in,out] CANx the can peripheral to use
 @param[in] local_addr the address of this node
 @param[in] CAN_Mode mode to initialize the CAN peripheral into
 @param[in] FIFO0_IRQ interrupt channel of own address FIFO.
 @param[in] FIFO1_IRQ interrupt channel of broadcast FIFO.
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

  assert_param(IS_CAN_ALL_PERIPH(CANx));
  //Address must be in between 0x01 and 0x1E (1 and 30), 29 possible.
  assert_param(local_addr < 0x1F);
  assert_param(local_addr != 0x1F);
  assert_param(local_addr != 0x00);
  assert_param(IS_CAN_MODE(CAN_Mode));

  handycan.CANx = CANx;
  handycan.local_address = local_addr;

#if HC_KEEP_STATISTICS
  handycan.tx_packets = 0;
  handycan.rx_packets = 0;
  handycan.failures = 0;
#endif

  //Configure the CAN
  CAN_DeInit(handycan.CANx);
  CAN_StructInit(&CAN_InitStruct);
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
  CAN_Init(handycan.CANx, &CAN_InitStruct);

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
  CAN_FilterInit(&CAN_FilterInitStruct);

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
  CAN_FilterInit(&CAN_FilterInitStruct);

  //Setup receive interrupt for FIFO0(own address) data
  NVIC_InitStruct.NVIC_IRQChannel = FIFO0_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  CAN_ITConfig(handycan.CANx, CAN_IT_FMP0, ENABLE);
  NVIC_EnableIRQ(FIFO0_IRQ);

  //Setup for FIFO1(broadcast) data
  NVIC_InitStruct.NVIC_IRQChannel = FIFO1_IRQ;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
  CAN_ITConfig(handycan.CANx, CAN_IT_FMP1, ENABLE);
  NVIC_EnableIRQ(FIFO1_IRQ);

  return 0;
}

/*!
 @brief Takes a CAN Message and translates it into a HandyCAN packet
 @param[in] FIFONumber Which FIFO to read
 @param[out] package output HandyCAN package
 @note Parts taken from MCD library
 @return 0: ok, -1 error, -2 non handyCAN message
 @todo Error handling, improve reading data from RDxR register
 */
inline int8_t
HandyCAN_recievePackage (uint8_t FIFONumber, struct HandyCAN_package* package)
{
  // The standard identifier. This is between 0 and 0x7FF and will contain addressing data
  uint32_t StdId;
  // The type of identifier for the message being received.
  // HandyCAN only uses the standard ID
  uint8_t IDE;
  //Type of frame for the recieved message, CAN_remote_transmission_request
  //uint8_t RTR;
  //The index of the filter that was used to match this message. Can be 0 to 0xFF
  //uint8_t FMI;

  assert_param(IS_CAN_FIFO(FIFONumber));

  ///HandyCAN only uses the standard ID.
  IDE = (uint8_t) 0x04 & handycan.CANx->sFIFOMailBox[FIFONumber].RIR;
  if (IDE != CAN_Id_Standard)
    {
#if HC_KEEP_STATISTICS
      handycan.failures++;
#endif
      return -2;
    }
  StdId = (uint32_t) 0x000007FF
      & (handycan.CANx->sFIFOMailBox[FIFONumber].RIR >> 21);

  //These are not used at the moment
  //FMI = (uint8_t) 0xFF & (handycan.CANx->sFIFOMailBox[FIFONumber].RDTR >> 8);
  //RTR = (uint8_t) 0x02 & handycan.CANx->sFIFOMailBox[FIFONumber].RIR;

  //Get the package data length
  package->len = (uint8_t) 0x0F & handycan.CANx->sFIFOMailBox[FIFONumber].RDTR;

  //Retrieve the data. TODO: optimize this.
  package->data[0] = (uint8_t) 0xFF
      & handycan.CANx->sFIFOMailBox[FIFONumber].RDLR;
  package->data[1] = (uint8_t) 0xFF
      & (handycan.CANx->sFIFOMailBox[FIFONumber].RDLR >> 8);
  package->data[2] = (uint8_t) 0xFF
      & (handycan.CANx->sFIFOMailBox[FIFONumber].RDLR >> 16);
  package->data[3] = (uint8_t) 0xFF
      & (handycan.CANx->sFIFOMailBox[FIFONumber].RDLR >> 24);
  package->data[4] = (uint8_t) 0xFF
      & handycan.CANx->sFIFOMailBox[FIFONumber].RDHR;
  package->data[5] = (uint8_t) 0xFF
      & (handycan.CANx->sFIFOMailBox[FIFONumber].RDHR >> 8);
  package->data[6] = (uint8_t) 0xFF
      & (handycan.CANx->sFIFOMailBox[FIFONumber].RDHR >> 16);
  package->data[7] = (uint8_t) 0xFF
      & (handycan.CANx->sFIFOMailBox[FIFONumber].RDHR >> 24);

  //Release the FIFO mailbox
  if (FIFONumber == CAN_FIFO0)
    handycan.CANx->RF0R |= CAN_RF0R_RFOM0;
  else
    handycan.CANx->RF1R |= CAN_RF1R_RFOM1;

  // Decode the addresses
  // Again STDID first 5 bits destination address, then 5 bits source addr, then 1 reserved
  package->source_adress = (StdId & HC_SRC_MASK) >> HC_SRC_OFFSET;
  package->dest_adress = StdId & HC_DEST_MASK;

#if HC_KEEP_STATISTICS
  handycan.rx_packets++;
#endif

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
 @brief Transmit a handycan package
 @param destination: Address to transmit package to
 @param data: data to transmit to destination
 @param len: Length of data. Must be in between 1 and 8.
 @note Queues message for transmit. Transmits as soon as older packages are sent.
 @note Parts taken from MCD library
 @return 0: transmit queued. -1: no mailboxes available, -2:
 @todo Error handling, better construction of TDxR data register
 */
int8_t
HandyCAN_transmit (uint8_t destination, uint8_t data[], uint8_t len)
{
  uint32_t StdId;
  uint8_t transmit_mailbox = 0;

  assert_param(destination <= 0x1F);
  assert_param(len <= 8);
  assert_param(len > 0);

  // The ID is first 5 bits of destination address,
  // then 5 bits of source address ending with 1 bit reserved
  StdId = destination | (handycan.local_address << HC_SRC_OFFSET);
  assert_param(IS_CAN_STDID(StdId));

  // Select one empty transmit mailbox
  if ((handycan.CANx->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
    transmit_mailbox = 0;
  else if ((handycan.CANx->TSR & CAN_TSR_TME1) == CAN_TSR_TME1)
    transmit_mailbox = 1;
  else if ((handycan.CANx->TSR & CAN_TSR_TME2) == CAN_TSR_TME2)
    transmit_mailbox = 2;
  else
    // we could return -1 here, but i want to keep multiple exit points to a minimum
    transmit_mailbox = CAN_TxStatus_NoMailBox;

  if (transmit_mailbox != CAN_TxStatus_NoMailBox)
    {
      // Clear any old transmit request
      handycan.CANx->sTxMailBox[transmit_mailbox].TIR &= TMIDxR_TXRQ;
      // Setup the StdId (address) field
      handycan.CANx->sTxMailBox[transmit_mailbox].TIR |= ((StdId << 21) |
      CAN_RTR_DATA);

      // Setup the DLC (length) field
      len &= (uint8_t) 0x0000000F;
      handycan.CANx->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t) 0xFFFFFFF0;
      handycan.CANx->sTxMailBox[transmit_mailbox].TDTR |= len;

      // setup the output data registers
      handycan.CANx->sTxMailBox[transmit_mailbox].TDLR = (((uint32_t) data[3]
	  << 24) | ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8)
	  | ((uint32_t) data[0]));
      handycan.CANx->sTxMailBox[transmit_mailbox].TDHR = (((uint32_t) data[7]
	  << 24) | ((uint32_t) data[6] << 16) | ((uint32_t) data[5] << 8)
	  | ((uint32_t) data[4]));

      // start tranmission
      handycan.CANx->sTxMailBox[transmit_mailbox].TIR |= TMIDxR_TXRQ;
    }

  if (transmit_mailbox == CAN_TxStatus_NoMailBox)
    {
#if HC_KEEP_STATISTICS
      handycan.failures++;
#endif
      return -1;
    }

#if HC_KEEP_STATISTICS
  handycan.tx_packets++;
#endif

  return 0;
}

/*!
 @brief Check if there are packets currently being transmitted
 @note checks if there is a mailbox in use
 @return 1: transmit packet queued. 0: idle.
 */
int8_t
HandyCAN_isTransmitting (void)
{
  if (HandyCAN_remainingMailboxes() < 3)
    return 1;
  else
    return 0;
}

/*!
 @brief returns the amount of packages sent since init
 @note returns 0 if statistics are disabled. Set #HC_KEEP_STATISTICS to 1 if required
 @return amount of packages sent, 0 if disabled
 */
uint64_t
HandyCAN_sentPackets (void)
{
#if HC_KEEP_STATISTICS
  return handycan.tx_packets;
#else
  return 0;
#endif
}

/*!
 @brief returns the amount of packages received since init
 @note returns 0 if statistics are disabled. Set #HC_KEEP_STATISTICS to 1 if required
 @return amount of packages received, 0 if disabled
 */
uint64_t
HandyCAN_recievedPackets (void)
{
#if HC_KEEP_STATISTICS
  return handycan.rx_packets;
#else
  return 0;
#endif
}

/*!
 @brief returns the failures since init
 @note returns 0 if statistics are disabled. Set #HC_KEEP_STATISTICS to 1 if required
 @return amount of failures, 0 if disabled
 */
uint64_t
HandyCAN_failCount (void)
{
#if HC_KEEP_STATISTICS
  return handycan.failures;
#else
  return 0;
#endif
}

/*!
 @brief For debug. Dumps a received HandyCAN Packet to STDOUT
 @param [in] package: incoming handyCAN package to dump
 */
void
HandyCAN_dumpRxPackage (struct HandyCAN_package* package)
{
  if (package->dest_adress == HC_BROADCAST_ADDR)
    {
      trace_printf("Incoming BROADCAST from %#x with intent: %u\n",
		   package->source_adress, package->data[0]);
    }
  else
    {
      trace_printf("Received message from %#x with intent: %u\n",
		   package->source_adress, package->data[0]);
    }

  for (uint8_t i = 1; i < package->len; i++)
    trace_printf("Data[%u]: %#x %u\n", i, package->data[i], package->data[i]);
}

/*!
 @brief For debug. Dumps a received can message to STDOUT
 @param [in] rx_msg: incoming can message to dump
 */
void
HandyCAN_dumpCanRxMessage (CanRxMsg* rx_msg)
{
  trace_printf("StdId: %#x\n"
	       "ExtID: %#x\n"
	       "IDE: %#x\n"
	       "RTR: %#x\n"
	       "DLC: %u\n"
	       "FMI: %#x\n",
	       rx_msg->StdId, rx_msg->ExtId, rx_msg->IDE, rx_msg->RTR,
	       rx_msg->DLC, rx_msg->FMI);

  for (uint8_t i = 0; i < 8; i++)
    trace_printf("Data[%u]: %#x %u\n", i, rx_msg->Data[i], rx_msg->Data[i]);
}

