/**
 * @file    HandyCAN.h
 * @author  SirVolta
 * @ide     Emacs
 * @date    Sep 18, 2017
 * @brief   HandyCAN node library
 * @note    Function descriptions are in source file
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
#ifndef HANDYCAN_H_
#define HANDYCAN_H_
#include "stm32f10x_conf.h"


#define HC_DEST_MASK 0x1F

#define HC_SRC_MASK 0x3E0
#define HC_SRC_OFFSET 5

#define HC_RESERVED_BIT 0x400
#define HC_RESERVED_BIT_OFFSET 10

#define HC_BROADCAST_ADDR 0x1F

#define HC_CAN_SEND_TIMEOUT 0xFFFFF


/// @brief local state and configuration info
struct HandyCAN_package
{
  uint8_t dest_adress;
  uint8_t source_adress;
  uint8_t len;
  uint8_t data[8];
};



int8_t
HandyCAN_INIT (CAN_TypeDef* CANx, uint8_t local_addr, uint8_t CAN_Mode,
	       IRQn_Type FIFO0_IRQ, IRQn_Type FIFO1_IRQ);

int8_t
HandyCAN_Transmit (uint8_t destination, uint8_t * data, uint8_t len);

int8_t
HandyCAN_decodeCanRxMsg (CanRxMsg* rx_msg, struct HandyCAN_package* package);


int8_t
HandyCAN_canTransmit (void);

int8_t
HandyCAN_isTransmitting (void);



//For debug only
void
HandyCAN_dumpCanRxMessage (CanRxMsg* rx_msg);

#endif /* HANDYCAN_H_ */
