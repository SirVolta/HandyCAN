/*
 * DHT1122.h
 *
 *  Created on: Dec 30, 2015
 *      Author: pelle
 */

#ifndef DHT1122_H_
#define DHT1122_H_
#include <stdint.h>
#include "stm32f10x_gpio.h"

/*!
 @brief contains data about one DHT11 or DHT22 sensor
 @section DHT1122 Example
 @code
 DHT1122struct dht;
 dht.IO_Pin = GPIO_Pin_1;
 dht.IO_Port = GPIOA;
 dht1122_read(&dht);
 printf("%u.%u C, %u.%u %%",
        dht.temp_intergral, dht.temp_decimal,
        dht.hum_intergral, dht.hum_decimal);
 @endcode
 */
typedef struct DHT1122struct
{
 uint16_t IO_Pin; /**< IO Pin DHT is connected to*/
 GPIO_TypeDef* IO_Port; /**< IO Port DHT is connected to*/
 uint8_t temp_intergral; /**< Integral part of temperature reading*/
 uint8_t temp_decimal; /**< decimal part of temperature reading, only avaliable on DHT22*/
 uint8_t hum_intergral;/**< Integral part of humidity reading*/
 uint8_t hum_decimal; /**< decimal part of humidity reading, only avaliable on DHT22*/
} DHT1122struct;

unsigned char
dht1122_get_byte (DHT1122struct* device);
unsigned char
dht1122_read (DHT1122struct* device);
#endif /* DHT1122_H_ */
