/*!
 * @file DHT1122.c
 * @author Pelle Jansen
 * @date Dec 30, 2015
 * @brief DHT11 or DHT22 interface library
 */

#include "DHT1122.h"
#include <stdint.h>
#include "svlib_stm32f10x.h"

/*!
 @brief  Reads one byte of data from DHT11/22 chip
 Requires IO pin to be set to INPUT
 @param[in] device
 pointer to an DHT1122struct instance with IO pin and port set
 @return one byte of data read from DHT device
 */
uint8_t
dht1122_get_byte (DHT1122struct* device)
{
 uint8_t s = 0;
 uint8_t value = 0;

 for (s = 0; s < 8; s += 1)
  {
   value <<= 1;
   while (!GPIO_ReadInputDataBit (device->IO_Port, device->IO_Pin))
    ;
   delayUs (30);

   if (GPIO_ReadInputDataBit (device->IO_Port, device->IO_Pin))
    {
     value |= 1;
    }
   while (GPIO_ReadInputDataBit (device->IO_Port, device->IO_Pin))
    ;
  }
 return value;
}

/**!
 @brief  Reads data from DHT11/22 chip and stores it in struct
 @param[in,out] device
 pointer to an DHT1122struct instance with IO pin and port set
 Data will be written to data section of struct
 Integral data is only available with DHT22. Will be zero otherwise
 @return success of reception from dht
 - 0: all ok
 - 1: no response, expected high, got low
 - 2: no response, expected low, got high
 - Both most likely due to device not present or not responding
 - 3: checksum failed
 - 30: any other fault
 */
uint8_t
dht1122_read (DHT1122struct* device)
{
 uint8_t chk = 0;
 uint8_t s = 0;
 uint8_t check_sum = 0;
 uint8_t data[5];

 GPIO_InitTypeDef dht11_io;
 dht11_io.GPIO_Pin = device->IO_Pin;
 dht11_io.GPIO_Mode = GPIO_Mode_Out_PP;
 dht11_io.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init (device->IO_Port, &dht11_io);

 device->hum_intergral = 255;
 device->hum_decimal = 255;
 device->temp_intergral = 255;
 device->temp_decimal = 255;

 //DHT11_lat = 1;
 GPIO_ResetBits (device->IO_Port, device->IO_Pin);
 delayUs (25000);
 GPIO_SetBits (device->IO_Port, device->IO_Pin);
 delayUs (20);

 dht11_io.GPIO_Mode = GPIO_Mode_IPU;
 GPIO_Init (device->IO_Port, &dht11_io);

 chk = GPIO_ReadInputDataBit (device->IO_Port, device->IO_Pin);
 if (chk)
  {
   return 1;
  }
 delayUs (80);

 chk = GPIO_ReadInputDataBit (device->IO_Port, device->IO_Pin);
 if (!chk)
  {
   return 2;
  }
 delayUs (80);

 for (s = 0; s <= 4; s += 1)
  {
   data[s] = dht1122_get_byte (device);
  }

 dht11_io.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_Init (device->IO_Port, &dht11_io);

 GPIO_SetBits (device->IO_Port, device->IO_Pin);

 for (s = 0; s < 4; s += 1)
  {
   check_sum += data[s];
  }

 if (check_sum != data[4])
  {
   return 3;
  }
 else
  {
   device->hum_intergral = data[0];
   device->hum_decimal = data[1];
   device->temp_intergral = data[2];
   device->temp_decimal = data[3];
   return 0;
  }
 return 30;
}
