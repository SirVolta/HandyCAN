# Sensor node
This example node reads light, temprature+humidity and motion
Its address is set to 0x02.  
The node type of the Sensor node is 2

# Intents
- 0x01: get node uptime
- 0x02: get temperature
- 0x03: motion detected
- 0x04: get light level
- 0x05: temperature alarm reached
- 0x06: set temperatrue alarm
- 0x07: get temperature alarm
- 0x08: light alarm
- 0x09: set light alarm
- 0x0A: get light alarm

It will respond with 0xFD if a invalid intent is sent.

# Pinout
- B3: DHT22 sensor and pullup resistor
- A6: Light sensor analog. 0V = no light, 3.3V = max light.
- A11: motion sensor. 5V tolerant.
- B8: CAN RX
- B9: CAN TX
- C13: LED _Open drain_

