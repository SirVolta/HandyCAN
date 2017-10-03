# LED Node
This example node has 4 CAN controllable PWM outputs.  
Its address is set to 0x03.  
The node type of the LED Node is 3.

# Intents
- 0x01: get node uptime
- 0x02: Set color (RGB, 0-255)
- 0x03: Set power, 0-255

It will respond with 0xFD if a invalid intent is sent.

# Pinout
- A0: PWM0, RED
- A1: PWM1: Green
- A2: PWM2: Blue
- A3: PWM3: PWR/White
- B8: CAN RX
- B9: CAN TX
- C13: LED _Open drain_
