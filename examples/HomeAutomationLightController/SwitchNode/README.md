# Switch node
This example node has 4 CAN controllable PWM outputs.  
Its address is set to 0x01.  
The node type of the Switch Node is 1.
All switches switch to ground.

# Intents
- 0x01: get node uptime
- 0x02: Switch changed
- 0x03: Get switch state

It will respond with 0xFD if a invalid intent is sent.

# Pinout
- A11: Switch 
- A12: Switch
- A15: Switch
- B8: CAN RX
- B9: CAN TX
- C13: LED _Open drain_
 
