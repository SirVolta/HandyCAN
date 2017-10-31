# Keyboard node
This example node reads a PS/2 keyboard and drives a color touchscreen.  
It allows internal comms (chat) or a possible command line interface.  
Its address is set to 0x0A.  
The node type of the Keyboard Node is 8

# CAN Format
Keyboard data is sent every time the user hits the RETURN key.  
It is send in pure 8-bit ASCII, with a serial number to indicate location.  
The buffer can hold at most 1500 characters.  
Data layout(A=ASCII char): [intent, serialNum, A1, A2, A3, A4, A5, A6]  
Example: Hello, world<Return>  
[0x0A, 0, 'H', 'e', 'l', 'l', 'o', ',']  
[0x0A, 1, 'w', 'o', 'r', 'l', 'd', '\0']  
All messages will be acknowledged by the receiver before the next is sent.  
Ack response: [AckIntent, serialNum].  
If a response is not recieved within 1ms, the message is transmitted again
for at most 10 times. After that transmission to that node is stopped.
If there are more nodes, these are tried next

For keyboard nodes to find each other, keyboard nodes broadcast keyboard node detect
after the user requested a broadcast transmission.  
All keyboard nodes will then respond with their short name (max 7 characters).  
Example: [0x1A, 'J', 'o', 'h', 'n'] in response of broadcast of 0x1A.

# First (demo) version
This first version only has part of the functionality I want to include in the real keyboard node.  
It will have: 
- Command sending to interlink at address 0
- Chat to all connected keyboard nodes
To be added later in the form of an external advanced chat library:
- One to one chat, select able from list of keyboard nodes
- Proper notifications
- Keep some history
- Scrollback
- Settings menu with:
  - Customizable name
  - interlink address
  - Notification options
  - 

# Intents
0x0A: Outgoing keyboard data
0x0B: Incoming display data
0x0C: Keyboard data ACK.
0x0D: Display data ACK.
0x1A: Keyboard Node Detect (broadcast response)

In case of chat, keyboard data is sent with the display data intent

It will respond with 0xFD if a invalid intent is sent.


# EEPROM
An I2C EEPROM is used to keep configuration data like the node's name.  


# Pinout
- A1: Speaker
- A11: CAN RX
- A12: CAN TX
- B6: I2C EEPROM CLOCK
- B7: I2C EEPROM DAT
- C13: LED _Open drain_

