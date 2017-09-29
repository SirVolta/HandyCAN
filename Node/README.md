# Node
Nodes are the main part of HandyCAN and can communicate with each other, and the PC.

The nodes have a full implementation of HandyCAN and is ready to use on a STM32F10.  
It should be trivial to port HandyCAN to an other STM, as long as the standard peripheral driver is avaliable for that controller. 

## Pinout
- A0: PC->Can node. Disables CAN recieve and UART transmit.  
- 11: CAN RX  
- A12: CAN TX  
- C13: LED _(open drain)_  
