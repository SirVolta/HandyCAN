# Interlink Node
The interlink node faclitates comms between the PC an the CAN network

## Clear To Send
Active low output indicating weather the interlink node is ready to send a packet. 
This way the PC can know when the package sent to the interlink node is acturally sent onto the CAN bus. 
It is not requirered, but then the send speed of the PC must be dramatically lowered to prevent
sending packets faster then the CAN bus can handle. 

## Not enough CPU power
If the CAN bus is run at over 125kbaud, the STM32F10 is not powerful enough to recieve and transmit all packages in realtime.  
But, it can receive OR transmit at a maximum of 500kbaud.  
So if a speed lower then 500k but higher then 125k is required you can use two Interlink nodes: one for receive, one for transmit.  
Connect the UART RX of the pc to the UART TX of the first interlink node connected to the CAN bus  
Connect the UART TX of the PC to the UART RX of the second interlink node connected to the CAN bus  
Also connect A0 of the second interlink node to ground during startup of the node. This indicates that it is PC->CAN only, and it should not decode incoming can messages.  

If a speed higher then 500k is required, a more powerful microcontroller like the STM32F4 is needed  
In the future i might build a STMF4 or better based interlink node that is fast enough to send AND recieve at 1Mbaud.  
This is why there is a seperate STM32F1 folder.  

## STM32F1
This is tested to run on A STM32F103C6.  

### Pinout:
- A0: PC->Can node. Disables CAN recieve and UART transmit if connected to ground. (jumper).
- A1: CTS_N: Clear To Send. Is high while a package is being sent.
- A9: UART TX (unused if A0 is low during startup)  
- A10: UART RX  
- A11: CAN RX  
- A12: CAN TX  
- C13: LED _(open drain)_  



### notes
There is no limit to the amount of interlink nodes in a setup and they are not required if PC comms is not needed.  
