#Protocol Documentation

#CAN ID: Addressing.
HandyCAN uses the normal 11-bit CAN ID mode, allowing for 29 unique node addresses.  
With some modification it is possible to change this to 29 bit id, allowing for 16 381 nodes!  

The CAN ID bits are used as following:  
[MSb]        [LSb]  
rsssssddddd  
d: [0..4] destination node adress  
s: [5..10] source node address  
r: [11] reserved  

Special addresses  
0x00: reserved/interlink node  
0x1F: broadcast  

#Data (8 byte)  
The first data byte is the intent byte.  
This specifies the purpose of the package and allows the recieving node  
to determine what the following 7 data bytes contain.  
The amount of data bits is variable between at 1 and 8.  

