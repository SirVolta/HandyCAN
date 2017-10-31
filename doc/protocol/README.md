# Protocol Documentation
Here is where I go into full detail of both the HandyCAN protocol and the interlink node UART protocol.  

 This file is part of HandyCAN  

 HandyCAN is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.  
  
 HandyCAN is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 See the GNU General Public License for more details.  

 You should have received a copy of the GNU General Public License
 along with HandyCAN.  If not, see <http://www.gnu.org/licenses/>.  

## The Interlink protocol
The interlink protocol is the protocol used between the interlink node and the PC.  
It is used to send and receive raw CAN messages from the PC using the interlink node via a serial port.  
If you are developing a application for the PC, this is the protocol you need to implement in your program 
to allow communication to and from the interlink node.  
  
### The start and end of message issue
Because a serial port is used, a can message must be packed into several bytes to be sent over the UART.  
This raises the issue of indicating when a CAN message starts and ends.  
Using a ASCII system, a simple linefeed \n would solve this.
But also make the protocol extremely processor intensive and inefficient, as every
binary data point will need to be converted into a string and back again.  
This is why the interlink protocol is binary.
But, using just binary data transfer, it is impossible to know where the packet starts and stops.
This would require a unique identifier, but the data may also contain this identifier, 
falsely indicating the start or end of a message.  
To solve this, a two byte start and stop is used, and the data is scanned weather this byte combination
is present in the data. If so, the byte is incremented, and the position of this incremented byte is stored 
so the receiver can decrement it again.  
To improve reliability, the amount of incremented bytes is also sent. By verifying this number
against the actual number of bytes received, a message with a missing byte can be discarded instead 
of returning corrupted data.  

### Message structure
The first and last two bytes the sync bytes.
They are constant and combined they are unique in the message.  
Following the two start bytes is the data length byte. 
This can range from 1 to 8, and indicates the amount of data bytes in the message.  
Next is the amount of shift bytes in the message that require decrementing.  
After this come the data bytes.  
And then the shift bytes if any.  

The message structure will look like this:  
[startsync1, startsync2, datalength, shiftlength, StdId low, StdId high, databytes, checkbytes, endsync1, endsync2]

The values of the magic sync bytes are:  
startsync1: 0xF0
startsync2: 0xFA
endsync1: 0xE0
endsync2: 0xEE

The magic bytes are chosen quite big, so verification of the length bytes is not needed, and more importantly,
the shift addresses themselves cannot become syncbytes as their value can never be larger then the amount of 
bytes in a message.  

Say you want to transmit the data [0x04, 0x08] to address 1, from address 0, the package will look like this:  
[0xF0, 0xFA, 2, 0, 1, 0, 0x04, 0x08, 0xE0, 0xEE]

A few more examples:
data: [0x04, 0x08, 0xF0, 0xFA] to address 1, from address 0:  
[0xF0, 0xFA, 2, 1, 1, 0, 0x04, 0x08, 0xF0, 0xFB, 0x10, 0xE0, 0xEE]

data: [0x04, 0x08, 0xE0, 0xEE] to address 1, from address 0:  
[0xF0, 0xFA, 2, 1, 1, 0, 0x04, 0x08, 0xE0, 0xEF, 0x10, 0xE0, 0xEE]


### More about _the sync_
Sender:  
First, the sender constructs the message with the start, length, ID, and data.
Now the sender loops over this data, excluding the start and length, and checks for the magic start or end somewhere in the package.  
If this is the case, it increments the last byte by one, increments the shiftlength, and places the index of that
incremented byte after the data in the message.  
Now it appends the two end bytes and the message is ready to be sent.  

Receiver:  
The receiver processes all incoming bytes from the UART.  
If the two start sync bytes are detected, it starts storing all further bytes, until the two end sync bytes are detected.  
Now a raw interlink message is received, the receiver first checks the first two and last two bytes again.  
After this the two length bytes are extracted, summed and checked against the total length of the message.  
If the message length is not equal to the calculated length plus message overhead, a byte was lost and the message must be discarded.  
Now the receiver must check if there are syncbytes. If so, extract their indexes from after the data section and decrement them.  
Now one complete CAN message is available to the receiver


## The HandyCAN protocol
This is the protocol that is sent over the CAN bus between (interlink) nodes.
If you are developing a PC application, you will have to implement this protocol to encode and decode 
to be sent or received using the interlink protocol.
To recap: PC -> HandyCAN -> Interlink -> CAN BUS, or, CAN BUS -> Interlink -> HandyCAN -> PC
A python implementation of this is available in the PC folder.

In the node, the HandyCAN library will handle the encoding and decoding of packages.  
So if you are just using nodes, you do not have do anything to use the HandyCAN protocol.  

### CAN ID: Addressing
HandyCAN uses the normal 11-bit CAN ID mode, allowing for 29 unique node addresses.  
With some modification it is possible to change this to 29 bit id, allowing for 16 381 nodes!  

The CAN ID bits are used as following:  
[MSb]        [LSb]  
rsssssddddd  
d: [0..4] destination node address  
s: [5..10] source node address  
r: [11] reserved  

This way a unique ID is guaranteed, and allows the CAN prioritizing to work properly both ways.  
The destination node with the lowest address will go before nodes with a higher address.  
If the destination is the same, which often happens with the interlink node, the source address
will now determine the priority.  

#### Addresses
Normal node addresses can be in between 0x01 and 0x1E (1 and 30)
Every node on the bus MUST have its own address!  
If two nodes on the same bus share a address, bus collisions and undefined behavior will happen!  
The interlink node address is controlled by the PC software, but nodes can always transmit to 0x00
to reach all interlink nodes, and nothing more.  
There also is a broadcast address to address all nodes at the same time.  
This can be especially useful for device detection and similar operations.  

##### Special addresses:
-  0x00: reserved/interlink node
-  0x1F: broadcast


### Data (8 byte)  
The first data byte is the intent byte.  
This specifies the purpose of the package and allows the receiving node  
to determine what the following 7 data bytes contain.  
The amount of data bits is variable between at 1 and 8.  


