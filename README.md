# HandyCAN
The HandyCAN is a communication solution to link many
STM32 microcontrollers and a computer using a CAN bus network.
The HandyCAN specification specifies three different type of devices:
Node, InterlinkNode, PC

Every node on the network has a address and a base type.
Nodes listen only to its own address, and a special broadcast address.
This way any node can communicate with any other node without bothering others,
while also being able to address all nodes at the same time.
An Interlink node serves as a translator between the CAN bus and a PC.
This allows a computer program to send and receive CAN packets and,
for example, control nodes from the internet.
Interlink nodes are connected to the computer using a high speed (1-2Mbaud) UART.

Every HandyCAN packet has an intent. This intent indicates what a recipient should do when
it receives the package. This can mean manipulating hardware, configuring itself,
sending status information, reading sensors or any other thing a node could possibly do.
It is a good idea to use different intents for different types of node so that when a packet
is sent to the wrong node, it will do nothing.
Intent 255 is reserved for device detection. When one (interlink) node broadcasts intent 255,
all other nodes will in turn respond with its address and type.
This way it is possible to discover all devices on the bus and get their type and address.

Following this, every node must have:
- an 5 bit address. This must be unique to every node.
- an 8 bit node type. Used to indicate what the function of a node is.
  One type can be shared with many nodes.


Following is a example of HandyCAN being used in a simple home automation setup:
Here HandyCAN is used to connect a light switch, RGB LED Strip, and a light and motion sensor to a
Raspberry Pi. This makes it possible for the user to finely tweak the lighting in a room.
For example: which color and intensity of light he/she wants, at which time of day,
at a specific ambient light level, only when motion is detected, etc.
This While still being able to force it on and off using a hardware toggle switch.
![HomeAutomationExample](https://raw.githubusercontent.com/SirVolta/HandyCAN/master/doc/resources/HandyCan_HomeAut_Example.png "HandyCAN Home Automation Example")



For the moment this is just empty shell, but HandyCAN is under rapid development
and will see an implementation soon.
Example usages and the protocol specification is coming soon.
