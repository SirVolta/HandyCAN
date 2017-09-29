# PC Application
The interlink node sends encapsulated CAN packets over the UART to the PC,
and transmits encapsulated CAN packets to the interlink node, which in turn sends them over the CAN bus.
This way the PC software has maximum flexibility, and the relatively processor intensive job of
decoding all CAN packets is done by the PC, not the interlink node.

This folder contains a python implementation of the [interlink protocol described here.](https://github.com/SirVolta/HandyCAN/tree/master/doc/protocol)  

Example usage of the python module will be in the examples folder.  


