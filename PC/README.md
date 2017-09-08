# PC Application
The interlink nodes sends encapsulated CAN packets over the UART to the PC,
and transmits encapsulated CAN packets to the interlink node, which in turn sends them over the CAN bus.
This way the PC software has maximum flexibility, and the processor intensive job of
decoding all CAN packets is done by the PC, not the interlink node.

This folder contains a description of the UART protocol used by the interlink node
and a implementation of it as a python module.

Example usage of the python module will be in the examples folder.
