# HandyCAN interlink node library
# Copyright (C) 2017 Pelle Sepp Florens Jansen
#
# This file is part of HandyCAN 
#
# HandyCAN is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# HandyCAN is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with HandyCAN.  If not, see <http://www.gnu.org/licenses/>.
"""
HandyCAN interlink node library

Communicates to an interlink node using a serial port,
and allows recieving and transmitting of HandyCAN packages

\mainpage

\section intro_sec Introduction
This is a python libary and example implementation of both the
Interlink Protocol and the HandyCAN protocol.

\section nav_sec Navigation
The library description can be found here: pyHandyCAN.HandyCAN \n
An example implementation is here: pyHandyCAN.py

\see https://github.com/SirVolta/HandyCAN/tree/master/examples

"""

import logging
import threading
import datetime

logging.basicConfig(level=logging.DEBUG)


class HandyCANPackage(object):
    """
    Represents one HandyCANPackage
    """
    ## Amount of data bytes in the package
    length = None
    ## data (maximum of 8)
    data = []
    ## package source address
    source = None
    ## package destination address
    dest = None
    ## message decode error
    error = None

    def __str__(self):
        """
        String representation of the package
        """
        prettyOutput = "Incoming CAN packet from {} ({}) to {} ({}) with length {}\nContaining data {} {} ".format(
            self.source, hex(self.source),
            self.dest, hex(self.dest),
            self.length, self.data, [hex(elem) for elem in self.data])
        if self.error:
            prettyOutput += "Error: {} ".format(self.error)
        return prettyOutput
    
    

class HandyCAN(object):
    """
    Handles both the HandyCAN and the interlink protocol.
    
    It allows sending and recieving HandyCAN packages through a serial port connected to a interlink node.
    """
    ## Always the first byte of a interlink mesage
    startsyncbyte1 = 0xF0
    ## Always the secondbyte of a interlink mesage
    startsyncbyte2 = 0xFA
    ## Indicates the end of a interlink message
    endsyncbyte1 = 0xE0
    ## Must come right after #endsyncbyte1 to indicate end
    endsyncbyte2 = 0xEE
    ## Index of the length byte in interlink message
    lenidx = 2
    ## index of the amount of check bytes in a interlink message
    checklenidx = 3
    ## index of the StdId low byte in a interlink message
    StdIdLidx = 4
    ## index of the StdId high byte in a interlink message
    StdIdHidx = 5
    ## index of the start of the data in a interlink message
    dataidx = 6
    ## offset of the source bits in a STDID HandyCANPackage
    sourceoffset = 5
    ## Minimum time between output packages in microseconds.\n
    ## This is required as the PC has no idea how many free mailboxes the interlink
    ## node still has avaliable, a temporary solution is to dramatiaclly lower the max send
    ## speed to prevent overruns. All packages must be ack'd anyways to ensure delivery.
    minimum_time_between_packages = 1000 #microseconds
        
    def __init__(self, own_address):
        """
        Sets the local adress

        Examples:
        hc = HandyCAN(0x0)
        """
        ## Logging class
        self.log = logging.getLogger("HandyCAN")
        ## Last time a package was sent
        self.last_package_sent_time = datetime.datetime.now()
        ## our own adress used by the send() function
        self.own_address = own_address
        
    def init_serial(self, ser, recieveCallback):
        """
        Initializes the serial port

        Args:
        ser: A Serial.serial instance connected to the interlink node
        recieveCallback: the function called when a package is recieved. It takes one argument: the package.

        Examples:
        >>> def rx(message):\
          print (message)
        
        ser = serial.Serial('/dev/ttyUSB0',2000000, timeout=0.25)
        hc.init_serial(ser, rx)
        """
        ## the function to be called when a valid package has arrived
        self.recieveCallback = recieveCallback
        ## the serial port connected to the interlink node
        self.ser = ser
        ## Serial data recieve thread
        self.interlinkThread = threading.Thread(target=self.recieveThread)
        self.interlinkThread.daemon = True
        self.interlinkThread.name = 'InterlinkNode to PC'
        self.interlinkThread.start()

    def sendPack(self, package):
        """
        Sends a HandyCANPackage

        Args:
        package: the HandyCANPackage to send

        Returns:
        The amount of bytes actually sent

        Examples:
        pack = HandyCANPackage()
        pack.source = 0x00
        pack.dest = 0x03
        pack.data = [1, 2, 3]
        pack.length = len(data)
        hc.sendPack(pack)
        """
        
        timediff = datetime.datetime.now() - self.last_package_sent_time
        if timediff.microseconds < self.minimum_time_between_packages:
            return 0

        try:
            message = self.encodeCANMessage(package)
        except IOError as e:
            raise e
        self.log.debug(" Sending: {}".format([hex(elem) for elem in message]))
        
        bytesmessage = [elem.to_bytes(1, byteorder='big') for elem in message]
        bytes_sent = 0
        for byte in bytesmessage:
            bytes_sent += self.ser.write(byte)
            
        self.last_package_sent_time = datetime.datetime.now()
        return bytes_sent

    def send(self, dest, data):
        """
        Sends data to a node

        Args:
        dest: Destination node adress
        data: data to send

        returns:
        Amount of bytes sent to interlink node

        Examples:
        hc.send(0x03, [1, 2, 3]
        """
        pack = HandyCANPackage()
        pack.source = self.own_address
        pack.dest = dest
        pack.data = data
        pack.length = len(data)
        try:
            return self.sendPack(pack)
        except IOError as e:
            raise e
        

    def recieveThread(self):
        """
        Runs in the background, recieving bytes from the interlink node.
        
        When a valid message is detected, it calls the callback

        """
        alive = True
        message = []
        previousByte = None

        while alive:
            indata = self.ser.read(1)
            if indata:
                #check if this is the start of the message
                if indata == self.startsyncbyte2.to_bytes(1, byteorder='big'):
                    if previousByte == self.startsyncbyte1.to_bytes(1, byteorder='big') :
                        #if so, clear the message buffer
                        message = [int.from_bytes(previousByte, byteorder='big')]
                #check if this is the beginning of the end of the message
                if indata == self.endsyncbyte2.to_bytes(1, byteorder='big'):
                    if previousByte == self.endsyncbyte1.to_bytes(1, byteorder='big'):
                        #if so, decode it and call the callback
                        message.append(int.from_bytes(indata, byteorder='big'))
                        package = self.decodeCANMessage(message)
                        self.recieveCallback(package)
                # this is a data byte. append it to the package.
                message.append(int.from_bytes(indata, byteorder='big'))                        
                previousByte = indata


    
    def decodeCANMessage(self, message):
        """
        Takes a interlink protocol message and converts it into HandyCAN

        Args:
        message: interlink message from interlink node

        returns:
        A HandyCAN package

        examples:
        >>> message = [240, 250, 6, 2, 34, 0, 0, 240, 251, 0, 224, 239, 8, 11, 224, 238]
        >>> hc = HandyCAN(0)
        >>> pack = hc.decodeCANMessage(message)
        >>> pack.length == 6
        True
        >>> pack.data == [0, 0xF0, 0xFA, 0, 0xE0, 0xEE]
        True
        >>> pack.source == 1
        True
        >>> pack.dest == 2
        True
        >>> hc.encodeCANMessage(pack)
        [240, 250, 6, 2, 34, 0, 0, 240, 251, 0, 224, 239, 8, 11, 224, 238]
        
        """
        pack = HandyCANPackage()

        self.log.debug(" Recieved: {}".format([hex(elem) for elem in message]))
        
        if message[0] != self.startsyncbyte1 or message[1] != self.startsyncbyte2:
            pack.error = "not a valid handycan message: invalid header. Expected {} {} got {} {}".format(hex(self.startsyncbyte1), hex(self.startsyncbyte2), hex(message[0]), hex(message[1]))
            return pack
        if message[-1] != self.endsyncbyte2 or message[-2] != self.endsyncbyte1:
            pack.error = "not a valid handycan message: invalid footer. Expected {} {} got {} {}".format(hex(self.endsyncbyte1), hex(self.endsyncbyte2), hex(message[-2]), hex(message[-1]))
            return pack
        
        #First of all, get the data length.
        #We need this to properly decode the package
        pack.length = message[self.lenidx]
        # get the total length of the package, including overhead
        
        #total lenght of the message is startsync + len + checklen + idl + idh + data + checkbytes + endsync
        # or 2 + 1 + 1 + 1 + 1 + length + checklen +  2
        # or  8 + length + checklen
        total_length = 8 + message[self.lenidx] + message[self.checklenidx]
       
        if total_length != len(message):
            pack.error = "Invalid message length: expected {} got {} ".format(total_length, len(message))
            log.error(pack.error)
            return pack
        
        # Now we need to test wether there are bytes needing decrementing
        # and decrement them.
        n_checkbytes = message[self.checklenidx]
        if n_checkbytes:
            # the offset of the checkbytes is startsync + len + checklen + idl + idh + data
            # or 2 + 1 + 1 + 1 + 1 + length or 6 + length'
            checkbytes_start = 6 + pack.length
            #fetch which bytes need incrementing
            checkbytes = [message[i + checkbytes_start] for i in range(n_checkbytes)]
            if len (checkbytes) != n_checkbytes:
                raise IOError ("Missing byte in checkbytes!")
            # then increment them
            for checkbyte in checkbytes:
                message[checkbyte] -= 1
    
        StdId = message[self.StdIdLidx] | message[self.StdIdHidx] << 8;
        pack.source = (StdId & 0x3E0) >> 5;
        pack.dest = StdId & 0x1F
        pack.data = []
        for i in range(self.dataidx, pack.length + 6, 1):
            pack.data.append(message[i])

        return pack
        

    def encodeCANMessage(self, pack):
        """
        Takes a HandyCANPackage and converts it into a interlink message

        Args:
        pack: HandyCANPackage to convert

        Returns:
        Interlink message

        
        Examples:
        >>> hcp = HandyCANPackage()
        >>> hcp.length = 6
        >>> hcp.data = [0, 0xF0, 0xFA, 0, 0xE0, 0xEE]
        >>> hcp.source = 1
        >>> hcp.dest = 2
        >>> hc = HandyCAN(0)
        >>> hc.encodeCANMessage(hcp)
        [240, 250, 6, 2, 34, 0, 0, 240, 251, 0, 224, 239, 8, 11, 224, 238]
        >>> msg = hc.encodeCANMessage(hcp)
        >>> hcp2 = hc.decodeCANMessage(msg)
        >>> hcp.length == 6
        True
        >>> hcp.data == [0, 0xF0, 0xFA, 0, 0xE0, 0xEE]
        True
        >>> hcp.source == 1
        True
        >>> hcp.dest == 2
        True
        
        """
        StdId = pack.dest | (pack.source << self.sourceoffset)
        message = [
            #0, 1 start sync
            self.startsyncbyte1, self.startsyncbyte2,
            #2 length
            pack.length,
            #3: number of checkbytes
            0,
            #4: stdid low
            StdId & 0xFF,
            #5: stdid high
            ((StdId & 0xFF00) >> 8) & 0xFF,
        ]
        dataOffset = len(message)
        # todo: check if length <= length of list pack.data
        for i in range(pack.length):
            message.append(pack.data[i])

        shift = []
        #Generate sync
        for check in range(3, len(message), 1):
             if (message[check - 1] == self.startsyncbyte1 and message[check] == self.startsyncbyte2) or (message[check - 1] == self.endsyncbyte1 and message[check] == self.endsyncbyte2):
                 message[check] += 1
                 shift.append(check)

        for checkloc in shift:
            message.append(checkloc)

        message[3]= len(shift)

        message.append(self.endsyncbyte1)
        message.append(self.endsyncbyte2)

        return message
    


if __name__ == "__main__":
    """
    Now run some tests and demos
    """
    import doctest
    import time
    doctest.testmod()

    

        
    
    

