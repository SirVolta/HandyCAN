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
An example implementation is here: hcdemo.py

\see https://github.com/SirVolta/HandyCAN/tree/master/examples

"""
import queue
import logging
import threading
import time
import datetime


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
        prettyOutput = "HandyCAN package from {} ({}) to {} ({}) with length {} Containing data {} {} ".format(
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
    minimum_time_between_packages = 10000 #microseconds
    ## A better solution to delay is to use the CTS line from the interlink node
    ## this is set to true if it is to be used
    cts = False
        
    def __init__(self, own_address, cts=False, delay=10000):
        """
        Sets the local adress

        Examples:
        hc = HandyCAN(0x0)
        """
        ## Logging class
        self.log = logging.getLogger("HandyCAN")
        self.cts = cts
        self.delay = delay
        ## Last time a package was sent
        self.last_package_sent_time = datetime.datetime.now()
        ## our own adress used by the send() function
        self.own_address = own_address
        self.recieveQueue = queue.Queue()
        self.transmitQueue = queue.Queue()
        ## Transmit enable
        self.txe = True
        
        
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
        
        self.recieveCallbackThread = threading.Thread(target=self.recieveCallbackThread)
        self.recieveCallbackThread.daemon = True
        self.recieveCallbackThread.name = 'InterlinkNode to PC Callback caller'
        self.recieveCallbackThread.start()
        
        self.transmitThread = threading.Thread(target=self.transmitThread)
        self.transmitThread.daemon = True
        self.transmitThread.name = 'PC To Interlink node'
        self.transmitThread.start()

        self.log.info("Serial initialized")


    def transmitThread(self):
        while True:
            if not self.cts:
                time.sleep(0.000001) # short cpu-unload delay
                #cts is not used, wait for the minimum time between packs
                timediff = datetime.datetime.now() - self.last_package_sent_time
                self.log.debug("waiting to transmit")
                if timediff.microseconds > self.minimum_time_between_packages:
                    message = self.transmitQueue.get(block=True, timeout=None)
                    self.log.debug("Delay sending: {} ".format([hex(elem) for elem in message]))
                    self.ser.write(message)
                    self.last_package_sent_time = datetime.datetime.now()
                    self.transmitQueue.task_done()
            else:
                #wait for 200 microseconds for cts line delay
                #maximum speed: 4000 packages per second
                time.sleep(200.0 * (10.0 ** -6))
                if self.txe:
                    if self.ser.cts:
                        message = self.transmitQueue.get(block=True, timeout=None)
                        self.log.debug("CTS Sending: {} ".format([hex(elem) for elem in message]))
                        bytes_sent = self.ser.write(message)
                        self.last_package_sent_time = datetime.datetime.now()
                        self.transmitQueue.task_done()
                        

    def sendPack(self, package):
        """
        Queues a HandyCANPackage for send.

        Args:
        package: the HandyCANPackage to send

        Returns:
        nothing

        Examples:
        pack = HandyCANPackage()
        pack.source = 0x00
        pack.dest = 0x03
        pack.data = [1, 2, 3]
        pack.length = len(data)
        hc.sendPack(pack)
        """
        try:
            message = self.encodeCANMessage(package)
        except IOError as e:
            raise e
        self.log.debug("Queued for send: {} ".format([hex(elem) for elem in message]))
        self.transmitQueue.put(list(message))


    def send(self, dest, data):
        """
        Sends data to a node

        Args:
        dest: Destination node adress
        data: data to send

        returns:
        Amount of bytes sent to interlink node

        Examples:
        hc.send(0x03, [1, 2, 3])
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
                        #if so, place it into the queue
                        message.append(int.from_bytes(indata, byteorder='big'))
                        self.recieveQueue.put(list(message))
                # this is a data byte. append it to the package.
                message.append(int.from_bytes(indata, byteorder='big'))                        
                previousByte = indata

    def recieveCallbackThread(self):
        while 1:
            # microsecond delay for safety
            time.sleep(1.0 * 10.0 ** -6)
            message = self.recieveQueue.get(block=True, timeout=None)
            package = self.decodeCANMessage(message)
            self.recieveCallback(package)
            self.recieveQueue.task_done()

    
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

        self.log.debug("Decoding:  {}".format([hex(elem) for elem in message]))
        
        if message[0] != self.startsyncbyte1 or message[1] != self.startsyncbyte2:
            pack.error = "not a valid handycan message: invalid header. Expected {} {} got {} {}".format(hex(self.startsyncbyte1), hex(self.startsyncbyte2), hex(message[0]), hex(message[1]))
            self.log.error(pack.error)
            return pack
        if message[-1] != self.endsyncbyte2 or message[-2] != self.endsyncbyte1:
            pack.error = "not a valid handycan message: invalid footer. Expected {} {} got {} {}".format(hex(self.endsyncbyte1), hex(self.endsyncbyte2), hex(message[-2]), hex(message[-1]))
            self.log.error(pack.error)
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
            self.log.error(pack.error)
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
    logging.basicConfig(level=logging.DEBUG)
    import doctest
    import time
    doctest.testmod()

