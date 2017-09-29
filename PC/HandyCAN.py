import serial
import logging
import threading

logging.basicConfig(level=logging.DEBUG)


class HandyCANPackage(object):
    length = None
    data = []
    source = None
    dest = None

    def __str__(self):
        prettyOutput = "Incoming CAN packet from {} ({}) to {} ({}) with length {}\nContaining data {} {}".format(
            self.source, hex(self.source),
            self.dest, hex(self.dest),
            self.length, self.data, [hex(elem) for elem in self.data])
        return prettyOutput
    
    

class HandyCAN(object):
    def __init__(self, own_address):
        self.log = logging.getLogger("HandyCAN")
        self.startsyncbyte1 = 0xF0
        self.startsyncbyte2 = 0xFA
        self.endsyncbyte1 = 0xE0
        self.endsyncbyte2 = 0xEF
        self.lenidx = 2
        self.checklenidx = 3
        self.StdIdLidx = 4
        self.StdIdHidx = 5
        self.dataidx = 6
        self.sourceoffset = 5
        self.own_address = own_address
        
    def init_serial(self, ser, recieveCallback):
        self.recieveCallback = recieveCallback
        self.ser = ser
        self.thread_read = threading.Thread(target=self.recieveThread)
        self.thread_read.daemon = True
        self.thread_read.name = 'InterlinkNode to PC'
        self.thread_read.start()

    def sendPack(self, package):
        message = self.encodeCANMessage(package)
        bytesmessage = [elem.to_bytes(1, byteorder='big') for elem in message]
        for byte in bytesmessage:
            self.ser.write(byte)

    def send(self, dest, data):
        pack = HandyCANPackage()
        pack.source = self.own_address
        pack.dest = dest
        pack.data = data
        pack.length = len(data)
        self.sendPack(pack)
        

    def recieveThread(self):
        alive = True
        message = []
        previousByte = None

        while alive:
            indata = self.ser.read(1)
            if indata:
                #check if this is the start of the message
                if indata == b'\xfa':
                    if previousByte ==  b'\xf0':
                        #if so, clear the message buffer
                        message = [int.from_bytes(previousByte, byteorder='big')]
                #check if this is the beginning of the end of the message
                if indata == b'\xef':
                    if previousByte == b'\xe0':
                        #if so, decode it and call the callback
                        message.append(int.from_bytes(indata, byteorder='big'))
                        package = self.decodeCANMessage(message)
                        self.recieveCallback(package)
                # this is a data byte. append it to the package.
                message.append(int.from_bytes(indata, byteorder='big'))                        
                previousByte = indata


    
    def decodeCANMessage(self, message):
        pack = HandyCANPackage()

        self.log.debug([hex(elem) for elem in message])
        
        if message[0] != 0xF0 or message[1] != 0xFA:
            raise IOError("not a valid handycan message: invalid header")
        if message[-1] != 0xEF or message[-2] != 0xE0:
            raise IOError("not a valid handycan message: invalid footer")
        
        #First of all, get the data length.
        #We need this to properly decode the package
        pack.length = message[self.lenidx]
        # get the total length of the package, including overhead
        
        #total lenght of the message is startsync + len + checklen + idl + idh + data + checkbytes + endsync
        # or 2 + 1 + 1 + 1 + 1 + length + checklen +  2
        # or  8 + length + checklen
        total_length = 8 + message[self.lenidx] + message[self.checklenidx]
       
        if total_length != len(message):
            print("Message too long or too short")

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
        >>> hcp = HandyCANPackage()
        >>> hcp.length = 6
        >>> hcp.data = [0, 0xF0, 0xFA, 0, 0xE0, 0xEF]
        >>> hcp.source = 1
        >>> hcp.dest = 2
        >>> hc = HandyCAN(0)
        >>> hc.encodeCANMessage(hcp)
        [240, 250, 6, 2, 34, 0, 0, 240, 251, 0, 224, 240, 8, 11, 224, 239]
        >>> msg = hc.encodeCANMessage(hcp)
        >>> hcp2 = hc.decodeCANMessage(msg)
        >>> hcp.length == 6
        True
        >>> hcp.data == [0, 0xF0, 0xFA, 0, 0xE0, 0xEF]
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
             if (message[check - 1] == 0xF0 and message[check] == 0xFA) or (message[check - 1] == 0xE0 and message[check] == 0xEF):
                 message[check] += 1
                 shift.append(check)

        for checkloc in shift:
            message.append(checkloc)

        message[3]= len(shift)

        message.append(0xE0)
        message.append(0xEF)

        #print ([hex(elem) for elem in message])
        return message
    


if __name__ == "__main__":
    import doctest
    import time
    doctest.testmod()

    
    ser = serial.Serial('/dev/ttyUSB0',1152000, timeout=0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    def rx(package):
        print(package)

    hc = HandyCAN(0)
    hc.init_serial(ser, rx)

    while 1:
        hc.send(0x0A, [2, 0])
        time.sleep(0.1)
        hc.send(0x0A, [2, 1])
        time.sleep(0.1)
    
    

