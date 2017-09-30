# HandyCAN interlink node library demo
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
Demonstration of the python HandyCAN library

"""

import serial
import logging
import pyHandyCAN
import time

logging.basicConfig(level=logging.DEBUG)
## get a logger
log = logging.getLogger("hcdemo")

if __name__ == "__main__":
    ## open a serial port to the interlink node
    ser = serial.Serial('/dev/ttyUSB0',2000000, timeout=0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    ## previous byte used for overrun test
    previousDat = 0
    
    def rx(package):
        """
        Called by the HandyCAN library when a package arrives

        Args:
        package: the recieved package

        Examples:
        doxypypy bug requires example section...
        """
        global previousDat
        
        if package.error:
            print("error: ", package.error)
            return
        
        dat = package.data[4]
        
        if dat != previousDat +1 and dat !=0 and previousDat != 255:
            log.warning (" Overrun detected: {} {}".format( dat, previousDat))
        previousDat = dat
        log.info(package)

    ## The main HandyCAN class, set address to 0 for now
    hc = pyHandyCAN.HandyCAN(0)
    # connect the serial port and recieve function
    hc.init_serial(ser, rx)

    while 1:
        try:
            hc.send(0x0A, [2, 0, 0xE0, 0xEE])
        except IOError as e:
            log.error(e)
        time.sleep(2.1)
        hc.send(0x0A, [2, 1, 0xF0, 0xFA])
        time.sleep(2.1)

        
