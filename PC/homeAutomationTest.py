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
Demonstration of the python HandyCAN library in a home automation setup

"""

import serial
import logging
import pyHandyCAN
import time

logging.basicConfig(level=logging.INFO)
## get a logger
log = logging.getLogger("hcdemo")

uptime_intent = 0x01
switch_change_intent = 0x02
discover_intent = 0xFF
discover_response_intent = 0xFE
get_switch_state_intent = 0x03

temp_intent = 0x02
motion_intent = 0x03
light_intent = 0x04
temp_alarm_intent = 0x05

light_set_color_intent = 0x02
light_set_power_intent = 0x03

broadcastaddr = 0x1F
switchaddr = 0x01
sensoraddr = 0x02
lightaddr = 0x03

if __name__ == "__main__":
    ## open a serial port to the interlink node
    ser = serial.Serial('/dev/ttyUSB0',2000000, timeout=0.25)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    def rx(package):
        if package.error:
            print("error: ", package.error)
            return
        log.debug(package)

        intent = package.data[0]
        if intent == discover_response_intent:
            out = "Found a {} node at address {}".format(package.data[1], hex(package.source))
            log.info(out)

        if intent == uptime_intent:
            if len(package.data) < 5:
                package.data.append(0)
            out = "Node {} is up for {} days, {} hours, {} minutes and {} seconds".format(package.source,
                                                                                          package.data[4], package.data[3],
                                                                                          package.data[2], package.data[1])
            log.info(out)

        if package.source == switchaddr:
            if intent == switch_change_intent:
                out = "switch {} on node {} is now {}".format(package.data[1], package.source, package.data[2])
                log.info(out)

            if intent == get_switch_state_intent:
                out = "switch {} on node {} is {}".format(package.data[1], package.source, package.data[2])
                log.info(out)

        if package.source == sensoraddr:
            if intent == motion_intent:
                if package.data[2]:
                    out = "Motion detected on sensor {} of node {}".format(package.data[1], package.source)
                else:
                    out = "Motion stopped on sensor {} of node {}".format(package.data[1], package.source)
                    
                log.info(out)

            if intent == light_intent:
                lightlevel = package.data[1] | (package.data[2] << 8)
                out = "light level on node {} is {}".format(package.source, lightlevel)
                log.info(out)

            if intent == temp_intent:
                error = package.data[1]
                if not error:
                    temp_decimal = package.data[2]
                    temp_intergral = package.data[3]
                    hum_decimal = package.data[4]
                    hum_intergal = package.data[5]
                    temp = (temp_intergral & 0x7F) * 256 + temp_decimal
                    if temp_intergral &0x80:
                        temp *= -1
                    hum = hum_intergal *256 + hum_decimal

                    out = "Temp: {}c, hum: {}% at node {}".format(temp / 10, hum / 10, package.source)
                else:
                    out = "DHT22 error on node{}: {}".format(package.source, error)
                log.info(out)


    ## The main HandyCAN class, set address to 0 for now
    hc = pyHandyCAN.HandyCAN(0)
    # connect the serial port and recieve function
    hc.init_serial(ser, rx)

    time.sleep(1)
    hc.send(broadcastaddr, [discover_intent])
    hc.send(lightaddr, [light_set_color_intent, 0x00, 0xAA, 0x00])

    while 1:
        time.sleep (2)
        #hc.send(sensoraddr, [light_intent])
        #continue
        hc.send(sensoraddr, [temp_intent])
        time.sleep(1)
        hc.send(lightaddr, [light_set_color_intent, 0xFA, 0x24, 0x14])
        time.sleep(5)
        hc.send(sensoraddr, [light_intent])
        time.sleep(5)
        hc.send(sensoraddr, [uptime_intent])
        time.sleep(1)
        hc.send(lightaddr, [light_set_color_intent, 0xAA, 0x45, 0x14])
        time.sleep(5)
        hc.send(switchaddr, [uptime_intent])
        time.sleep(5)
        hc.send(lightaddr, [uptime_intent])
        time.sleep(1)
        hc.send(lightaddr, [light_set_color_intent, 0xFF, 0x68, 0xBA])
        time.sleep(5)
        

        
