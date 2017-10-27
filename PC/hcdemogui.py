# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QColorDialog
from PyQt5.QtGui import *
import sys
import serial
import logging
import pyHandyCAN
import time


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(463, 484)
        self.centralWidget = QtWidgets.QWidget(MainWindow)
        self.centralWidget.setObjectName("centralWidget")
        self.listWidget = QtWidgets.QLabel(self.centralWidget)
        self.listWidget.setGeometry(QtCore.QRect(10, 180, 401, 81))
        self.listWidget.setObjectName("listWidget")
        self.up = QtWidgets.QLabel(self.centralWidget)
        self.up.setGeometry(QtCore.QRect(10, 320, 401, 120))
        self.up.setObjectName("up")

        self.pushButton = QtWidgets.QPushButton(self.centralWidget)
        self.pushButton.setGeometry(QtCore.QRect(10, 270, 94, 36))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralWidget)
        self.pushButton_2.setGeometry(QtCore.QRect(10, 10, 94, 36))
        self.pushButton_2.setObjectName("pushButton_2")
        self.temp = QtWidgets.QLabel(self.centralWidget)
        self.temp.setGeometry(QtCore.QRect(10, 60, 421, 20))
        self.temp.setObjectName("temp")
        self.sw1 = QtWidgets.QLabel(self.centralWidget)
        self.sw1.setGeometry(QtCore.QRect(10, 130, 63, 30))
        self.sw1.setObjectName("sw1")
        self.sw2 = QtWidgets.QLabel(self.centralWidget)
        self.sw2.setGeometry(QtCore.QRect(90, 130, 63, 30))
        self.sw2.setObjectName("sw2")
        self.sw3 = QtWidgets.QLabel(self.centralWidget)
        self.sw3.setGeometry(QtCore.QRect(180, 130, 63, 30))
        self.sw3.setObjectName("sw3")
        self.lightlevel = QtWidgets.QLabel(self.centralWidget)
        self.lightlevel.setGeometry(QtCore.QRect(10, 90, 400, 20))
        self.lightlevel.setObjectName("lightlevel")
        self.motion = QtWidgets.QLabel(self.centralWidget)
        self.motion.setGeometry(QtCore.QRect(270, 130, 63, 30))
        self.motion.setObjectName("motion")
        MainWindow.setCentralWidget(self.centralWidget)
        self.menuBar = QtWidgets.QMenuBar(MainWindow)
        self.menuBar.setGeometry(QtCore.QRect(0, 0, 463, 30))
        self.menuBar.setObjectName("menuBar")
        MainWindow.setMenuBar(self.menuBar)
        self.mainToolBar = QtWidgets.QToolBar(MainWindow)
        self.mainToolBar.setObjectName("mainToolBar")
        MainWindow.addToolBar(QtCore.Qt.TopToolBarArea, self.mainToolBar)
        self.statusBar = QtWidgets.QStatusBar(MainWindow)
        self.statusBar.setObjectName("statusBar")
        MainWindow.setStatusBar(self.statusBar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.pushButton.setText(_translate("MainWindow", "Detect Nodes"))
        self.pushButton_2.setText(_translate("MainWindow", "Set Color"))
        self.temp.setText(_translate("MainWindow", "Temp"))
        self.sw1.setText(_translate("MainWindow", "SW1"))
        self.sw2.setText(_translate("MainWindow", "SW2"))
        self.sw3.setText(_translate("MainWindow", "SW3"))
        self.lightlevel.setText(_translate("MainWindow", "LightLevel"))
        self.motion.setText(_translate("MainWindow", "Motion"))

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
log = logging.getLogger("HcDemoGUI")

        
class HcDemoGUI(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(HcDemoGUI, self).__init__()
        self.setupUi(self)
        self.setAcceptDrops(True)
        
        self.sw1.setPixmap(QPixmap('res/amber-off-32.png'))
        self.sw2.setPixmap(QPixmap('res/blue-off-32.png'))
        self.sw3.setPixmap(QPixmap('res/yellow-off-32.png'))
        self.motion.setPixmap(QPixmap('res/red-off-32.png'))

        self.updateTimer = QtCore.QTimer()
        self.updateTimer.timeout.connect(self.update)
        self.updateTimer.start(100)
        self.cnt = 0
        
        self.pushButton.clicked.connect(self.onDetect)
        self.pushButton_2.clicked.connect(self.onColor)

        self.nodes = []
        self.upts = ['', '', '', '']

        ser = serial.Serial('/dev/ttyUSB0',2000000)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        
        ## The main HandyCAN class, set address to 0 for now
        ## and enable CTS send mode
        self.hc = pyHandyCAN.HandyCAN(0, cts=True)
        # connect the serial port and recieve function
        self.hc.init_serial(ser, self.rx)
        
        self.show()

    def update(self):
        if self.cnt == 1:
            self.hc.send(sensoraddr, [temp_intent])
        elif self.cnt == 2:
            self.hc.send(sensoraddr, [light_intent])
        elif self.cnt == 3:
            self.hc.send(sensoraddr, [uptime_intent])
        elif self.cnt == 4:
            self.hc.send(switchaddr, [uptime_intent])
        elif self.cnt == 5:
            self.hc.send(lightaddr, [uptime_intent])
        elif self.cnt == 6:
            self.hc.send(switchaddr, [get_switch_state_intent, 11])
        elif self.cnt == 7:
            self.hc.send(switchaddr, [get_switch_state_intent, 12])
        elif self.cnt == 8:
            self.hc.send(switchaddr, [get_switch_state_intent, 15])                        
        else:
            self.cnt = 0
        self.cnt += 1
        

        pass

    def onDetect(self):
        self.hc.send(broadcastaddr, [discover_intent])
        self.nodes=[]

    def onColor(self):
        color = QColorDialog.getColor()
 
        if color.isValid():
            r, g, b, alpha = color.getRgb()
            self.hc.send(lightaddr, [light_set_color_intent, r, g, b])
        
    def rx(self, package):
        if package.error:
            print("error: ", package.error)
            return
        log.debug(package)

        intent = package.data[0]
        if intent == discover_response_intent:
            out = "Found a {} node at address {}".format(package.data[1], hex(package.source))
            log.debug(out)
            if not out in self.nodes:
                self.nodes.append(out)
            self.listWidget.setText('\n'.join(self.nodes))

        if intent == uptime_intent:
            if len(package.data) < 5:
                package.data.append(0)
            out = "Node {} is up for {} days, {} hours, {} minutes and {} seconds".format(package.source,
                                                                                          package.data[4], package.data[3],
                                                                                          package.data[2], package.data[1])

            self.upts[package.source] = out
            self.up.setText('\n'.join(self.upts))
            log.debug(out)

        if package.source == switchaddr:
            if intent == switch_change_intent or intent == get_switch_state_intent:
                out = "switch {} on node {} is now {}".format(package.data[1], package.source, package.data[2])
                log.debug(out)
                switch = package.data[1]
                state = package.data[2]
                if switch == 11:
                    if state:
                        self.sw1.setPixmap(QPixmap('res/amber-off-32.png'))
                    else:
                        self.sw1.setPixmap(QPixmap('res/amber-on-32.png'))
                if switch == 12:
                    if state:
                        self.sw2.setPixmap(QPixmap('res/blue-off-32.png'))
                    else:
                        self.sw2.setPixmap(QPixmap('res/blue-on-32.png'))

                if switch == 15:
                    if state:
                        self.sw3.setPixmap(QPixmap('res/yellow-off-32.png'))
                    else:
                        self.sw3.setPixmap(QPixmap('res/yellow-on-32.png'))


                        

        if package.source == sensoraddr:
            if intent == motion_intent:
                if package.data[2]:
                    self.motion.setPixmap(QPixmap('res/red-on-32.png'))
                    out = "Motion detected on sensor {} of node {}".format(package.data[1], package.source)
                else:
                    self.motion.setPixmap(QPixmap('res/red-off-32.png'))
                    out = "Motion stopped on sensor {} of node {}".format(package.data[1], package.source)
                    
                log.debug(out)

            if intent == light_intent:
                lightlevel = package.data[1] | (package.data[2] << 8)
                out = "light level on node {} is {}".format(package.source, lightlevel)
                self.lightlevel.setText(out)
                log.debug(out)

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

                self.temp.setText(out)
                log.debug(out)

    
        
        
        
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    app = QApplication(sys.argv)
    ex = HcDemoGUI()
    sys.exit(app.exec_())
