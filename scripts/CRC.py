from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtCore import pyqtSignal as Signal

import sys
import socket

## ------ Check if the robot is connected to the socket server ------##
class Check_robot_connection(QThread):
    Robot_is_connceted = Signal(bool)
    def __init__(self):
        super().__init__()
        self.ip = None
        
    def run(self): 
        try:
            HOST = self.ip
            PORT = 30003
            s = socket.socket()
            s.settimeout(3)
            s.connect((HOST, PORT))
            s.settimeout(None)
            s.close()    
            self.Robot_is_connceted.emit(True)
        except:
            self.Robot_is_connceted.emit(False)
            
    def stop(self):
        self.quit()
