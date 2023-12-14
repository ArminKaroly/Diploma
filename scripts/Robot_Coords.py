from PyQt6 import uic
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtCore import pyqtSignal as Signal

import sys
import time
import socket
import numpy as np

from state import State
from urStateReceiver import UrStateReceiver

class Get_robot_coordinates(QThread):
    Robot_coordinates = Signal(np.ndarray)  
    def __init__(self):
        super().__init__()
        self.robot_ip = None
        self.receiver = None

    def run(self):
        try:
            HOST = self.robot_ip
            PORT = 30003
            TIMEOUT = 3
            self.receiver = UrStateReceiver(HOST, PORT)
            values = np.zeros((2,6))
            while True:
                start = time.perf_counter()
                try:
                    received = self.receiver.PollDataFromSocket(TIMEOUT)
                except socket.error as msg:
                    sys.exit(1)
                q = 0
                for iii in received:
                    if q > 11:
                        break
                    if iii.visible == 'True':
                        values[int(np.floor(q/6)),q%6] = iii.value
                        q = q + 1   
                self.Robot_coordinates.emit(values) 
        except Exception as e:
            print(e)          
