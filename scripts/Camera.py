from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtCore import pyqtSignal as Signal, pyqtSlot as Slot

import sys
import cv2 as cv
import cv2.aruco as aruco

## ------ Create a camera class that refresh the camera image (right side of the main window) ------##
class Camera(QThread):
    ImageUpdate = Signal(QImage)
    def __init__(self):
        super().__init__()
        self.Camera_index = None
        self.frame = None
        self.checked = None
        self.detector = None
        self.Capture = None
        self.board = None
        self.mtx = None
        self.dist = None
        
    def run(self):
        self.ThreadActive = True
        self.Capture = cv.VideoCapture(self.Camera_index)
        self.Capture.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        self.Capture.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        while self.ThreadActive:
            ret, self.frame = self.Capture.read()
            if ret:
                image = cv.cvtColor(self.frame, cv.COLOR_BGR2RGB)
                if self.checked:
                    gray = cv.cvtColor(image, cv.COLOR_RGB2GRAY)
                    m_corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
                    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
                    for corner in m_corners:
                        cv.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)                        
                    image = aruco.drawDetectedMarkers(image, m_corners, borderColor = (255,0,0))
                ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(1280,720,Qt.AspectRatioMode.KeepAspectRatio)
                self.ImageUpdate.emit(Pic)
    
    def stop(self):
        self.ThreadActive = False
        self.quit()
        self.Capture.release()  
