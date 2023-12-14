from PyQt6 import uic
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtCore import pyqtSignal as Signal

import os
import sys
import cv2 as cv
import numpy as np
from cv2 import aruco

class Camera_calibration_class(QThread):
    ProgbarValue = Signal(int)
    Matrices = Signal(list)
    def __init__(self):
        super().__init__()
        self.detector = None
        self.board = None
        self.images = None
    def Read_chessboard_from_images(self):
        allCorners = []
        allIds = []
        decimator = 0
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.00001)       
        for i in range(self.images):
            frame = cv.imread(os.path.join(os.getcwd(), "tmp") + "/" + str(i) + ".jpg", cv.IMREAD_COLOR)
            gray = cv.cvtColor(frame, cv.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
            if len(corners)>0:               
                for corner in corners:
                    cv.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria) 
                res2 = 	cv.aruco.interpolateCornersCharuco(corners,ids,gray,self.board)
                if res2[1] is not None and res2[2] is not None and len(res2[1])>3 and decimator%1==0:
                    allCorners.append(res2[1])
                    allIds.append(res2[2])
            decimator+=1
            self.ProgbarValue.emit(int(decimator/self.images*100)-5)
            imsize = gray.shape
        return allCorners,allIds,imsize

    def Camera_calibration_method(self,allCorners,allIds,imsize):
        cameraMatrixInit = np.array([[ 1000, 0, imsize[0]/2.], [0, 1000, imsize[1]/2.], [0, 0, 1]])

        distCoeffsInit = np.zeros((12,1))
        flags = (cv.CALIB_USE_INTRINSIC_GUESS + cv.CALIB_RATIONAL_MODEL + cv.CALIB_FIX_ASPECT_RATIO)
        (ret, camera_matrix, distortion_coefficients0,
        rotation_vectors, translation_vectors,
        stdDeviationsIntrinsics, stdDeviationsExtrinsics,
        perViewErrors) = cv.aruco.calibrateCameraCharucoExtended(
                        charucoCorners=allCorners,
                        charucoIds=allIds,
                        board=self.board,
                        imageSize=imsize,
                        cameraMatrix=cameraMatrixInit,
                        distCoeffs=distCoeffsInit,
                        flags=flags,
                        criteria=(cv.TERM_CRITERIA_EPS & cv.TERM_CRITERIA_COUNT, 10000, 1e-9))
        return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors       
    
    
    def Get_Board_CSC(self,mtx,dist):
        rvecs = None
        tvecs = None
        for i in range(self.images):
            gray = cv.cvtColor(cv.imread(os.path.join(os.getcwd(), "tmp") + "/" + str(i) + ".jpg", cv.IMREAD_COLOR), cv.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            for corner in corners:
                cv.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)                        
            rvec = np.float32(np.array([0,0,1]))
            tvec = np.float32(np.array([0,0,0]))
            if len(corners) > 0:
                charucoretval, charucoCorners, charucoIds = cv.aruco.interpolateCornersCharuco(corners, ids, gray, self.board)
                retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, self.board, mtx, dist,rvec,tvec)
            if rvecs is None and tvecs is None:
                rvecs = np.reshape(rvec,(1,3))
                tvecs = np.reshape(tvec,(1,3))/1000
            else:
                rvec = np.reshape(rvec,(1,3))
                rvecs = np.append(rvecs,rvec,axis = 0)
                tvec = np.reshape(tvec,(1,3))/1000
                tvecs = np.append(tvecs,tvec,axis = 0)
        return rvecs,tvecs
                
    def run(self):
        allCorners,allIds,imsize = self.Read_chessboard_from_images() 
        ret, mtx, dist, rvecs, tvecs = self.Camera_calibration_method(allCorners,allIds,imsize)
        rvecs,tvecs = self.Get_Board_CSC(mtx,dist)
        self.ProgbarValue.emit(100)
        self.Matrices.emit([mtx,dist,rvecs,tvecs])  
    
    def stop(self):
        self.quit()