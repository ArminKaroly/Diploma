from PyQt6 import uic
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtCore import pyqtSignal as Signal

import os
import sys
import time
import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R

## ------ Take images automatically ------##
class Auto_image_taker(QThread):
    Images = Signal(int)
    Robot_joint_values = Signal(np.ndarray)
    def __init__(self,start_pose,positions,camera,robot_coord_class,widgets,num_of_images):
        super().__init__()
        self.robot_coord_class = robot_coord_class
        self.start_pose = start_pose
        self.positions = positions
        self.camera = camera
        self.widgets = widgets
        self.robot_coordinates = None
        self.robot_joints = None
        self.images = num_of_images
        self.joint_values = np.zeros((1,6))
        
    def run(self):
        while(self.robot_coordinates is None):
            time.sleep(0.1)
            j = 0
        for i in self.positions:
            rotvec = R.from_euler('xyz',i[3:6],degrees = False).as_rotvec()
            pos = "movej(p["+str(i[0])+","+str(i[1])+","+str(i[2])+","+str(rotvec[0])+","+str(rotvec[1])+","+str(rotvec[2])+"], a=0.2, v=0.5)" + "\n"
            self.robot_coord_class.s.send(pos.encode())
            iters = 0
            while((np.abs(i[0]-self.robot_coordinates[0])>=0.001 or np.abs(i[1]-self.robot_coordinates[1])>=0.001 or np.abs(i[2]-self.robot_coordinates[2])>=0.001 or np.abs(i[3]-self.robot_coordinates[3])/np.pi*180>=1 or np.abs(i[4]-self.robot_coordinates[4])/np.pi*180>=1 or np.abs(i[5]-self.robot_coordinates[5])/np.pi*180>=1) and iters<75):
                time.sleep(0.1)
                iters = iters + 1
            time.sleep(1.5)
            try:
                cv.imwrite(os.path.join(os.getcwd(), "tmp") + "/" + str(self.images + j)+".jpg",self.camera.frame)
                self.joint_values = np.append(self.joint_values,self.robot_joints,axis = 0)
                j += 1
            except Exception as e:
                print(e)
        self.joint_values = self.joint_values[1:,:]
        rotvec = R.from_euler('xyz',self.start_pose[3:6],degrees = False).as_rotvec()
        start_pos = "movej(p["+str(self.start_pose[0]/1000)+","+str(self.start_pose[1]/1000)+","+str(self.start_pose[2]/1000)+","+str(rotvec[0])+","+str(rotvec[1])+","+str(rotvec[2])+"], a=0.2, v=0.5)" + "\n"    
        self.robot_coord_class.s.send(start_pos.encode()) 
        time.sleep(3)
        
        
        ####### FORCILUSSAL ###############
        
        self.widgets[0].setTabEnabled(0,True)
        self.widgets[0].setTabEnabled(2,True)
        self.widgets[1].setDisabled(False)
        self.widgets[2].setDisabled(False)
        self.widgets[3].setDisabled(False)
        self.widgets[4].setDisabled(False)
        self.widgets[5].setDisabled(False)       
        self.widgets[6].setDisabled(False)
        self.widgets[7].setDisabled(False)
        self.widgets[8].setDisabled(False)
        self.widgets[9].setDisabled(False)      
        self.widgets[10].setDisabled(False)
        self.widgets[11].setDisabled(False)
        self.widgets[12].setDisabled(False)
        self.widgets[13].setDisabled(False)
        self.Images.emit(self.images + j)
        self.Robot_joint_values.emit(self.joint_values)

    def Refresh_robot_pose(self,Robot_coordinates):
        euler = R.from_rotvec(Robot_coordinates[1,3:6]).as_euler("xyz", degrees = False)
        translation = Robot_coordinates[1,0:3]
        self.robot_joints = Robot_coordinates[0,:]
        self.robot_joints = np.reshape(self.robot_joints,(1,6))
        self.robot_coordinates = np.append(translation,euler)
        
    def stop(self):
        self.quit()
