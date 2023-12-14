from PyQt6 import uic
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtCore import pyqtSignal as Signal

import os
import sys
import csv
import numpy as np
from scipy import optimize
from scipy.spatial.transform import Rotation as R
from Used_functions import rodrigues_vec_to_rotation_mat, invHT, prodHT, prodHTs, OptimalTransformation

class Pre_Calculation(QThread):
    Progbar_value = Signal(int)
    T_B2b = Signal(np.ndarray)
    T_T2C = Signal(np.ndarray)
    def __init__(self,T_Init,Robot):
        super().__init__()
        self.T_Base2Board = T_Init
        self.Robot = Robot
        self.joints = None
        self.rvecs = None
        self.tvecs = None
        self.T_full = None
        self.R_full = None
        self.Q = None
        self.R_Base2Board = None
    def run(self):
        ##------ Open things that are save in temporary folder ------##
            if True:
                with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
                        joints = np.array(list(csv.reader(csvfile)))
                        self.joints = joints.astype(float)
                
                with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
                        rvecs = np.array(list(csv.reader(csvfile)))
                        self.rvecs = rvecs.astype(float)
                
                with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
                        tvecs = np.array(list(csv.reader(csvfile)))
                        self.tvecs = tvecs.astype(float)
                
                
                T_C2Board = np.zeros((4,4,self.rvecs.shape[0]))
                
                ##------ flag vector, there maybe several pictures that cann't be used for calibration ------##
                flags = np.zeros(self.rvecs.shape[0])
                
                ##------ Generate matrices (4,4,N) ------##
                for i in range(rvecs.shape[0]):
                    if not np.all(self.rvecs[i,:] == np.array([[0,0,1]])) and not np.all(self.tvecs[i,:] == np.array([[0,0,0]])):
                        flags[i] = 1
                        Rot = rodrigues_vec_to_rotation_mat(self.rvecs[i,:])
                        tra = np.reshape(self.tvecs[i,:],(3,1))
                        T_C2Board[:,:,i] = np.append(np.append(Rot,tra,axis = 1),np.array([[0,0,0,1]]),axis = 0)                          
                T_Base2TCP = self.Robot.T(np.transpose(self.joints))         
                T_C2Board = T_C2Board[:,:,flags == 1]
                T_Base2TCP = T_Base2TCP[:,:,flags == 1]
                T_TCP2Base = invHT(T_Base2TCP)
                T_TCP2Board = prodHTs(T_TCP2Base,self.T_Base2Board)
                
                ##------ Board origo calculated from camera, and from TCP ------##
                points_from_camera = prodHT(T_C2Board,np.array([[0],[0],[0]]))
                points_from_TCP = prodHT(T_TCP2Board,np.array([[0],[0],[0]]))
                
                
                ##------ Calculate Optimal transformation between TCP and Camera frame ------##
                T_TCP2C = np.reshape(OptimalTransformation(points_from_camera,points_from_TCP)[0],(4,4,1))
                self.Progbar_value.emit(33)
                
                
                ##------ Calculate full loop ------##
                self.T_full = prodHTs(prodHTs(prodHTs(T_Base2TCP,T_TCP2C),T_C2Board),invHT(self.T_Base2Board))            
                self.R_full = self.T_full[:3,:3,:]
                self.Q = self.R_full.shape[2]
                
                ##------ Optimize T_Base2Board with least square algorithm using Cost_R and Cost_t funcions ------##     
                R_Base2Board_ls = optimize.least_squares(self.Cost_R,np.array([0,0,0]),ftol = 1e-15, xtol = 1e-15, gtol = 1e-15)
                self.Progbar_value.emit(67)            
                self.R_Base2Board = R.from_euler( 'xyz', R_Base2Board_ls.x, degrees = False).as_matrix()
                
                t_Base2Board_ls = optimize.least_squares(self.Cost_t,np.array([0,0,0]),ftol = 1e-15, xtol = 1e-15, gtol = 1e-15)
                
                T_diff = np.reshape(np.append(np.append(self.R_Base2Board,np.reshape(t_Base2Board_ls.x,(3,1)), axis = 1),np.array([[0,0,0,1]]),axis = 0),(4,4,1))  
                T_Base2Board = prodHTs(self.T_Base2Board,T_diff)
                self.Progbar_value.emit(100)
                self.T_B2b.emit(T_Base2Board)
                self.T_T2C.emit(T_TCP2C)
            #except Exception as e:
            #    print(e)
            
    ##------ Cost function, we want to reach euler('xyz'[0,0,0]) ------##
    def Cost_R(self,ori_par):
        output = np.zeros((self.Q,3))
        Rotmat = R.from_euler('xyz',ori_par,degrees = False).as_matrix()
        for i in range(self.Q):
            output[i,:] =  np.reshape(R.from_matrix(np.matmul(self.R_full[:,:,i],Rotmat)).as_euler('xyz',degrees = False),(1,3))
        #print(np.max(np.absolute(output), axis = 0))
        return np.sum(output*output,axis = 0)
    
    ##------ Cost function, we want to reach minimal distances ------##
    def Cost_t(self,tra_par):
        output = np.zeros((self.Q,3))
        Tramat = np.append(np.append(self.R_Base2Board,np.array([[tra_par[0]],[tra_par[1]],[tra_par[2]]]),axis = 1),np.array([[0,0,0,1]]),axis = 0)
        for i in range(self.Q):
            output[i,:] =  np.reshape(np.matmul(np.matmul(self.T_full[:,:,i],Tramat),np.array([[0],[0],[0],[1]]))[:3],(1,3))
        #print(np.max(np.absolute(output), axis = 0))    
        return np.sum(output*output,axis = 0)
        
    def stop(self):
        self.quit()
    
