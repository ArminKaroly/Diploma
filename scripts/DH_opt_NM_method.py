from PyQt6 import uic
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtCore import pyqtSignal as Signal

import os
import csv
import time
import numpy as np
from scipy import optimize
from scipy.spatial.transform import Rotation as R
from Used_functions import rodrigues_vec_to_rotation_mat, invHT, prodHT, prodHTs, OptimalTransformation

class NM_Optimalization(QThread):
    Progbar_value = Signal(int)
    DH_params = Signal(np.ndarray)
    T_T2C = Signal(np.ndarray)
    def __init__(self,Robot):
        super().__init__()
        self.Robot = Robot
        self.joints = None
        self.rvecs = None
        self.tvecs = None
        self.iternum = 0
    
    def run(self):
        with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
            joints = np.array(list(csv.reader(csvfile)))
            self.joints = joints.astype(float)
        with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
            rvecs = np.array(list(csv.reader(csvfile)))
            self.rvecs = rvecs.astype(float)
        with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
            tvecs = np.array(list(csv.reader(csvfile)))
            self.tvecs = tvecs.astype(float)
        with open(os.path.join(os.getcwd(), "tmp") + "/T_TCP2C.csv", newline='\n') as csvfile:
            T_TCP2C = np.array(list(csv.reader(csvfile)))
            T_TCP2C = T_TCP2C.astype(float)
            T_TCP2C = np.reshape(T_TCP2C,(4,4,1))    
        with open(os.path.join(os.getcwd(), "tmp") + "/T_Base2Board.csv", newline='\n') as csvfile:
            T_Base2Board = np.array(list(csv.reader(csvfile)))
            T_Base2Board = T_Base2Board.astype(float)
            T_Base2Board = np.reshape(T_Base2Board,(4,4,1))
        
        ##------ flag vector, there maybe several pictures that cann't be used for calibration ------##
        Flags = np.zeros(self.rvecs.shape[0])
        T_C2Board = np.zeros((4,4,self.rvecs.shape[0]))
        
        ##------ Generate T_C2Board matrices (4,4,N) and change FLAG vector to which matrices are correct ------##
        for i in range(self.rvecs.shape[0]):
            if not np.all(self.rvecs[i,:] == np.array([[0,0,1]])) and not np.all(self.tvecs[i,:] == np.array([[0,0,0]])):
                Flags[i] = 1
                Rot = rodrigues_vec_to_rotation_mat(self.rvecs[i,:])
                tra = np.reshape(self.tvecs[i,:],(3,1))
                T_C2Board[:,:,i] = np.append(np.append(Rot,tra,axis = 1),np.array([[0,0,0,1]]),axis = 0)
        
        T_C2Board = T_C2Board[:,:,Flags == 1]    
        p_cam = prodHT(prodHTs(T_Base2Board,invHT(T_C2Board)),np.array([[0],[0],[0]]))
        
        joints = self.joints[Flags == 1,:] 
        joints = np.transpose(joints)
        K = joints.shape[0]
        
        flags = np.ones(K)
        x = self.Robot.GetParam(flags)
        
        self.Robot.SetParam(x,flags)
        p_robot = prodHTs(self.Robot.T(joints),T_TCP2C)
        p_robot = prodHT(p_robot,np.array([[0],[0],[0]]))
        
        elotte = (p_robot-p_cam)        
        elotte_sum = np.sum(elotte*elotte,axis = 1)
        elotte_max = np.max(elotte*elotte,axis = 1)
        elotte_mean = np.mean(elotte*elotte,axis = 1)
        
        ido = time.time()
        x = optimize.fmin(self.Cost_DH,x,(p_cam,self.Robot,joints,T_TCP2C,flags),ftol = 1e-3, xtol = 1e-3,maxiter = 3000)
        print(time.time()-ido)
        self.Progbar_value.emit(3000)
        self.DH_params.emit(x)
        
        print(x)
        
        self.Robot.SetParam(x,flags)
        p_robot = prodHTs(self.Robot.T(joints),T_TCP2C)
        p_robot = prodHT(p_robot,np.array([[0],[0],[0]]))
        
        T_diff = OptimalTransformation(p_cam,p_robot)
        
        T_diff = np.reshape(T_diff[0],(4,4,1))
        T_TCP2C = prodHTs(T_TCP2C,T_diff)
        
        self.T_T2C.emit(T_TCP2C)
        
        p_robot = prodHTs(self.Robot.T(joints),T_TCP2C)
        p_robot = prodHT(p_robot,np.array([[0],[0],[0]]))        
        utana = (p_robot-p_cam)
        
        utana_sum = np.sum(utana*utana,axis = 1)
        utana_max = np.max(utana*utana,axis = 1)
        utana_mean = np.mean(utana*utana, axis = 1)
        print("________________________")
        print("________________________")
        print("Elotte:")
        print("sum:",np.sqrt(elotte_sum))
        print("max:",np.sqrt(elotte_max))
        print("mean:",np.sqrt(elotte_mean))
        print("________________________")
        print("________________________")
        print("Utana:")
        print("sum:",np.sqrt(utana_sum))
        print("max:",np.sqrt(utana_max))
        print("mean:",np.sqrt(utana_mean))
        print("________________________")
        print("________________________")
        
        '''
        elotte = np.sqrt(np.sum(elotte * elotte,axis = 0))
        utana = np.sqrt(np.sum(utana * utana,axis = 0))
        
        kimenetek1 = np.append(np.reshape(elotte_sum,(1,3)),np.reshape(utana_sum,(1,3)),axis = 1)
        kimenetek2 = np.append(np.reshape(elotte_max,(1,3)),np.reshape(utana_max,(1,3)),axis = 1)
        kimenetek3 = np.append(np.reshape(elotte_mean,(1,3)),np.reshape(utana_mean,(1,3)),axis = 1)
        kimenetek = np.append(np.append(kimenetek1,kimenetek2,axis = 0),kimenetek3,axis = 0)
        '''
        #np.savetxt(os.getcwd() + "/" + 'out.csv', kimenetek, delimiter=',')
        
    def Cost_DH(self,x,p_cam,Robot,joints,T_TCP2C,flags):
        Robot.SetParam(x,flags)
        p_robot = prodHTs(Robot.T(joints),T_TCP2C)
        p_robot = prodHT(p_robot,np.array([[0],[0],[0]]))
        value = OptimalTransformation(p_cam,p_robot)
        self.iternum += 1
        self.Progbar_value.emit(self.iternum)
        #print(value[1][1])
        return value[1][1]
        
    def stop(self):
        self.quit()
