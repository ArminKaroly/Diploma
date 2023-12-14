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
from sympy import * # Symbol, cos, sin, diff, subs
from Used_functions import rodrigues_vec_to_rotation_mat, invHT, prodHT, prodHTs, OptimalTransformation, RLink
class Grad_Optimalization(QThread):
    T_T2C = Signal(np.ndarray)
    DH_params = Signal(np.ndarray)
    def __init__(self,Robot,learning_rate,step_size,step_size_decay):
        super().__init__()
        self.Robot = Robot
        self.learning_rate = learning_rate
        self.step_size = step_size
        self.step_size_decay = step_size_decay
        self.joints = None
        self.rvecs = None
        self.tvecs = None
        self.joints_kalib= None
        self.joints_test = None
        self.p_cam_kalib = None
        self.p_cam_test = None
        self.iternum = 0
        
    def run(self):
        with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
            joints = np.array(list(csv.reader(csvfile)))
            joints = joints.astype(float)
        with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
            rvecs = np.array(list(csv.reader(csvfile)))
            rvecs = rvecs.astype(float)
        with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
            tvecs = np.array(list(csv.reader(csvfile)))
            tvecs = tvecs.astype(float)
        with open(os.path.join(os.getcwd(), "tmp") + "/T_TCP2C.csv", newline='\n') as csvfile:
            T_TCP2C = np.array(list(csv.reader(csvfile)))
            T_TCP2C = T_TCP2C.astype(float)      
        with open(os.path.join(os.getcwd(), "tmp") + "/T_Base2Board.csv", newline='\n') as csvfile:
            T_Base2Board = np.array(list(csv.reader(csvfile)))
            T_Base2Board = T_Base2Board.astype(float)
            T_Base2Board = np.reshape(T_Base2Board,(4,4,1))
        
        DH_parameters_general_symbolic = np.repeat(np.array([[Symbol('filler_d'),Symbol('filler_theta'),Symbol('filler_a'),Symbol('filler_alpha')]]),joints.shape[1],axis = 0)
        
        for i in range(0,joints.shape[1]):
            DH_parameters_general_symbolic[i,:] = np.array([[Symbol('d'+str(i)),Symbol('theta'+str(i)),Symbol('a'+str(i)),Symbol('alpha'+str(i))]])
        
        symbolic_T = np.eye(4)    
        for i in range(0,joints.shape[1]):
            symbolic_T = np.matmul(symbolic_T,self.Symbolic_traf_DH(DH_parameters_general_symbolic[i,:]))
        symbolic_T = np.matmul(symbolic_T,T_TCP2C)
        
        T_TCP2C = np.reshape(T_TCP2C,(4,4,1))
        Jacobi_general_symbolic = self.Generate_Jacobian(symbolic_T,DH_parameters_general_symbolic)
        print("##########################################")
        print("##------ ENTERED CALIBRATION AREA ------##")
        print("##########################################")
        
        self.Generate_calibration_and_test(5,joints,rvecs,tvecs,T_Base2Board)
        print(self.Calculate_Cost(T_TCP2C))
        for e in range(10):
                    
            Flags = (np.random.rand(int(self.p_cam_kalib.shape[0]/10))*self.p_cam_kalib.shape[0]).astype(int) 
            
            Errors_matrix = self.Generate_Error(Flags,T_TCP2C)
            
            weights = np.sqrt(np.sum(Errors_matrix*Errors_matrix,axis = 1))
                    
            Numeric_DHS = self.Generate_numeric_DHs(Flags)
            
            Numeric_jacobian = self.Calculate_Jacobian(Jacobi_general_symbolic,DH_parameters_general_symbolic,Numeric_DHS)
            
            Delta_DHs = np.zeros((Numeric_jacobian.shape[2],self.joints_kalib.shape[1]*4))
            
            for i in range(Numeric_jacobian.shape[2]):
                Delta_DHs[i,:] = np.reshape(self.Calculate_Delta_DH(Numeric_jacobian[:,:,i],np.reshape(Errors_matrix[i,:],(3,1))),(self.joints_kalib.shape[1]*4,))
            
            self.step_size = self.step_size*self.step_size_decay
            
            sum_weight = sum(weights)
            weights = np.reshape(weights,(-1,1))
            
            Delta_DHs = np.sum(Delta_DHs*weights,axis = 0)/sum_weight
                    
            DHS_Actual = self.Robot.GetParam(np.ones(self.joints_kalib.shape[1])) + Delta_DHs
            
            self.Robot.SetParam(DHS_Actual,np.ones(self.joints_kalib.shape[1]))
            print(    "##------ DH ------##")
            print(np.reshape(self.Robot.GetParam(np.ones(self.joints_kalib.shape[1])),(-1,4)))
            print(    "##------ Hiba ------##")
            print(self.Calculate_Cost(T_TCP2C))
                        
    def Calculate_Cost(self,T_TCP2C):
        p_robot = np.transpose(prodHT(prodHTs(self.Robot.T(np.transpose(self.joints_test)),T_TCP2C),np.array([[0],[0],[0]])))
        difference = self.p_cam_test-p_robot
        difference = np.sqrt(np.sum((difference*difference),axis = 0))
        return difference
    
    '''##------ Functions for symbolic calibration ------##'''    
    # Generate a symbolic Homogeneous transformation matrix, usin DH parameters
    # Inputs:  DH parameters -> (a,d,alpha,theta) where a(float), d(float), alpha(float), theta(float)
    # Outputs:  Symbolic 4x4 homogeneous transformation matric
    def Symbolic_traf_DH(self,dh):
        d = dh[0]
        theta = dh[1]
        a = dh[2]
        alpha = dh[3]
        sa = sin(alpha)
        ca = cos(alpha)
        st = sin(theta)
        ct = cos(theta)
        return np.array([[ct, -st*ca, st*sa, a*ct],[st, ct*ca, -ct*sa, a*st],[0, sa, ca, d],[0,0,0,1]])     
    
    def Generate_Jacobian(self,Trafo,DH_parameters):
    # Return a Jacobian matrix that defines the delta(x,y,z) movement, when one of the DH parameter change
    # Inputs: Trafo: Symbolic 4x4 numpy.array(Symbolic) homogeneous transformation matrix
    #         DH_parameters: A symbolic Nx4 np.array(Symbolic) matrix that contains the symbolic DH parameters
    # Outputs:  Jacobian_sym: 3x4K np.array(Symbolic) Jacobian matrix where K is the number of the joints of the robot
        Jacobian_sym = np.repeat(np.array([[Symbol("filler_x")],[Symbol("filler_y")],[Symbol("filler_z")]]),DH_parameters.shape[0]*4,axis = 1)
        parameters = DH_parameters.flatten('F')
        for i in range(parameters.shape[0]):
            Jacobian_sym[:,i] = np.array([diff(Trafo[0,3],parameters[i]),diff(Trafo[1,3], parameters[i]),diff(Trafo[2,3], parameters[i])])
        return Jacobian_sym    
    
    def Generate_calibration_and_test(self,part_test,joints,rvecs,tvecs,T_Base2Board):
        ##------ flag vector, there maybe several pictures that cann't be used for calibration ------##
        Flags = np.zeros(rvecs.shape[0])
        T_C2Board = np.zeros((4,4,rvecs.shape[0]))
        ##------ Generate T_C2Board matrices (4,4,N) and change FLAG vector to which matrices are correct ------##
        for i in range(rvecs.shape[0]):
            if not np.all(rvecs[i,:] == np.array([[0,0,1]])) and not np.all(tvecs[i,:] == np.array([[0,0,0]])):
                Flags[i] = 1
                Rot = rodrigues_vec_to_rotation_mat(rvecs[i,:])
                tra = np.reshape(tvecs[i,:],(3,1))
                T_C2Board[:,:,i] = np.append(np.append(Rot,tra,axis = 1),np.array([[0,0,0,1]]),axis = 0)
        
        T_C2Board = T_C2Board[:,:,Flags == 1]    
        p_cam = np.transpose(prodHT(prodHTs(T_Base2Board,invHT(T_C2Board)),np.array([[0],[0],[0]])))
        joints = joints[Flags == 1,:]
        
        flag = np.zeros(joints.shape[0])
        random_values = np.random.rand(joints.shape[0])
        flag[random_values <= 1/part_test] = 1
        self.joints_kalib = joints[flag == 0,:]
        self.joints_test = joints[flag == 1,:]
        self.p_cam_kalib = p_cam[flag == 0,:]
        self.p_cam_test = p_cam[flag == 1,:]
        
    def Generate_Error(self,Flags,T_TCP2C):
        p_robot = prodHT(prodHTs(self.Robot.T(np.transpose(self.joints_kalib[Flags,:])),T_TCP2C),np.array([[0],[0],[0]]))
        p_cam_need = self.p_cam_kalib[Flags,:]
        p_robot = np.transpose(p_robot)
        return p_robot-p_cam_need
    
    def Generate_numeric_DHs(self,Flags):
        used_joint_coords = self.joints_kalib[Flags,:]
        flags = np.ones(used_joint_coords.shape[1])
        Numeric_DHS = np.repeat(np.reshape(self.Robot.GetParam(flags),(used_joint_coords.shape[1],4,1),order = 'F'),used_joint_coords.shape[0],axis = 2)
        for i in range(used_joint_coords.shape[0]):
            for j in range(used_joint_coords.shape[1]):
                if type(self.Robot.data[j]) == type(RLink(np.array([0,0,0,0]))):
                    Numeric_DHS[j,1,i] = Numeric_DHS[j,1,i] + used_joint_coords[i,j]
                else:
                    Numeric_DHS[j,0,i] = Numeric_DHS[j,0,i] + used_joint_coords[i,j] 
        return Numeric_DHS

    def Calculate_Jacobian(self,Jacobian_symbolic,sym_parameters,num_parameters):
    # Returns numerical value of the (Generate_Jacobian) Jacobian matrix (connection between x,y,z and the DH parameters) 
    # Inputs:   Jacobian: 3x4K np.array(Symbolic) Jacobian matrix where K is the number of the joints of the robot
    #           Sym_parameters: 1x4K np.array(Symbolic) vector that contains all the Symbolic DH parameters
    #           num_parameters: 1x4k np.array(float) vector that contsains all the numerical value of the DH parameters
    # Output:   Calculate_Jacobian: 3x4K np.array(float) numerical Jacobian matrix (connection between x,y,z and the DH parameters)
        Numeric_Jacobian = np.zeros((Jacobian_symbolic.shape[0],Jacobian_symbolic.shape[1],num_parameters.shape[2]))
        symbolic_parameters = sym_parameters.flatten('F')
        for i in range(num_parameters.shape[2]):
            numeric_parameters = num_parameters[:,:,i].flatten('F')
            for j in range(Jacobian_symbolic.shape[1]):
                x = Jacobian_symbolic[0,j]
                y = Jacobian_symbolic[1,j]
                z = Jacobian_symbolic[2,j]
                for k in range(symbolic_parameters.shape[0]):
                    x = x.subs(symbolic_parameters[k],numeric_parameters[k])
                    y = y.subs(symbolic_parameters[k],numeric_parameters[k])
                    z = z.subs(symbolic_parameters[k],numeric_parameters[k])
                Numeric_Jacobian[:,j,i] = np.array([x,y,z])
            #print("   ##---- JACOBIAN " + str(i) + " CALCULATED ----##")
        return Numeric_Jacobian
        
    def Calculate_Delta_DH(self,Jacobian,Error):
    # Calculate the gradiens delta DH values 
    # Inputs:   Jacobian: 3x4K numpy.array(float) numerical JAcobian matrix (connection between x,y,z and the DH parameters)
    #           Error: 3x1 numpy.array(float) vector that contains the deviation of the measured and the calculated point position
    #           learning_rate: float, that describe the gradient learning 
    #           step_size: float, define the step size of the algorithm
    # Outputs:  Calculate_Delta_DH: 4Kx1 numpy.array(float) vector that contains the delta DHs 
        return np.matmul(np.matmul(np.linalg.inv(np.matmul(np.transpose(Jacobian),Jacobian) + self.learning_rate*np.eye(4*self.joints_kalib.shape[1])), np.transpose(Jacobian)),Error) * self.step_size
    
    def stop(self):
        self.quit()    


