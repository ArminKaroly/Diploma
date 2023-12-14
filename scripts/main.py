from PyQt6 import uic
from PyQt6.QtGui import *
from PyQt6.QtCore import *
from PyQt6.QtWidgets import *
from PyQt6 import QtWidgets, QtGui
from PyQt6.QtCore import pyqtSignal as Signal

import os
import csv
import sys
import time
import json
import sympy
import shutil
import cv2 as cv
import numpy as np 
from cv2 import aruco
from scipy import optimize
from scipy.spatial.transform import Rotation as R


##------ Import from python files ------##
from Used_functions import *
from functions_screw import * 
from pose_generator import PoseGenerator

##------ QThread classes ------##
from Camera import Camera
from CRC import Check_robot_connection
from Robot_Coords import Get_robot_coordinates
from Camera_calibration_class import Camera_calibration_class
from Auto_image_taker import Auto_image_taker
from Pre_calculations import Pre_Calculation
from DH_opt_NM_method import NM_Optimalization
from DH_opt_Grad_method import Grad_Optimalization
from move_robot import Set_robot_coordinates
        
##------ Main window ------##            
class MainWindow(QtWidgets.QMainWindow):   
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
        ##------ Define variables used for config ------##
        self.Robot_IP_add = None
        self.Board_size_width = None
        self.Board_size_height = None
        self.Square_size_m = None
        self.Marker_size_m = None
        self.Camera_index_used = 0
        self.Board_dict_used = "4X4_50"
        self.time_check_start = None
        self.socket_works = False        
        
        ##------ Define variables, these used for the calibration -------##
        self.Number_of_images = 0
        self.current_robot_joint_parameters = None
        self.dictionary_aruco_calibration = None
        self.image_shown = 0 
        self.Detector_camera_calibration = None
        self.mtx = None
        self.dist = None
        self.Charuco_board= False
        
        uic.loadUi("app.ui", self)
        
        ##------ Define tabpages ------##
        self.Pages = self.findChild(QTabWidget, "Pages")
        self.Config_page = self.findChild(QWidget, "Config")
        self.Cam_calib_page = self.findChild(QWidget, "Camera_calibration")
        self.SVD_page = self.findChild(QWidget, "SVD")
        self.DH_page = self.findChild(QWidget, "DH")
        
        ##------ Disable tabs ------##
        self.Pages.setTabEnabled(1,False) 
        self.Pages.setTabEnabled(2,False)
        self.Pages.setTabEnabled(3,False)
        
        ##------ Check if the tab index is changed ------##
        self.Pages.currentChanged.connect(self.Pages_changed)

        ######################################################################
        # TAB 0 ui-ból beolvas Qwidgets
        ######################################################################                
        
        ##------ Define combo boxes ------##
        self.Dict_comb_box = self.findChild(QComboBox, "Dict_comb_box")
        self.Cam_index_comb_box = self.findChild(QComboBox, "Cam_index_comb_box")
        
        ##------ Define camera images as qlabel ------##
        self.Cam_image = self.findChild(QLabel, "Cam_image")
        
        ##------ Define lineedits ------##
        self.Board_height = self.findChild(QLineEdit, "Board_height")
        self.Board_width = self.findChild(QLineEdit, "Board_width")
        self.Robot_IP = self.findChild(QLineEdit, "Robot_IP")
        self.Marker_size = self.findChild(QLineEdit, "marker_size")
        self.Square_size = self.findChild(QLineEdit, "square_size")
        
        ##------ Define checkbox ------##
        self.Robot_conn = self.findChild(QCheckBox, "Robot_connection")
        self.Use_robot = self.findChild(QCheckBox, "Use_robot")
        
        ##------ Define buttons ------##
        self.Accept1 = self.findChild(QPushButton, "Accept1")
        self.Refresh_cam_index = self.findChild(QPushButton, "Refresh_cam_index")
        self.Save_config = self.findChild(QPushButton, "Save_config_button")
        self.Open_config = self.findChild(QPushButton, "Open_config_button")
        
        ######################################################################
        # TAB 1 ui-ból beolvas Qwidgets
        ######################################################################        

        ##------ Define buttons ------##
        self.Accept2 = self.findChild(QPushButton, "Accept_camera_calibration")
        self.Save_camera_calibration = self.findChild(QPushButton, "Save_camera_calibration")
        self.Open_camera_calibration = self.findChild(QPushButton, "Open_camera_calibration")
        self.Take_picture_cam_calib = self.findChild(QPushButton, "Take_calib_picture")
        self.Calculate_cam_calib_params = self.findChild(QPushButton, "Calculate_camera_parameters")
        self.Left = self.findChild(QPushButton, "Left_button")
        self.Right = self.findChild(QPushButton, "Right_button")
        self.Delete_calib_image = self.findChild(QPushButton, "Delet_calibration_image")
        self.Give_x_value_p = self.findChild(QPushButton, "Give_x_value_p")
        self.Give_x_value_m = self.findChild(QPushButton, "Give_x_value_m")
        self.Give_y_value_p = self.findChild(QPushButton, "Give_y_value_p")
        self.Give_y_value_m = self.findChild(QPushButton, "Give_y_value_m")
        self.Give_z_value_p = self.findChild(QPushButton, "Give_z_value_p")
        self.Give_z_value_m = self.findChild(QPushButton, "Give_z_value_m")
        self.Give_rx_value_p = self.findChild(QPushButton, "Give_rx_value_p")
        self.Give_rx_value_m = self.findChild(QPushButton, "Give_rx_value_m")
        self.Give_ry_value_p = self.findChild(QPushButton, "Give_ry_value_p")
        self.Give_ry_value_m = self.findChild(QPushButton, "Give_ry_value_m")
        self.Give_rz_value_p = self.findChild(QPushButton, "Give_rz_value_p")
        self.Give_rz_value_m = self.findChild(QPushButton, "Give_rz_value_m")

        ##------ Define camera images as a qLabel that taken for the calibration ------##
        self.Calibration_image = self.findChild(QLabel, "Calibratin_images")
        
        ##------ Define checkbox to show dertected markers ------##
        self.Show_marker_calib = self.findChild(QCheckBox, "Show_detected_markers")

        ##------ Define calibration progressbar ------##
        self.Calibration_prog_bar = self.findChild(QProgressBar, "Calibration_prog_bar")
        
        ##------ Define Lineedit for the TCP coordinates ------##
        self.line_edit_x_value = self.findChild(QLineEdit, "line_edit_x_value")
        self.line_edit_y_value = self.findChild(QLineEdit, "line_edit_y_value")
        self.line_edit_z_value = self.findChild(QLineEdit, "line_edit_z_value")
        self.line_edit_rx_value = self.findChild(QLineEdit, "line_edit_rx_value")
        self.line_edit_ry_value = self.findChild(QLineEdit, "line_edit_ry_value")
        self.line_edit_rz_value = self.findChild(QLineEdit, "line_edit_rz_value")       
        
        ##------ Define label for the distorsion parameters (QLabel) ------##
        self.F_x = self.findChild(QLabel, "F_x")
        self.F_y = self.findChild(QLabel, "F_y")
        self.u_v = self.findChild(QLabel, "u_v")
        self.w_v = self.findChild(QLabel, "w_v")
        self.k1 = self.findChild(QLabel, "k1")
        self.k2 = self.findChild(QLabel, "k2")
        self.k3 = self.findChild(QLabel, "k3")
        self.k4 = self.findChild(QLabel, "k4")
        self.k5 = self.findChild(QLabel, "k5")
        self.k6 = self.findChild(QLabel, "k6")
        self.p1 = self.findChild(QLabel, "p1")
        self.p2 = self.findChild(QLabel, "p2")
        self.s1 = self.findChild(QLabel, "s1")
        self.s2 = self.findChild(QLabel, "s2")
        self.s3 = self.findChild(QLabel, "s3")
        self.s4 = self.findChild(QLabel, "s4")
        
        ##------ Define spinboxes ------##
        self.Speher_radius_spinbox = self.findChild(QDoubleSpinBox, "Sphere_radius")
        self.Sphere_grid_width_spinbox = self.findChild(QSpinBox, "Sphere_grid_width")
        self.Sphere_grid_height_spinbox = self.findChild(QSpinBox, "Sphere_grid_height")
        self.Sphere_grid_hor_dist_spinbox = self.findChild(QDoubleSpinBox, "Sphere_grid_hor_dist")
        self.Sphere_grid_ver_dist_spinbox = self.findChild(QDoubleSpinBox, "Sphere_grid_ver_dist")
        
        ##------ Define spinboxes ------##
        self.Ori_grid_width_spinbox = self.findChild(QSpinBox, "Ori_grid_width")
        self.Ori_grid_height_spinbox = self.findChild(QSpinBox, "Ori_grid_height")
        self.Ori_grid_hor_dist_spinbox = self.findChild(QDoubleSpinBox, "Ori_grid_hor_dist")
        self.Ori_grid_ver_dist_spinbox = self.findChild(QDoubleSpinBox, "Ori_grid_ver_dist")
        
        ##------ Define spinboxes  ------##
        self.Layer_num_spinbox = self.findChild(QSpinBox, "Layer_num")
        self.Layer_distance_spinbox = self.findChild(QDoubleSpinBox, "Layer_dist")
        
        ##------ Define button  ------##
        self.Start_auto_button = self.findChild(QPushButton, "Start_auto")
        
        ##------ Define tabs  ------##
        self.Pic_taker_tabs = self.findChild(QTabWidget,"tabWidget")
        
        ######################################################################
        # TAB 2 ui-ból beolvas Qwidgets
        ######################################################################        
        
        self.Tabs2 = self.findChild(QTabWidget,"tabWidget_3")
        
        ##------ Define buttons ------##
        self.Accept3 = self.findChild(QPushButton, "Accept_pre")
        self.Save_pre = self.findChild(QPushButton, "Save_pre")
        self.Open_pre = self.findChild(QPushButton, "Open_pre")
        self.Take_pic_pre = self.findChild(QPushButton, "Take_pic_pre")
        self.Calculate_pre = self.findChild(QPushButton, "Calculate_pre")
        self.Left_pre = self.findChild(QPushButton, "Left_pre")
        self.Right_pre = self.findChild(QPushButton, "Right_pre")
        self.Delete_pre = self.findChild(QPushButton, "Delete_pre")
        self.Give_x_value_p_2 = self.findChild(QPushButton, "Give_x_value_p_2")
        self.Give_x_value_m_2 = self.findChild(QPushButton, "Give_x_value_m_2")
        self.Give_y_value_p_2 = self.findChild(QPushButton, "Give_y_value_p_2")
        self.Give_y_value_m_2 = self.findChild(QPushButton, "Give_y_value_m_2")
        self.Give_z_value_p_2 = self.findChild(QPushButton, "Give_z_value_p_2")
        self.Give_z_value_m_2 = self.findChild(QPushButton, "Give_z_value_m_2")
        self.Give_rx_value_p_2 = self.findChild(QPushButton, "Give_rx_value_p_2")
        self.Give_rx_value_m_2 = self.findChild(QPushButton, "Give_rx_value_m_2")
        self.Give_ry_value_p_2 = self.findChild(QPushButton, "Give_ry_value_p_2")
        self.Give_ry_value_m_2 = self.findChild(QPushButton, "Give_ry_value_m_2")
        self.Give_rz_value_p_2 = self.findChild(QPushButton, "Give_rz_value_p_2")
        self.Give_rz_value_m_2 = self.findChild(QPushButton, "Give_rz_value_m_2")
        
        ##------ Define Lineedit for the TCP coordinates ------##
        self.line_edit_x_value_2 = self.findChild(QLineEdit, "line_edit_x_value_2")
        self.line_edit_y_value_2 = self.findChild(QLineEdit, "line_edit_y_value_2")
        self.line_edit_z_value_2 = self.findChild(QLineEdit, "line_edit_z_value_2")
        self.line_edit_rx_value_2 = self.findChild(QLineEdit, "line_edit_rx_value_2")
        self.line_edit_ry_value_2 = self.findChild(QLineEdit, "line_edit_ry_value_2")
        self.line_edit_rz_value_2 = self.findChild(QLineEdit, "line_edit_rz_value_2")       
        
        ##------ Define checkbox to show dertected markers ------##
        self.Show_marker_pre = self.findChild(QCheckBox, "Show_marker_pre")

        ##------ Define calibration progressbar ------##
        self.Pre_prog_bar = self.findChild(QProgressBar, "Pre_prog_bar")
        
        self.Calibration_image_2 = self.findChild(QLabel, "Calibratin_images_2")
        
        ##------ Define Lineedit for the TCP coordinates ------##
        self.ig_x = self.findChild(QLineEdit, "ig_x")
        self.ig_y = self.findChild(QLineEdit, "ig_y")
        self.ig_z = self.findChild(QLineEdit, "ig_z")
        self.ig_rx = self.findChild(QLineEdit, "ig_rx")
        self.ig_ry = self.findChild(QLineEdit, "ig_ry")
        self.ig_rz = self.findChild(QLineEdit, "ig_rz")       
        
        ##------ Define label for the distorsion parameters (QLabel) ------##
        self.T_bb_11 = self.findChild(QLabel, "T_bb_11")
        self.T_bb_12 = self.findChild(QLabel, "T_bb_12")
        self.T_bb_13 = self.findChild(QLabel, "T_bb_13")
        self.T_bb_14 = self.findChild(QLabel, "T_bb_14")
        self.T_bb_21 = self.findChild(QLabel, "T_bb_21")
        self.T_bb_22 = self.findChild(QLabel, "T_bb_22")
        self.T_bb_23 = self.findChild(QLabel, "T_bb_23")
        self.T_bb_24 = self.findChild(QLabel, "T_bb_24")
        self.T_bb_31 = self.findChild(QLabel, "T_bb_31")
        self.T_bb_32 = self.findChild(QLabel, "T_bb_32")
        self.T_bb_33 = self.findChild(QLabel, "T_bb_33")
        self.T_bb_34 = self.findChild(QLabel, "T_bb_34")
        
        self.T_tc_11 = self.findChild(QLabel, "T_tc_11")
        self.T_tc_12 = self.findChild(QLabel, "T_tc_12")
        self.T_tc_13 = self.findChild(QLabel, "T_tc_13")
        self.T_tc_14 = self.findChild(QLabel, "T_tc_14")
        self.T_tc_21 = self.findChild(QLabel, "T_tc_21")
        self.T_tc_22 = self.findChild(QLabel, "T_tc_22")
        self.T_tc_23 = self.findChild(QLabel, "T_tc_23")
        self.T_tc_24 = self.findChild(QLabel, "T_tc_24")
        self.T_tc_31 = self.findChild(QLabel, "T_tc_31")
        self.T_tc_32 = self.findChild(QLabel, "T_tc_32")
        self.T_tc_33 = self.findChild(QLabel, "T_tc_33")
        self.T_tc_34 = self.findChild(QLabel, "T_tc_34")
        
        ##------ Define spinboxes ------##
        self.Speher_radius_spinbox_2 = self.findChild(QDoubleSpinBox, "Sphere_radius_2")
        self.Sphere_grid_width_spinbox_2 = self.findChild(QSpinBox, "Sphere_grid_width_2")
        self.Sphere_grid_height_spinbox_2 = self.findChild(QSpinBox, "Sphere_grid_height_2")
        self.Sphere_grid_hor_dist_spinbox_2 = self.findChild(QDoubleSpinBox, "Sphere_grid_hor_dist_2")
        self.Sphere_grid_ver_dist_spinbox_2 = self.findChild(QDoubleSpinBox, "Sphere_grid_ver_dist_2")
        
        ##------ Define spinboxes ------##
        self.Ori_grid_width_spinbox_2 = self.findChild(QSpinBox, "Ori_grid_width_2")
        self.Ori_grid_height_spinbox_2 = self.findChild(QSpinBox, "Ori_grid_height_2")
        self.Ori_grid_hor_dist_spinbox_2 = self.findChild(QDoubleSpinBox, "Ori_grid_hor_dist_2")
        self.Ori_grid_ver_dist_spinbox_2 = self.findChild(QDoubleSpinBox, "Ori_grid_ver_dist_2")
        
        ##------ Define spinboxes  ------##
        self.Layer_num_spinbox_2 = self.findChild(QSpinBox, "Layer_num_2")
        self.Layer_distance_spinbox_2 = self.findChild(QDoubleSpinBox, "Layer_dist_2")
                
        ##------ Define button  ------##
        self.Start_auto_button_2 = self.findChild(QPushButton, "Start_auto_2")
                
        ######################################################################
        # TAB 3 ui-ból beolvas Qwidgets
        ###################################################################### 
        
        self.Tabs3 = self.findChild(QTabWidget,"tabWidget_5")
        
        ##------ Define buttons ------##
        self.Accept_end = self.findChild(QPushButton, "Accept_end")
        self.Save_end = self.findChild(QPushButton, "Save_end")
        self.Open_end = self.findChild(QPushButton, "Open_end")
        self.Take_pic_end = self.findChild(QPushButton, "Take_pic_end")
        self.Calculate_end = self.findChild(QPushButton, "Calculate_end")
        self.Left_end = self.findChild(QPushButton, "Left_end")
        self.Right_end = self.findChild(QPushButton, "Right_end")
        self.Delete_end = self.findChild(QPushButton, "Delete_end")
        self.Give_x_value_p_3 = self.findChild(QPushButton, "Give_x_value_p_3")
        self.Give_x_value_m_3 = self.findChild(QPushButton, "Give_x_value_m_3")
        self.Give_y_value_p_3 = self.findChild(QPushButton, "Give_y_value_p_3")
        self.Give_y_value_m_3 = self.findChild(QPushButton, "Give_y_value_m_3")
        self.Give_z_value_p_3 = self.findChild(QPushButton, "Give_z_value_p_3")
        self.Give_z_value_m_3 = self.findChild(QPushButton, "Give_z_value_m_3")
        self.Give_rx_value_p_3 = self.findChild(QPushButton, "Give_rx_value_p_3")
        self.Give_rx_value_m_3 = self.findChild(QPushButton, "Give_rx_value_m_3")
        self.Give_ry_value_p_3 = self.findChild(QPushButton, "Give_ry_value_p_3")
        self.Give_ry_value_m_3 = self.findChild(QPushButton, "Give_ry_value_m_3")
        self.Give_rz_value_p_3 = self.findChild(QPushButton, "Give_rz_value_p_3")
        self.Give_rz_value_m_3 = self.findChild(QPushButton, "Give_rz_value_m_3")
        
        ##------ Define Lineedit for the TCP coordinates ------##
        self.line_edit_x_value_3 = self.findChild(QLineEdit, "line_edit_x_value_3")
        self.line_edit_y_value_3 = self.findChild(QLineEdit, "line_edit_y_value_3")
        self.line_edit_z_value_3 = self.findChild(QLineEdit, "line_edit_z_value_3")
        self.line_edit_rx_value_3 = self.findChild(QLineEdit, "line_edit_rx_value_3")
        self.line_edit_ry_value_3 = self.findChild(QLineEdit, "line_edit_ry_value_3")
        self.line_edit_rz_value_3 = self.findChild(QLineEdit, "line_edit_rz_value_3")       
        
        
        ##------ Define checkbox to show dertected markers ------##
        self.Show_marker_robcalib = self.findChild(QCheckBox, "Show_marker_end")     
        
        self.Calibration_image_3 = self.findChild(QLabel, "Calibratin_images_3")         
        
        ##------ Define spinboxes ------##
        self.Speher_radius_spinbox_3 = self.findChild(QDoubleSpinBox, "Sphere_radius_3")
        self.Sphere_grid_width_spinbox_3 = self.findChild(QSpinBox, "Sphere_grid_width_3")
        self.Sphere_grid_height_spinbox_3 = self.findChild(QSpinBox, "Sphere_grid_height_3")
        self.Sphere_grid_hor_dist_spinbox_3 = self.findChild(QDoubleSpinBox, "Sphere_grid_hor_dist_3")
        self.Sphere_grid_ver_dist_spinbox_3 = self.findChild(QDoubleSpinBox, "Sphere_grid_ver_dist_3")
        
        ##------ Define spinboxes ------##
        self.Ori_grid_width_spinbox_3 = self.findChild(QSpinBox, "Ori_grid_width_3")
        self.Ori_grid_height_spinbox_3 = self.findChild(QSpinBox, "Ori_grid_height_3")
        self.Ori_grid_hor_dist_spinbox_3 = self.findChild(QDoubleSpinBox, "Ori_grid_hor_dist_3")
        self.Ori_grid_ver_dist_spinbox_3 = self.findChild(QDoubleSpinBox, "Ori_grid_ver_dist_3")
        
        ##------ Define spinboxes  ------##
        self.Layer_num_spinbox_3 = self.findChild(QSpinBox, "Layer_num_3")
        self.Layer_distance_spinbox_3 = self.findChild(QDoubleSpinBox, "Layer_dist_3")
                
        ##------ Define button  ------##
        self.Start_auto_button_3 = self.findChild(QPushButton, "Start_auto_3")        
        
        ##------ Define DH table ------##
        self.DH_table = self.findChild(QTableWidget,"DH_parameters_table")
        
        ##------ Define Progressbar ------##
        self.Cal_progbar = self.findChild(QProgressBar, "CAL_DH_PB")
        
        ######################################################################
        # TAB 0 action-ök, és
        ###################################################################### 
        
        ##------ Define camera indexes combo box, to get all the camera indexes (if there is less than 11 cameras connected to the system) ------##
        self.Cam_indexes = np.array([]) 
        for i in range(0,10):
            cap = cv.VideoCapture(i)
            if cap.read()[0]:
                self.Cam_indexes = np.append(self.Cam_indexes,i)
            cap.release()       
        self.Cam_indexes = self.Cam_indexes.astype("int64").astype("str") 
        self.Cam_index_comb_box.addItems(self.Cam_indexes) 
        
        ##------ Define camera to generate camera image (aruco board and aruco detector is not used here) ------##
        self.Camera = Camera()
        self.Camera.Camera_index = self.Camera_index_used
        self.Camera.checked = self.Show_marker_calib.isChecked()
        self.Camera.detector = self.Detector_camera_calibration
        self.Camera.board = self.Charuco_board
        self.Camera.mtx = self.mtx
        self.Camera.dist = self.dist
        self.Camera.start()
        self.Camera.ImageUpdate.connect(self.ImageUpdateSlot)
        
        ##------ Define lineedit changed text to save the new values to variables ------##
        self.Board_height.textEdited.connect(self.Board_height_edited)
        self.Board_width.textEdited.connect(self.Board_width_edited)
        self.Robot_IP.textEdited.connect(self.Robot_IP_edited)
        self.Marker_size.textEdited.connect(self.Marker_size_edited)
        self.Square_size.textEdited.connect(self.Square_size_edited)
                
        ##------ Define combo box changed text to save the new values to variables ------##
        self.Dict_comb_box.currentTextChanged.connect(self.dict_comb_text_changed)
        self.Cam_index_comb_box.currentTextChanged.connect(self.cam_index_changed)
        
        ##------ Define buttons to do smthg. ------##
        self.Refresh_cam_index.clicked.connect(self.Refresh_camera_indexes)
        self.Save_config.clicked.connect(self.Save_config_to_json)
        self.Open_config.clicked.connect(self.Open_config_from_json)
        self.Accept1.clicked.connect(self.To_page1)
        
        ##------ Define check robot connection class to check if the robot is connected to socket ------##
        self.Chck_rob_conn = Check_robot_connection()
        self.Chck_rob_conn.Robot_is_connceted.connect(self.change_robot_conn_tick)
        
        ######################################################################
        # TAB 1 actions
        ###################################################################### 
        
        ##------ Define button pressed to move the robot ------##
        self.Give_x_value_p.pressed.connect(self.Set_robot_x_p)
        self.Give_x_value_m.pressed.connect(self.Set_robot_x_m)
        self.Give_y_value_p.pressed.connect(self.Set_robot_y_p)
        self.Give_y_value_m.pressed.connect(self.Set_robot_y_m)
        self.Give_z_value_p.pressed.connect(self.Set_robot_z_p)
        self.Give_z_value_m.pressed.connect(self.Set_robot_z_m)
        self.Give_rx_value_p.pressed.connect(self.Set_robot_rx_p)
        self.Give_rx_value_m.pressed.connect(self.Set_robot_rx_m)
        self.Give_ry_value_p.pressed.connect(self.Set_robot_ry_p)
        self.Give_ry_value_m.pressed.connect(self.Set_robot_ry_m)
        self.Give_rz_value_p.pressed.connect(self.Set_robot_rz_p)
        self.Give_rz_value_m.pressed.connect(self.Set_robot_rz_m)
        
        ##------ Define buttons release actions to stop the robot movement ------##
        self.Give_x_value_p.released.connect(self.Stop_robot_movement)
        self.Give_x_value_m.released.connect(self.Stop_robot_movement)
        self.Give_y_value_p.released.connect(self.Stop_robot_movement)
        self.Give_y_value_m.released.connect(self.Stop_robot_movement)
        self.Give_z_value_p.released.connect(self.Stop_robot_movement)
        self.Give_z_value_m.released.connect(self.Stop_robot_movement)
        self.Give_rx_value_p.released.connect(self.Stop_robot_movement)
        self.Give_rx_value_m.released.connect(self.Stop_robot_movement)
        self.Give_ry_value_p.released.connect(self.Stop_robot_movement)
        self.Give_ry_value_m.released.connect(self.Stop_robot_movement)
        self.Give_rz_value_p.released.connect(self.Stop_robot_movement)
        self.Give_rz_value_m.released.connect(self.Stop_robot_movement)
        
        ##------ Define buttons to do smthg. ------##
        self.Take_picture_cam_calib.clicked.connect(self.Take_picture_camera_calibration)
        self.Calculate_cam_calib_params.clicked.connect(self.Calculate_camera_calibration)
        self.Right.clicked.connect(self.Right_image)
        self.Left.clicked.connect(self.Left_image)
        self.Delete_calib_image.clicked.connect(self.Delete_image)
        self.Open_camera_calibration.clicked.connect(self.Open_camera_calib)
        self.Save_camera_calibration.clicked.connect(self.Save_camera_calib)
        self.Accept2.clicked.connect(self.To_page2)
        self.Start_auto_button.clicked.connect(self.Auto_calibration)
            
        ##------ Define checkbox to show the markers that are detected by the camera ------##
        self.Show_marker_calib.stateChanged.connect(self.Show_markers_on_image)
                
        ##------ Define the camera calibration class ------##
        self.CCC = Camera_calibration_class()        
        self.CCC.ProgbarValue.connect(self.update_progress)
        self.CCC.Matrices.connect(self.complete)      
        
        ######################################################################
        # TAB 2 actions
        ###################################################################### 
        
        ##------ Define button pressed to move the robot ------##
        self.Give_x_value_p_2.pressed.connect(self.Set_robot_x_p)
        self.Give_x_value_m_2.pressed.connect(self.Set_robot_x_m)
        self.Give_y_value_p_2.pressed.connect(self.Set_robot_y_p)
        self.Give_y_value_m_2.pressed.connect(self.Set_robot_y_m)
        self.Give_z_value_p_2.pressed.connect(self.Set_robot_z_p)
        self.Give_z_value_m_2.pressed.connect(self.Set_robot_z_m)
        self.Give_rx_value_p_2.pressed.connect(self.Set_robot_rx_p)
        self.Give_rx_value_m_2.pressed.connect(self.Set_robot_rx_m)
        self.Give_ry_value_p_2.pressed.connect(self.Set_robot_ry_p)
        self.Give_ry_value_m_2.pressed.connect(self.Set_robot_ry_m)
        self.Give_rz_value_p_2.pressed.connect(self.Set_robot_rz_p)
        self.Give_rz_value_m_2.pressed.connect(self.Set_robot_rz_m)
        
        ##------ Define buttons release actions to stop the robot movement ------##
        self.Give_x_value_p_2.released.connect(self.Stop_robot_movement)
        self.Give_x_value_m_2.released.connect(self.Stop_robot_movement)
        self.Give_y_value_p_2.released.connect(self.Stop_robot_movement)
        self.Give_y_value_m_2.released.connect(self.Stop_robot_movement)
        self.Give_z_value_p_2.released.connect(self.Stop_robot_movement)
        self.Give_z_value_m_2.released.connect(self.Stop_robot_movement)
        self.Give_rx_value_p_2.released.connect(self.Stop_robot_movement)
        self.Give_rx_value_m_2.released.connect(self.Stop_robot_movement)
        self.Give_ry_value_p_2.released.connect(self.Stop_robot_movement)
        self.Give_ry_value_m_2.released.connect(self.Stop_robot_movement)
        self.Give_rz_value_p_2.released.connect(self.Stop_robot_movement)
        self.Give_rz_value_m_2.released.connect(self.Stop_robot_movement)
        
        ##------ Define buttons to do smthg. ------##
        self.Take_pic_pre.clicked.connect(self.Take_picture_camera_calibration)
        self.Calculate_pre.clicked.connect(self.Calculate_preveous)
        self.Right_pre.clicked.connect(self.Right_image)
        self.Left_pre.clicked.connect(self.Left_image)
        self.Delete_pre.clicked.connect(self.Delete_image)
        self.Open_pre.clicked.connect(self.Open_pre_func)
        self.Save_pre.clicked.connect(self.Save_pre_func)
        self.Accept3.clicked.connect(self.To_page3)
        self.Start_auto_button_2.clicked.connect(self.Auto_calibration)
            
        ##------ Define checkbox to show the markers that are detected by the camera ------##
        self.Show_marker_pre.stateChanged.connect(self.Show_markers_on_image)
        
        ######################################################################
        # TAB 3 actions
        ###################################################################### 
        ##------ Define button pressed to move the robot ------##
        self.Give_x_value_p_3.pressed.connect(self.Set_robot_x_p)
        self.Give_x_value_m_3.pressed.connect(self.Set_robot_x_m)
        self.Give_y_value_p_3.pressed.connect(self.Set_robot_y_p)
        self.Give_y_value_m_3.pressed.connect(self.Set_robot_y_m)
        self.Give_z_value_p_3.pressed.connect(self.Set_robot_z_p)
        self.Give_z_value_m_3.pressed.connect(self.Set_robot_z_m)
        self.Give_rx_value_p_3.pressed.connect(self.Set_robot_rx_p)
        self.Give_rx_value_m_3.pressed.connect(self.Set_robot_rx_m)
        self.Give_ry_value_p_3.pressed.connect(self.Set_robot_ry_p)
        self.Give_ry_value_m_3.pressed.connect(self.Set_robot_ry_m)
        self.Give_rz_value_p_3.pressed.connect(self.Set_robot_rz_p)
        self.Give_rz_value_m_3.pressed.connect(self.Set_robot_rz_m)
        
        ##------ Define buttons release actions to stop the robot movement ------##
        self.Give_x_value_p_3.released.connect(self.Stop_robot_movement)
        self.Give_x_value_m_3.released.connect(self.Stop_robot_movement)
        self.Give_y_value_p_3.released.connect(self.Stop_robot_movement)
        self.Give_y_value_m_3.released.connect(self.Stop_robot_movement)
        self.Give_z_value_p_3.released.connect(self.Stop_robot_movement)
        self.Give_z_value_m_3.released.connect(self.Stop_robot_movement)
        self.Give_rx_value_p_3.released.connect(self.Stop_robot_movement)
        self.Give_rx_value_m_3.released.connect(self.Stop_robot_movement)
        self.Give_ry_value_p_3.released.connect(self.Stop_robot_movement)
        self.Give_ry_value_m_3.released.connect(self.Stop_robot_movement)
        self.Give_rz_value_p_3.released.connect(self.Stop_robot_movement)
        self.Give_rz_value_m_3.released.connect(self.Stop_robot_movement)
        
        self.Take_pic_end.clicked.connect(self.Take_picture_camera_calibration)
        self.Right_end.clicked.connect(self.Right_image)
        self.Left_end.clicked.connect(self.Left_image)
        self.Delete_end.clicked.connect(self.Delete_image)
        self.Open_end.clicked.connect(self.Open_end_func)
        self.Save_end.clicked.connect(self.Save_end_func)
        self.Accept_end.clicked.connect(QApplication.instance().quit)
        
        self.Show_marker_robcalib.stateChanged.connect(self.Show_markers_on_image)
        
        self.Calculate_end.clicked.connect(self.DH_Cal)
        
    ######################################################################
    # General functions
    ######################################################################            
        
    ##------ Set pixmap, so change the camera image ------##            
    def ImageUpdateSlot(self, image):
        self.Cam_image.setPixmap(QPixmap.fromImage(image))  
        if self.Pages.currentIndex() == 0:
            self.check_values_for_config()

    ##------ Disable tabs if index is lower ------##
    def Pages_changed(self, s):
        for i in range(s+1,self.Pages.count()):
            self.Pages.setTabEnabled(i,False)
            
    ######################################################################
    # TAB 0 functions
    ######################################################################             
    
    ##------ Set robot connection ip and start checking the connection ------##
    def Check_robot_connection(self):
        self.Chck_rob_conn.ip = self.Robot_IP_add
        self.Chck_rob_conn.start()   
    
    ##------ Change the "Robot connected to socket" tick on the UI ------##
    def change_robot_conn_tick(self,robot_is_connected):
        if robot_is_connected:
            self.Robot_conn.setDisabled(False) 
            self.Robot_conn.setCheckState(Qt.CheckState.Checked)
            self.Robot_conn.setDisabled(True)
        else:
            self.Robot_conn.setDisabled(False) 
            self.Robot_conn.setCheckState(Qt.CheckState.Unchecked )
            self.Robot_conn.setDisabled(True) 

    ##------ Check if all of the values are not None, so the config values are valid ------##     
    def check_values_for_config(self):
        if  self.Board_size_width is not None and self.Board_size_height is not None and self.Square_size_m is not None and self.Marker_size_m is not None and ((self.Robot_conn.checkState() == Qt.CheckState.Checked and self.Robot_IP_add is not None) or self.Use_robot.checkState() == Qt.CheckState.Unchecked):
            self.Accept1.setDisabled(False) 
            self.Save_config.setDisabled(False)                     
        else:
            self.Accept1.setDisabled(True)
            self.Save_config.setDisabled(True)
    
    ##------ Open a json config file, load saved config parameters ------##
    def Open_config_from_json(self):
        self.file_dialog_open = QFileDialog()
        self.fname_open = self.file_dialog_open.getOpenFileName(self,"Open saved config","","JSON Files (*.json)")
        self.file_dialog_open.setFixedSize(600,400)
        try:
            if self.fname_open[0] != "":
                self.saved_file_name = self.fname_open[0]
                open_json = open(self.saved_file_name)
                data = json.load(open_json)             
                self.Robot_IP_add = data["Robot_IP"]
                self.Board_size_width = data["Board_size_width"]
                self.Board_size_height = data["Board_size_height"]
                self.Square_size_m = data["Square_size"]
                self.Marker_size_m = data["Marker_size"]
                self.Camera_index_used = data["Camera_index"]
                self.Board_dict_used = data["Board_dictionary"]       
                
                self.Board_height.setText(str(self.Board_size_height))
                self.Board_width.setText(str(self.Board_size_width))
                self.Robot_IP.setText(self.Robot_IP_add)
                self.Marker_size.setText(str(self.Marker_size_m))
                self.Square_size.setText(str(self.Square_size_m))
                self.Dict_comb_box.setCurrentIndex(self.Dict_comb_box.findText(self.Board_dict_used))
                self.Cam_index_comb_box.setCurrentIndex(self.Cam_index_comb_box.findText(str(self.Camera_index_used)))
                self.Check_robot_connection()
        except Exception as e:
            print(e)
            
    ##------ Save config file to .json ------##
    def Save_config_to_json(self):      
        if self.Robot_IP_add is not None and self.Board_size_width is not None and self.Board_size_height is not None and self.Square_size_m is not None and self.Marker_size_m is not None: 
            self.file_dialog_save = QFileDialog()
            self.fname_saved = self.file_dialog_save.getSaveFileName(self,"Save config","config.json","JSON Files (*.json)")
            self.file_dialog_save.setFixedSize(600,400)
            dictionary = {
            "Robot_IP": self.Robot_IP_add,
            "Camera_index": self.Camera_index_used,
            "Board_dictionary": self.Board_dict_used,
            "Board_size_width": self.Board_size_width,
            "Board_size_height": self.Board_size_height,
            "Square_size": self.Square_size_m,
            "Marker_size": self.Marker_size_m
            }
            path = self.fname_saved[0]
            if path != "":
                json_object = json.dumps(dictionary, indent=4)
                with open(path, "w") as outfile:
                    outfile.write(json_object)           
            
    ##------ Refresh camera indexes ------##
    def Refresh_camera_indexes(self, s): 
        self.Cam_index_comb_box.clear()
        self.Cam_indexes = np.array([])
        for i in range(0,10):
            cap = cv.VideoCapture(i)
            if cap.read()[0] or i == self.Camera_index_used:
                self.Cam_indexes = np.append(self.Cam_indexes,i)
            cap.release()       
        self.Cam_indexes = self.Cam_indexes.astype("int64").astype("str") 
        self.Cam_index_comb_box.addItems(self.Cam_indexes)
        self.Cam_index_comb_box.setCurrentIndex(self.Cam_index_comb_box.findText(str(self.Camera_index_used)))
        
    ##------ Change current tab to tab2 ------##            
    def To_page1(self):
        self.Pages.setTabEnabled(1,True)
        self.Pages.setCurrentWidget(self.Pages.findChild(QWidget, "Camera_calibration"))
        
        ##------ Load the correct dictionary to the selected one ------##
        if self.Board_dict_used == "4x4_50":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50) 
            
        elif self.Board_dict_used == "4x4_100":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)
            
        elif self.Board_dict_used == "4x4_250":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
            
        elif self.Board_dict_used == "4x4_1000":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
            
        elif self.Board_dict_used == "5x5_50":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
            
        elif self.Board_dict_used == "5x5_100":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
            
        elif self.Board_dict_used == "5x5_250":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_250)
            
        elif self.Board_dict_used == "5x5_1000":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)
            
        elif self.Board_dict_used == "6x6_50":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_50)
            
        elif self.Board_dict_used == "6x6_100":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_100)
            
        elif self.Board_dict_used == "6x6_250":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
            
        elif self.Board_dict_used == "6x6_1000":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_1000)
            
        elif self.Board_dict_used == "7x7_50":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_50)
            
        elif self.Board_dict_used == "7x7_100":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_100)
            
        elif self.Board_dict_used == "7x7_250":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_250)
            
        elif self.Board_dict_used == "7x7_1000":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_7X7_1000)
            
        elif self.Board_dict_used == "ARUCO_ORIGINAL":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
            
        elif self.Board_dict_used == "APRILTAG_16h5":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_16h5)
            
        elif self.Board_dict_used == "APRILTAG_25h9":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_25h9)
            
        elif self.Board_dict_used == "APRILTAG_36h10":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h10)
            
        elif self.Board_dict_used == "APRILTAG_36h11":
            self.dictionary_aruco_calibration = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_APRILTAG_36h11)
        
        ##------ Define the aruco board and the aruco detector ------##           
        self.Charuco_board = cv.aruco.CharucoBoard((self.Board_size_width,self.Board_size_height), self.Square_size_m, self.Marker_size_m, self.dictionary_aruco_calibration)	
        self.Detector_camera_calibration = cv.aruco.ArucoDetector(self.dictionary_aruco_calibration,  cv.aruco.DetectorParameters())
        
        ##------ Redefine the camera, so it can detect aruco markers ------##
        self.Camera.stop()
        self.Camera.Camera_index = self.Camera_index_used
        self.Camera.checked = self.Show_marker_calib.isChecked()
        self.Camera.detector = self.Detector_camera_calibration
        self.Camera.board = self.Charuco_board
        self.Camera.mtx = self.mtx
        self.Camera.dist = self.dist
        self.Camera.start()
        self.Camera.ImageUpdate.connect(self.ImageUpdateSlot)       
        
        ##------ Create a Camera calibration class, to run the camera calibration via a qthread ------##
        self.CCC.detector = self.Detector_camera_calibration
        self.CCC.board = self.Charuco_board
        self.CCC.images = self.Number_of_images
        
        ##------ Stop robot connection checking ------##
        self.Chck_rob_conn.stop()
                
        if self.Use_robot.checkState() == Qt.CheckState.Checked:
            self.Pic_taker_tabs.setTabEnabled(0,True)
            self.Pic_taker_tabs.setTabEnabled(1,True)
            
            self.Tabs2.setTabEnabled(1,True)
            self.Tabs2.setTabEnabled(2,True)
            
            self.Tabs3.setTabEnabled(0,True)
            self.Tabs3.setTabEnabled(1,True)
            
            self.Get_robot_coordinates = Get_robot_coordinates()
            
            self.Get_robot_coordinates.robot_ip = self.Robot_IP_add
            self.Get_robot_coordinates.start()
            
            self.Get_robot_coordinates.Robot_coordinates.connect(self.print_robot_coords) 
            
            self.Set_robot_coordinates_class = Set_robot_coordinates(self.Robot_IP_add)
            
        else:
            self.Pic_taker_tabs.setTabEnabled(0,False) 
            self.Pic_taker_tabs.setTabEnabled(1,False)
            self.Tabs2.setTabEnabled(1,False)
            self.Tabs2.setTabEnabled(2,False)
            self.Tabs3.setTabEnabled(0,False)
            self.Tabs3.setTabEnabled(1,False)
            
        ##------ Define UR16e SerialManipulator ------##
        ##------ ______________________________ ------##
        L1 = RLink(np.array([0.1807,0,0,np.pi/2]))
        L2 = RLink(np.array([0,0,-0.4784,0]))
        L3 = RLink(np.array([0,0,-0.36,0]))
        L4 = RLink(np.array([0.17415,0,0,np.pi/2]))
        L5 = RLink(np.array([0.11985,0,0,-np.pi/2]))
        L6 = RLink(np.array([0.11655,0,0,0]))
        self.Robot = SerialManipulator([L1,L2,L3,L4,L5,L6])
        
    ##------ Load UI values to variables that are used int the main code ------##      
    def Board_height_edited(self, s):
        try:
            if int(s)>0:
                self.Board_size_height = int(s)
            else: self.Board_size_height = None
        except:
            self.Board_size_height = None
        self.check_values_for_config()
            
    def Board_width_edited(self, s):
        try:
            if int(s)>0:
                self.Board_size_width = int(s)
            else: 
                self.Board_size_width = None
        except:
            self.Board_size_width = None
        self.check_values_for_config()
        
    def Robot_IP_edited(self, s):
        if s !="":
            self.Robot_IP_add = s
            self.Check_robot_connection()
        else:
            self.Robot_IP_add = None
        self.check_values_for_config()
        
    def Marker_size_edited(self, s):
        try:
            if float(s)>0:
                self.Marker_size_m = float(s)
            else:
                self.Marker_size_m = None
        except:
            self.Marker_size_m = None
        self.check_values_for_config()
        
    def Square_size_edited(self, s):
        try:
            if float(s)>0:
                self.Square_size_m = float(s)
            else:
                self.Square_size_m = None
        except:
            self.Square_size_m = None
        self.check_values_for_config()
    
    def dict_comb_text_changed(self, s):
        self.Board_dict_used = s
    
    def cam_index_changed(self, s):
        try:
            self.Camera_index_used = int(s)
            self.Camera.stop()
            self.Camera.Camera_index = self.Camera_index_used
            self.Camera.checked = self.Show_marker_calib.isChecked()
            self.Camera.detector = self.Detector_camera_calibration
            self.Camera.board = self.Charuco_board
            self.Camera.mtx = self.mtx
            self.Camera.dist = self.dist
            self.Camera.start()
            self.Camera.ImageUpdate.connect(self.ImageUpdateSlot)
        except Exception as e:
            print(e)

    ######################################################################
    # TAB 1 functions
    ###################################################################### 
    
    ##------ Start camera calibration via qthread ------##
    def Calculate_camera_calibration(self):
        self.Take_picture_cam_calib.setDisabled(True)
        self.CCC.images = self.Number_of_images
        self.CCC.start()
    
    ##------ Print robot coordinates to the UI ------##    
    def print_robot_coords(self, values):                         
        Joint_values = np.reshape(values[0,:],(1,6))
        
        #Joint_values = Joint_values + np.array([np.pi,0,0,0,0,0])
        
        T_tcp = self.Robot.T(np.transpose(Joint_values))
        
        #T_tcp = Forward_kin_screw(Screw_parameters,Home,Joint_values)
                
        eulers = R.from_matrix(T_tcp[0:3,0:3,0]).as_euler("xyz",degrees = False)    
                
        self.line_edit_x_value.setText(str(T_tcp[0,3,0])[0:8])        
        self.line_edit_y_value.setText(str(T_tcp[1,3,0])[0:8])
        self.line_edit_z_value.setText(str(T_tcp[2,3,0])[0:8])
        
        self.line_edit_rx_value.setText(str(eulers[0])[0:8])
        self.line_edit_ry_value.setText(str(eulers[1])[0:8])
        self.line_edit_rz_value.setText(str(eulers[2])[0:8])  
        
        self.line_edit_x_value_2.setText(str(T_tcp[0,3,0])[0:8])        
        self.line_edit_y_value_2.setText(str(T_tcp[1,3,0])[0:8])
        self.line_edit_z_value_2.setText(str(T_tcp[2,3,0])[0:8])
        
        self.line_edit_rx_value_2.setText(str(eulers[0])[0:8])
        self.line_edit_ry_value_2.setText(str(eulers[1])[0:8])
        self.line_edit_rz_value_2.setText(str(eulers[2])[0:8]) 
        
        self.line_edit_x_value_3.setText(str(T_tcp[0,3,0])[0:8])        
        self.line_edit_y_value_3.setText(str(T_tcp[1,3,0])[0:8])
        self.line_edit_z_value_3.setText(str(T_tcp[2,3,0])[0:8])
        
        self.line_edit_rx_value_3.setText(str(eulers[0])[0:8])
        self.line_edit_ry_value_3.setText(str(eulers[1])[0:8])
        self.line_edit_rz_value_3.setText(str(eulers[2])[0:8])
        
        self.current_robot_joint_parameters = Joint_values
    
    ##------ Set robot velocities to move the robot as long as the button is pressed ------##
    def Set_robot_x_p(self):
        self.Set_robot_coordinates_class.Move_robot_x_p()
        
    def Set_robot_x_m(self):
        self.Set_robot_coordinates_class.Move_robot_x_m()
        
    def Set_robot_y_p(self):
        self.Set_robot_coordinates_class.Move_robot_y_p()
        
    def Set_robot_y_m(self):
        self.Set_robot_coordinates_class.Move_robot_y_m()
        
    def Set_robot_z_p(self):
        self.Set_robot_coordinates_class.Move_robot_z_p()
        
    def Set_robot_z_m(self):
        self.Set_robot_coordinates_class.Move_robot_z_m()
        
    def Set_robot_rx_p(self):
        self.Set_robot_coordinates_class.Move_robot_rx_p()
        
    def Set_robot_rx_m(self):
        self.Set_robot_coordinates_class.Move_robot_rx_m()
        
    def Set_robot_ry_p(self):
        self.Set_robot_coordinates_class.Move_robot_ry_p()
        
    def Set_robot_ry_m(self):
        self.Set_robot_coordinates_class.Move_robot_ry_m()
    
    def Set_robot_rz_p(self):
        self.Set_robot_coordinates_class.Move_robot_rz_p()
        
    def Set_robot_rz_m(self):
        self.Set_robot_coordinates_class.Move_robot_rz_m()
    
    def Stop_robot_movement(self):
        self.Set_robot_coordinates_class.Stop_robot()
    
    ##------ Update the calibration progressbar ------##
    def update_progress(self, v):
        self.Calibration_prog_bar.setValue(v)
    
    ##------ Load the results of the calibration to the UI ------##
    def complete(self, mtx_dist_list):
        self.Calibration_prog_bar.setValue(100)
        self.mtx = mtx_dist_list[0]
        self.dist = mtx_dist_list[1]
        np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'rvecs.csv', mtx_dist_list[2], delimiter=',')
        np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'tvecs.csv', mtx_dist_list[3], delimiter=',')
        
        self.Take_picture_cam_calib.setDisabled(False)
        self.F_x.setText(str(self.mtx[0,0])[0:10])
        self.F_y.setText(str(self.mtx[1,1])[0:10])
        self.u_v.setText(str(self.mtx[0,2])[0:10])
        self.w_v.setText(str(self.mtx[1,2])[0:10])
        
        self.k1.setText("k1 = " + str(self.dist[0,0])[0:10])
        self.k2.setText("k2 = " + str(self.dist[1,0])[0:10])
        self.k3.setText("k3 = " + str(self.dist[4,0])[0:10])
        self.k4.setText("k4 = " + str(self.dist[5,0])[0:10])
        self.k5.setText("k5 = " + str(self.dist[6,0])[0:10])
        self.k6.setText("k6 = " + str(self.dist[7,0])[0:10])
        self.p1.setText("p1 = " + str(self.dist[2,0])[0:10])
        self.p2.setText("p2 = " + str(self.dist[3,0])[0:10])
        self.s1.setText("s1 = " + str(self.dist[8,0])[0:10])
        self.s2.setText("s2 = " + str(self.dist[9,0])[0:10])
        self.s3.setText("s3 = " + str(self.dist[10,0])[0:10])
        self.s4.setText("s4 = " + str(self.dist[11,0])[0:10])
        
        self.Accept2.setDisabled(False)
        self.Save_camera_calibration.setDisabled(False)
        
        self.Camera.stop()
        self.Camera.Camera_index = self.Camera_index_used
        self.Camera.checked = self.Show_marker_calib.isChecked()
        self.Camera.detector = self.Detector_camera_calibration
        self.Camera.board = self.Charuco_board
        self.Camera.mtx = self.mtx
        self.Camera.dist = self.dist
        self.Camera.start()
        self.Camera.ImageUpdate.connect(self.ImageUpdateSlot)

    ##------ Take a picture and save it to the taken picture list ------##
    def Take_picture_camera_calibration(self):
        try:
            cv.imwrite(os.path.join(os.getcwd(), "tmp") + "/" + str(self.Number_of_images)+".jpg", self.Camera.frame)
            self.Number_of_images = self.Number_of_images + 1
            with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
                joints = np.array(list(csv.reader(csvfile)))
                joints = joints.astype(float)
            if joints.size == 0 :
                joints = self.current_robot_joint_parameters
            else:
                joints = np.append(joints,self.current_robot_joint_parameters,axis = 0)
        except Exception as e:
            print(e)
        if self.Pages.currentIndex() != 1:
            rvecs = None
            tvecs = None
            gray = cv.cvtColor(self.Camera.frame, cv.COLOR_RGB2GRAY)
            corners, ids, rejectedImgPoints = self.Camera.detector.detectMarkers(gray)
            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
            for corner in corners:
                cv.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)                        
            rvec = np.float32(np.array([0,0,1]))
            tvec = np.float32(np.array([0,0,0]))
            if len(corners) > 0:
                charucoretval, charucoCorners, charucoIds = cv.aruco.interpolateCornersCharuco(corners, ids, gray, self.Camera.board)
                retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, self.Camera.board, self.mtx, self.dist,rvec,tvec)
            
            rvec = np.reshape(rvec,(1,3))
            tvec = np.reshape(tvec,(1,3))/1000
            with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
                rvecs = np.array(list(csv.reader(csvfile)))
                rvecs = rvecs.astype(float)
                rvecs = np.append(rvecs,rvec,axis = 0)
            np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'rvecs.csv', rvecs, delimiter=',') 
            
            with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
                tvecs = np.array(list(csv.reader(csvfile)))
                tvecs = tvecs.astype(float)
                tvecs = np.append(tvecs,tvec,axis = 0)
            np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'tvecs.csv', tvecs, delimiter=',') 
        
        
        np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'joints.csv', joints, delimiter=',') 
        if self.Number_of_images > 0:
            image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/0.jpg", cv.IMREAD_COLOR)
            ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
            Pic = ConvertToQtFormat.scaled(391,220,Qt.AspectRatioMode.KeepAspectRatio)
            self.Calibration_image.setPixmap(QPixmap(Pic))
            self.Calibration_image_2.setPixmap(QPixmap(Pic))
            self.Calibration_image_3.setPixmap(QPixmap(Pic))
        if self.Number_of_images >= 6:
            self.Calculate_cam_calib_params.setDisabled(False)
        
    ##------ Jump to the next taken image if it is possible ------##        
    def Right_image(self):
        if self.Number_of_images > 1:
            self.image_shown += 1
            if self.image_shown > self.Number_of_images-1:
                self.image_shown = self.Number_of_images-1
            image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/"+ str(self.image_shown) +".jpg", cv.IMREAD_COLOR)
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
            Pic = ConvertToQtFormat.scaled(391,220,Qt.AspectRatioMode.KeepAspectRatio)
            self.Calibration_image.setPixmap(QPixmap(Pic))
            self.Calibration_image_2.setPixmap(QPixmap(Pic))
            self.Calibration_image_3.setPixmap(QPixmap(Pic))
    
    ##------ Jump to the preveous image if it is possible ------##                              
    def Left_image(self):
        if self.Number_of_images > 1:
            self.image_shown += -1
            if self.image_shown < 0:
                self.image_shown = 0
            image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/"+ str(self.image_shown) +".jpg", cv.IMREAD_COLOR)
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
            Pic = ConvertToQtFormat.scaled(391,220,Qt.AspectRatioMode.KeepAspectRatio)
            self.Calibration_image.setPixmap(QPixmap(Pic))
            self.Calibration_image_2.setPixmap(QPixmap(Pic))
            self.Calibration_image_3.setPixmap(QPixmap(Pic))
    
    ##------ Delete current image and jump to the preveous one ------##        
    def Delete_image(self):
        if self.Number_of_images > 1:
            os.remove(os.path.join(os.getcwd(), "tmp") + "/"+ str(self.image_shown) +".jpg")   
            with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
                    joints = np.array(list(csv.reader(csvfile)))
                    joints = joints.astype(float)
                    joints = np.delete(joints,self.image_shown,0)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/joints.csv", joints, delimiter=",")
            if os.path.isfile(os.path.join(os.getcwd(), "tmp") + "/" + 'rvecs.csv'):
                with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
                    rvecs = np.array(list(csv.reader(csvfile)))
                    rvecs = rvecs.astype(float)
                    rvecs = np.delete(rvecs,self.image_shown,0)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", rvecs, delimiter=",")
            if os.path.isfile(os.path.join(os.getcwd(), "tmp") + "/" + 'tvecs.csv'):
                with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
                    tvecs = np.array(list(csv.reader(csvfile)))
                    tvecs = tvecs.astype(float)
                    tvecs = np.delete(tvecs,self.image_shown,0)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", tvecs, delimiter=",")  
                
            for i in range(self.image_shown,self.Number_of_images-1):
                my_source = os.path.join(os.getcwd(), "tmp") + "/"+ str(i+1) +".jpg"
                my_dest = os.path.join(os.getcwd(), "tmp") + "/"+ str(i) +".jpg"
                os.rename(my_source, my_dest)
            self.Number_of_images = self.Number_of_images - 1
            self.image_shown +=-1
            if self.image_shown < 0:
                self.image_shown = 0
            image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/"+ str(self.image_shown) +".jpg", cv.IMREAD_COLOR)
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
            Pic = ConvertToQtFormat.scaled(391,220,Qt.AspectRatioMode.KeepAspectRatio)
            self.Calibration_image.setPixmap(QPixmap(Pic))
            self.Calibration_image_2.setPixmap(QPixmap(Pic))
            self.Calibration_image_3.setPixmap(QPixmap(Pic))
        if self.Number_of_images<6:
            self.Calculate_cam_calib_params.setDisabled(True)
    
    ##------ Show detected markers on camera image ------##    
    def Show_markers_on_image(self,s):
        
        if self.Pages.currentIndex() == 1:
            self.Show_marker_pre.setChecked(self.Show_marker_calib.isChecked())
            self.Show_marker_robcalib.setChecked(self.Show_marker_calib.isChecked())
        
        elif self.Pages.currentIndex() == 2:
            self.Show_marker_calib.setChecked(self.Show_marker_pre.isChecked())
            self.Show_marker_robcalib.setChecked(self.Show_marker_pre.isChecked())
            
        elif self.Pages.currentIndex() == 3:    
            self.Show_marker_calib.setChecked(self.Show_marker_robcalib.isChecked())
            self.Show_marker_pre.setChecked(self.Show_marker_robcalib.isChecked())
        
        self.Camera.Camera_index = self.Camera_index_used
        self.Camera.checked = self.Show_marker_calib.isChecked() or self.Show_marker_pre.isChecked() or self.Show_marker_robcalib.isChecked()
        self.Camera.detector = self.Detector_camera_calibration
        self.Camera.board = self.Charuco_board
        self.Camera.mtx = self.mtx
        self.Camera.dist = self.dist
        self.Camera.start()
        self.Camera.ImageUpdate.connect(self.ImageUpdateSlot)

    ##------ Open camera calibration from config .json file ------##        
    def Open_camera_calib(self):
        self.Number_of_images = 0 
        self.file_dialog_open_calibration = QFileDialog()
        self.fname_open_calibration = self.file_dialog_open_calibration.getExistingDirectory(self,"Open camera calibration from directory")
        self.file_dialog_open_calibration.setFixedSize(600,400)
        if self.fname_open_calibration != "":
            i = 0
            while True:
                if os.path.isfile(self.fname_open_calibration + "/" + str(i)+".jpg"):
                    image = cv.imread(self.fname_open_calibration + "/" + str(i)+".jpg", cv.IMREAD_COLOR)
                    cv.imwrite(os.path.join(os.getcwd(), "tmp") + "/" + str(i)+".jpg", image)
                    i = i + 1
                    self.Number_of_images += 1 
                else:
                    break
        self.CCC.detector = self.Detector_camera_calibration
        self.CCC.board = self.Charuco_board
        self.CCC.images = self.Number_of_images
        try: 
            if self.fname_open_calibration != "":
                with open(self.fname_open_calibration + "/" + 'mtx.csv', newline='\n') as csvfile:
                    self.mtx = np.array(list(csv.reader(csvfile)))
                with open(self.fname_open_calibration + "/" + 'dist.csv', newline='\n') as csvfile:
                    self.dist = np.array(list(csv.reader(csvfile)))
                with open(self.fname_open_calibration + "/" + 'joints.csv', newline='\n') as csvfile:
                    joints = np.array(list(csv.reader(csvfile)))
                    joints = joints.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/joints.csv", joints, delimiter=",")
                with open(self.fname_open_calibration + "/" + 'rvecs.csv', newline='\n') as csvfile:
                    rvecs = np.array(list(csv.reader(csvfile)))
                    rvecs = rvecs.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", rvecs, delimiter=",")
                with open(self.fname_open_calibration + "/" + 'tvecs.csv', newline='\n') as csvfile:
                    tvecs = np.array(list(csv.reader(csvfile)))
                    tvecs = tvecs.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", tvecs, delimiter=",")
                self.mtx = self.mtx.astype(float)
                self.dist = self.dist.astype(float)
                        
            if self.Number_of_images > 0:
                ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(391,220,Qt.AspectRatioMode.KeepAspectRatio)
                self.Calibration_image.setPixmap(QPixmap(Pic))
                self.Calibration_image_2.setPixmap(QPixmap(Pic))
                self.Calibration_image_3.setPixmap(QPixmap(Pic))
            if self.Number_of_images >= 6:
                self.Calculate_cam_calib_params.setDisabled(False)
                self.Save_camera_calibration.setDisabled(False)
                self.Accept2.setDisabled(False)
                self.Calibration_prog_bar.setValue(100)          
                
                self.F_x.setText(str(self.mtx[0,0])[0:10])
                self.F_y.setText(str(self.mtx[1,1])[0:10])
                self.u_v.setText(str(self.mtx[0,2])[0:10])
                self.w_v.setText(str(self.mtx[1,2])[0:10])
                
                self.k1.setText("k1 = " + str(self.dist[0,0])[0:10])
                self.k2.setText("k2 = " + str(self.dist[1,0])[0:10])
                self.k3.setText("k3 = " + str(self.dist[4,0])[0:10])
                self.k4.setText("k4 = " + str(self.dist[5,0])[0:10])
                self.k5.setText("k5 = " + str(self.dist[6,0])[0:10])
                self.k6.setText("k6 = " + str(self.dist[7,0])[0:10])
                self.p1.setText("p1 = " + str(self.dist[2,0])[0:10])
                self.p2.setText("p2 = " + str(self.dist[3,0])[0:10])
                self.s1.setText("s1 = " + str(self.dist[8,0])[0:10])
                self.s2.setText("s2 = " + str(self.dist[9,0])[0:10])
                self.s3.setText("s3 = " + str(self.dist[10,0])[0:10])
                self.s4.setText("s4 = " + str(self.dist[11,0])[0:10])
        except Exception as e:
            print(e)    
    ##------ Save images and .txt file of the calibration ------##
    def Save_camera_calib(self):  
        self.file_dialog_save_calib = QFileDialog()
        self.fname_saved_calib = self.file_dialog_save_calib.getExistingDirectory(self,"Save camera calibration into directory")
        self.file_dialog_save_calib.setFixedSize(600,400)
        if self.fname_saved_calib != "":
            for i in range(self.Number_of_images):
                image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/" + str(i) +".jpg", cv.IMREAD_COLOR)
                cv.imwrite(self.fname_saved_calib + "/" + str(i)+".jpg", image)
        if self.fname_saved_calib != "":
            np.savetxt(self.fname_saved_calib + "/" + 'mtx.csv', self.mtx, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'dist.csv', self.dist, delimiter=',')
            with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
                    joints = np.array(list(csv.reader(csvfile)))
                    joints = joints.astype(float)
            with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
                    rvecs = np.array(list(csv.reader(csvfile)))
                    rvecs = rvecs.astype(float)
            with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
                    tvecs = np.array(list(csv.reader(csvfile)))
                    tvecs = tvecs.astype(float)        
            np.savetxt(self.fname_saved_calib + "/" + 'joints.csv', joints, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'rvecs.csv', rvecs, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'tvecs.csv', tvecs, delimiter=',')        
    
    ##------ Start auto calibration ------##
    def Auto_calibration(self): 
        if self.Pages.currentIndex() == 1:
            ##------ Define spinboxes ------##
            self.Speher_radius_spinbox = self.findChild(QDoubleSpinBox, "Sphere_radius")
            self.Sphere_grid_width_spinbox = self.findChild(QSpinBox, "Sphere_grid_width")
            self.Sphere_grid_height_spinbox = self.findChild(QSpinBox, "Sphere_grid_height")
            self.Sphere_grid_hor_dist_spinbox = self.findChild(QDoubleSpinBox, "Sphere_grid_hor_dist")
            self.Sphere_grid_ver_dist_spinbox = self.findChild(QDoubleSpinBox, "Sphere_grid_ver_dist")
            
            ##------ Define spinboxes ------##
            self.Ori_grid_width_spinbox = self.findChild(QSpinBox, "Ori_grid_width")
            self.Ori_grid_height_spinbox = self.findChild(QSpinBox, "Ori_grid_height")
            self.Ori_grid_hor_dist_spinbox = self.findChild(QDoubleSpinBox, "Ori_grid_hor_dist")
            self.Ori_grid_ver_dist_spinbox = self.findChild(QDoubleSpinBox, "Ori_grid_ver_dist")
            
            ##------ Define spinboxes  ------##
            self.Layer_num_spinbox = self.findChild(QSpinBox, "Layer_num")
            self.Layer_distance_spinbox = self.findChild(QDoubleSpinBox, "Layer_dist")
        
        elif self.Pages.currentIndex() == 2:
            ##------ Define spinboxes ------##
            self.Speher_radius_spinbox = self.findChild(QDoubleSpinBox, "Sphere_radius_2")
            self.Sphere_grid_width_spinbox = self.findChild(QSpinBox, "Sphere_grid_width_2")
            self.Sphere_grid_height_spinbox = self.findChild(QSpinBox, "Sphere_grid_height_2")
            self.Sphere_grid_hor_dist_spinbox = self.findChild(QDoubleSpinBox, "Sphere_grid_hor_dist_2")
            self.Sphere_grid_ver_dist_spinbox = self.findChild(QDoubleSpinBox, "Sphere_grid_ver_dist_2")
            
            ##------ Define spinboxes ------##
            self.Ori_grid_width_spinbox = self.findChild(QSpinBox, "Ori_grid_width_2")
            self.Ori_grid_height_spinbox = self.findChild(QSpinBox, "Ori_grid_height_2")
            self.Ori_grid_hor_dist_spinbox = self.findChild(QDoubleSpinBox, "Ori_grid_hor_dist_2")
            self.Ori_grid_ver_dist_spinbox = self.findChild(QDoubleSpinBox, "Ori_grid_ver_dist_2")
            
            ##------ Define spinboxes  ------##
            self.Layer_num_spinbox = self.findChild(QSpinBox, "Layer_num_2")
            self.Layer_distance_spinbox = self.findChild(QDoubleSpinBox, "Layer_dist_2")
        
        elif self.Pages.currentIndex() == 3:
            ##------ Define spinboxes ------##
            self.Speher_radius_spinbox = self.findChild(QDoubleSpinBox, "Sphere_radius_3")
            self.Sphere_grid_width_spinbox = self.findChild(QSpinBox, "Sphere_grid_width_3")
            self.Sphere_grid_height_spinbox = self.findChild(QSpinBox, "Sphere_grid_height_3")
            self.Sphere_grid_hor_dist_spinbox = self.findChild(QDoubleSpinBox, "Sphere_grid_hor_dist_3")
            self.Sphere_grid_ver_dist_spinbox = self.findChild(QDoubleSpinBox, "Sphere_grid_ver_dist_3")
            
            ##------ Define spinboxes ------##
            self.Ori_grid_width_spinbox = self.findChild(QSpinBox, "Ori_grid_width_3")
            self.Ori_grid_height_spinbox = self.findChild(QSpinBox, "Ori_grid_height_3")
            self.Ori_grid_hor_dist_spinbox = self.findChild(QDoubleSpinBox, "Ori_grid_hor_dist_3")
            self.Ori_grid_ver_dist_spinbox = self.findChild(QDoubleSpinBox, "Ori_grid_ver_dist_3")
            
            ##------ Define spinboxes  ------##
            self.Layer_num_spinbox = self.findChild(QSpinBox, "Layer_num_3")
            self.Layer_distance_spinbox = self.findChild(QDoubleSpinBox, "Layer_dist_3")
        
        self.Pic_taker_tabs.setTabEnabled(0,False)
        self.Pic_taker_tabs.setTabEnabled(2,False)
        self.Speher_radius_spinbox.setDisabled(True)
        self.Sphere_grid_width_spinbox.setDisabled(True)
        self.Sphere_grid_height_spinbox.setDisabled(True)
        self.Sphere_grid_hor_dist_spinbox.setDisabled(True)
        self.Sphere_grid_ver_dist_spinbox.setDisabled(True)
        
        self.Ori_grid_width_spinbox.setDisabled(True)
        self.Ori_grid_height_spinbox.setDisabled(True)
        self.Ori_grid_hor_dist_spinbox.setDisabled(True)
        self.Ori_grid_ver_dist_spinbox.setDisabled(True)
        
        self.Layer_num_spinbox.setDisabled(True)
        self.Layer_distance_spinbox.setDisabled(True)
        
        self.Start_auto_button.setDisabled(True)
        self.Open_camera_calibration.setDisabled(True)
        
        x = float(self.line_edit_x_value.text())*1000   
        y = float(self.line_edit_y_value.text())*1000
        z = float(self.line_edit_z_value.text())*1000
        rx = float(self.line_edit_rx_value.text())
        ry = float(self.line_edit_ry_value.text())
        rz = float(self.line_edit_rz_value.text())
        
        start_pose = [x,y,z,rx,ry,rz]
        
        pose_generator = PoseGenerator()
        
        rad = float(self.Speher_radius_spinbox.value())
        orbiting_num_pos_x = int(self.Sphere_grid_width_spinbox.value())
        orbiting_num_pos_y = int(self.Sphere_grid_height_spinbox.value()) 
        orbiting_angle_pos_x = float(self.Sphere_grid_hor_dist_spinbox.value())
        orbiting_angle_pos_y = float(self.Sphere_grid_ver_dist_spinbox.value())
        
        
        orbiting_poses = pose_generator.generate_orbitings(start_pose=np.array(start_pose), num_pos_x=orbiting_num_pos_x, num_pos_y=orbiting_num_pos_y, angle_pos_x=orbiting_angle_pos_x/180*np.pi, angle_pos_y=orbiting_angle_pos_y/180*np.pi, radius=rad)
        
        layer_num = int(self.Layer_num_spinbox.value())
        layer_dist = float(self.Layer_distance_spinbox.value())
        
        sphere_poses = pose_generator.generate_spheres(orbiting_poses=orbiting_poses, r_step=layer_dist, steps=layer_num)
        
        orient_num_x = int(self.Ori_grid_width_spinbox.value())
        orient_num_y = int(self.Ori_grid_height_spinbox.value()) 
        orient_angle_x = float(self.Ori_grid_hor_dist_spinbox.value()) 
        orient_angle_y = float(self.Ori_grid_ver_dist_spinbox.value())
        
        
        calibration_photo_poses = pose_generator.generate_poses(poses=sphere_poses, num_ori_x=orient_num_x, num_ori_y=orient_num_y, ori_x_angle=orient_angle_x/180*np.pi, ori_y_angle=orient_angle_y/180*np.pi)
        calibration_photo_poses[:,0:3] = calibration_photo_poses[:,0:3] / 1000
        
        used_poses = np.asarray(calibration_photo_poses)
            
        widgets = (self.Pic_taker_tabs,self.Speher_radius_spinbox,self.Sphere_grid_width_spinbox,self.Sphere_grid_height_spinbox,self.Sphere_grid_hor_dist_spinbox,self.Sphere_grid_ver_dist_spinbox,        self.Ori_grid_width_spinbox,self.Ori_grid_height_spinbox,self.Ori_grid_hor_dist_spinbox,self.Ori_grid_ver_dist_spinbox,self.Layer_num_spinbox,self.Layer_distance_spinbox,      self.Start_auto_button,self.Open_camera_calibration)
        
        self.Auto_calibration_class = Auto_image_taker(start_pose,used_poses,self.Camera,self.Set_robot_coordinates_class,widgets,self.Number_of_images)
        self.Get_robot_coordinates.Robot_coordinates.connect(self.Auto_calibration_class.Refresh_robot_pose)
        self.Auto_calibration_class.start()
        self.Auto_calibration_class.Images.connect(self.Get_images) 
        self.Auto_calibration_class.Robot_joint_values.connect(self.Get_joint_coordinates)
        self.Auto_calibration_class.stop()             
    
    def Get_joint_coordinates(self,joint_coordinates):
        with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
            joints = np.array(list(csv.reader(csvfile)))
            joints = joints.astype(float)
        if joints.size == 0 :
            joints = joint_coordinates
        else:
            joints = np.append(joints,joint_coordinates,axis = 0)
        np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'joints.csv', joints, delimiter=',') 
        
    
    ##------ Open emitted images ------##    
    def Get_images(self,images):
        if self.Pages.currentIndex() != 1:
            rvecs_c = None
            tvecs_c = None
            for i in range(self.Number_of_images,images):
                current_image = image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/" + str(i) + ".jpg", cv.IMREAD_COLOR)
                gray = cv.cvtColor(current_image, cv.COLOR_RGB2GRAY)
                corners, ids, rejectedImgPoints = self.Camera.detector.detectMarkers(gray)
                criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
                for corner in corners:
                    cv.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)                        
                rvec = np.float32(np.array([0,0,1]))
                tvec = np.float32(np.array([0,0,0]))
                if len(corners) > 0:
                    charucoretval, charucoCorners, charucoIds = cv.aruco.interpolateCornersCharuco(corners, ids, gray, self.Camera.board)
                    retval, rvec, tvec = aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, self.Camera.board, self.mtx, self.dist,rvec,tvec)
                if rvecs_c is None and tvecs_c is None:
                    rvecs_c = np.reshape(rvec,(1,3))
                    tvecs_c = np.reshape(tvec,(1,3))/1000
                else:
                    rvec = np.reshape(rvec,(1,3))
                    rvecs_c = np.append(rvecs_c,rvec,axis = 0)
                    tvec = np.reshape(tvec,(1,3))/1000
                    tvecs_c = np.append(tvecs_c,tvec,axis = 0)
                
            with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
                rvecs = np.array(list(csv.reader(csvfile)))
                rvecs = rvecs.astype(float)
                rvecs = np.append(rvecs,rvecs_c,axis = 0)
            np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'rvecs.csv', rvecs, delimiter=',') 
            
            with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
                tvecs = np.array(list(csv.reader(csvfile)))
                tvecs = tvecs.astype(float)
                tvecs = np.append(tvecs,tvecs_c,axis = 0)
            np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'tvecs.csv', tvecs, delimiter=',') 
            self.Number_of_images = images
        else:
            self.Number_of_images = images
        if self.Number_of_images >=1:
            image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/0.jpg", cv.IMREAD_COLOR)
            ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
            Pic = ConvertToQtFormat.scaled(391,220,Qt.AspectRatioMode.KeepAspectRatio)
            self.Calibration_image.setPixmap(QPixmap(Pic))
            self.Calibration_image_2.setPixmap(QPixmap(Pic))
            self.Calibration_image_3.setPixmap(QPixmap(Pic))
        if self.Number_of_images >=6:
            self.Calculate_cam_calib_params.setDisabled(False)    
            self.CCC.detector = self.Detector_camera_calibration
            self.CCC.board = self.Charuco_board
            self.CCC.images = self.Number_of_images

    ##------ Open the 3rd tab ------##
    def To_page2(self):
        self.Pages.setTabEnabled(2,True)
        self.Pages.setCurrentWidget(self.Pages.findChild(QWidget, "SVD")) 
        self.CCC.stop()  

    ######################################################################
    # TAB 3 functions
    ###################################################################### 
    def Calculate_preveous(self):
        
        ig_xf = float(self.ig_x.text())
        ig_yf = float(self.ig_y.text()) 
        ig_zf = float(self.ig_z.text()) 
        ig_rxf = float(self.ig_rx.text())
        ig_ryf = float(self.ig_ry.text())
        ig_rzf = float(self.ig_rz.text())
        
        ##------ Define Initial guess matrix ------##          
        R_Base2Board = R.from_euler('xyz',np.array([ig_rxf,ig_ryf,ig_rzf]),degrees = False).as_matrix()
        T_Base2Board = np.append(np.append(R_Base2Board,np.array([[ig_xf],[ig_yf],[ig_zf]]),axis = 1),np.array([[0,0,0,1]]),axis = 0)            
        T_Base2Board = np.reshape(T_Base2Board,(4,4,1))
        
        self.Pre_Calculation_thread = Pre_Calculation(T_Base2Board,self.Robot)
        self.Pre_Calculation_thread.Progbar_value.connect(self.Set_Progbar_pre)
        self.Pre_Calculation_thread.T_B2b.connect(self.Get_T_B2b)
        self.Pre_Calculation_thread.T_T2C.connect(self.Get_T_T2C)
        self.Pre_Calculation_thread.start()
        self.Pre_Calculation_thread.stop()

    def Set_Progbar_pre(self,value):
        self.Pre_prog_bar.setValue(value)
    
    def Get_T_B2b(self,T_Base2Board):
        self.T_bb_11.setText(str(T_Base2Board[0,0,0])[0:8])
        self.T_bb_12.setText(str(T_Base2Board[0,1,0])[0:8])
        self.T_bb_13.setText(str(T_Base2Board[0,2,0])[0:8])
        self.T_bb_14.setText(str(T_Base2Board[0,3,0])[0:8])
        self.T_bb_21.setText(str(T_Base2Board[1,0,0])[0:8])
        self.T_bb_22.setText(str(T_Base2Board[1,1,0])[0:8])
        self.T_bb_23.setText(str(T_Base2Board[1,2,0])[0:8])
        self.T_bb_24.setText(str(T_Base2Board[1,3,0])[0:8])
        self.T_bb_31.setText(str(T_Base2Board[2,0,0])[0:8])
        self.T_bb_32.setText(str(T_Base2Board[2,1,0])[0:8])
        self.T_bb_33.setText(str(T_Base2Board[2,2,0])[0:8])
        self.T_bb_34.setText(str(T_Base2Board[2,3,0])[0:8])  
        np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'T_Base2Board.csv', np.reshape(T_Base2Board,(4,4)), delimiter=',') 
        
    def Get_T_T2C(self,T_TCP2C):
        self.T_tc_11.setText(str(T_TCP2C[0,0,0])[0:8])
        self.T_tc_12.setText(str(T_TCP2C[0,1,0])[0:8])
        self.T_tc_13.setText(str(T_TCP2C[0,2,0])[0:8])
        self.T_tc_14.setText(str(T_TCP2C[0,3,0])[0:8])
        self.T_tc_21.setText(str(T_TCP2C[1,0,0])[0:8])
        self.T_tc_22.setText(str(T_TCP2C[1,1,0])[0:8])
        self.T_tc_23.setText(str(T_TCP2C[1,2,0])[0:8])
        self.T_tc_24.setText(str(T_TCP2C[1,3,0])[0:8])
        self.T_tc_31.setText(str(T_TCP2C[2,0,0])[0:8])
        self.T_tc_32.setText(str(T_TCP2C[2,1,0])[0:8])
        self.T_tc_33.setText(str(T_TCP2C[2,2,0])[0:8])
        self.T_tc_34.setText(str(T_TCP2C[2,3,0])[0:8])     
        np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'T_TCP2C.csv', np.reshape(T_TCP2C,(4,4)), delimiter=',')
        self.Accept3.setDisabled(False)
        self.Save_pre.setDisabled(False)
        
    def Open_pre_func(self):
        self.Number_of_images = 0 
        self.file_dialog_open_calibration = QFileDialog()
        self.fname_open_calibration = self.file_dialog_open_calibration.getExistingDirectory(self,"Open Precalculations from directory")
        self.file_dialog_open_calibration.setFixedSize(600,400)
        if self.fname_open_calibration != "":
            i = 0
            while True:
                if os.path.isfile(self.fname_open_calibration + "/" + str(i)+".jpg"):
                    image = cv.imread(self.fname_open_calibration + "/" + str(i)+".jpg", cv.IMREAD_COLOR)
                    cv.imwrite(os.path.join(os.getcwd(), "tmp") + "/" + str(i)+".jpg", image)
                    i = i + 1
                    self.Number_of_images += 1 
                else:
                    break
        self.CCC.detector = self.Detector_camera_calibration
        self.CCC.board = self.Charuco_board
        self.CCC.images = self.Number_of_images
        try: 
            with open(self.fname_open_calibration +  '/joints.csv', newline='\n') as csvfile:
                    joints = np.array(list(csv.reader(csvfile)))
                    joints = joints.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/joints.csv", joints, delimiter=",")
            with open(self.fname_open_calibration + "/" + 'rvecs.csv', newline='\n') as csvfile:
                    rvecs = np.array(list(csv.reader(csvfile)))
                    rvecs = rvecs.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", rvecs, delimiter=",")
            with open(self.fname_open_calibration + "/" + 'tvecs.csv', newline='\n') as csvfile:
                    tvecs = np.array(list(csv.reader(csvfile)))
                    tvecs = tvecs.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", tvecs, delimiter=",")
            with open(self.fname_open_calibration + "/T_Base2Board.csv", newline='\n') as csvfile:
                    T_Base2Board = np.array(list(csv.reader(csvfile)))
                    T_Base2Board = T_Base2Board.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/T_Base2Board.csv", T_Base2Board, delimiter=",")
                    
            with open(self.fname_open_calibration + "/T_TCP2C.csv", newline='\n') as csvfile:
                    T_TCP2C = np.array(list(csv.reader(csvfile)))
                    T_TCP2C = T_TCP2C.astype(float)    
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/T_TCP2C.csv", T_TCP2C, delimiter=",")
                    
            with open(self.fname_open_calibration + "/mtx.csv", newline='\n') as csvfile:
                    mtx = np.array(list(csv.reader(csvfile)))
                    self.mtx= mtx.astype(float)    
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/mtx.csv", self.mtx, delimiter=",")
            
            with open(self.fname_open_calibration + "/dist.csv", newline='\n') as csvfile:
                    dist = np.array(list(csv.reader(csvfile)))
                    self.dist = dist.astype(float)    
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/dist.csv", self.dist, delimiter=",")                
            
            if self.Number_of_images > 0:
                ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(391,220,Qt.AspectRatioMode.KeepAspectRatio)
                self.Calibration_image.setPixmap(QPixmap(Pic))
                self.Calibration_image_2.setPixmap(QPixmap(Pic))
                self.Calibration_image_3.setPixmap(QPixmap(Pic))
            if self.Number_of_images >= 6:
                self.Accept3.setDisabled(False)
                self.Save_pre.setDisabled(False)
                self.Pre_prog_bar.setValue(100)  
                
            self.F_x.setText(str(self.mtx[0,0])[0:10])
            self.F_y.setText(str(self.mtx[1,1])[0:10])
            self.u_v.setText(str(self.mtx[0,2])[0:10])
            self.w_v.setText(str(self.mtx[1,2])[0:10])
            
            self.k1.setText("k1 = " + str(self.dist[0,0])[0:10])
            self.k2.setText("k2 = " + str(self.dist[1,0])[0:10])
            self.k3.setText("k3 = " + str(self.dist[4,0])[0:10])
            self.k4.setText("k4 = " + str(self.dist[5,0])[0:10])
            self.k5.setText("k5 = " + str(self.dist[6,0])[0:10])
            self.k6.setText("k6 = " + str(self.dist[7,0])[0:10])
            self.p1.setText("p1 = " + str(self.dist[2,0])[0:10])
            self.p2.setText("p2 = " + str(self.dist[3,0])[0:10])
            self.s1.setText("s1 = " + str(self.dist[8,0])[0:10])
            self.s2.setText("s2 = " + str(self.dist[9,0])[0:10])
            self.s3.setText("s3 = " + str(self.dist[10,0])[0:10])
            self.s4.setText("s4 = " + str(self.dist[11,0])[0:10])    
                
            self.T_bb_11.setText(str(T_Base2Board[0,0])[0:8])
            self.T_bb_12.setText(str(T_Base2Board[0,1])[0:8])
            self.T_bb_13.setText(str(T_Base2Board[0,2])[0:8])
            self.T_bb_14.setText(str(T_Base2Board[0,3])[0:8])
            self.T_bb_21.setText(str(T_Base2Board[1,0])[0:8])
            self.T_bb_22.setText(str(T_Base2Board[1,1])[0:8])
            self.T_bb_23.setText(str(T_Base2Board[1,2])[0:8])
            self.T_bb_24.setText(str(T_Base2Board[1,3])[0:8])
            self.T_bb_31.setText(str(T_Base2Board[2,0])[0:8])
            self.T_bb_32.setText(str(T_Base2Board[2,1])[0:8])
            self.T_bb_33.setText(str(T_Base2Board[2,2])[0:8])
            self.T_bb_34.setText(str(T_Base2Board[2,3])[0:8])
            
            self.T_tc_11.setText(str(T_TCP2C[0,0])[0:8])
            self.T_tc_12.setText(str(T_TCP2C[0,1])[0:8])
            self.T_tc_13.setText(str(T_TCP2C[0,2])[0:8])
            self.T_tc_14.setText(str(T_TCP2C[0,3])[0:8])
            self.T_tc_21.setText(str(T_TCP2C[1,0])[0:8])
            self.T_tc_22.setText(str(T_TCP2C[1,1])[0:8])
            self.T_tc_23.setText(str(T_TCP2C[1,2])[0:8])
            self.T_tc_24.setText(str(T_TCP2C[1,3])[0:8])
            self.T_tc_31.setText(str(T_TCP2C[2,0])[0:8])
            self.T_tc_32.setText(str(T_TCP2C[2,1])[0:8])
            self.T_tc_33.setText(str(T_TCP2C[2,2])[0:8])
            self.T_tc_34.setText(str(T_TCP2C[2,3])[0:8])   
            
        except Exception as e:
            print(e)   
    
    def Save_pre_func(self):
        self.file_dialog_save_calib = QFileDialog()
        self.fname_saved_calib = self.file_dialog_save_calib.getExistingDirectory(self,"Save Precalculation into directory")
        self.file_dialog_save_calib.setFixedSize(600,400)
        if self.fname_saved_calib != "":
            for i in range(self.Number_of_images):
                image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/" + str(i) +".jpg", cv.IMREAD_COLOR)
                cv.imwrite(self.fname_saved_calib + "/" + str(i)+".jpg", image)
        if self.fname_saved_calib != "":
            np.savetxt(self.fname_saved_calib + "/" + 'mtx.csv', self.mtx, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'dist.csv', self.dist, delimiter=',')
            with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
                    joints = np.array(list(csv.reader(csvfile)))
                    joints = joints.astype(float)
            with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
                    rvecs = np.array(list(csv.reader(csvfile)))
                    rvecs = rvecs.astype(float)
            with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
                    tvecs = np.array(list(csv.reader(csvfile)))
                    tvecs = tvecs.astype(float)   
                    
            with open(os.path.join(os.getcwd(), "tmp") + "/T_Base2Board.csv", newline='\n') as csvfile:
                    T_b2b = np.array(list(csv.reader(csvfile)))
                    T_b2b = T_b2b.astype(float)
                    
            with open(os.path.join(os.getcwd(), "tmp") + "/T_TCP2C.csv", newline='\n') as csvfile:
                    T_T2C = np.array(list(csv.reader(csvfile)))
                    T_T2C = T_T2C.astype(float)     
            np.savetxt(self.fname_saved_calib + "/" + 'joints.csv', joints, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'rvecs.csv', rvecs, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'tvecs.csv', tvecs, delimiter=',')     
            np.savetxt(self.fname_saved_calib + "/" + 'T_Base2Board.csv', T_b2b, delimiter=',')     
            np.savetxt(self.fname_saved_calib + "/" + 'T_TCP2C.csv', T_T2C, delimiter=',')     
            
    def To_page3(self):
        self.Pages.setTabEnabled(3,True)
        self.Pages.setCurrentWidget(self.Pages.findChild(QWidget, "DH"))

    ######################################################################
    # TAB 3 functions
    ###################################################################### 
    
    def Open_end_func(self):
        self.Number_of_images = 0 
        self.file_dialog_open_calibration = QFileDialog()
        self.fname_open_calibration = self.file_dialog_open_calibration.getExistingDirectory(self,"Open Precalculations from directory")
        self.file_dialog_open_calibration.setFixedSize(600,400)
        if self.fname_open_calibration != "":
            i = 0
            while True:
                if os.path.isfile(self.fname_open_calibration + "/" + str(i)+".jpg"):
                    image = cv.imread(self.fname_open_calibration + "/" + str(i)+".jpg", cv.IMREAD_COLOR)
                    cv.imwrite(os.path.join(os.getcwd(), "tmp") + "/" + str(i)+".jpg", image)
                    i = i + 1
                    self.Number_of_images += 1 
                else:
                    break
        self.CCC.detector = self.Detector_camera_calibration
        self.CCC.board = self.Charuco_board
        self.CCC.images = self.Number_of_images
        try: 
            with open(self.fname_open_calibration +  '/joints.csv', newline='\n') as csvfile:
                    joints = np.array(list(csv.reader(csvfile)))
                    joints = joints.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/joints.csv", joints, delimiter=",")
            with open(self.fname_open_calibration + "/" + 'rvecs.csv', newline='\n') as csvfile:
                    rvecs = np.array(list(csv.reader(csvfile)))
                    rvecs = rvecs.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", rvecs, delimiter=",")
            with open(self.fname_open_calibration + "/" + 'tvecs.csv', newline='\n') as csvfile:
                    tvecs = np.array(list(csv.reader(csvfile)))
                    tvecs = tvecs.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", tvecs, delimiter=",")
            with open(self.fname_open_calibration + "/T_Base2Board.csv", newline='\n') as csvfile:
                    T_Base2Board = np.array(list(csv.reader(csvfile)))
                    T_Base2Board = T_Base2Board.astype(float)
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/T_Base2Board.csv", T_Base2Board, delimiter=",")
                    
            with open(self.fname_open_calibration + "/T_TCP2C.csv", newline='\n') as csvfile:
                    T_TCP2C = np.array(list(csv.reader(csvfile)))
                    T_TCP2C = T_TCP2C.astype(float)    
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/T_TCP2C.csv", T_TCP2C, delimiter=",")
                    
            with open(self.fname_open_calibration + "/mtx.csv", newline='\n') as csvfile:
                    mtx = np.array(list(csv.reader(csvfile)))
                    self.mtx= mtx.astype(float)    
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/mtx.csv", self.mtx, delimiter=",")
            
            with open(self.fname_open_calibration + "/dist.csv", newline='\n') as csvfile:
                    dist = np.array(list(csv.reader(csvfile)))
                    self.dist = dist.astype(float)    
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/dist.csv", self.dist, delimiter=",") 
                    
            with open(self.fname_open_calibration + "/DHs.csv", newline='\n') as csvfile:
                    DHs = np.array(list(csv.reader(csvfile)))
                    self.DHs = DHs.astype(float)    
                    np.savetxt(os.path.join(os.getcwd(), "tmp") + "/DHs.csv", self.DHs, delimiter=",")
            
            if self.Number_of_images > 0:
                ConvertToQtFormat = QImage(image.data,image.shape[1],image.shape[0],QImage.Format.Format_RGB888)
                Pic = ConvertToQtFormat.scaled(391,220,Qt.AspectRatioMode.KeepAspectRatio)
                self.Calibration_image.setPixmap(QPixmap(Pic))
                self.Calibration_image_2.setPixmap(QPixmap(Pic))
                self.Calibration_image_3.setPixmap(QPixmap(Pic))
            if self.Number_of_images >= 6:
                self.Accept3.setDisabled(False)
                self.Save_pre.setDisabled(False)
                self.Pre_prog_bar.setValue(100)  
                
            self.F_x.setText(str(self.mtx[0,0])[0:10])
            self.F_y.setText(str(self.mtx[1,1])[0:10])
            self.u_v.setText(str(self.mtx[0,2])[0:10])
            self.w_v.setText(str(self.mtx[1,2])[0:10])
            
            self.k1.setText("k1 = " + str(self.dist[0,0])[0:10])
            self.k2.setText("k2 = " + str(self.dist[1,0])[0:10])
            self.k3.setText("k3 = " + str(self.dist[4,0])[0:10])
            self.k4.setText("k4 = " + str(self.dist[5,0])[0:10])
            self.k5.setText("k5 = " + str(self.dist[6,0])[0:10])
            self.k6.setText("k6 = " + str(self.dist[7,0])[0:10])
            self.p1.setText("p1 = " + str(self.dist[2,0])[0:10])
            self.p2.setText("p2 = " + str(self.dist[3,0])[0:10])
            self.s1.setText("s1 = " + str(self.dist[8,0])[0:10])
            self.s2.setText("s2 = " + str(self.dist[9,0])[0:10])
            self.s3.setText("s3 = " + str(self.dist[10,0])[0:10])
            self.s4.setText("s4 = " + str(self.dist[11,0])[0:10])    
                
            self.T_bb_11.setText(str(T_Base2Board[0,0])[0:8])
            self.T_bb_12.setText(str(T_Base2Board[0,1])[0:8])
            self.T_bb_13.setText(str(T_Base2Board[0,2])[0:8])
            self.T_bb_14.setText(str(T_Base2Board[0,3])[0:8])
            self.T_bb_21.setText(str(T_Base2Board[1,0])[0:8])
            self.T_bb_22.setText(str(T_Base2Board[1,1])[0:8])
            self.T_bb_23.setText(str(T_Base2Board[1,2])[0:8])
            self.T_bb_24.setText(str(T_Base2Board[1,3])[0:8])
            self.T_bb_31.setText(str(T_Base2Board[2,0])[0:8])
            self.T_bb_32.setText(str(T_Base2Board[2,1])[0:8])
            self.T_bb_33.setText(str(T_Base2Board[2,2])[0:8])
            self.T_bb_34.setText(str(T_Base2Board[2,3])[0:8])
            
            self.T_tc_11.setText(str(T_TCP2C[0,0])[0:8])
            self.T_tc_12.setText(str(T_TCP2C[0,1])[0:8])
            self.T_tc_13.setText(str(T_TCP2C[0,2])[0:8])
            self.T_tc_14.setText(str(T_TCP2C[0,3])[0:8])
            self.T_tc_21.setText(str(T_TCP2C[1,0])[0:8])
            self.T_tc_22.setText(str(T_TCP2C[1,1])[0:8])
            self.T_tc_23.setText(str(T_TCP2C[1,2])[0:8])
            self.T_tc_24.setText(str(T_TCP2C[1,3])[0:8])
            self.T_tc_31.setText(str(T_TCP2C[2,0])[0:8])
            self.T_tc_32.setText(str(T_TCP2C[2,1])[0:8])
            self.T_tc_33.setText(str(T_TCP2C[2,2])[0:8])
            self.T_tc_34.setText(str(T_TCP2C[2,3])[0:8])   
            
            for i in range(self.DHs.shape[0]):
                for j in range(self.DHs.shape[1]):
                    self.DH_table.setItem(i,j,QTableWidgetItem(str(self.DHs[i,j])))
            self.Cal_progbar.setValue(3000)
            self.Accept_end.setDisabled(False)
            self.Save_end.setDisabled(False)
            
        except Exception as e:
            print(e)   
    
    def Save_end_func(self):
        DH_parameters = np.zeros((7,4))
        for i in range(7):
            for j in range(4):
                try:
                    DH_parameters[i,j] = self.DH_table.item(i,j).text()
                except Exception as e:
                    print(e)
                    
        self.file_dialog_save_calib = QFileDialog()
        self.fname_saved_calib = self.file_dialog_save_calib.getExistingDirectory(self,"Save robotcalibration into directory")
        self.file_dialog_save_calib.setFixedSize(600,400)
        if self.fname_saved_calib != "":
            for i in range(self.Number_of_images):
                image = cv.imread(os.path.join(os.getcwd(), "tmp") + "/" + str(i) +".jpg", cv.IMREAD_COLOR)
                cv.imwrite(self.fname_saved_calib + "/" + str(i)+".jpg", image)
        if self.fname_saved_calib != "":
            np.savetxt(self.fname_saved_calib + "/" + 'mtx.csv', self.mtx, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'dist.csv', self.dist, delimiter=',')
            with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", newline='\n') as csvfile:
                    joints = np.array(list(csv.reader(csvfile)))
                    joints = joints.astype(float)
            with open(os.path.join(os.getcwd(), "tmp") + "/rvecs.csv", newline='\n') as csvfile:
                    rvecs = np.array(list(csv.reader(csvfile)))
                    rvecs = rvecs.astype(float)
            with open(os.path.join(os.getcwd(), "tmp") + "/tvecs.csv", newline='\n') as csvfile:
                    tvecs = np.array(list(csv.reader(csvfile)))
                    tvecs = tvecs.astype(float)   
                    
            with open(os.path.join(os.getcwd(), "tmp") + "/T_Base2Board.csv", newline='\n') as csvfile:
                    T_b2b = np.array(list(csv.reader(csvfile)))
                    T_b2b = T_b2b.astype(float)
                    
            with open(os.path.join(os.getcwd(), "tmp") + "/T_TCP2C.csv", newline='\n') as csvfile:
                    T_T2C = np.array(list(csv.reader(csvfile)))
                    T_T2C = T_T2C.astype(float)     
            np.savetxt(self.fname_saved_calib + "/" + 'joints.csv', joints, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'rvecs.csv', rvecs, delimiter=',')
            np.savetxt(self.fname_saved_calib + "/" + 'tvecs.csv', tvecs, delimiter=',')     
            np.savetxt(self.fname_saved_calib + "/" + 'T_Base2Board.csv', T_b2b, delimiter=',')     
            np.savetxt(self.fname_saved_calib + "/" + 'T_TCP2C.csv', T_T2C, delimiter=',') 
            np.savetxt(self.fname_saved_calib + "/" + 'DHs.csv', DH_parameters, delimiter=',') 
            
    def DH_Cal(self):
    ##------ Open things that are save in temporary folder ------##
        #self.Opt_DH = Grad_Optimalization(self.Robot,0.2,1,0.5)
        #self.Opt_DH.start()
        #self.Opt_DH.stop()

        self.Opt_DH = NM_Optimalization(self.Robot)
        self.Opt_DH.Progbar_value.connect(self.Set_Progbar_end)
        self.Opt_DH.DH_params.connect(self.Get_DH)
        self.Opt_DH.T_T2C.connect(self.Get_T_T2C_cal)
        self.Opt_DH.start()
        self.Opt_DH.stop()
        
    def Set_Progbar_end(self,value):
        self.Cal_progbar.setValue(value)
    
    def Get_DH(self,DH_s):
        for i in range(DH_s.shape[0]):
            self.DH_table.setItem(int(i/4),int(i%4),QTableWidgetItem(str(DH_s[i])))
        np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'DHS.csv', np.reshape(DH_s,(-1,4)), delimiter=',')    
        self.Accept_end.setDisabled(False)
        self.Save_end.setDisabled(False)
        
    def Get_T_T2C_cal(self,T_TCP2C):
        np.savetxt(os.path.join(os.getcwd(), "tmp") + "/" + 'T_TCP2C.csv', np.reshape(T_TCP2C,(4,4)), delimiter=',')
        
##------ Start the main window ------##    

try:
    os.rmdir("tmp")
except Exception as e:
    print(e)
try:
    shutil.rmtree("tmp")    
except Exception as e:
    print(e)
os.mkdir(os.path.join(os.getcwd(), "tmp"))
with open(os.path.join(os.getcwd(), "tmp") + "/joints.csv", "w") as my_empty_csv:
    pass
app = QtWidgets.QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()
