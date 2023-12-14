from PyQt6.QtCore import *
import sys
import time
import socket

## ------ Set the robot velocities via socket ------##
class Set_robot_coordinates(QThread):
    def __init__(self,robot_ip):
        super().__init__()
        self.HOST = robot_ip 
        self.PORT = 30002
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.settimeout(3)
            self.s.connect((self.HOST, self.PORT))
        except Exception as e:
            print(e)            

    def Move_robot_x_p(self):
        self.robot_pos = "speedl([0.05,0,0,0,0,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
    
    def Move_robot_x_m(self):
        self.robot_pos = "speedl([-0.05,0,0,0,0,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
        
    def Move_robot_y_p(self):
        self.robot_pos = "speedl([0,0.05,0,0,0,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
    
    def Move_robot_y_m(self):
        self.robot_pos = "speedl([0,-0.05,0,0,0,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())    

    def Move_robot_z_p(self):
        self.robot_pos = "speedl([0,0,0.05,0,0,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
    
    def Move_robot_z_m(self):
        self.robot_pos = "speedl([0,0,-0.05,0,0,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
        
    def Move_robot_rx_p(self):
        self.robot_pos = "speedl([0,0,0,0.1,0,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())

    def Move_robot_rx_m(self):
        self.robot_pos = "speedl([0,0,0,-0.1,0,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
        
    def Move_robot_ry_p(self):
        self.robot_pos = "speedl([0,0,0,0,0.1,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
        
    def Move_robot_ry_m(self):
        self.robot_pos = "speedl([0,0,0,0,-0.1,0], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())    

    def Move_robot_rz_p(self):
        self.robot_pos = "speedl([0,0,0,0,0,0.1], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
    
    def Move_robot_rz_m(self):
        self.robot_pos = "speedl([0,0,0,0,0,-0.1], 0.15, 200)\n"
        self.s.send(self.robot_pos.encode())
    
    def Stop_robot(self):
        self.robot_pos = "stopl(2)\n"
        self.s.send(self.robot_pos.encode())

    def stop(self):
        self.s.close()
        self.quit()

