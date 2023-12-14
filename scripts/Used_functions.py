import numpy as np 
import numpy as np
import sys
from sympy import *
'''##------ Rotation stuff ------##'''    
def rodrigues_vec_to_rotation_mat(rodrigues_vec):
# Create a rotational matrix from axis-angle orientation representation 
# Input:    rodrigues_vec: 1x3 numpy.array(float) vector, an orientation representation in axis angle
# Output:   rotation_mat: 3x3 np.array(float) rotation matrix, an orientation representation
    theta = np.linalg.norm(rodrigues_vec)
    if theta < sys.float_info.epsilon:              
        rotation_mat = np.eye(3, dtype=float)
    else:
        r = rodrigues_vec / theta
        I = np.eye(3, dtype=float)
        r_rT = np.array([
            [r[0]*r[0], r[0]*r[1], r[0]*r[2]],
            [r[1]*r[0], r[1]*r[1], r[1]*r[2]],
            [r[2]*r[0], r[2]*r[1], r[2]*r[2]]
        ])
        r_cross = np.array([
            [0, -r[2], r[1]],
            [r[2], 0, -r[0]],
            [-r[1], r[0], 0]
        ])
        rotation_mat = np.cos(theta) * I + (1 - np.cos(theta)) * r_rT + np.sin(theta) * r_cross
    return rotation_mat

'''##------ Optimalization stuff ------##'''    
        
def OptimalTransformation(points1,points2):
#OptimalTransformation: to find a transformation such that (points1)-T*(points2) is minimal
# Inputs:
#  - points1, points2: 3xN matrices containing the points
#
# Outputs:
#  - T: 4x4 homogeneous transformation matrix
#  - errors.vec: matrix with size 3xN
#  - errors.norms: vector with length N
#  - errors.sum: scalar
#
# KJ - based on the content of http://nghiaho.com/?page_id=671    
    N = points1.shape[1]
    cog1 = np.reshape(np.mean(points1,axis = 1),(3,1))
    cog2 = np.reshape(np.mean(points2,axis = 1),(3,1))       
    points1_ = points1[:] - cog1
    points2_ = points2[:] - cog2
    toSVD = np.zeros((3,3))
    for i in range(N):  
        toSVD = toSVD + np.matmul(np.reshape(points2_[:,i],(3,1)),np.reshape(points1_[:,i],(1,3)))
    U,S,V0 = np.linalg.svd(toSVD)
    rotation = np.matmul(U,V0)
    if np.linalg.det(rotation)<0:
        V = np.matmul(np.transpose(V0),np.diag(np.array([1,1,-1])))
    else:
        V = np.transpose(V0)
    rotation = np.matmul(U,np.transpose(V)) 
    offset = cog2 - np.matmul(rotation,cog1)
    T = np.append(np.append(rotation,offset,axis=1),np.array([[0,0,0,1]]),axis = 0)
    errors_vec = points1-np.matmul(rotation,points2)-np.repeat(offset,N,axis = 1)
    errors_sum = np.sum(np.diag(np.matmul(np.transpose(errors_vec),errors_vec)))
    errors_norms = np.sqrt(np.diag(np.matmul(np.transpose(errors_vec),errors_vec)))
    errors = (errors_vec,errors_sum,errors_norms)
    return (T,errors)

'''##------ Matrix, and vector products functions ------##'''

def prodRM(T,p):
# Returns the product of rotation matrices and vectors
# Inputs:   T: 3x3xN numpy.array(float) rotation matrix
#           p: 3xN or 4xN numpy.array(float) vector, that represnt a point in 3D 
# Output:   out: 3xN or 4xN numpy.array(float) vector that contains the rotated points
    toincrease = False
    if p.shape[0]==4:
        p = p[:3,:]
        toincrease = True    
    if T.shape[2] == 1:
        out = np.matmul(np.reshape(T,(3,3)),p)
        if toincrease:
            out = np.append(out,np.ones((1,p.shape[1])),axis = 0)
    else:
        N = T.shape[2]
        if p.shape[1] == 1:
            out = np.zeros((3,N))
            for i in range(0,N):
                out[:,:,i] = np.reshape(np.matmul(T[:,:,i],p),(3,)) 
        elif (N==p.shape[1]):
            out = np.zeros((3,N))
            for i in range(N):
                out[:,i] = np.reshape(np.matmul(T[:,:,i],p[:,i]),(3,))
        if toincrease:
            out = np.append(out,np.ones((1,N)),axis = 0)
    return out


def prodHT(T,p):
# Returns the product of transformation matrices and vectors
# Inputs:   T: 4x4 or 4x4xN numpy.array(float) transformation matrices 
#           p: 3xN or 4xN numpy.array(float) matrix that represents a 3D points
# Output:   out: 3xN or 4xN numpy.array(float) matrix that represents the transformated points
    toreduce = False
    if p.shape[0] == 3:
        p = np.append(p,np.ones((1,p.shape[1])),axis = 0)
        toreduce = True      
    if T.shape[2] == 1:
        out = np.matmul(np.reshape(T,(4,4)),p)
        out = np.reshape(out,(4,p.shape[1]))
    else:
        N = T.shape[2]
        if p.shape[1] == 1:
            out = np.zeros((4,N))
            for i in range(0,N):
                out[:,i] = np.reshape(np.matmul(T[:,:,i],p),(4,))  
        elif N == p.shape[1]:
            out = np.zeros((4,N))
            for i in range(0,N):
                out[:,i] = np.reshape(np.matmul(T[:,:,i],p[:,i]),(4,))
    if toreduce:
        out = out[:3,:]
    return out

def invHT(T):
# Returns the inverse of transformation matrixes
# Inputs:   T: 4x4xN numpy.array(float) homogeneous transformation matrices
# Outputs:  out: 4x4xN numpy.array(float) homogeneous transformation matrices
    N = T.shape[2] 
    R = np.reshape(np.transpose(T[:3,:3,:],(1,0,2)),(3,3,N))
    p = prodRM(R,np.reshape(-T[:3,3,:],(3,N)))
    out = np.zeros((4,4,N))
    for i in range(0,N):
        out[:,:,i] = np.append(np.append(np.reshape(R[:,:,i],(3,3)),np.reshape(p[:,i],(3,1)),axis = 1),np.array([[0,0,0,1]]),axis = 0)
    return out

def prodHTs(T1,T2):
# Returns the product of transformation matrices
# Inputs:   T1: 4x4 or 4x4xN numpy.array(float) homogeneous transformation matrices
#           T2: 4x4 or 4x4xN numpy.array(float) homogeneous transformation matrices
# Output:   out: 4x4 or 4x4xN numpy.array(float) homogeneous transformation matrices
    bool1 = T1.shape[2] == 1
    bool2 = T2.shape[2] == 1
    bool3 = T1.shape[2] == T2.shape[2] 
    
    if bool1 and bool2:
        out = np.matmul(np.reshape(T1,(4,4)),np.reshape(T2,(4,4)))  
    if bool1 and (not bool2):
        N = T2.shape[2]
        out = np.zeros((4,4,N))
        for i in range(0,N):
            out[:,:,i] = np.matmul(np.reshape(T1,(4,4)),T2[:,:,i])          
    if (not bool1) and bool2:
        N = T1.shape[2]
        out = np.zeros((4,4,N))
        for i in range(0,N):
            out[:,:,i] = np.matmul(T1[:,:,i],np.reshape(T2,(4,4)))
    if bool3:
        N = T1.shape[2]
        out = np.zeros((4,4,N))
        for i in range(0,N):
            out[:,:,i] = np.matmul(T1[:,:,i],T2[:,:,i])   
    return out

'''##------ Links and manipulator classes ------##'''
class Link():
# Link: abstract class for robot segments
    def __init__(self,parameters):
        self.parametervector = parameters.flatten()
        self.d = self.parametervector[0]
        self.theta = self.parametervector[1]
        self.a = self.parametervector[2]
        self.alpha = self.parametervector[3]
        
    def Set(self,value,name):
    # Set a DH parameter
    # It is slow, avoid its usage! [d,theta,a,alpha]
        if name == 'd':
            self.parametervector[0] = value
        elif name == 'theta':
            self.parametervector[1] = value
        elif name == 'a':
            self.parametervector[2] = value
        elif name == 'alpha':
            self.parametervector[3] = value    
    
    def Setparam(self,x,flags):
    # Set a subset of parameters based on the flags to values that are
    # stored in x variable as
    # flags: 4x1
    # x: sum(flags)x1   
        self.parametervector[flags == 1] = x

    def Getparam(self,flags):
    # Get a subset of parameters based on the flags
    # flags: 4x1
    # out: sum(flags)x1  
        return np.reshape(self.parametervector[flags == 1],(1,-1)) 
        
    def T0(self):
        sa = np.sin(self.alpha)
        ca = np.cos(self.alpha)
        st = np.sin(self.theta)
        ct = np.cos(self.theta)
        return np.array([[ct, -st*ca, st*sa, self.a*ct],[st, ct*ca, -ct*sa, self.a*st],[0, sa, ca, self.d],[0,0,0,1]])      


class TLink():
#Class that implements the spacialities of translational joints to the Link class 
    def __init__(self,link):
        if type(link) == type(Link(np.array([0,0,0,0]))):
            self.parametervector = link.parametervector
            self.d = link.d
            self.theta = link.theta
            self.a = link.a
            self.alpha = link.alpha
        elif type(link) == type(np.array([])):
            self.parametervector = link.flatten()
            self.d = self.parametervector[0]
            self.theta = self.parametervector[1]
            self.a = self.parametervector[2]
            self.alpha = self.parametervector[3]             
        
    def T(self,q):
    # Returns the transformation matrices for given joint coordinates
    # q: 1xN
    # out: 4x4xN
        self.d = self.parametervector[0]
        self.theta = self.parametervector[1]
        self.a = self.parametervector[2]
        self.alpha = self.parametervector[3]
        q = np.reshape(q,(1,-1))
        sa = np.sin(self.alpha)
        ca = np.cos(self.alpha)
        st = np.sin(self.theta)
        ct = np.cos(self.theta)
        N = q.shape[1]
        on = np.ones((1,N))
        ze = np.zeros((1,N))
        T = np.array([[ct*on, -st*ca*on, st*sa*on, self.a*ct*on],[st*on, ct*ca*on, -ct*sa*on, self.a*st*on],[ze, sa*on, ca*on, self.d+q],[ze,ze,ze,on]])
        T = np.reshape(T,(4,4,N))
        return T
        
        
class RLink():
#Class that implements the spacialities of translational joints to the Link class 
    def __init__(self,link):
        if type(link) == type(Link(np.array([0,0,0,0]))):
            self.parametervector = link.parametervector
            self.d = link.d
            self.theta = link.theta
            self.a = link.a
            self.alpha = link.alpha
        elif type(link) == type(np.array([])):
            self.parametervector = link
            self.d = self.parametervector[0]
            self.theta = self.parametervector[1]
            self.a = self.parametervector[2]
            self.alpha = self.parametervector[3]    
        
    def T(self,q):
    # Returns the transformation matrices for given joint coordinates
    # q: 1xN
    # out: 4x4xN
        self.d = self.parametervector[0]
        self.theta = self.parametervector[1]
        self.a = self.parametervector[2]
        self.alpha = self.parametervector[3]
        q = np.reshape(q,(1,-1))
        sa = np.sin(self.alpha)
        ca = np.cos(self.alpha)
        st = np.sin(self.theta + q)
        ct = np.cos(self.theta + q)
        N = q.shape[1]
        on = np.ones((1,N))
        ze = np.zeros((1,N))
        T = np.array([[ct, -st*ca, st*sa, self.a*ct],[st, ct*ca, -ct*sa, self.a*st],[ze, sa*on, ca*on, self.d*on],[ze,ze,ze,on]])
        T = np.reshape(T,(4,4,N))
        return T        

class SerialManipulator():
# Class for robot kinematics with serial structure taking into account
# the computation requirements of calibration process
    def __init__(self,data):
        if (type(data[0]) == type(RLink(np.array([0,0,0,0]))) or type(data[0]) == type(TLink(np.array([0,0,0,0])))):
            self.data = data
            self.K = len(data)
            self.parametervector = np.zeros((len(data)*4))
            for i in range(len(data)):
                self.parametervector[i*4:(i+1)*4] = data[i].parametervector
        elif type(data[0]) == type(np.array([])):
            self.parametervector = data[0].flatten()
            self.K = len(data[1])
            self.data = [None] * len(data[1])
            for i in range(self.K):
                if data[1][i] == 'R':
                    self.data[i] = RLink(self.parametervector[i*4:(i+1)*4])
                if data[1][i] == 'T':
                    self.data[i] = TLink(self.parametervector[i*4:(i+1)*4])
    
    def GetParam(self,flags):
    # Get the parameters of the links noted in the input flags
    # flags: 4*K x 1
    # out: 1 x sum(flags)
        flags = np.repeat(flags, 4, axis = 0).flatten('F')        
        return self.parametervector[flags == 1]
        
    def SetParam(self,x,flags):
    # Set the parameters of the links noted in the input flags
    # flags: 4*K x 1
    # x: 1 x sum(flags)
        flags = np.repeat(flags, 4, axis = 0).flatten('F')        
        self.parametervector[flags == 1] = x
        for i in range(int(flags.shape[0]/4)):
            self.data[i].parametervector = x[i*4:(i+1)*4]      
        
    def AddLink(self,link):
    # Add a link to the structure
        self.data.append(link)
        self.parametervector = np.append(self.parametervector,link.parametervector.flatten())    
        self.K += 1

    def Prune(self,i):
    # Remove on link from the structure
        self.data.pop(i)
        self.parametervector = np.append(self.parametervector[:i*4],self.parametervector[i*4+4:])   
        self.K -= 1   
        
    def subSerialManipulator(self,i,j):
    # Returns a serial manipulator object from the i-th to j-th links example(2-3-4-5) 
        return SerialManipulator(self.data[i:j]) 
    
    def T(self,q):
    # Returns the transformation matrices for given joint coordinates
    # q: KxN
    # out: 4x4xN
        N = q.shape[1]
        if self.K == 0:
            out = np.zeros((4,4,N))
            for i in range(N):
                out[:,:,i] = np.eye(4) 
        else:
            out = self.data[0].T(q[0,:])
            for i in range(1,self.K):
                out = prodHTs(out,self.data[i].T(q[i,:]))
        return out  