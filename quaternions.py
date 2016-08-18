# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 13:32:57 2016

@author: lvanhulle
"""
import numpy as np

rotMat = 0
w = 0
x = 1
y = 2
z = 3
    
def mat2quat(m):
    q1 = np.sqrt(m[0,0] + m[1,1] + m[2,2] + 1)/2
    q2 = (np.sqrt(m[0,0] - m[1,1] - m[2,2] + 1)/2) * np.sign(m[2,1] - m[1,2])
    q3 = (np.sqrt(m[1,1] - m[0,0] - m[2,2] + 1)/2) * np.sign(m[0,2] - m[2,0])
    q4 = (np.sqrt(m[2,2] - m[0,0] - m[1,1] + 1)/2) * np.sign(m[1,0] - m[0,1])
    q = (q1, q2, q3, q4)
    r = sum(i**2 for i in q)
    return q, r
    
def quat2mat(q):
    m1 = np.array([[q[w], q[z], -q[y], q[x]],
                   [-q[z], q[w], q[x], q[y]],
                   [q[y], -q[x], q[w], q[z]],
                   [-q[x], -q[y], -q[z], q[w]]])

    m2 = np.array([[q[w], q[z], -q[y], -q[x]],
                   [-q[z], q[w], q[x], -q[y]],
                   [q[y], -q[x], q[w], -q[z]],
                   [q[x], q[y], q[z], q[w]]])

    return np.dot(m1, m2)[:3,:3]
    
def rotate(axis, deg):
    global rotMat
    rad = deg/360.0*2*np.pi
    cos = np.cos(rad)
    sin = np.sin(rad)
    if axis is 'x':
        matrix = np.array([[1, 0, 0],
                           [0, cos, -sin],
                           [0, sin, cos]])
    elif axis is 'y':
        matrix = np.array([[cos, 0, sin],
                           [0, 1, 0],
                           [-sin, 0, cos]])
    elif axis is 'z':
        matrix = np.array([[cos, -sin, 0],
                           [sin, cos, 0],
                           [0, 0, 1]])
    else:
        print('Invalid axis: ', axis)
    
    rotMat = matrix    
    origin = np.array([[1, 0, 0],[0, -1, 0],[0, 0, -1]])
    return mat2quat(matrix.dot(origin))
    
    
def rotation_quat(axis, deg):
    axis = 'wxyz'.index(axis)
    fRad = deg/360.0*np.pi
    quat = [0]*4
    quat[w] = np.cos(fRad)
    quat[axis] = np.sin(fRad)
    return Quat(quat)
    
class Quat:
    def __init__(self, w, x=None, y=None, z=None):
        try:
            self.w, self.x, self.y, self.z = w
        except Exception:
            self.w = w
            self.x = x
            self.y = y
            self.z = z

    def __iter__(self):
        return (i for i in (self.w, self.x, self.y, self.z))
            
    def rotate(self, axis, deg):
        axis = 'wxyz'.index(axis)
        fRad = deg/360.0*np.pi
        quat = [0]*4
        quat[w] = np.cos(fRad)
        quat[axis] = np.sin(fRad)
        return Quat(quat)*self
    
    def __mul__(self, other):
        quat = [0]*4
        quat[w] = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        quat[x] = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        quat[y] = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        quat[z] = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quat(quat)
    
    def __repr__(self):
        return 'Quat(' + ', '.join(str(i) for i in self) + ')'
       
        
        
        
        
        
        