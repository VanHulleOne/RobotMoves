# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 13:32:57 2016

@author: lvanhulle
"""
import numpy as np

rotMat = 0
W = 0
X = 1
Y = 2
Z = 3

CW = -1
CCW = 1

class Quat:
    def __init__(self, w, x=None, y=None, z=None):
        self.PRECISION = 7
        try:
            self.w, self.x, self.y, self.z = w
        except Exception:
            self.w = w
            self.x = x
            self.y = y
            self.z = z

    def _key(self):
        return self.w, self.x, self.y, self.z

    def __iter__(self):
        return (i for i in self._key())
            
    def rotate(self, axis, deg):
        """
        Rotation quaternion creation from here:
        http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-17-quaternions/
        """
        axis = 'wxyz'.index(axis)
        rad = -deg/360.0*np.pi
        quat = [0]*4
        quat[W] = np.cos(rad)
        quat[axis] = np.sin(rad)
        return Quat(quat)*self
    
    def __mul__(self, other):
        """
        Multiplication from here:
        http://www.cprogramming.com/tutorial/3d/quaternions.html
        """
        quat = [0]*4
        quat[W] = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        quat[X] = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        quat[Y] = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        quat[Z] = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
        return Quat(quat)
    
    def __str__(self):
        return '[' + ', '.join('{:0.7g}'.format(i) for i in self) + ']'
    
    def __repr__(self):
        return 'Quat(' + ', '.join('{:0.7g}'.format(i) for i in self) + ')'

def circle(*, centerX, centerY, centerZ=0, radius, numPoints=20, dir_, startAngle):
    
    startRad = startAngle/360.0*np.pi*2
    for i in range(numPoints):
        radians = (2*np.pi/numPoints)*i*dir_*-1 + startRad
        yield centerX + radius*np.cos(radians), centerY + -radius*np.sin(radians), centerZ
        
def linearMove(endPoint, quat, config, speed):
    tool = 'tNozzle'
    workObj = 'wobjPlatform'

    return ('\t\tMoveL [[' +
            ', '.join(['{:0.3f}'.format(i) for i in endPoint]) + '], ' + # X, Y, Z
            str(quat) + ', ' + # Quaternion
            str(config) + # Configuration
            ', [' + ', '.join(['9E+09']*6) + ']], ' + # List of 9E+09 which are robot default for ignore
            'v{:.0f}, z0, '.format(speed) + # Speed
            tool + ', \\Wobj := ' + workObj + ';\n') # Tool and work object 

def circle_quat(numPoints=20, quat=None):
    if quat is None:
        quat = Quat(0.707107, -0.707107, 0, 0)
    for i in range((numPoints+1)*2):
#        yield quat
        yield quat.rotate('z', -360.0/numPoints*i)
        
def move_robot_circle(*, centerX, centerY, centerZ=0, radius, numPoints=20):
    config = [-1,-1,-2,0]
    for point, quat in zip(circle(centerX=centerX, centerY=centerY, centerZ=centerZ,
                                  radius=radius, numPoints=numPoints),
                           circle_quat(numPoints)):
        yield linearMove(point, quat, config, 30)

#with open('circle.txt', 'w') as outfile:    
#    for move in move_robot_circle(centerX=95, centerY=-10, centerZ=90, radius=75):
#        print(move, end='')
#        outfile.write(move)
    

    
def mat2quat(m):
    q1 = np.sqrt(m[0,0] + m[1,1] + m[2,2] + 1)/2
    q2 = (np.sqrt(m[0,0] - m[1,1] - m[2,2] + 1)/2) * np.sign(m[2,1] - m[1,2])
    q3 = (np.sqrt(m[1,1] - m[0,0] - m[2,2] + 1)/2) * np.sign(m[0,2] - m[2,0])
    q4 = (np.sqrt(m[2,2] - m[0,0] - m[1,1] + 1)/2) * np.sign(m[1,0] - m[0,1])
    q = (q1, q2, q3, q4)
    r = sum(i**2 for i in q)
    return q, r
    
def quat2mat(q):
    m1 = np.array([[q[W], q[Z], -q[Y], q[X]],
                   [-q[Z], q[W], q[X], q[Y]],
                   [q[Y], -q[X], q[W], q[Z]],
                   [-q[X], -q[Y], -q[Z], q[W]]])

    m2 = np.array([[q[W], q[Z], -q[Y], -q[X]],
                   [-q[Z], q[W], q[X], -q[Y]],
                   [q[Y], -q[X], q[W], -q[Z]],
                   [q[X], q[Y], q[Z], q[W]]])

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
    quat[W] = np.cos(fRad)
    quat[axis] = np.sin(fRad)
    return Quat(quat)

    

       
        
        
        
        
        
        