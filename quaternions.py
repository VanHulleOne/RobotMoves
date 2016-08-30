# -*- coding: utf-8 -*-
"""
Created on Mon Aug 15 13:32:57 2016

@author: lvanhulle
"""
import numpy as np
from collections import namedtuple

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
            
    def rotate_deg(self, axis, deg):
        rad = deg/360.0*2*np.pi
        return self.rotate_rad(axis, rad)

    def rotate_rad(self, axis, rad):
        """
        Rotation quaternion creation from here:
        http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-17-quaternions/
        """
#        rad *= -1
        axis = 'wxyz'.index(axis)
        quat = [0]*4
        quat[W] = np.cos(rad/2)
        quat[axis] = np.sin(rad/2)
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
        return '[' + ', '.join('{:0.7g}'.format(round(i,self.PRECISION)) for i in self) + ']'
    
    def __repr__(self):
        return 'Quat(' + ', '.join('{:0.7g}'.format(i) for i in self) + ')'
        
def moveL(endPoint, quat, config, speed):
    tool = 'tNozzle'
    workObj = 'wobjPlatform'

    return ('\t\tMoveL [[' +
            ', '.join(['{:0.3f}'.format(i) for i in endPoint]) + '], ' + # X, Y, Z
            str(quat) + ', ' + # Quaternion
            str(list(config)) + # Configuration
            ', [' + ', '.join(['9E+09']*6) + ']], ' + # List of 9E+09 which are robot default for ignore
            'v{:.0f}, z0, '.format(speed) + # Speed
            tool + ', \\Wobj := ' + workObj + ';\n') # Tool and work object 

def moveJ(point, quat, config, speed):
    tool = 'tNozzle'
    workObj = 'wobjPlatform'
    
    return ('\t\tMoveJ [[' +
        ', '.join(['{:0.3f}'.format(i) for i in point]) + '], ' + # X, Y, Z
        str(quat) + ', ' + # Quaternion
        str(list(config)) + # Configuration
        ', [' + ', '.join(['9E+09']*6) + ']], ' + # List of 9E+09 which are robot default for ignore
        'v{:.0f}, z0, '.format(speed) + # Speed
        tool + ', \\Wobj := ' + workObj + ';\n') # Tool and work object 

def circle(*, centerX, centerY, centerZ=0, radius, numPoints, dir_, startAngle):    
    startRad = startAngle/360.0*np.pi*2
    for i in range(numPoints):
        radians = (2*np.pi/numPoints)*i*dir_ + startRad
        yield centerX + radius*np.cos(radians), centerY + radius*np.sin(radians), centerZ

def circle_quat(*, numPoints, dir_, startAngle):
    startRad = startAngle/360.0*2*np.pi
    quat = Quat(0.5, -0.5, -0.5, 0.5) # Side 3 up is our reference zero
    step = np.pi*2/numPoints
    for i in range(numPoints):
        yield quat.rotate_rad('z', startRad+i*step*dir_)

def reorient():
    yield '\t\tjRepo := CJointT();\n'
    yield '\t\tjRepo.robax.rax_3 := jRepo.robax.rax_3 - 60;\n'
    yield '\t\tMoveAbsJ jRepo, v300, z10, tNozzle\WObj:=wobjPlatform;\n'
    yield '\t\tjRepo.robax.rax_4 := jRepo.robax.rax_4 + 180;\n'
    yield '\t\tjRepo.robax.rax_5 := jRepo.robax.rax_5 + 120;\n'
    yield '\t\tjRepo.robax.rax_6 := jRepo.robax.rax_6 + 180;\n'
        
    yield '\t\tMoveAbsJ jRepo, v300, z10, tNozzle\WObj:=wobjPlatform;\n'
    yield '\t\tjRepo.robax.rax_3 := jRepo.robax.rax_3 + 60;\n'
    yield '\t\tMoveAbsJ jRepo, v300, z10, tNozzle\WObj:=wobjPlatform;\n'
        
def move_circle(*, centerX, centerY, centerZ, radius, numPoints, dir_, startAngle):
    START_RAD = startAngle/360.0*2*np.pi
    STEP_RAD = np.pi*2/numPoints
    REF_QUAT = Quat(0.5, -0.5, -0.5, 0.5) # Side 3 up is our reference zero
    A4_CONFIGS = [2, 1, 0, -1, -2]
    first = True
    for i in range(numPoints//2):
        currAngle = START_RAD + i*STEP_RAD*dir_
        point = [centerX + radius*np.cos(currAngle), centerY + radius*np.sin(currAngle), centerZ]
        quat = REF_QUAT.rotate_rad('z', currAngle)
        config = [-1, A4_CONFIGS[int(i/((numPoints/2)/len(A4_CONFIGS)))], 0, 1]
        if first:
            first = False
            yield moveJ([point[0], point[1], point[2]+75], quat, config, 300)
        yield moveL(point, quat, config, 30)

    point[2] += 75
    yield moveL(point, quat, config, 30)         
     
    yield from reorient()
    first = True
    for i in range(numPoints//2, numPoints):
        currAngle = START_RAD + i*STEP_RAD*dir_
        point = [centerX + radius*np.cos(currAngle), centerY + radius*np.sin(currAngle), centerZ]
        quat = REF_QUAT.rotate_rad('z', currAngle)
        config = [-1, A4_CONFIGS[int((i-numPoints//2)/((numPoints/2)/len(A4_CONFIGS)))], 3, 0]
        if first:
            first = False
            yield moveJ([point[0], point[1], point[2]+75], quat, config, 300)
        yield moveL(point, quat, config, 30)
        
        
def move_robot_circle(*, centerX, centerY, centerZ=0, radius, numPoints, dir_, startAngle):
    Config = namedtuple('Config', 'axis1 axis4 axis6 axisX')
    _4up = Config(-1, 0, 2, 0)
    _2up = Config(-1, 0, 0, 1)
    _1up1st = Config(-1, 2, 3, 0)
    _1up2nd = Config(-1, -2, 0, 1)
    _3up1st = Config(-1, -2, 2, 0)
    _3up2nd = Config(-1, 2, 0, 1)
    configs = [[_1up1st, _4up, _3up1st], [_3up2nd, _2up, _1up2nd]]
    locs = list(zip(circle(centerX=centerX,
                                  centerY=centerY,
                                  centerZ=centerZ,
                                  radius=radius,
                                  numPoints=numPoints,
                                  dir_=dir_,
                                  startAngle=startAngle),
                           circle_quat(numPoints=numPoints,
                                       dir_=dir_,
                                       startAngle=startAngle)))

    for x, (point, quat) in enumerate(locs[:len(locs)//2]):
        yield moveL(point, quat, configs[1][x//(len(locs)//(2*3))], 30)
        
    yield '\nDo something for reposition\n'
    
    for x, (point, quat) in enumerate(locs[len(locs)//2:]):
        yield moveL(point, quat, configs[0][x//(len(locs)//(2*3))], 30)

#with open('circle.txt', 'w') as outfile:    
for move in move_robot_circle(centerX=100, centerY=50, centerZ=10, radius=37, numPoints=24, dir_=CCW, startAngle=0):
    print(move, end='')
    
print('\n\nSecond Batch\n\n')
for move in move_circle(centerX=100, centerY=50, centerZ=10, radius=37, numPoints=24, dir_=CCW, startAngle=0):
    print(move, end='')
#    outfile.write(move)
    

    
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

    

       
        
        
        
        
        
        