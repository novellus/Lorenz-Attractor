# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 15:40:08 2014

@author: Nick Walker; Wesley Gardner
"""

import OpenGL.GLUT as GLUT
import OpenGL.GL as GL
import OpenGL.GLU as GLU
import numpy as np
import time as t
import sys

def getVector(vector_a, vector_b):
    #the difference between two vectors
    return np.array(vector_b) - np.array(vector_a)

def getVectorMagnitude(vector_a, vector_b):
    # r = sqrt(x^2 + y^2 + z^2)
    return np.sqrt((vector_b[0] - vector_a[0])**2 + (vector_b[1] - vector_a[1])**2 + (vector_b[2] - vector_a[2])**2)

def rotationMatrix(axis,theta):
    #the euler-rodrigues formula
    axis = axis / np.sqrt(np.dot(axis,axis))
    a = np.cos(theta / 2)
    b, c, d = - axis * np.sin(theta/2)
    return np.array([[a * a + b * b - c * c - d * d, 2 * (b * c - a * d), 2 * (b * d + a * c)], [2 * (b * c + a * d), a * a + c * c - b * b - d * d, 2 * (c * d - a * b)], [2 * (b * d - a * c), 2 * (c * d + a * b), a * a + d * d - b * b - c * c]])

class LorenzGenerator:
	x = 1
	y = 1
	z = 1
	sigma = 10
	rho = 28
	beta = 8.0/3
	dt = .01

	def __init__(self, sigma = 10, rho = 28, beta = 8.0/3):
		self.sigma = sigma
		self.rho = rho
		self.beta = beta

	def start(self,x0 = 1, y0 = 1, z0 = 1, timeStep = 0.01):
		self.x = x0
		self.y = y0
		self.z = z0
		self.dt = timeStep

	def setTimestep(self, timeStep):
		self.dt = timeStep

	def getNextPoint(self):
		dx = (self.sigma*(self.y - self.x)) * self.dt
		dy=(self.x*(self.rho-self.z) - self.y) * self.dt
		dz=(self.x * self.y - self.beta * self.z) * self.dt
		self.x += dx
		self.y += dy
		self.z += dz
		return (self.x, self.y, self.z)

class Camera:
    """A Class for Managing a Camera"""

    def __init__(self, camera = [10.0, 0.0, 5.0], target = [0.0, 0.0, 0.0], up = [0.0, 0.0, 1.0]):
        """Takes Camera Position, Target Position, Transverse Axis, and Maximum Range as Inputs"""
        self._camera = np.array(camera)
        self._target = np.array(target)
        self._up = np.array(up)
        self._fv = (np.array(target) - np.array(camera)) / getVectorMagnitude(np.array(target), np.array(camera))
        self._transverse = np.cross(self._up, self._fv)

    def calcTargetVector(self):
        """Calculates the location of the Target point"""
        self._target = self._camera + self._fv

    def calcUpVector(self):
        """Calculates the unit vector of the transverse vector of the Camera"""
        self._up = np.cross(self._transverse, self._fv)

    def calcTransverseVector(self):
        """Calculates the unit vector of the transverse vector of the Camera"""
        self._transverse = np.cross(self._up, self._fv)

    def getCamera(self):
        """Returns the Position of the Camera"""
        return self._camera

    def getTarget(self):
        """Returns the Position of the Target"""
        return self._target

    def getUp(self):
        """Returns the Position of the Up vector"""
        return self._up

    def getTransverse(self):
        """Returns the Transverse Axis"""
        return self._transverse

    def getFacingVector(self):
        """Returns the unit vector of the Camera facing"""
        return self._fv

    def setCamera(self, camera):
        """Sets the Camera Position"""
        self._camera = camera

    def setTarget(self, target):
        """Sets the Target Position"""
        self._target = target

    def forward(self):
        """Moves the Camera Forward"""
        step = .5 * self._fv
        self._target = self._target + step
        self._camera = self._camera + step
        GLUT.glutPostRedisplay()

    def backward(self):
        """Moves the Camera Backward"""
        step = .5 * self._fv
        self._target = self._target - step
        self._camera = self._camera - step
        GLUT.glutPostRedisplay()

    def up(self):
        """Moves the Camera Facing Up"""
        self._fv = np.dot(rotationMatrix(self._transverse, -2 * np.pi / 180.), np.array(self._fv))
        self.calcTargetVector()
        #self.calcUpVector()
        GLUT.glutPostRedisplay()

    def down(self):
        """Moves the Camera Facing Down"""
        self._fv = np.dot(rotationMatrix(self._transverse, 2 * np.pi / 180.), np.array(self._fv))
        self.calcTargetVector()
        #self.calcUpVector()
        GLUT.glutPostRedisplay()

    def left(self):
        """Moves the Camera Facing Left"""
        self._fv = np.dot(rotationMatrix(self._up, -2 * np.pi / 180.), np.array(self._fv))
        self._target = self._camera + self._fv
        self.calcTransverseVector()
        GLUT.glutPostRedisplay()

    def right(self):
        """Moves the Camera Facing Right"""
        self._fv = np.dot(rotationMatrix(self._up, 2 * np.pi / 180.), np.array(self._fv))
        self._target = self._camera + self._fv
        self.calcTransverseVector()
        GLUT.glutPostRedisplay()

    def facing(self, x, y, w, h):
        """Changes the Facing of the Camera Based on the Position of the Mouse"""
        ax = 0.0
        ay = 0.0
        if x > 2 * w / 3.0:
            ax = (x - w / 2.0) / (w / 2.0)
        if x < 1 * w / 3.0:
            ax = - (w / 2.0 - x) / (w / 2.0)
        if y > 2 * h / 3.0:
            ay = (y - h / 2.0) / (h / 2.0)
        if y < 1 * h / 3.0:
            ay = - (h / 2.0 - y) / (h / 2.0)
        self._fv = np.dot(rotationMatrix(self._up, ax * np.pi / 180.), np.array(self._fv))
        self._fv = np.dot(rotationMatrix(self._transverse, ay * np.pi / 180.), np.array(self._fv))
        self._target = self._camera + self._fv
        self.calcTransverseVector()
        GLUT.glutPostRedisplay()

p = LorenzGenerator()
p.start()

P = []

start = t.time()
frames = 0.0

#the position of the camera is set at the edge of the system
camera = [75.0, -75.0, 75.0]
#the point the camera is looking at is set to the center of the system
target = [0.0, 0.0, 0.0]
#the up vector determines what is up
up = [0.0, 0.0, 1.0]
#the maximum viewing range
frange = 1000.0
#ambient light vector
lambient =  [0.2, 0.2, 0.2, 1.0]
#diffuse light vector
ldiffuse =  [0.5, 0.5, 0.5, 1.0]
#specular light vector
lspecular =  [0.5, 0.5, 0.5, 1.0]
#light model ambient vector
lmodelambient = [0.2, 0.2, 0.2, 1.0]
#light position is set a little bit up the z axis
lposition =  [0.0, 0.0, 0.0, 0.0]

fx = 256
fy = 256

#initialize our camera, which is a ship, because we're in space
cam = Camera(camera, target, up)

#this makes it so everything starts static
running = False

#idle function to update all object positions and camera facing
def Calculations():
    global objects
    global test
    global frames
    global start
    global p
    global P
    if running:
        P.append(p.getNextPoint())
    w = GLUT.glutGet(GLUT.GLUT_WINDOW_WIDTH)
    h = GLUT.glutGet(GLUT.GLUT_WINDOW_HEIGHT)
    cam.facing(fx, fy, w, h)
    GLUT.glutPostRedisplay()
    frames = frames + 1
    if frames > 100:
        elapsed = t.time() - start
        sys.stdout.write('\r')
        sys.stdout.write(str(frames / elapsed))
        sys.stdout.flush()
        frames = 0
        start = t.time()

#calls to move the ship, used by mouse function
def forward():
    Calculations()
    cam.forward()

def backward():
    Calculations()
    cam.backward()

#passive mouse function that gets the mouse position when it is moved
def motion(x, y):
    global fx
    global fy
    fx = x
    fy = y

def init():
    #lighting is setup
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, lambient)
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, ldiffuse)
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_SPECULAR, lspecular)
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, lposition)
    GL.glLightModelfv(GL.GL_LIGHT_MODEL_AMBIENT, lmodelambient)
    GL.glEnable(GL.GL_LIGHTING)
    GL.glEnable(GL.GL_LIGHT0)
    GL.glEnable(GL.GL_DEPTH_TEST)
    #display list for the axes and the orbitals
    GL.glNewList(1, GL.GL_COMPILE)
    GL.glPushMatrix()
    GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
    GL.glMaterialfv(GL.GL_FRONT, GL.GL_AMBIENT, [1.0, 1.0, 1.0, 1.0])
    GL.glMaterialfv(GL.GL_FRONT, GL.GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
    GL.glMaterialfv(GL.GL_FRONT, GL.GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
    GL.glMaterialf(GL.GL_FRONT, GL.GL_SHININESS, .5)
    GL.glBegin(GL.GL_LINES)
    GL.glVertex3f(0.0, 0.0, 0.0)
    GL.glVertex3f(50.0, 0.0, 0.0)
    GL.glEnd()
    GL.glBegin(GL.GL_LINES)
    GL.glVertex3f(0.0, 0.0, 0.0)
    GL.glVertex3f(0.0, 50.0, 0.0)
    GL.glEnd()
    GL.glBegin(GL.GL_LINES)
    GL.glVertex3f(0.0, 0.0, 0.0)
    GL.glVertex3f(0.0, 0.0, 50.0)
    GL.glEnd()
    GL.glPopMatrix()
    GL.glEndList()

def display():
    GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
    GL.glLoadIdentity()
    #the camera is located where it is and is facing the target with the positive z axis up
    GLU.gluLookAt(cam.getCamera()[0], cam.getCamera()[1], cam.getCamera()[2], cam.getTarget()[0], cam.getTarget()[1], cam.getTarget()[2], cam.getUp()[0], cam.getUp()[1], cam.getUp()[2])
    GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, lposition)
    #the display list is called
    GL.glCallList(1)
    GL.glPushMatrix()
    GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
    GL.glMaterialfv(GL.GL_FRONT, GL.GL_AMBIENT, [0.0, 1.0, 0.0, 1.0])
    GL.glMaterialfv(GL.GL_FRONT, GL.GL_DIFFUSE, [1.0, 1.0, 1.0, 1.0])
    GL.glMaterialfv(GL.GL_FRONT, GL.GL_SPECULAR, [1.0, 1.0, 1.0, 1.0])
    GL.glMaterialf(GL.GL_FRONT, GL.GL_SHININESS, .5)
    #GL.glBegin(GL.GL_LINE_LOOP)
    #GL.glBegin(GL.GL_LINES)
    GL.glBegin(GL.GL_LINE_STRIP)
    for x, y, z in P:
        GL.glVertex3f(x, y, z)
    GL.glEnd()
    GL.glPopMatrix()
    GLUT.glutSwapBuffers()

#modifies the perspective for when the window size changes
def reshape(w, h):
    GL.glViewport(0, 0, w, h)
    GL.glMatrixMode(GL.GL_PROJECTION)
    GL.glLoadIdentity()
    GLU.gluPerspective(60.0, float(w)/h, .01, 1000.0)
    GL.glMatrixMode(GL.GL_MODELVIEW)
    GL.glLoadIdentity()

#keyboard commands
def keyboard(key, x, y):
    global running
    if key == 'w':
        cam.forward()
    elif key == 's':
        cam.backward()
    elif key == 'a':
        cam.left()
    elif key == 'd':
        cam.right()
    elif key == 'q':
        cam.down()
    elif key == 'e':
        cam.up()
    elif key == '':
        sys.exit()
    elif key == 'p':
        running = not running

#the mouse is used to control the camera
def mouse(button, state, x, y):
    if button == GLUT.GLUT_LEFT_BUTTON:
        if (state == GLUT.GLUT_DOWN):
            GLUT.glutIdleFunc(forward)
        elif (state == GLUT.GLUT_UP):
            GLUT.glutIdleFunc(Calculations)
    elif button == GLUT.GLUT_RIGHT_BUTTON:
        if (state == GLUT.GLUT_DOWN):
            GLUT.glutIdleFunc(backward)
        elif (state == GLUT.GLUT_UP):
            GLUT.glutIdleFunc(Calculations)

GLUT.glutInit(sys.argv)
GLUT.glutInitDisplayMode (GLUT.GLUT_DOUBLE | GLUT.GLUT_RGB | GLUT.GLUT_DEPTH)
GLUT.glutInitWindowSize (1024, 576)
GLUT.glutInitWindowPosition (100, 100)
GLUT.glutCreateWindow('Lorentz Attractor')
init()
GLUT.glutDisplayFunc(display)
GLUT.glutReshapeFunc(reshape)
GLUT.glutKeyboardFunc(keyboard)
GLUT.glutMouseFunc(mouse)
GLUT.glutPassiveMotionFunc(motion)
GLUT.glutMotionFunc(motion)
GLUT.glutIdleFunc(Calculations)
GLUT.glutMainLoop()