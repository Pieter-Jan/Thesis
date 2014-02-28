#!/usr/bin/env python
"""
Created on Thu Nov 29 10:19:42 2012

@author: Jonas
"""
from numpy import *
import socket
import marshal
import matplotlib.pyplot as plt
import OncillaVisualization as OV
import pygame
from pygame.locals import *

XDIM = 900
YDIM = 400
WINSIZE = [XDIM, YDIM]

class OncillaHardware:
    """ ON WHERE TO FIND THE DATA IN THE COMMAND
        command[ FRONT_LEFT ][ HIP ] = angle_in_degrees """
    HIP = 0
    KNEE = 1
    SERVO = 2

    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    HIND_LEFT = 3
    HIND_RIGHT = 4
  
    SIMULATION = False
    screen = None
    showSupportPolygon = False
    swingLeg = 1
 
    """ This connects to a robot-server, found on IP:PORT
     The robot's position will start on the position corresponding to the command {1:[0,135,0],2:[0,135,0],3:[0,135,0],4:[0,135,0]}"""
    def __init__(self, simulation=False, IP = '157.193.205.249', PORT=31708):
      if simulation == False:
        self.closed = False
        self.BUFFER_SIZE = 4096 
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((IP, PORT))
      else:
        self.SIMULATION = True
        print 'Running Oncilla in simulation mode' 
        
        pygame.init()
        self.screen = pygame.display.set_mode(WINSIZE) 

    def close(self):
        if self.closed:
            return
        self.s.close()
        self.closed = True
        print "Connection to robot closed"

    def __del__(self):
        self.close()

    def sendCommand(self, angles):
        d = marshal.dumps(self._anglesToFloat(angles))
        self.s.send(d)
        d = self.s.recv(self.BUFFER_SIZE)
        data = marshal.loads(d)
        return data

    def sendConfiguration(self, q):
      if self.SIMULATION == False:
        # q: configuration in radians
        data = self.sendCommand({1:[q.item(0)*180.0/math.pi, q.item(1)*180.0/math.pi, q.item(2)*180.0/math.pi-90.0], 
                               2:[q.item(3)*180.0/math.pi, q.item(4)*180.0/math.pi, q.item(5)*180.0/math.pi-90.0], 
                               3:[q.item(6)*180.0/math.pi, q.item(7)*180.0/math.pi, q.item(8)*180.0/math.pi-90.0], 
                               4:[q.item(9)*180.0/math.pi, q.item(10)*180.0/math.pi, q.item(11)*180.0/math.pi-90.0]}) 
        return data

      else:
        OV.VisualizeRobot(self, q, swingLeg = self.swingLeg, showSupportPolygon=self.showSupportPolygon)

    def getConfiguration(self):
      # Get q
      # TODO
      q = None
      return q

    def _anglesToFloat(self,angles):
        for leg in angles:
            for i in xrange(3):
                if isinstance(angles[leg][i], generic):
                    angles[leg][i] = asscalar(angles[leg][i])
        return angles
