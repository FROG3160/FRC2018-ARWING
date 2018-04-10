'''
Created on Mar 26, 2018

@author: paule
'''
import wpilib
from wpilib.command import Subsystem


class FROGLights(Subsystem):
    #Sends info to Arduino to control lights

    i2c = wpilib.i2c.I2C(wpilib.I2C.Port.kOnboard, 84)
    
    def __init__(self):
        pass
    
    def write(self, value):
        #???self.i2c.transaction
        self.i2c.write(0,value)
        
    def sendDontClimb(self):
        print('sent')
        self.write(0x01)
        
    def sendOKToClimb(self):
        print('sent')
        self.write(0x02)
        
    