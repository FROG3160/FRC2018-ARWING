#!/usr/bin/env python3
"""
Main code for Robot
"""

import wpilib
import robotmap
from wpilib import Joystick
from subsystems.drivetrain import DriveTrain as Drive


class Robot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        
        self.kDriveTrain = robotmap.DriveTrain
        self.kSticks = robotmap.Sticks
        self.dStick = Joystick(self.kSticks['drive'])
        self.cStick = Joystick(self.kSticks['control'])
        self.drive = Drive(self)
        
        self.fielddata = None


    def robotPeriodic(self):
        pass

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        self.drive.stop()

    def autonomousInit(self):
        """
        This function is run once each time the robot enters autonomous mode.
        """
        self.drive.driveLeftMaster.setQuadraturePosition(0, 0)
        self.drive.driveRightMaster.setQuadraturePosition(0, 0)     

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        
        ## get field data
        #if not self.fielddata:
            #self.fielddata = wpilib.DriverStation.getInstance().getGameSpecificMessage()          
            #nearswitch, scale, farswitch = list(self.fielddata)
        
        #if self.fielddata[0] == 'R':
            #self.drive.arcade(.5, .2)
        #else:
            #self.drive.arcade(.5, -.2)
            
        self.drive.arcade(.5, 0)

    def teleopInit(self):
        self.drive.driveLeftMaster.setQuadraturePosition(0, 0)
        self.drive.driveRightMaster.setQuadraturePosition(0, 0)

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        speed = self.dStick.getY() * -1
        rotation = self.dStick.getTwist()
        # self.drive.moveSpeed(speed, speed)
        self.drive.arcade(speed, rotation)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
