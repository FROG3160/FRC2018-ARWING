#!/usr/bin/env python3
"""
Main code for Robot
"""

import wpilib
import robotmap
from wpilib import Joystick
from subsystems.drivetrain import DriveTrain as Drive
from subsystems.grabber import Grabber
from subsystems.elevator import Elevator
from subsystems.climber import Climber


class Robot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.pneumaticControlModuleCANID = robotmap.PCM
        self.kDriveTrain = robotmap.DriveTrain
        self.kCubeGrabber = robotmap.CubeGrabber
        self.kElevator = robotmap.Elevator
        self.kSticks = robotmap.Sticks
        self.kClimber = robotmap.Climber
        self.dStick = Joystick(self.kSticks['drive'])
        self.cStick = Joystick(self.kSticks['control'])
        
        self.drive = Drive(self)
        self.cubeGrabber = Grabber(self)
        self.elevator = Elevator(self)
        self.climber = Climber(self)


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
        # get field data
        #self.fielddata = wpilib.DriverStation.getInstance().getGameSpecificMessage()        

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        #nearswitch, scale, farswitch = list(self.fielddata)
#         
#         if nearswitch == 'R':
#             self.drive.arcade(.5, .2)
#         else:
#             self.drive.arcade(.5, -.2)
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        speed = self.dStick.getY() * -1
        rotation = self.dStick.getTwist()
        # self.drive.moveSpeed(speed, speed)
        
        self.drive.arcadeWithRPM(speed, rotation, 2800)
        
        self.cubeGrabber.grabberFunction()
        
        self.elevator.elevatorFunction()
        
        self.climber.climberFunction()

    def testInit(self):
        pass

    def testPeriodic(self):
        wpilib.LiveWindow.setEnabled(True)
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
