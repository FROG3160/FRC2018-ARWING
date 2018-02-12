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
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        self.drive.arcade(.5 , 0)

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        speed = self.dStick.getY() * -1
        rotation = self.dStick.getTwist()
        # self.drive.moveSpeed(speed, speed)
        self.drive.arcade(speed, rotation)

    def testInit(self):
        pass

    def testPeriodic(self):
        wpilib.LiveWindow.setEnabled(True)
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
