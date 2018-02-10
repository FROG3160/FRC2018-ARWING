#!/usr/bin/env python3
"""
Main code for Robot
"""

import wpilib

from wpilib import Joystick
from subsystems.drivetrain import DriveTrain as Drive


class Robot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.drive = Drive()
        self.stick = Joystick(0)

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
        self.drive.moveToPosition(10000, 'left')

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        speed = self.stick.getY() * -1
        rotation = self.stick.getTwist()
        # self.drive.moveSpeed(speed, speed)
        self.drive.arcade(speed, rotation)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
