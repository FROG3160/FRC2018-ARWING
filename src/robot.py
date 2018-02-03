#!/usr/bin/env python3
"""
Main code for Robot
"""

import wpilib

__author__ = "F.R.O.G. - Team 3160"
__copyright__ = "Copyright 2018, Grand Lake Area Robotics Education"
__credits__ = []
__license__ = "MIT"
__version__ = "1.0.0"
__maintainer__ = "F.R.O.G. - Team 3160"
__email__ = "frog3160web@gmail.com"
__status__ = "Development"

class Robot(wpilib.IterativeRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        pass

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        pass

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        pass

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        pass


if __name__ == "__main__":
    wpilib.run(Robot)