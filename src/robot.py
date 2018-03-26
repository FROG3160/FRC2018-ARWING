#!/usr/bin/env python3
"""
Main code for Robot
"""

import wpilib
import robotmap
from wpilib import Joystick
from subsystems.drivetrain import DriveTrain as Drive
from subsystems.grabber import cubeGrabber
from subsystems.elevator import Elevator
from subsystems.climber import Climber
from subsystems.autonomous import Autonomous
# from robotpy_ext.common_drivers.navx import AHRS


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
        self.cubeGrabber = cubeGrabber(self)
        self.elevator = Elevator(self)
        self.climber = Climber(self)
        
        
        
    


    def robotPeriodic(self):
        pass

    def disabledInit(self):
        pass

    def disabledPeriodic(self):
        self.drive.stop()

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.autonomous = Autonomous(self)
        self.autonomous.reset()
        self.drive.autoInit()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        #self.autonomous.testMove(36, -1, False)
        #self.autonomous.testAngle(-90, -1)
        #self.elevator.setElevatorPosition(self.elevator.kScale)
       
        #self.autonomous.start()
        self.autonomous.run()
        #self.elevator.setElevatorPosition(-20000)
        
        self.autonomous.telemetry()
        
    def teleopInit(self):
        self.drive.teleInit()

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        speed = (self.dStick.getY() * -1)**3
        rotation = self.dStick.getTwist()/(1.1+self.dStick.getRawAxis(3))
        # self.drive.moveSpeed(speed, speed)
         
        self.drive.arcadeWithRPM(speed, rotation, 2800)
          
        self.cubeGrabber.grabberFunction()
#          
        self.elevator.elevatorFunction()
        self.elevator.telemetry()
          
        self.climber.climberFunction()
        


    def testInit(self):
        pass

    def testPeriodic(self):
        wpilib.LiveWindow.setEnabled(True)
        pass
    


if __name__ == "__main__":
    wpilib.run(Robot)
