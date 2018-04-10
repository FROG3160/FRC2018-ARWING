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
from wpilib.sendablechooser import SendableChooser
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
        
        self.sendableChooser()
        
        
        
    


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
        #self.autonomous.testMove(self.autonomous.WALL_TO_SCALE, -1, False)
        #self.autonomous.testAngle(-90, -1)
        #self.elevator.setElevatorPosition(self.elevator.kScale)
       
        #self.autonomous.start()
        self.autonomous.run()
        #self.elevator.setElevatorPosition(-20000)
        
        #self.autonomous.telemetry()
        
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
        #self.elevator.telemetry()
          
        self.climber.climberFunction()
        


    def testInit(self):
        pass

    def testPeriodic(self):
        wpilib.LiveWindow.setEnabled(True)
        pass
    

    
    def sendableChooser(self):
        self.startingChooser = SendableChooser()
        self.startingChooser.addDefault('Move Forward Only', '!')
        self.startingChooser.addObject('Starting Left', 'L')
        self.startingChooser.addObject('Starting Middle', 'M')
        self.startingChooser.addObject('Starting Right', 'R')
        
        wpilib.SmartDashboard.putData('Starting Side', self.startingChooser)
        
        self.startingDelayChooser = SendableChooser()
        self.startingDelayChooser.addDefault('0', 0)
        self.startingDelayChooser.addObject('1', 1)
        self.startingDelayChooser.addObject('2', 2)
        self.startingDelayChooser.addObject('3', 3)
        self.startingDelayChooser.addObject('4', 4)
        self.startingDelayChooser.addObject('5', 5)
        self.startingDelayChooser.addObject('6', 6)
        self.startingDelayChooser.addObject('7', 7)
        self.startingDelayChooser.addObject('8', 8)
        self.startingDelayChooser.addObject('9', 9)
        self.startingDelayChooser.addObject('10', 10)
        self.startingDelayChooser.addObject('11', 11)
        self.startingDelayChooser.addObject('12', 12)
        self.startingDelayChooser.addObject('13', 13)
        self.startingDelayChooser.addObject('14', 14)
        self.startingDelayChooser.addObject('15', 15)
        
        wpilib.SmartDashboard.putData('Delay Time(sec)', self.startingDelayChooser)
        
        self.switchOrScale = SendableChooser()
        self.switchOrScale.addDefault('Switch', 'Switch')
        self.switchOrScale.addObject('Scale', 'Scale')
        
        wpilib.SmartDashboard.putData('Switch or Scale', self.switchOrScale)

if __name__ == "__main__":
    wpilib.run(Robot)
