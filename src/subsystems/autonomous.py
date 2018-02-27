from wpilib.command.subsystem import Subsystem
import wpilib
import math

class Autonomous(Subsystem):
    
    def __init__(self, robot):
        self.robot = robot

#         self.autoInstructionsFile = open("subsystems/AutoInstructions.txt", "r")
        self.autoInstructions = [0, 'L']
#         self.autoInstructions[index].strip()
#         self.autoInstructionsFile.close()
        
        self.armSolenoid = self.robot.cubeGrabber.armSolenoid
        self.armSolenoid.set(self.robot.cubeGrabber.armClosePosition)
        
        self.driveLeft = self.robot.drive.driveLeftMaster
        self.driveRight = self.robot.drive.driveRightMaster
        
        self.elevator = self.robot.elevator.elevator
        
        self.leftGrab = self.robot.cubeGrabber.leftArm
        self.rightGrab = self.robot.cubeGrabber.rightArm
        
        self.gamedata = "LLL"
        
        self.startingYaw = 0
        
        self.autoStep = 0
        self.chunkStep = 0 
        self.startLocation = self.autoInstructions[1]
        self.delayTime = self.autoInstructions[0]
        
        self.distanceVariables()
        
        

    def magEncoderInchesToTicks(self, inches):
        RADIUS_OF_WHEEL = 3
        CIRCUMFERENCE_OF_WHEEL = RADIUS_OF_WHEEL*2*math.pi
        TICKS_PER_REVOLUTION = 4096
        
        rotations = inches/CIRCUMFERENCE_OF_WHEEL
        
        ticks = rotations*TICKS_PER_REVOLUTION
        
        self.ticks = ticks
        
        return self.ticks
        
    def distanceVariables(self):
        self.ANGLE_TOLERANCE = 2
               
        self.WALL_TO_SWITCH = 228
        self.SWITCH_TO_PLATFORM_ALLEY = 96
        self.PLATFORM_ALLEY_TO_SCALE = 84
        self.WALL_TO_STRAIGHT_SWITCH = 60
        self.ALLEY_TO_CUBE = 24
        self.ALLEY_TO_ALLEY = 270
        self.CUBE_TO_SWITCH = 16
        
        #324
        
        if self.startLocation == 'R':
            self.VERTICAL_ALLEY_TO_CUBE_ANGLE = -135
            self.TURN = -90
            self.SCALE_DUMP_ANGLE = -45
            self.SWITCH_DUMP_ANGLE = -90
        else:
            self.VERTICAL_ALLEY_TO_CUBE_ANGLE = 135
            self.TURN = 90
            self.SCALE_DUMP_ANGLE = 45
            self.SWITCH_DUMP_ANGLE = 90
       
        self.WALL_TO_SCALE = self.WALL_TO_SWITCH + self.SWITCH_TO_PLATFORM_ALLEY + self.PLATFORM_ALLEY_TO_SCALE
        
        self.WALL_TO_PLATFORM_ALLEY = self.WALL_TO_SWITCH + self.SWITCH_TO_PLATFORM_ALLEY
        
    def getGameData(self):
        self.gamedata = wpilib.DriverStation.getInstance().getGameSpecificMessage
      
    def run(self):
        self.telemetry()
        
        if self.autoStep == 0:
            self.start()
        elif self.autoStep == 1:
            self.delay(self.delayTime)
        elif self.autoStep == 2:
            print(self.chunkStep)
            print(self.robot.drive.isInGyroPosition())
            self.autoChooser()
            

    def start(self):
        self.reset()
        self.robot.drive.ahrs.reset()
        
        if self.chunkStep == 0:
            self.resetGyro()
        if self.chunkStep == 1:
            if self.robot.elevator.getBottomLimit():
                self.chunkStep += 1
            else: 
                self.robot.elevator.calibrateBottomAutonomous()
        elif self.chunkStep == 2:
            self.robot.elevator.setElevatorPosition(self.robot.elevator.kStart)
            if self.elevator.getSelectedSensorPosition(0) >= self.robot.elevator.kStart:
                self.chunkStep += 1
        elif self.chunkStep == 3:
            self.robot.elevator.setElevatorPosition(self.robot.elevator.kBottom)
            if self.robot.elevator.getBottomLimit:
                
                self.chunkStep += 1
        elif self.chunkStep == 4:
            self.reset()
            self.chunkStep = 0
            self.autoStep += 1
        
    def autoChooser(self):
        if (self.gamedata == "LRL" and self.startLocation == "L") or (self.gamedata == "RLR" and self.startLocation == "R"):
            self.sameDifferent()
        elif (self.gamedata == "RLR" and self.startLocation == "L") or (self.gamedata == "LRL" and self.startLocation == "R"):
            self.differentSame()
        elif (self.gamedata == "LLL" and self.startLocation == "L") or (self.gamedata == "RRR" and self.startLocation == "R"):
            self.sameSame()
        elif (self.gamedata == "RRR" and self.startLocation == "L") or (self.gamedata == "LLL" and self.startLocation == "R"):
            self.differentDifferent()
            
        
    def sameDifferent(self):
        if self.chunkStep == 0:
            self.autoMove(self.WALL_TO_SWITCH, self.robot.elevator.kSwitch, False)
        elif self.chunkStep == 1:
            self.autoAngle(self.SWITCH_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 2:
            self.dropCube()
        elif self.chunkStep == 3:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 4:
            self.autoMove(self.SWITCH_TO_PLATFORM_ALLEY, self.robot.elevator.kBottom, False)
        elif self.chunkStep == 5:
            self.autoAngle(self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 6:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 7:
            self.autoMove(-self.ALLEY_TO_CUBE, -1, False)
        elif self.chunkStep == 8:
            self.autoAngle(self.TURN, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 9:
            self.autoMove(self.ALLEY_TO_ALLEY, self.robot.elevator.kScale, False)
        elif self.chunkStep == 10:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 11:
            self.autoMove(self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 12:    
            self.autoAngle(-self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 13:
            self.dropCube()
        elif self.chunkStep == 14:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 15:
            self.autoMove(-self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 16:
            self.autoAngle(-self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, self.robot.elevator.kBottom)
        elif self.chunkStep == 17:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 18:
            self.autoMove(-self.ALLEY_TO_CUBE, self.robot.elevator.kScale, False)
        elif self.chunkStep == 19:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 20:
            self.autoMove(self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 21:
            self.autoAngle(-self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 22:
            self.dropCube()
        else:
            self.reset()
            
    def differentSame(self):
        if self.chunkStep == 0:
            self.autoMove(self.WALL_TO_SCALE, self.robot.elevator.kScale, False)
        elif self.chunkStep == 1:
            self.autoAngle(self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 2:
            self.dropCube()
        elif self.chunkStep == 3:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 4:
            self.autoMove(-self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 5:
            self.autoAngle(self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, -self.robot.elevator.kBottom)
        elif self.chunkStep == 6:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 7:
            self.autoMove(-self.ALLEY_TO_CUBE, self.robot.elevator.kScale, False)
        elif self.chunkStep == 8:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 9:
            self.autoMove(self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 10:
            self.autoAngle(self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 11:
            self.dropCube()
        elif self.chunkStep == 12:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 13:
            self.autoMove(-self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 14:
            self.autoAngle(self.TURN, self.ANGLE_TOLERANCE, self.robot.elevator.kBottom)
        elif self.chunkStep == 15:
            self.autoMove(self.ALLEY_TO_ALLEY, -1, False)
        elif self.chunkStep == 16:
            self.autoAngle(-self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 17:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 18:
            self.autoMove(0, self.robot.elevator.kSwitch, False)
        elif self.chunkStep == 19:
            self.autoMove(self.CUBE_TO_SWITCH, -1, False)
        elif self.chunkStep == 20:
            self.dropCube()
        else:
            self.reset()
            
    def sameSame(self):
        if self.chunkStep == 0:
            self.autoMove(self.WALL_TO_SCALE, self.robot.elevator.kScale, False)
        elif self.chunkStep == 1:
            self.autoAngle(self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 2:
            self.dropCube()
        elif self.chunkStep == 3:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 4:
            self.autoMove(-self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 5:
            self.autoAngle(self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, self.robot.elevator.kBottom)
        elif self.chunkStep == 6:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 7:
            self.autoMove(0, self.robot.elevator.kSwitch, False)
        elif self.chunkStep == 8:
            self.autoMove(self.CUBE_TO_SWITCH, -1, False)
        elif self.chunkStep == 9:
            self.dropCube()
        else:
            self.reset()
         
    def differentDifferent(self):
        if self.chunkStep == 0:
            self.autoMove(self.WALL_TO_PLATFORM_ALLEY, self.robot.elevator.kSwitch, False)
        elif self.chunkStep == 1:
            self.autoAngle(self.TURN, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 2:
            self.autoMove(self.ALLEY_TO_ALLEY, self.robot.elevator.kScale, False)
        elif self.chunkStep == 3:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 4:
            self.autoMove(self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 5:
            self.autoAngle(-self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 6:
            self.dropCube()
        elif self.chunkStep == 7:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 8:
            self.autoMove(-self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 9:
            self.autoAngle(-self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, self.robot.elevator.kBottom)
        elif self.chunkStep == 10:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 11:
            self.autoMove(0, self.robot.elevator.kSwitch, False)
        elif self.chunkStep == 12:
            self.autoMove(self.CUBE_TO_SWITCH, -1, False)
        elif self.chunkStep == 13:
            self.dropCube()
        else:
            self.reset()
        
    def straightSwitch(self):
        if self.chunkStep == 0:
            self.autoMove(self.WALL_TO_STRAIGHT_SWITCH, self.robot.elevator.kSwitch, False)
        elif self.chunkStep == 1:
            self.dropCube()
        elif self.chunkStep == 2:
            self.autoStep += 1
                         
    def autoMove(self, distanceIN, elevatorPosition, isAutoPickUp):
        self.robot.drive.moveToPosition(self.magEncoderInchesToTicks(distanceIN))
        
        if elevatorPosition >= 0:
            self.robot.elevator.setElevatorPosition(elevatorPosition)
        else:
            self.elevator.set(0)
            
        if isAutoPickUp:
            self.robot.cubeGrabber.grabberFunction()
            
            
        if abs(self.driveLeft.getSelectedSensorPosition(0)) >= abs(self.magEncoderInchesToTicks(distanceIN)) and abs(self.driveRight.getSelectedSensorPosition(0)) >= abs(self.magEncoderInchesToTicks(distanceIN)):
            if elevatorPosition >= 0: 
                
                if self.elevator.getSelectedSensorPosition(0) >= elevatorPosition:
                    
                    self.reset()
                    self.chunkStep += 1
            else:   
                self.reset()
                self.chunkStep += 1
      
    def autoAngle(self, angle, tolerance, elevatorPosition):
        self.adjustedAngleTarget = angle + self.startingYaw
        self.robot.drive.setAngle(self.adjustedAngleTarget, tolerance) 
        
        if elevatorPosition >= 0:
            self.robot.elevator.setElevatorPosition(elevatorPosition)
        else:
            self.elevator.set(0)    
            
        if self.robot.drive.isInGyroPosition():
            if elevatorPosition >= 0: 
                if self.elevator.getSelectedSensorPosition(0) >= elevatorPosition:
                    self.reset()
                    self.chunkStep += 1
            else:   
                self.reset()
                self.chunkStep += 1       
                                      
    def pickUpCube(self):
        self.reset()
        self.robot.cubeGrabber.armGrabAuto()
        
        if self.robot.cubeGrabber.getSwitch():
            self.chunkStep += 1
     
    def dropCube(self):
        self.reset()
        self.leftGrab.set(-1)
        self.rightGrab.set(1)
        
        wpilib.Timer.delay(1)
        
        self.reset()
        self.chunkStep += 1    
                                
    def delay(self, seconds):
        self.reset()
        wpilib.Timer.delay(seconds)
        self.chunkStep = 0
        self.autoStep = 2
                         
    def reset(self):
        self.elevator.set(0)
        self.driveLeft.set(0)
        self.driveRight.set(0)
        self.leftGrab.set(0)
        self.rightGrab.set(0)
        
        
        self.robot.cubeGrabber.armSolenoid.set(self.robot.cubeGrabber.armClosePosition)
        
        self.driveLeft.setSelectedSensorPosition(0, 0, 0)
        self.driveRight.setSelectedSensorPosition(0, 0, 0)
        
    def resetGyro(self):
        self.startingYaw = self.robot.drive.ahrs.getYaw()
        self.chunkStep += 1
    
    def testMove(self, distance, elevatorPosition, isAutoPickUp):
        if self.chunkStep == 0:
            self.autoMove(distance, elevatorPosition, isAutoPickUp)
        else:
            self.reset()
            
    def testAngle(self, angle, elevatorPosition):
        if self.chunkStep == 0:
            self.resetGyro()
        if self.chunkStep == 1:
            self.autoAngle(angle, 3, elevatorPosition)
        else:
            self.reset()
                
    def telemetry(self):
        wpilib.SmartDashboard.putNumber('Auto Step', self.autoStep)
        wpilib.SmartDashboard.putNumber('Chunk Step', self.chunkStep)
        wpilib.SmartDashboard.putNumber('Delay Time', self.delayTime)
        wpilib.SmartDashboard.putString('Game Data', self.gamedata)
        wpilib.SmartDashboard.putString('Starting Position', self.startLocation)
        wpilib.SmartDashboard.putNumber('Angle', self.robot.drive.ahrs.getAngle())
        wpilib.SmartDashboard.putNumber('Left Position', self.driveLeft.getSelectedSensorPosition(0))
        wpilib.SmartDashboard.putNumber('Right Position', self.driveRight.getSelectedSensorPosition(0))
        wpilib.SmartDashboard.putNumber('Starting Angle', self.startingYaw)
        wpilib.SmartDashboard.putNumber('Adjusted Angle', self.startingYaw + self.robot.drive.ahrs.getYaw())