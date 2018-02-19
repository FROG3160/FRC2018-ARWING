from wpilib.command.subsystem import Subsystem
import wpilib
import ctre
from _operator import index

class Autonomous(Subsystem):
    
    def __init__(self, robot):
        self.robot = robot

        self.autoInstructionsFile = open("AutoInstructions.txt", "r")
        self.autoInstructions = self.autoInstructionsFile.readlines()
        self.autoInstructions[index].strip()
        self.autoInstructionsFile.close()
        
        self.armSolenoid.set(self.armClosePosition)
        
        self.driveLeft = self.robot.Drive.driveLeftMaster
        self.driveRight = self.robot.Drive.driveRightMaster
        
        self.elevator = self.robot.Elevator.mainLift
        
        self.leftGrab = self.robot.Grabber.leftArm
        self.rightGrab = self.robot.Grabber.rightArm
        
        self.gamedata = " "
        
        self.autoStep = 0
        self.chunkStep = 0 
        self.startLocation = self.autoInstructions[1]
        self.delayTime = self.autoInstructions[0]
        

    
    def distanceVariables(self):
        self.ANGLE_TOLERANCE = 3
               
        self.WALL_TO_SWITCH = 0
        self.SWITCH_TO_PLATFORM_ALLEY = 0
        self.PLATFORM_ALLEY_TO_SCALE = 0
        self.WALL_TO_STRAIGHT_SWITCH = 0
        self.ALLEY_TO_CUBE = 0
        self.ALLEY_TO_ALLEY = 0
        self.CUBE_TO_SWITCH = 0
        
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
       
        self.WALL_TO_SCALE = self.WALL_TO_NEAR_SWITCH + self.NEAR_SWITCH_TO_PLATFORM_ALLEY + self.PLATFORM_ALLEY_TO_SCALE
        
        self.WALL_TO_PLATFORM_ALLEY = self.WALL_TO_NEAR_SWITCH + self.NEAR_SWITCH_TO_PLATFORM_ALLEY
        
    def getGameData(self):
        self.gamedata = wpilib.DriverStation.getGameSpecificMessage() 
      
    def run(self):
        self.telemetry()
        
        if self.autoStep == 0:
            self.start()
        elif self.autoStep == 1:
            self.delay(self.delayTime)
        elif self.autoStep == 2:
            self.autoChooser()

    def start(self):
        self.reset()
        self.robot.Drive.ahrs.reset()
        
        if self.chunkStep == 0:
            if self.robot.Elevator.getBottomLimit():
                self.chunkStep = 1
            else: 
                self.robot.Elevator.calibrateBottomAutonomous()
        elif self.chunkStep == 1:
            self.robot.Elevator.setElevatorPosition(self.robot.Elevator.kStart)
            if self.elevator.getPulseWidthPosition() >= self.robot.Elevator.kStart:
                self.chunkStep = 2
        elif self.chunkStep == 2:
            self.robot.Elevator.setElevatorPosition(self.robot.Elevator.kBottom)
            if self.robot.Elevator.getBottomLimit:
                self.chunkStep = 3
        elif self.chunkStep == 3:
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
            self.autoMove(self.WALL_TO_SWITCH, self.robot.Elevator.kSwitch, False)
        elif self.chunkStep == 1:
            self.autoAngle(self.SWITCH_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 2:
            self.dropCube()
        elif self.chunkStep == 3:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 4:
            self.autoMove(self.SWITCH_TO_PLATFORM_ALLEY, self.robot.Elevator.kBottom, False)
        elif self.chunkStep == 5:
            self.autoAngle(self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 6:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 7:
            self.autoMove(-self.ALLEY_TO_CUBE, -1, False)
        elif self.chunkStep == 8:
            self.autoAngle(self.TURN, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 9:
            self.autoMove(self.ALLEY_TO_ALLEY, self.robot.Elevator.kScale, False)
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
            self.autoAngle(-self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, self.robot.Elevator.kBottom)
        elif self.chunkStep == 17:
            self.self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 18:
            self.autoMove(-self.ALLEY_TO_CUBE, self.robot.Elevator.kScale, False)
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
            self.autoMove(self.WALL_TO_SCALE, self.robot.Elevator.kScale, False)
        elif self.chunkStep == 1:
            self.autoAngle(self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 2:
            self.dropCube()
        elif self.chunkStep == 3:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 4:
            self.autoMove(-self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 5:
            self.autoAngle(self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, -self.robot.Elevator.kBottom)
        elif self.chunkStep == 6:
            self.self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 7:
            self.autoMove(-self.ALLEY_TO_CUBE, self.robot.Elevator.kScale, False)
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
            self.autoAngle(self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, self.robot.Elevator.kBottom)
        elif self.chunkStep == 15:
            self.autoMove(self.ALLEY_TO_ALLEY, -1, False)
        elif self.chunkStep == 16:
            self.autoAngle(-self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 17:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 18:
            self.autoMove(0, self.robot.Elevator.kSwitch, False)
        elif self.chunkStep == 19:
            self.autoMove(self.CUBE_TO_SWITCH, -1, False)
        elif self.chunkStep == 20:
            self.dropCube()
        else:
            self.reset()
            
    def sameSame(self):
        if self.chunkStep == 0:
            self.autoMove(self.WALL_TO_SCALE, self.robot.Elevator.kScale, False)
        elif self.chunkStep == 1:
            self.autoAngle(self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 2:
            self.dropCube()
        elif self.chunkStep == 3:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 4:
            self.autoMove(-self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 5:
            self.autoAngle(self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, self.robot.Elevator.kBottom)
        elif self.chunkStep == 6:
            self.self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 7:
            self.autoMove(0, self.robot.Elevator.kSwitch, False)
        elif self.chunkStep == 8:
            self.autoMove(self.CUBE_TO_SWITCH, -1, False)
        elif self.chunkStep == 9:
            self.dropCube()
        else:
            self.reset()
         
    def differentDifferent(self):
        if self.chunkStep == 0:
            self.autoMove(self.WALL_TO_PLATFORM_ALLEY, self.robot.Elevator.kSwitch, False)
        elif self.chunkStep == 1:
            self.autoAngle(self.TURN, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 2:
            self.autoMove(self.ALLEY_TO_ALLEY, self.robot.Elevator.kScale, False)
        elif self.chunkStep == 3:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 4:
            self.autoMove(self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 5:
            self.autoAngle(-self.SCALE_DUMP_ANGLE, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 6:
            self.autoAngle(0, self.ANGLE_TOLERANCE, -1)
        elif self.chunkStep == 7:
            self.autoMove(-self.PLATFORM_ALLEY_TO_SCALE, -1, False)
        elif self.chunkStep == 8:
            self.autoAngle(-self.VERTICAL_ALLEY_TO_CUBE_ANGLE, self.ANGLE_TOLERANCE, self.robot.Elevator.kBottom)
        elif self.chunkStep == 9:
            self.autoMove(self.ALLEY_TO_CUBE, -1, True)
        elif self.chunkStep == 10:
            self.autoMove(0, self.robot.Elevator.kSwitch, False)
        elif self.chunkStep == 11:
            self.autoMove(self.CUBE_TO_SWITCH, -1, False)
        elif self.chunkStep == 12:
            self.dropCube()
        else:
            self.reset()
        
    def straightSwitch(self):
        if self.chunkStep == 0:
            self.autoMove(self.WALL_TO_STRAIGHT_SWITCH, self.robot.Elevator.kSwitch, False)
        elif self.chunkStep == 1:
            self.dropCube()
        elif self.chunkStep == 2:
            self.autoStep += 1
                         
    def autoMove(self, distance, elevatorPosition, isAutoPickUp):
        self.driveLeft.set(ctre.talonsrx.TalonSRX.FeedbackDevice.PulseWidthEncodedPosition, distance)
        self.driveRight.set(ctre.talonsrx.TalonSRX.FeedbackDevice.PulseWidthEncodedPosition, distance)
        
        if elevatorPosition >= 0:
            self.robot.Elevator.setElevatorPosition(elevatorPosition)
        else:
            self.elevator.set(0)
            
        if isAutoPickUp:
            self.robot.Grabber.armGrabAuto()
            
            
        if self.driveLeft.getPulseWidthPosition() >= distance and self.driveRight.getPulseWidthPosition() >= distance:
            if elevatorPosition >= 0: 
                if self.elevator.getPulseWidthPosition() >= elevatorPosition:
                    self.reset()
                    self.chunkStep += 1
            else:   
                self.reset()
                self.chunkStep += 1
      
    def autoAngle(self, angle, tolerance, elevatorPosition):
        self.robot.Drive.setAngle(angle, tolerance) 
        
        if elevatorPosition >= 0:
            self.robot.Elevator.setElevatorPosition(elevatorPosition)
        else:
            self.elevator.set(0)    
            
        if self.robot.Drive.isInGyroPosition():
            if elevatorPosition >= 0: 
                if self.elevator.getPulseWidthPosition() >= elevatorPosition:
                    self.reset()
                    self.chunkStep += 1
            else:   
                self.reset()
                self.chunkStep += 1       
                                      
    def pickUpCube(self):
        self.reset()
        self.robot.Grabber.armGrabAuto()
        
        if self.robot.Grabber.getSwitch():
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
        
        
        self.robot.Grabber.armSolenoid.set(self.robot.Grabber.armClosePosition)
        
        self.driveLeft.setPulseWidthPosition(0, 0)
        self.driveRight.setPulseWidthPosition(0, 0)
                
    def telemetry(self):
        wpilib.SmartDashboard.putNumber('Auto Step', self.autoStep)
        wpilib.SmartDashboard.putNumber('Chunk Step', self.chunkStep)
        wpilib.SmartDashboard.putNumber('Delay Time', self.delayTime)
        wpilib.SmartDashboard.putString('Game Data', self.gamedata)
        wpilib.SmartDashboard.putString('Starting Position', self.startLocation)
        wpilib.SmartDashboard.putNumber('Angle', self.robot.Drive.ahrs.getAngle())
        wpilib.SmartDashboard.putNumber('Left Position', self.driveLeft.getSelectedSensorPosition(0))
        wpilib.SmartDashboard.putNumber('Right Position', self.driveRight.getSelectedSensorPosition(0))