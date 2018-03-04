import ctre
import wpilib
import math
from ctre import WPI_TalonSRX as Talon
from wpilib.drive.differentialdrive import DifferentialDrive
from wpilib.speedcontrollergroup import SpeedControllerGroup
from wpilib.smartdashboard import SmartDashboard as SD
from wpilib.command import Subsystem
from robotpy_ext.common_drivers.navx import AHRS


class DriveTrain(Subsystem):
    '''
    'Tank Drive' system set up with 2 motors per side, one a "master"
    with a mag encoder attached and the other "slave" controller set
    to follow the "master".
    '''

    def __init__(self, robot):

        self.robot = robot
        
        self.ahrs = AHRS.create_spi()    
        # Initialize all controllers
        self.driveLeftMaster = Talon(self.robot.kDriveTrain['left_master'])
        self.driveLeftSlave = Talon(self.robot.kDriveTrain['left_slave'])
        self.driveRightMaster = Talon(self.robot.kDriveTrain['right_master'])
        self.driveRightSlave = Talon(self.robot.kDriveTrain['right_slave'])

        wpilib.LiveWindow.addActuator("DriveTrain",
                                      "LeftMaster", self.driveLeftMaster)
        wpilib.LiveWindow.addActuator("DriveTrain",
                                      "RightMaster", self.driveRightMaster)

        # Connect the slaves to the masters on each side
        self.driveLeftSlave.follow(self.driveLeftMaster)
        self.driveRightSlave.follow(self.driveRightMaster)
        

        self.driveLeftMaster.configNominalOutputForward(0, 0)
        self.driveLeftMaster.configNominalOutputReverse(0, 0)

        self.driveRightMaster.configNominalOutputForward(0, 0)        
        self.driveRightMaster.configNominalOutputReverse(0, 0)
                
        self.speed = .4
        self.driveLeftMaster.configPeakOutputForward(self.speed, 0)
        self.driveLeftMaster.configPeakOutputReverse(-self.speed, 0)
        
        self.driveRightMaster.configPeakOutputForward(self.speed, 0)
        self.driveRightMaster.configPeakOutputReverse(-self.speed, 0)
        


        self.driveLeftMaster.setSafetyEnabled(False)
        self.driveRightMaster.setSafetyEnabled(False)

        # Makes sure both sides' controllers show green and use positive
        # values to move the bot forward.
        self.driveLeftSlave.setInverted(False)
        self.driveLeftMaster.setInverted(False)
        self.driveRightSlave.setInverted(True)
        self.driveRightMaster.setInverted(True)
        
        self.PID()
        
        
        """
        Initializes the count for toggling which side of the 
        robot will be considered the front when driving.
        """
        self.robotFrontToggleCount = 2

        # Configures each master to use the attached Mag Encoders
        self.driveLeftMaster.configSelectedFeedbackSensor(
            ctre.talonsrx.TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.driveRightMaster.configSelectedFeedbackSensor(
            ctre.talonsrx.TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)

        # Reverses the encoder direction so forward movement always
        # results in a positive increase in the encoder ticks.
        self.driveLeftMaster.setSensorPhase(True)
        self.driveRightMaster.setSensorPhase(True)
        
        self.driveLeftMaster.setSelectedSensorPosition(0, 0, 0)
        self.driveRightMaster.setSelectedSensorPosition(0, 0, 0)

        # these supposedly aren't part of the WPI_TalonSRX class
        # self.driveLeftMaster.setSelectedSensorPostion(0, 0, 10)
        # self.driveRightMaster.setSelectedSensorPosition(0, 0, 10)

        # Throw data on the SmartDashboard so we can work with it.
        # SD.putNumber(
        #     'Left Quad Pos.',
        #     self.driveLeftMaster.getQuadraturePosition())
        # SD.putNumber(
        #     'Right Quad Pos.',
        #     self.driveRightMaster.getQuadraturePosition())

        self.leftVel = None
        self.leftPos = None
        self.rightVel = None
        self.rightPos = None
        self.leftMaxVel = 0
        self.rightMaxVel = 0
        self.leftMinVel = 0
        self.rightMinVel = 0
        
        # self.driveLeftMaster.config_kP(0, .3, 10)

        self.driveControllerLeft = SpeedControllerGroup(self.driveLeftMaster)
        self.driveControllerRight = SpeedControllerGroup(self.driveRightMaster)
        self.driveControllerRight.setInverted(True)
        self.drive = DifferentialDrive(self.driveControllerLeft,
                                       self.driveControllerRight)
        self.drive.setSafetyEnabled(False)

        self.previousError = 0

        super().__init__()
    def autoInit(self):
        self.speed = .4
        self.driveLeftMaster.configPeakOutputForward(self.speed, 0)
        self.driveLeftMaster.configPeakOutputReverse(-self.speed, 0)
        
        self.driveRightMaster.configPeakOutputForward(self.speed, 0)
        self.driveRightMaster.configPeakOutputReverse(-self.speed, 0) 
        
#         self.driveLeftMaster.config_kP(0, .055, 0)
#         self.driveRightMaster.config_kP(0, .055, 0)

        self.driveLeftMaster.config_kP(0, 20, 0)
        self.driveRightMaster.config_kP(0, 20, 0)
        
        self.driveLeftMaster.config_kF(0, 0.0, 0)
        self.driveRightMaster.config_kF(0, 0.0, 0)
        
  
        
    def teleInit(self):
        self.speed = .75
        self.driveLeftMaster.configPeakOutputForward(self.speed, 0)
        self.driveLeftMaster.configPeakOutputReverse(-self.speed, 0)
        
        self.driveRightMaster.configPeakOutputForward(self.speed, 0)
        self.driveRightMaster.configPeakOutputReverse(-self.speed, 0)
        
        
        self.driveLeftMaster.config_kP(0, 0.0, 0)
        self.driveRightMaster.config_kP(0, 0.0, 0)
        
        self.driveLeftMaster.config_kF(0, 0.313, 0)
        self.driveRightMaster.config_kF(0, 0.313, 0)

    def moveToPosition(self, position, side='left'):
        self.driveLeftMaster.set(ctre.talonsrx.TalonSRX.ControlMode.Position, position)
        
        self.driveRightMaster.set(ctre.talonsrx.TalonSRX.ControlMode.Position, position)

    def stop(self):
        self.drive.stopMotor()

    def arcade(self, speed, rotation):
        self.updateSD()
        
        if self.robot.dStick.getRawButtonReleased(3):
            self.robotFrontToggleCount += 1
        
        """
        This if statement acts as a toggle to change which motors are 
        inverted, completely changing the "front" of the robot. This is
        useful for when we are about to climb.
        """
        if self.robotFrontToggleCount%2 == 0:
            self.drive.arcadeDrive(speed, rotation, True)      
        else:
            self.drive.arcadeDrive(-speed, rotation, True)
            
    def arcadeWithRPM(self, speed, rotation, maxRPM):
        self.updateSD()
        self.driveLeftMaster.setSafetyEnabled(False)
        
        if self.robot.dStick.getRawButtonReleased(3):
            self.robotFrontToggleCount += 1
            
        if self.robotFrontToggleCount%2 == 0:
            XSpeed = wpilib.RobotDrive.limit(speed)
        else:
            XSpeed = wpilib.RobotDrive.limit(-speed)
        
        XSpeed = self.applyDeadband(XSpeed, .02)

        ZRotation = wpilib.RobotDrive.limit(rotation)
        ZRotation = self.applyDeadband(ZRotation, .02) 
        
        XSpeed = math.copysign(XSpeed * XSpeed, XSpeed)
        ZRotation = math.copysign(ZRotation * ZRotation, ZRotation)

        maxInput = math.copysign(max(abs(XSpeed), abs(ZRotation)), XSpeed)

        if XSpeed >= 0.0:
            if ZRotation >= 0.0:
                leftMotorSpeed = maxInput
                rightMotorSpeed = XSpeed - ZRotation
            else:
                leftMotorSpeed = XSpeed + ZRotation
                rightMotorSpeed = maxInput
        else:
            if ZRotation >= 0.0:
                leftMotorSpeed = XSpeed + ZRotation
                rightMotorSpeed = maxInput
            else:
                leftMotorSpeed = maxInput
                rightMotorSpeed = XSpeed - ZRotation

        leftMotorSpeed = wpilib.RobotDrive.limit(leftMotorSpeed)
        rightMotorSpeed = wpilib.RobotDrive.limit(rightMotorSpeed)
        
        leftMotorRPM = leftMotorSpeed * maxRPM
        rightMotorRPM =  rightMotorSpeed * maxRPM
        
        self.driveLeftMaster.set(ctre.talonsrx.TalonSRX.ControlMode.Velocity, leftMotorRPM)
        self.driveRightMaster.set(ctre.talonsrx.TalonSRX.ControlMode.Velocity, rightMotorRPM)
        
    def updateSD(self):
        


        leftVel = self.driveLeftMaster.getSelectedSensorVelocity(0)
        leftPos = self.driveLeftMaster.getSelectedSensorPosition(0)

        rightVel = self.driveRightMaster.getSelectedSensorVelocity(0)
        rightPos = self.driveRightMaster.getSelectedSensorPosition(0)
        
        # keep the biggest velocity values
        if self.leftMaxVel < leftVel:
            self.leftMaxVel = leftVel
            
        if self.rightMaxVel < rightVel:
            self.rightMaxVel = rightVel

        # keep the smallest velocity values
        if self.leftMinVel > leftVel:
            self.leftMinVel = leftVel

        if self.rightMinVel > rightVel:
            self.rightMinVel = rightVel

        # calculate side deltas
        if self.leftVel:
            leftVelDelta = leftVel - self.leftVel
        else:
            leftVelDelta = 0

        if self.leftPos:
            leftPosDelta = leftPos - self.leftPos
        else:
            leftPosDelta = 0

        if self.rightVel:
            rightVelDelta = rightVel - self.rightVel
        else:
            rightVelDelta = 0

        if self.rightPos:
            rightPosDelta = rightPos - self.rightPos
        else:
            rightPosDelta = 0

        # calculate delta of delta
        differenceVel = leftVelDelta - rightVelDelta
        differencePos = leftPosDelta - rightPosDelta

        SD.putNumber("LeftSensorVel", leftVel)
        SD.putNumber("LeftSensorPos", leftPos)

        SD.putNumber("RightSensorVel", rightVel)
        SD.putNumber("RightSensorPos", rightPos)

        SD.putNumber('LeftVelDelta', leftVelDelta)
        SD.putNumber('LeftPosDelta', leftPosDelta)

        SD.putNumber('RightVelDelta', rightVelDelta)
        SD.putNumber('RightPosDelta', rightPosDelta)

        SD.putNumber('DifferenceVel', differenceVel)
        SD.putNumber('DifferencePos', differencePos)
        
        SD.putNumber('Angle', self.ahrs.getAngle())
        SD.putNumber('Adjusted Angle', self.ahrs.getAngle())

        SD.putNumber('Left Max Vel', self.leftMaxVel)
        SD.putNumber('Right Max Vel', self.rightMaxVel)
        
        SD.putNumber('Left Min Vel', self.leftMinVel)
        SD.putNumber('Right Min Vel', self.rightMinVel)

        self.leftVel = leftVel
        self.leftPos = leftPos
        self.rightVel = rightVel
        self.rightPos = rightPos
        

        # kP = self.driveLeftMaster.configGetParameter(
        #     self.driveLeftMaster.ParamEnum.eProfileParamSlot_P, 0, 10)

        # SmartDashboard.putNumber('Left Proportional', kP)

        # these may give the derivitive an integral of the PID once
        # they are set.  For now, they just show 0
        #SD.putNumber(
        #    'Left Derivative',
        #    self.driveLeftMaster.getErrorDerivative(0))
        #SD.putNumber(
        #    'Left Integral',
        #    self.driveLeftMaster.getIntegralAccumulator(0))
  
    def applyDeadband(self, value, deadband):
        """Returns 0.0 if the given value is within the specified range around zero. The remaining range
        between the deadband and 1.0 is scaled from 0.0 to 1.0.

        :param value: value to clip
        :param deadband: range around zero
        """
        if abs(value) > deadband:
            if value < 0.0:
                return (value - deadband) / (1.0 - deadband)
            else:
                return (value + deadband) / (1.0 - deadband)
        return 0.0
    
        
    def setAngle(self, angle, tolerance):
        #self.tolerance = tolerance
            
        self.turnController.setSetpoint(angle)
       
        
        if (self.ahrs.getYaw() <= abs(angle + tolerance)) and (self.ahrs.getYaw() >= abs(angle - tolerance)):            
            self.turnController.disable()
            
            self.driveLeftMaster.set(0)
            self.driveRightMaster.set(0)
            
        else:
            self.turnController.enable()
            
            self.drive.arcadeDrive(0, self.output)

            

            #self.leftTurnController.setSetpoint(angle)
    
            
    def isInGyroPosition(self):
        return ((self.ahrs.getYaw() - self.robot.autonomous.startingYaw) <= (self.turnController.getSetpoint() + self.robot.autonomous.ANGLE_TOLERANCE)) and ((self.ahrs.getYaw() - self.robot.autonomous.startingYaw) >= (self.turnController.getSetpoint() - self.robot.autonomous.ANGLE_TOLERANCE))
                   
    def PID(self):
        self.kP = 0.3
        self.kI = 0.00
        self.kD = 0.00
        self.kF = 0.00
        
        self.turnController = wpilib.PIDController(self.kP, self.kI, self.kD, self.kF, self.ahrs, output=self)
        
                
        self.turnController.setInputRange(-180, 180)
        self.turnController.setOutputRange(-0.5, 0.5)

        
        self.turnController.disable()

        
        
    def pidWrite(self, output):
        self.output = output       
                