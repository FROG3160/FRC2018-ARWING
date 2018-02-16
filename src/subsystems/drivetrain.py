import ctre
import wpilib
from ctre import WPI_TalonSRX as Talon
from wpilib.drive.differentialdrive import DifferentialDrive
from wpilib.speedcontrollergroup import SpeedControllerGroup
from wpilib.smartdashboard import SmartDashboard as SD
from wpilib.command import Subsystem


class DriveTrain(Subsystem):
    '''
    'Tank Drive' system set up with 2 motors per side, one a "master"
    with a mag encoder attached and the other "slave" controller set
    to follow the "master".
    '''

    def __init__(self, robot):

        self.robot = robot

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

        # Makes sure both sides' controllers show green and use positive
        # values to move the bot forward.
        self.driveLeftSlave.setInverted(False)
        self.driveLeftMaster.setInverted(False)
        self.driveRightSlave.setInverted(True)
        self.driveRightMaster.setInverted(True)

        # Configures each master to use the attached Mag Encoders
        self.driveLeftMaster.configSelectedFeedbackSensor(
            ctre.talonsrx.TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.driveRightMaster.configSelectedFeedbackSensor(
            ctre.talonsrx.TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)

        # Reverses the encoder direction so forward movement always
        # results in a positive increase in the encoder ticks.
        self.driveLeftMaster.setSensorPhase(True)
        self.driveRightMaster.setSensorPhase(True)

        self.driveLeftMaster.setQuadraturePosition(0, 0)
        self.driveRightMaster.setQuadraturePosition(0, 0)    

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
        # self.drive = DifferentialDrive(self.driveLeftMaster,
        #                               self.driveRightMaster)
        
        wpilib.LiveWindow.addActuator("DriveTrain", "Left Master", self.driveLeftMaster)
        wpilib.LiveWindow.addActuator("DriveTrain", "Right Master", self.driveRightMaster)
        #wpilib.LiveWindow.add(self.driveLeftSlave)
        #wpilib.LiveWindow.add(self.driveRightSlave)
        #wpilib.Sendable.setName(self.drive, 'Drive')
        #wpilib.LiveWindow.add(self.drive)
        #wpilib.Sendable.setName(self.driveLeftMaster, 'driveLeftMaster')
        #wpilib.LiveWindow.add(self.driveRightMaster)
        
        
        super().__init__()

    def moveToPosition(self, position, side='left'):

        if side == 'left':
            self.driveLeftMaster.setSafetyEnabled(False)
            self.driveLeftMaster.set(ctre.talonsrx.TalonSRX.ControlMode.Position, position)
        else:
            self.driveRightMaster.set(ctre.talonsrx.TalonSRX.ControlMode.Position, position)

    def stop(self):
        self.drive.stopMotor()

    def arcade(self, speed, rotation):
        self.updateSD()
        self.drive.arcadeDrive(speed, rotation, True)

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

