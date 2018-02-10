import ctre
from ctre import WPI_TalonSRX as Talon
from wpilib.drive.differentialdrive import DifferentialDrive
from wpilib.speedcontrollergroup import SpeedControllerGroup
from robotmap import kDriveTrain
from wpilib.smartdashboard import SmartDashboard as SD
from wpilib.command.subsystem import Subsystem


class DriveTrain(Subsystem):
    '''
    'Tank Drive' system set up with 2 motors per side, one a "master"
    with a mag encoder attached and the other "slave" controller set
    to follow the "master".
    '''

    def __init__(self):

        # Initialize all controllers
        self.driveLeftMaster = Talon(kDriveTrain.lmId)
        self.driveLeftSlave = Talon(kDriveTrain.lsId)
        self.driveRightMaster = Talon(kDriveTrain.rmId)
        self.driveRightSlave = Talon(kDriveTrain.rsId)

        # Connect the slaves to the masters on each side
        self.driveLeftSlave.follow(self.driveLeftMaster)
        self.driveRightSlave.follow(self.driveRightMaster)

        # Makes sure both sides' controllers show green and use positive
        # values to move the bot forward.
        # CURRENTLY DISABLED WHEN USING WITH DifferentialDrive
        # self.driveLeftSlave.setInverted(False)
        # self.driveLeftMaster.setInverted(False)
        self.driveRightSlave.setInverted(True)
        self.driveRightMaster.setInverted(True)

        # Configures each master to use the attached Mag Encoders
        self.driveLeftMaster.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.driveRightMaster.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)

        # Reverses the encoder direction so forward movement always
        # results in a positive increase in the encoder ticks.
        self.driveLeftMaster.setSensorPhase(True)
        self.driveRightMaster.setSensorPhase(True)

        # Throw data on the SmartDashboard so we can work with it.
        SD.putNumber(
            'Left Quad Pos.',
            self.driveLeftMaster.getQuadraturePosition())
        SD.putNumber(
            'Right Quad Pos.',
            self.driveRightMaster.getQuadraturePosition())

        # these may give the derivitive an integral of the PID once
        # they are set.  For now, they just show 0
        SD.putNumber(
            'Left Derivative',
            self.driveLeftMaster.getErrorDerivative(0))
        SD.putNumber(
            'Left Integral',
            self.driveLeftMaster.getIntegralAccumulator(0))

        self.leftVel = None
        self.leftPos = None
        self.rightVel = None
        self.rightPos = None

        # self.driveLeftMaster.config_kP(0, .3, 10)
        # kP = self.driveLeftMaster.configGetParameter(
        #     self.driveLeftMaster.ParamEnum.eProfileParamSlot_P, 0, 10)

        # SmartDashboard.putNumber('Left Proportional', kP)

        self.driveControllerLeft = SpeedControllerGroup(self.driveLeftMaster)
        self.driveControllerRight = SpeedControllerGroup(self.driveRightMaster)
        self.driveControllerRight.setInverted(True)
        self.drive = DifferentialDrive(self.driveControllerLeft,
                                       self.driveControllerRight)

    def moveToPosition(self, position, side='left'):

        if side == 'left':
            self.driveLeftMaster.setSafetyEnabled(False)
            self.driveLeftMaster.set(Talon.ControlMode.Position, position)
        else:
            self.driveRightMaster.set(Talon.ControlMode.Position, position)

    def stop(self):
        # self.driveLeftMaster.set(0)
        # self.driveRightMaster.set(0)
        self.drive.stopMotor()

    def arcade(self, speed, rotation):
        self.updateSD()
        self.drive.arcadeDrive(speed, rotation, True)

    def updateSD(self):

        leftVel = self.driveLeftMaster.getSelectedSensorVelocity(0)
        leftPos = self.driveLeftMaster.getSelectedSensorPosition(0)

        rightVel = self.driveRightMaster.getSelectedSensorVelocity(0)
        rightPos = self.driveRightMaster.getSelectedSensorPosition(0)

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

        self.leftVel = leftVel
        self.leftPos = leftPos
        self.rightVel = rightVel
        self.rightPos = rightPos
