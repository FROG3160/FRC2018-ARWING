from wpilib.command.subsystem import Subsystem
from ctre import WPI_TalonSRX as Talon
import ctre
import wpilib

class Elevator(Subsystem):
    
    kSwitch = 1000
    kScale = 3000
    kStart = 500
    kBottom = 0
    
    
    def __init__(self, robot):
        self.robot = robot
        
        self.elevator = Talon(self.robot.kElevator['elevator_motor'])
        self.elevator.setInverted(True)
        self.driverTwo = self.robot.cStick
        
        self.elevator.configSelectedFeedbackSensor(
        ctre.talonsrx.TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        
        self.elevator.configPeakOutputForward(1, 0)
        self.elevator.configPeakOutputReverse(-0.25, 0)
        self.elevator.configNominalOutputForward(0, 0)        
        self.elevator.configNominalOutputReverse(0, 0)
        
        self.elevator.config_kP(0, 3, 0)
        
        self.elevator.setSafetyEnabled(False)
        
        super().__init__()
        
    def elevatorFunction(self):
        if self.elevator.isRevLimitSwitchClosed():
            self.elevator.setPulseWidthPosition(0, 0)
              
        self.elevator.set(-self.driverTwo.getRawAxis(1))
        wpilib.SmartDashboard.putNumber('elevator amperage', self.elevator.getOutputCurrent())
    
#     def updateSD(self):
        
    
    def setElevatorPosition(self, position):
        if self.elevator.isRevLimitSwitchClosed():
            self.elevator.setSelectedSensorPosition(0, 0, 0)
            
        self.elevator.set(ctre.talonsrx.TalonSRX.ControlMode.Position, position)
        
    def calibrateBottomAutonomous(self):
        
        if self.elevator.isRevLimitSwitchClosed():
            self.elevator.setSelectedSensorPosition(0, 0, 0)
            self.elevator.set(0)
#             self.calibrateStep = 1
        else:
            self.elevator.set(-.5)
#             self.calibrateStep = 0
            
    def getBottomLimit(self):
        return self.elevator.isRevLimitSwitchClosed()
     
    def telemetry(self):
        wpilib.SmartDashboard.putNumber('Lift Position', self.elevator.getSelectedSensorPosition(0))        
        