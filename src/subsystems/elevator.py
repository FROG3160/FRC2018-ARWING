from wpilib.command.subsystem import Subsystem
from ctre import WPI_TalonSRX as Talon
import ctre
import wpilib

class Elevator(Subsystem):
    
    kSwitch = -20000
    kScale = -50000
    kStart = -5000
    kBottom = 0
    
    
    def __init__(self, robot):
        self.robot = robot
        
        self.elevator = Talon(self.robot.kElevator['elevator_motor'])
        self.elevator.setInverted(True)
        self.driverTwo = self.robot.cStick
        
        self.elevator.configSelectedFeedbackSensor(
        ctre.talonsrx.TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0)
        self.elevator.setSensorPhase(True)
        self.elevator.setSelectedSensorPosition(0, 0, 0)
        
        self.elevator.configPeakOutputForward(.25, 0)
        self.elevator.configPeakOutputReverse(-1, 0)
        self.elevator.configNominalOutputForward(0, 0)        
        self.elevator.configNominalOutputReverse(0, 0)
        
        
        self.elevator.setSafetyEnabled(False)
        
        self.smartToggle = False
        self.smartPosition = 0
        
        super().__init__()
        
    def elevatorFunction(self):
#         if self.elevator.isRevLimitSwitchClosed():
#             self.elevator.setSelectedSensorPosition(0, 0, 0)
              
        #self.elevator.set(self.driverTwo.getRawAxis(1))
        self.smartPositioning()
        wpilib.SmartDashboard.putNumber('elevator amperage', self.elevator.getOutputCurrent())
    
#     def updateSD(self):
        
    
    def setElevatorPosition(self, position):
#         if self.elevator.isRevLimitSwitchClosed():
#             self.elevator.setSelectedSensorPosition(0, 0, 0)
#             
        self.elevator.set(ctre.talonsrx.TalonSRX.ControlMode.Position, position)
        
    def smartPositioning(self):
        
        if self.driverTwo.getRawButtonReleased(5):
            self.smartPosition -= 1
            self.smartToggle = True
            
            if self.smartPosition < 0:
                self.smartPosition = 0
                
        elif self.driverTwo.getRawButtonReleased(6):
            self.smartPosition += 1
            self.smartToggle = True
            
            if self.smartPosition > 2:
                self.smartPosition = 2
                
        elif (self.driverTwo.getRawAxis(1) > 0.2) or (self.driverTwo.getRawAxis(1) < -0.2):
            self.smartToggle = False
                
        if self.smartToggle:
            if self.smartPosition == 2:
                self.setElevatorPosition(self.kScale)
            elif self.smartPosition == 1:
                self.setElevatorPosition(self.kSwitch)
            elif self.smartPosition == 0:
                self.elevator.set(0)
        else:
            self.elevator.set(self.driverTwo.getRawAxis(1))
            
            if self.elevator.getSelectedSensorPosition(0) < (self.kSwitch + self.kScale)/2:
                self.smartPosition = 2
            elif self.elevator.getSelectedSensorPosition(0) < self.kSwitch/2:
                self.smartPosition = 1
            else:
                self.smartPosition = 0
             
    def calibrateBottomAutonomous(self):
        
        if self.elevator.isRevLimitSwitchClosed():
            self.elevator.setSelectedSensorPosition(0, 0, 0)
            self.elevator.set(0)
#             self.calibrateStep = 1
        else:
            self.elevator.set(-.5)   
            
    def getBottomLimit(self):
        return self.elevator.isFwdLimitSwitchClosed()
     
    def telemetry(self):
        wpilib.SmartDashboard.putNumber('Lift Position', self.elevator.getSelectedSensorPosition(0))   
        wpilib.SmartDashboard.putBoolean('Forward Limit', self.elevator.isFwdLimitSwitchClosed())    
        wpilib.SmartDashboard.putBoolean('Reverse Limit', self.elevator.isRevLimitSwitchClosed())  
        