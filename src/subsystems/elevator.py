from wpilib.command.subsystem import Subsystem
from ctre import WPI_TalonSRX as Talon
import ctre
import wpilib

class Elevator(Subsystem):
    
    kSwitch = 0
    kScale = 0
    kStart = 0
    kBottom = 0
    
    
    def __init__(self, robot):
        self.robot = robot
        
        self.elevator = Talon(self.robot.kElevator['elevator_motor'])
        self.driverTwo = self.robot.cStick
        
        self.elevatorBottomSwitch = wpilib.DigitalInput(self.robot.kElevator['bottom_switch'])
        
        self.elevator.configSelectedFeedbackSensor(
        ctre.talonsrx.TalonSRX.FeedbackDevice.PulseWidthEncodedPosition, 0, 0)
        
        self.elevator.configPeakOutputForward(1, 0)
        self.elevator.configPeakOutputReverse(-1, 0)
        
        super().__init__()
        
    def elevatorFunction(self):
#         if self.elevatorBottomSwitch == 1:
#             self.elevator.setPulseWidthPosition(0, 0)
#             if -self.driverTwo.getRawAxis(1) < 0:
#                 self.liftPower = 0
#             else:
#                 self.liftPower = -self.driverTwo.getRawAxis(1)
#             
#         else:      
#             self.liftPower = -self.driverTwo.getRawAxis(1)
#             
#             
        self.elevator.set(self.driverTwo.getRawAxis(1))
    
#     def updateSD(self):
        
    
    def setElevatorPosition(self, position):
        self.elevator.set(ctre.talonsrx.TalonSRX.FeedbackDevice.PulseWidthEncodedPosition, position)
        
    def calibrateBottomAutonomous(self):
        
        if self.elevatorBottomSwitch == 1:
            self.elevator.set(0)
            self.elevator.setPulseWidthPosition(0, 0)
#             self.calibrateStep = 1
        else:
            self.elevator.set(-.5)
#             self.calibrateStep = 0
            
    def getBottomLimit(self):
        return self.elevatorBottomSwitch == 1
     
    def telemetry(self):
        wpilib.SmartDashboard.putNumber('Lift Position', self.elevator.getSelectedSensorPosition(0))        
        