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
        
        self.mainLift = Talon(self.robot.kElevator['elevator_motor'])
        self.driverTwo = self.robot.cStick
        
        self.elevatorBottomSwitch = wpilib.DigitalInput(self.robot.kElevator['bottom_switch'])
        
        
        
        super().__init__()
        
    def elevatorFunction(self):
        self.mainLift.set(-self.driverTwo.getRawAxis(1))
        
        if self.elevatorBottomSwitch == 1:
            self.elevator.setPulseWidthPosition(0, 0)
        
    def setElevatorPosition(self, position):
        self.mainLift.set(ctre.talonsrx.TalonSRX.FeedbackDevice.PulseWidthEncodedPosition, position)
        
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
     
            
        