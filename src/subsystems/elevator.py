from wpilib.command.subsystem import Subsystem
from ctre import WPI_TalonSRX as Talon

class Elevator(Subsystem):
    
    def __init__(self, robot):
        self.robot = robot
        
        self.mainLift = Talon(self.robot.kElevator['elevator_motor'])
        self.driverTwo = self.robot.cStick
        
        super().__init__()
        
    def elevatorFunction(self):
        self.mainLift.set(-self.driverTwo.getRawAxis(1))
        