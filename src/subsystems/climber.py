from wpilib.command.subsystem import Subsystem
from ctre import WPI_TalonSRX as Talon
import wpilib

class Climber(Subsystem):
    def __init__(self, robot):
        self.robot = robot
        
        self.climbMotor = Talon(self.robot.kClimber['climb_motor'])
        self.checkSwitch = wpilib.DigitalInput(self.robot.kClimber['check_switch'])
        self.solenoid = wpilib.Solenoid(self.robot.kClimber['solenoid'])
        
        self.driverOne = self.robot.dStick
        
        self.latchToggleCount = 2
        
    def climberFunction(self):
        if self.driverOne.getStickPOV() == 0:
            self.climbMotor.set(0.75)
        elif self.driverOne.getStickPOV() == 180:
            self.climbMotor.set(-0.75)
        else:
            self.climbMotor.set(0)
            
        if self.checkSwitch.get() == 1:
            self.climbLatch()
            
    def climbLatch(self):
        if self.driverOne.getRawButtonReleased(1):
            self.latchToggleCount += 1
            
        if self.latchToggleCount%2 == 0:
            self.solenoid.set(False)
        else:
            self.solenoid.set(True)
            
    def getCheckSwitch(self):
        return self.checkSwitch.get()
        
        