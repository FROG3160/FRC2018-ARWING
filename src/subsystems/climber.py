from wpilib.command.subsystem import Subsystem
from ctre import WPI_TalonSRX as Talon
import wpilib
from subsystems.lights import FROGLights

class Climber(Subsystem):
    def __init__(self, robot):
        self.robot = robot
        
        self.lights = FROGLights()
        
        self.climbMotor = Talon(self.robot.kClimber['climb_motor'])
        self.checkSwitch = wpilib.DigitalInput(self.robot.kClimber['check_switch'])
        self.solenoid = wpilib.DoubleSolenoid(self.robot.pneumaticControlModuleCANID, self.robot.kClimber['solenoid'], self.robot.kClimber['solenoid_2'])
        
        self.driverOne = self.robot.dStick
        
        self.latchToggleCount = 2
        
        self.handlerState = 0
        
        self.climbMotor.configPeakOutputForward(1, 0)
        self.climbMotor.configPeakOutputReverse(-1, 0)
        
    def climberFunction(self):
        self.climbLatch()
        
        if self.driverOne.getPOV() == 0:
            self.lightsHandler(2)
            self.climbMotor.set(0.75)
            
        elif self.driverOne.getPOV() == 180:
            self.lightsHandler(1)
            self.climbMotor.set(-0.75)
        else:
            self.climbMotor.set(0)
        wpilib.SmartDashboard.putBoolean('Latch Switch', self.checkSwitch.get() == 1)
        
            
    def climbLatch(self):
        if self.driverOne.getRawButtonReleased(1):
            self.latchToggleCount += 1
        if self.latchToggleCount%2 == 1:
            self.solenoid.set(self.solenoid.Value.kReverse)
        else:
            self.solenoid.set(self.solenoid.Value.kForward)
            
    def getCheckSwitch(self):
        return self.checkSwitch.get()
        if self.checkSwitch.get():
            self.lightsHandler(2)
            
    def lightsHandler(self, tempHandlerState):

            
        if tempHandlerState != self.handlerState:
            if tempHandlerState == 1:
                self.lights.sendOKToClimb()
            elif tempHandlerState == 2:
                self.lights.sendDontClimb()
            self.handlerState = tempHandlerState
        
        
        
        