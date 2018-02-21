import wpilib
from ctre import WPI_TalonSRX as Talon
from wpilib.command.subsystem import Subsystem
 
class cubeGrabber(Subsystem): 
       
    def __init__(self, robot):
        
        self.robot = robot
        
        self.leftArm = Talon(self.robot.kCubeGrabber['left_arm'])
        self.rightArm = Talon(self.robot.kCubeGrabber['right_arm'])
        
        self.armUS = wpilib.AnalogInput(self.robot.kCubeGrabber['ultra_sonic'])
        self.armSwitch = wpilib.DigitalInput(self.robot.kCubeGrabber['switch'])
        
        self.armClosePosition = self.robot.kCubeGrabber['close']
        self.armOpenPosition = self.robot.kCubeGrabber['open']
        
        self.driverTwo = self.robot.cStick
        
        self.armSolenoid = wpilib.Solenoid(self.robot.pneumaticControlModuleCANID, self.robot.kCubeGrabber['solenoid'])
        
        """
        Initializes the predetermined distances for grabber/cube interaction.
        """
        self.spinDistance = 12
        self.closeDistance = 7
        
        """
        Initializes Toggle Counts for the arm functionality.
        """
        self.armModeToggleCount = 2
        self.openToggleCount = 3
        
        super().__init__()
    
    def grabberFunction(self):
        """
        Updates the arm ultra sonic sensor divided by a 
        predetermined ratio to get the output to inches.
        """
        self.cubeDistanceIn = self.armUS.getValue()/8.417
        
        """
        If statement adding to the toggle count as the 
        driver changes from automatic grabbing to manual grabbing.
        """
        
        if self.robot.cStick.getRawButtonReleased(3):
            self.armModeToggleCount = self.armModeToggleCount + 1
            #self.armGrabReset()
    
        """
        If statement controlling whether the arm functionality 
        is automatic or manual, determined by the Arm Mode Toggle 
        remainder when divided by 2.
        """
        if self.armModeToggleCount%2 == 0:
            self.armGrabAuto()
            """Does not rumble the controller when in Automatic Grabbing"""
            self.robot.cStick.setRumble(0, 0)
        else:
            self.armGrabManual()
            """Does rumble the controller when in Manual Grabbing"""
            self.robot.cStick.setRumble(0, 0.5)
        
        """
        These send information over to the SmartDashboard
        """
        wpilib.SmartDashboard.putNumber("cubeGrabber Ultra Sonic", self.cubeDistanceIn)
        wpilib.SmartDashboard.putNumber("cubeGrabber Limit Switch", self.armSwitch.get())
        
    """
    Code for resetting the Cube cubeGrabber
    """
    def armGrabReset(self):
        """
        Stops the motors and returns arms to default open position.
        """
        self.leftArm.set(0)
        self.rightArm.set(0)
        
        self.armSolenoid.set(self.armOpenPosition)
    
    """
    Code for automatic cube grabbing
    """
    def armGrabAuto(self):
        """
        1) Checks if the left trigger on Driver Two's controller is pressed in more 
        than a quarter of the way down, then closes the Arms and spins the motors outwards.
        This spits out the cube and has first priority.
        
        2) Checks if the limit switch on the arm is pressed. 
        Will stop the motors and keep the arms closed and has second priority.
        
        3) Checks if the cube is close enough to start spinning the intake wheels inward and keeps the arms open.
        
        4) Checks if the cube is in range to close the arms. Spins motors inward and keeps arms closed.
        
        5)If nothing is sensed the arms will be in a default position with the motors stopped and the arms open.
        """
        if self.driverTwo.getRawAxis(2) > .25:
            self.leftArm.set(1)
            self.rightArm.set(-1)
            
            self.armSolenoid.set(self.armClosePosition)
            """This will prevent the arms from changing position when changing to Manual Mode."""
            self.openToggleCount = 2
        
        elif self.armSwitch.get() == 0:
            self.leftArm.set(0)
            self.rightArm.set(0)
            
            self.armSolenoid.set(self.armClosePosition)
            """This will prevent the arms from changing position when changing to Manual Mode."""
            self.openToggleCount = 2
            
        elif (self.cubeDistanceIn <= self.spinDistance) and (self.cubeDistanceIn > self.closeDistance):
            self.leftArm.set(-1)
            self.rightArm.set(1)
            
            self.armSolenoid.set(self.armOpenPosition)
            """This will prevent the arms from changing position when changing to Manual Mode."""
            self.openToggleCount = 3
            
        elif (self.cubeDistanceIn <= self.closeDistance):
            self.leftArm.set(-1)
            self.rightArm.set(1)
            
            self.armSolenoid.set(self.armClosePosition)
            """This will prevent the arms from changing position when changing to Manual Mode."""
            self.openToggleCount = 2
            
        else:
            self.leftArm.set(0)
            self.rightArm.set(0)
            
            self.armSolenoid.set(self.armOpenPosition)
            """This will prevent the arms from changing position when changing to Manual Mode."""
            self.openToggleCount = 3

    """
    Code for manual cube grabbing
    """
    def armGrabManual(self):
        """
        Combines the trigger axes on Driver Two's controller to create one "axis".
        """
        self.manualArmSpeed = -self.driverTwo.getRawAxis(2) + self.driverTwo.getRawAxis(3)
        
        """
        If statement adding to the toggle count that will determine whether the arms are closed or open.
        """
        if self.driverTwo.getRawButtonReleased(1):
            self.openToggleCount = self.openToggleCount + 1
           
        """
        If statement determining whether the arms are closed or opened 
        determined by the open toggle count remainder when divided by two.
        """ 
        if self.openToggleCount%2 == 0:
            self.armSolenoid.set(self.armClosePosition)
        else:
            self.armSolenoid.set(self.armOpenPosition)
            
        """
        Sets the speed of the intake wheels, using the makeshift axis made above as diret input.
        """    
        self.leftArm.set(-self.manualArmSpeed)
        self.rightArm.set(self.manualArmSpeed)
        
    def getSwitch(self):
        return self.armSwitch.get() == 1