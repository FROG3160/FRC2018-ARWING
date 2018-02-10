import wpilib

from wpilib import SmartDashboard
from wpilib.buttons import Button

class OperatorInterface:

	def __init__(self, robot):

		p_up = Button('')

        # Connect the buttons to commands
d_up.whenPressed(SetElevatorSetpoint(robot, 0.2))