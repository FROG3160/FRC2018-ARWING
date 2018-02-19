"""
These are all the input and output port numbers, 
as well as CAN IDs, edit this to match what is 
plugged into the robot.
"""

DriveTrain = {'left_master':1,
              'left_slave':3,
              'right_master':2,
              'right_slave':4,
}

CubeGrabber = {'left_arm':5,
               'right_arm':6,
               'solenoid':0,
               'ultra_sonic':3,
               'switch':0,
               'open':False,
               'close':True
}

Elevator = {'elevator_motor':7,
            'bottom_switch':2
}

Climber = {'climb_motor': 8,
           'solenoid':1,
           'check_switch':1,
           'lights': " "
}

"""PCM CAN ID"""
PCM = 0



Sticks = {'drive':0,
          'control':1,
}



