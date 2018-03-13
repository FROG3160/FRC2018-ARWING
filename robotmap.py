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
               'right_arm':7,
               'solenoid':1,
               'ultra_sonic':1,
               'switch':2,
               'close':False,
               'open':True
}

Elevator = {'elevator_motor':6,
            'bottom_switch':4
}

Climber = {'climb_motor': 8,
           'solenoid':0,
           'solenoid_2':2,
           'check_switch':0,
           'lights': " "
}

"""PCM CAN ID"""
PCM = 10



Sticks = {'drive':0,
          'control':1,
}



