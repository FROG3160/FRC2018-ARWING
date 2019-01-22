import time
import wpilib
from networktables import NetworkTables
from wpilib.command.subsystem import Subsystem

# To see messages from networktables, you must setup logging
import logging
logging.basicConfig(level=logging.DEBUG)

class vision(Subsystem):
    
    NetworkTables.initialize()
    table = NetworkTables.getTable('hatch')
    
    targetCenter_x = table.getNumberArray('x', (None))
    targetCenter_y = table.getNumberArray('y', (None))
    print (targetCenter_x)