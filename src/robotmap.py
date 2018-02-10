class RobotConstants():
    pass


kDriveTrain = RobotConstants()

# Controller IDs for the motors
# lmId is for the Left Master controller
# lsId is for the Left Slave controller
# Same applies for the right side: rmId, rsId
kDriveTrain.lmId = 1
kDriveTrain.rmId = 2
kDriveTrain.lsId = 3
kDriveTrain.rsId = 4

# Which slot to use for PID Gains
kDriveTrain.PIDslot = 0
# Which PID loop to use
kDriveTrain.PIDloop = 0
# Timout in ms to wait for command confirmation
kDriveTrain.PIDtimeout = 10


kStick = RobotConstants()

kStick.driveId = 0
kStick.controlId = 1
