from enum import Enum

class AFlag(Enum):
    """
    Represents the current progress of the static inspection A mission
    and what the vehicle's current objective is.
    """
    LEFT = 0 #Turn Steering all the way to the left
    RIGHT = 1
    CENTRE = 2 #Centre the steering wheel
    ACCELERATE = 3 #Accelerate to 200rpm
    HOLD = 4 #Hold for 9.5 seconds
    BRAKE = 5 #Brake to complete stop
    COMPLETE = 6 #Mission A has been complete

class BFlag(Enum):
    """
    Represents the current progress of the static inspection B mission
    and what the vehicle's current objective is. 
    """
    TIMER_START = 0
    ACCELERATE = 1 #Accelerate to 50rpm
    HOLD = 2
    EMGCBRAKE = 3 #Activate emergency brake
    WAIT_FOR_STOP = 4
    COMPLETE = 5

class demoFlag(Enum):
    """
    Represents the current progress of the autonomous demonstration
    mission and what the vehicle's current objective is. 
    """
    LEFT = 0 #Turn Steering all the way to the left
    RIGHT = 1
    CENTRE = 2 #Centre the steering wheel
    ACCELERATE = 3 #Accelerate for 10m
    BRAKE = 4 #Brake to complete stop
    REPEAT = 5 #Accelerate for another 10m
    EMGCBRAKE = 6 #Activate emergency brake

class SkidpanFlag(Enum):
    """
    Represents the current progress of the Skidpan mission
    and what the vehicle's current objective is.
    """
    StraightToTimekeepingLine = 0
    Right = 1
    Left = 2
    StopInZone = 3

class accelFlag(Enum):
    """
    Represents the current progress of the Accelleration
    mission and what the vehicle's current objective is. 
    """
    ACCELERATE = 0
    BRAKE = 1
    COMPLETE = 2


class autocrossFlag(Enum):
    """
    Represents the current progress of the Autocross mission
    and what the vehicles current objective is.
    """

    START = 0
    LAP = 1
    BRAKE = 2
    COMPLETE = 3
