from enum import Enum
import time
from nav_msgs.msg import Path


from mission import Mission
from model.vehical_model import Vehicle_State


class Static_B(Mission):

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

    def __init__(self, logger, trigger_ebs):
        super().__init__(logger=logger, trigger_ebs=trigger_ebs)

        self.timer_start = 0
        self.progress_flag = self.BFlag.LEFT

    def get_commands(self,current_state:Vehicle_State, desired_path:Path= None):
        acceleration = 0.0
        steering_angle = 0.0

        if self.progress_flag == self.BFlag.TIMER_START:
            self.timer_start = time.time()
            self.progress_flag = self.BFlag.ACCELERATE

        if self.progress_flag == self.BFlag.ACCELERATE:
            self.logger().info("Sub task: accelerate to 50rpm")
            if current_state.wheels_rpm < 50.0:
                acceleration = 0.1
            else:
                self.progress_flag = self.BFlag.HOLD
        
        if self.progress_flag == self.BFlag.HOLD:
            #NOTE: REPLACE WITH A PID CONTROLLER
            rpm_error = 50.0 - current_state.wheels_rpm
            kp = 0.01
            acceleration = max(min(kp * rpm_error, 0.25), -0.1)

            if time.time() - self.timer_start > 8:
                self.progress_flag = self.BFlag.EMGCBRAKE


        if self.progress_flag == self.BFlag.EMGCBRAKE:
            self.trigger_ebs()
            self.progress_flag = self.BFlag.WAIT_FOR_STOP
        
        if self.progress_flag == self.BFlag.WAIT_FOR_STOP:
            if current_state.directional_velocity <= 0.1:
                self.complete = True
                self.progress_flag = self.BFlag.ACCELERATE
        
        return float(acceleration), float(steering_angle)