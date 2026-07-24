from mission import Mission
from enum import Enum
from model.vehical_model import Vehicle_State
import time
from nav_msgs.msg import Path


class Autocross(Mission):

    class autocrossFlag(Enum):
        """
        Represents the current progress of the Autocross mission
        and what the vehicles current objective is.
        """

        START = 0
        LAP = 1
        BRAKE = 2
        COMPLETE = 3


    def __init__(self, logger, trigger_ebs, timer_period):
        super().__init__(logger=logger, trigger_ebs=trigger_ebs)

        self.timer_start = 0
        self.progress_flag = self.autocrossFlag.START

        self.time_at_event_start = 0
        self.timer_period = timer_period


    def get_commands(self,current_state:Vehicle_State, desired_path:Path):
        acceleration = 0.0
        steering_angle = 0.0
        self.logger().info("AS_Driving")
        #accelerate to start line
        if self.progress_flag == self.autocrossFlag.START:
            self.logger().info("Sub task: START")
            #go straight until start line crossed

            #if line crossed
            #    self.__autocrossFlag = autocrossFlag.LAP
        
        if self.progress_flag == self.autocrossFlag.LAP:
            self.logger().info("Sub task: LAP")
            #follow path using MPC inputs

            #if finish line crossed
            #    self.__autocrossFlag = autocrossFlag.BRAKE
            
        if self.progress_flag == self.autocrossFlag.BRAKE:
            self.logger().info("Sub task: BRAKE")
            #Brake until stopped

            if current_state.wheels_rpm <= 0.1:
                self.progress_flag = self.autocrossFlag.COMPLETE

        if self.progress_flag == self.autocrossFlag.COMPLETE:
            #set mission complete
            self.complete = True
            self.progress_flag = self.autocrossFlag.START


        return acceleration, steering_angle