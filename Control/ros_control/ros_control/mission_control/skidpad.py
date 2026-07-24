from enum import Enum
from nav_msgs.msg import Path


from mission import Mission
from model.vehical_model import Vehicle_State
from MPC.main import Model_Predictive_Control

class Skidpad(Mission):

    class SkidpanFlag(Enum):
        """
        Represents the current progress of the Skidpan mission
        and what the vehicle's current objective is.
        """
        StraightToTimekeepingLine = 0
        Right = 1
        Left = 2
        StopInZone = 3

    def __init__(self, logger, trigger_ebs, timer_period, mpc_unit:Model_Predictive_Control):
        super().__init__(logger=logger, trigger_ebs=trigger_ebs)

        self.timer_start = 0
        self.progress_flag = self.SkidpanFlag.StraightToTimekeepingLine

        self.time_at_event_start = 0
        self.timer_period = timer_period

        self.mpc_unit = mpc_unit


    def get_commands(self,current_state:Vehicle_State, desired_path:Path):
        self.logger().info("AS_Skidpan")

        acceleration = 0.0
        steering_angle = 0.0

        if self.progress_flag == self.SkidpanFlag.StraightToTimekeepingLine:
            # go straight until line crossed
            acceleration = 2.0
            # if line crossed
                #self.__skidpan_flag = SkidpanFlag.Right
        if self.progress_flag == self.SkidpanFlag.Right:
            pass
            # go around the right loop twice

            # if line crossed twice
                #self.__skidpan_flag = SkidpanFlag.Left
        if self.progress_flag == self.SkidpanFlag.Left:
            pass
            # go around the left loop twice

            # if timing line crossed twice
                #self.__skidpan_flag = SkidpanFlag.StopInZone
        if self.progress_flag == self.SkidpanFlag.StopInZone:
            # stop the car in the finish zone

            # rn just slow car down with -acc, in the future have a cost function specifically for stopping at a point
            acceleration = -5

            if current_state.wheels_rpm < 0.1 and not self.complete:
                self.complete = True


        return acceleration, steering_angle