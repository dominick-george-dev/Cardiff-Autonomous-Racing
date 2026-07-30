from enum import Enum
from typing import Optional
from nav_msgs.msg import Path


from mission_control.mission import Mission
from model.vehical_model import Vehicle_State
from MPC.main import Model_Predictive_Control

class Trackdrive(Mission):

    class TrackdriveFlag(Enum):
        """
        Represents the current progress of the Trackdrive mission
        and what the vehicle's current objective is.
        """
        TEMP = 0

    def __init__(self, trigger_ebs, timer_period, mpc_unit:Model_Predictive_Control):
        super().__init__(trigger_ebs=trigger_ebs)

        self.timer_start = 0
        self.progress_flag = self.TrackdriveFlag.TEMP

        self.time_at_event_start = 0
        self.timer_period = timer_period

        self.mpc_unit = mpc_unit


    def get_commands(self,current_state:Vehicle_State, desired_path: Optional[Path] = None):
        return 0.0,0.0