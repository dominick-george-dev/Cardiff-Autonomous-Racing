from typing import Optional
from nav_msgs.msg import Path


from mission_control.mission import Mission
from model.vehical_model import Vehicle_State
from controllers.main import Model_Predictive_Control

class NotSelected(Mission):


    def __init__(self, trigger_ebs):
        super().__init__(trigger_ebs=trigger_ebs)


    def get_commands(self,current_state:Vehicle_State, desired_path: Optional[Path] = None):
        return 0.0,0.0