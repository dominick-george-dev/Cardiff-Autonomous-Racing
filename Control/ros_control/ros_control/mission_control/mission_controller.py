from eufs_msgs.msg import CanState
from nav_msgs.msg import Path


from model.vehical_model import Vehicle_State
from MPC.main import Model_Predictive_Control
from mission_control.static_A import Static_A
from mission_control.static_B import Static_B
from mission_control.autonomous_demo import Autonomous_Demo
from mission_control.autocross import Autocross
from mission_control.acceleration import Acceleration
from mission_control.skidpad import Skidpad
from mission_control.trackdrive import Trackdrive

class Mission_Controller:
    def __init__(self,mpc_unit:Model_Predictive_Control,timer_period:float,logger,trigger_ebs) -> None:

        self.__ami_state = CanState.AMI_NOT_SELECTED
        self.__as_state = CanState.AS_OFF

        self.mpc_unit = mpc_unit
        self.timer_period = timer_period

        self.logger = logger#ros2 logger passed through from cmd node
        self.trigger_ebs = trigger_ebs# trigger ebs function passed through from cmd node

        self.current_mission = None
    

    def reset_mission_progress(self):
        self.current_mission.reset()

    def set_mission(self, AMI:CanState.ami_state):
        # choose mission based on new AMI
        if AMI == CanState.AMI_DDT_INSPECTION_A:
            self.current_mission = Static_A(logger=self.logger, trigger_ebs=self.trigger_ebs)
            self.logger().info(f"Mission switched to AMI: {AMI}, Static A")
        elif AMI == CanState.AMI_DDT_INSPECTION_B:
            self.current_mission = Static_B(logger=self.logger, trigger_ebs=self.trigger_ebs)
            self.logger().info(f"Mission switched to AMI: {AMI}, Static B")
        elif AMI == CanState.AMI_AUTONOMOUS_DEMO:
            self.current_mission = Autonomous_Demo(logger=self.logger, trigger_ebs=self.trigger_ebs, timer_period=self.timer_period)
            self.logger().info(f"Mission switched to AMI: {AMI}, Autonomous Demo")
        elif AMI == CanState.AMI_AUTOCROSS:
            self.current_mission = Autocross(logger=self.logger, trigger_ebs=self.trigger_ebs, timer_period=self.timer_period)
            self.logger().info(f"Mission switched to AMI: {AMI}, Autocross")
        elif AMI == CanState.AMI_ACCELERATION:
            self.current_mission = Acceleration(logger=self.logger, trigger_ebs=self.trigger_ebs, timer_period=self.timer_period, mpc_unit=self.mpc_unit)
            self.logger().info(f"Mission switched to AMI: {AMI}, Acceleration")
        elif AMI == CanState.AMI_SKIDPAD:
            self.current_mission = Skidpad(logger=self.logger, trigger_ebs=self.trigger_ebs, timer_period=self.timer_period, mpc_unit=self.mpc_unit)
            self.logger().info(f"Mission switched to AMI: {AMI}, Skidpad")
        elif AMI == CanState.AMI_TRACK_DRIVE:
            self.current_mission = Trackdrive(logger=self.logger, trigger_ebs=self.trigger_ebs, timer_period=self.timer_period, mpc_unit=self.mpc_unit)
            self.logger().info(f"Mission switched to AMI: {AMI}, Trackdrive")
        elif AMI == CanState.AMI_NOT_SELECTED:
            self.current_mission = None
            self.logger().info(f"Mission switched to AMI: {AMI}, Mission Not Selected")
        else:
            self.current_mission = None
            self.logger().warn(f"Attempted to switch to unknown mission, AMI: {AMI}. Switching to mission: None")

    def set_can_states(self,AMI:CanState.ami_state,AS:CanState.ami_state):

        if self.__ami_state != AMI: # if mission has changed
            self.reset_mission_progress()

            self.set_mission(AMI)
        
        self.__ami_state = AMI
        self.__as_state = AS
            

    def get_message(self,current_state:Vehicle_State,desired_path:Path=None) -> tuple[float, float]:

        # if not on a mission do not drive the car
        if self.current_mission == None:
            return 0.0, 0.0

        # if the current mission is not complete, and we are allowed to drive: get commands to send to the car
        if (not self.current_mission.is_complete()) and self.__as_state == CanState.AS_DRIVING:
            return self.current_mission.get_commands(current_state=current_state, desired_path=desired_path)
        else:
            return 0.0, 0.0

    
    def get_mission_complete(self) -> bool:
        if self.current_mission == None:
            return False
        else:
            return self.current_mission.is_complete()

if __name__ == "__main__":
    mission_controler = Mission_Controller()
    print(mission_controler.get_message())
