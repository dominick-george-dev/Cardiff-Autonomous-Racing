from enum import Enum
import math
from typing import Optional
from nav_msgs.msg import Path


from mission_control.mission import Mission
from model.vehical_model import Vehicle_State
from controllers.MPC import Model_Predictive_Control


class Acceleration(Mission):

    class accelFlag(Enum):
        """
        Represents the current progress of the Accelleration
        mission and what the vehicle's current objective is. 
        """
        ACCELERATE = 0
        BRAKE = 1
        COMPLETE = 2

    def __init__(self, trigger_ebs, timer_period, mpc_unit:Model_Predictive_Control):
        super().__init__(trigger_ebs=trigger_ebs)

        self.timer_start = 0
        self.progress_flag = self.accelFlag.ACCELERATE

        self.time_at_event_start = 0
        self.timer_period = timer_period

        self.mpc_unit = mpc_unit


    def get_commands(self,current_state:Vehicle_State, desired_path: Optional[Path] = None):
        #set default commands
        acceleration = 0.0
        steering_angle = 0.0

        # Make sure there is a path
        if desired_path == None:
            self.logger.error("No path given, commands 0, 0 sent")
            return acceleration, steering_angle
        
        self.logger.info("AS_Driving")
        # accelerate along path
        if self.progress_flag == self.accelFlag.ACCELERATE:
            self.logger.info("Sub task: Accelerate")
            distance_travelled = math.sqrt(current_state.xpos**2 + current_state.ypos**2) #distance to origin (0,0)
            if distance_travelled < 75:
                try:
                    commands = self.mpc_unit.main(initial_state=current_state,required_path=desired_path)#get required path from path planning, not sure where to get initial state from
                    acceleration = commands[0].acceleration 
                    steering_angle = commands[0].steering_angle
                except Exception as e:
                    acceleration = 0.0
                    steering_angle = 0.0
            else:
                self.progress_flag = self.accelFlag.BRAKE
        #Brake after 75m travelled 
        if self.progress_flag == self.accelFlag.BRAKE: 
            self.logger.info("Sub task: Brake")
            try:
                commands = self.mpc_unit.main(initial_state=current_state,required_path=desired_path)#let mpc brake and make any steering adjustments needed
                #cost function will need to now reward braking and massivly penalise accelerating or high speeds
                acceleration = commands[0].acceleration 
                steering_angle = commands[0].steering_angle
            except Exception as e:
                acceleration = -3.0 #might need to be increased
                steering_angle = 0.0

            if current_state.wheels_rpm <= 0.1:
                self.progress_flag = self.accelFlag.COMPLETE

        if self.progress_flag == self.accelFlag.COMPLETE and not self.complete:
            #set mission complete
            self.complete = True
            self.progress_flag = self.accelFlag.ACCELERATE
            #self.logger().info("Mission complete published!")

        # try:
        #     commands = self.mpc_unit.main(initial_state=current_state,required_path=desired_path)#get required path from path planning, not sure where to get initial state from
        #     acceleration = commands.acceleration 
        #     steering_angle = commands.steering_angle
        # except Exception as e:
        #     if current_state.directional_velocity < 5.0 or current_state.wheels_rpm < 200:
        #         pass
        
        # acceleration = 1.0 # make sure these are floats
        # steering_angle = 0.0
        # self.logger().info(f'created accelleration command')
        # # msg.drive.steering_angle_velocity
        # # msg.drive.jerk
        # #self.publisher_.publish(msg)
        # #self.logger().info(f'Publishing: "{msg.drive}"')
        # #self.logger().info(f'Publishing: "{msg.drive}" \n & {msg.header}')
        return float(acceleration),float(steering_angle)
