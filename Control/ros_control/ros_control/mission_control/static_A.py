from mission import Mission
from enum import Enum
from model.vehical_model import Vehicle_State
import time
from nav_msgs.msg import Path


class Static_A(Mission):

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

    def __init__(self, logger, trigger_ebs):
        super().__init__(logger=logger, trigger_ebs=trigger_ebs)

        self.timer_start = 0
        self.progress_flag = self.AFlag.LEFT

    def get_commands(self,current_state:Vehicle_State, desired_path:Path= None):
        self.logger().info("AS_Driving")
        
        acceleration = 0.0
        steering_angle = 0.0
        #steer all the way one way
        if self.progress_flag == self.AFlag.LEFT:
            self.logger().info("Sub task: Steer left")

            steering_angle = 0.5
            if current_state.steering_angle_rad <= -0.3316:#0.41:
                self.progress_flag = self.AFlag.RIGHT
        #steer all the way in the opposite direction
        if self.progress_flag == self.AFlag.RIGHT:
            self.logger().info("Sub task: Steer right")
            steering_angle = -0.5
            if current_state.steering_angle_rad >= 0.3316:#-0.41:
                self.progress_flag = self.AFlag.CENTRE
        #steering back to centre
        if self.progress_flag == self.AFlag.CENTRE:
            self.logger().info("Sub task: Steer centre")
            steering_angle = 0.0
            if abs(current_state.steering_angle_rad) <= 0.01:
                self.progress_flag = self.AFlag.ACCELERATE
                self.timer_start = time.time()
        #wheels to 200rpm
        if self.progress_flag == self.AFlag.ACCELERATE:
            self.logger().info("Sub task: Accelerate to 200rpm")
            self.logger().info(f"Current RPM: {current_state.wheels_rpm}")
            if current_state.wheels_rpm < 200.0:
                acceleration = 0.25
            else:
                self.progress_flag = self.AFlag.HOLD
                
        if self.progress_flag == self.AFlag.HOLD:
            self.logger().info("Hold for 10 seconds")
            self.logger().info(f"Current RPM: {current_state.wheels_rpm}")
            """if current_state.wheels_rpm < 195.0:
                acceleration = 0.25
            elif current_state.wheels_rpm < 205.0:
                acceleration = 0.1
            else:
                acceleration = 0.0"""

            #NOTE: REPLACE WITH A PID CONTROLLER
            if self.progress_flag == self.AFlag.HOLD:
                rpm_error = 200.0 - current_state.wheels_rpm
                kp = 0.01
                acceleration = max(min(kp * rpm_error, 0.25), -0.1)

            
            if time.time() - self.timer_start > 10:# if 10 seconds has passed
                self.progress_flag = self.AFlag.BRAKE
        
        #stop car
        if self.progress_flag == self.AFlag.BRAKE:
            self.logger().info("Sub task: Brake to zero")
            self.logger().info(f"Current RPM: {current_state.wheels_rpm}")

            acceleration = -2.0
            if current_state.wheels_rpm <= 0.1:
                self.progress_flag = self.AFlag.COMPLETE
        # set AS_FINISHED
        if self.progress_flag == self.AFlag.COMPLETE and not self.complete:
            #set mission complete
            self.complete = True
            self.progress_flag = self.AFlag.LEFT
            #self.logger().info("Mission complete published!")
        
        return float(acceleration), float(steering_angle)