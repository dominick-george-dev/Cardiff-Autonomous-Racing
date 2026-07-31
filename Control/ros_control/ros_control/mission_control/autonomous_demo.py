from enum import Enum
from time import time
from typing import Optional


from mission_control.mission import Mission
from model.vehical_model import Vehicle_State
from nav_msgs.msg import Path


class Autonomous_Demo(Mission):

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

    def __init__(self, trigger_ebs, timer_period):
        super().__init__(trigger_ebs=trigger_ebs)

        self.progress_flag = self.demoFlag.LEFT

        self.timer_start = 0
        self.delta_t = 0

        self.u = 0

        # NOTE: Should really check if perception is working here, 
        # then use that for distance measurements rather than suvat equations,
        # as we cannot guaruantee a constant and accurate "a" value


    def get_commands(self,current_state:Vehicle_State, desired_path: Optional[Path] = None):
        acceleration = 0.0
        steering_angle = 0.0

        #steering left and right and return to straight
        if self.progress_flag == self.demoFlag.LEFT:
            self.logger.info("Sub task: steering left")
            steering_angle = 0.5
            if current_state.steering_angle_rad <= -0.3316:
                self.progress_flag = self.demoFlag.RIGHT

        if self.progress_flag == self.demoFlag.RIGHT:
            self.logger.info("Sub task: steering right")
            steering_angle = -0.5
            if current_state.steering_angle_rad >= 0.3316:
                self.progress_flag = self.demoFlag.CENTRE

        if self.progress_flag == self.demoFlag.CENTRE:
            self.logger.info("Sub task: Steer centre")
            steering_angle = 0.0
            if abs(current_state.steering_angle_rad) <= 0.01:
                self.progress_flag = self.demoFlag.ACCELERATE
                self.timer_start = time()

        #accellerate for 10m to at least 15kph
        if self.progress_flag == self.demoFlag.ACCELERATE:
            self.logger.info("Sub task: accelleration for 10m")
            acceleration = 2.0
            self.delta_t = time() - self.timer_start
            # check if 10m have passed using suvat
            if 0.5 * acceleration * self.delta_t**2 >= 10: # s = ut + (1/2)at^2 (u=0, so not used)
                # NOTE: this currently isnt a very good interpretation,
                # really we should accelerate to 15kph, then stabilise speed and calculate distance based on that, if we dont have perception
                self.progress_flag = self.demoFlag.BRAKE
                self.timer_start = time()
                self.delta_t = 0
                self.u = current_state.directional_velocity

        #stop within a furthur 10m
        if self.progress_flag == self.demoFlag.BRAKE:
            self.logger.info("Sub task: break to stop")

            acceleration = -2.0
            self.delta_t = self.timer_start - time()

            # decelerate at -2 for 8 metres, if we are still moving then brake harder
            if self.u * self.delta_t + 0.5 * acceleration * self.delta_t**2 >= 8:
                acceleration = -10.0

            if abs(current_state.wheels_rpm) <= 0.1:
                self.progress_flag = self.demoFlag.REPEAT
                self.timer_start = time()
                self.u = 0
                self.delta_t = 0

        #accellerate for a furthur 10m to at least 15kph
        if self.progress_flag == self.demoFlag.REPEAT:
            # NOTE: entire section could probably be made into a private method
            self.logger.info("Sub task: Accelerate a further 10m")
            acceleration = 2.0
            self.delta_t = time() - self.timer_start

            #check if 10m have passed using suvat
            if self.u * self.delta_t + 0.5 * acceleration * self.delta_t**2 >= 10: # s = ut + (1/2)at^2
                self.progress_flag = self.demoFlag.EMGCBRAKE

        #deploy ebs
        if self.progress_flag == self.demoFlag.EMGCBRAKE:
            self.logger.info("Sub task: Deploy the EBS")
            self.trigger_ebs()

            self.complete = True
            
        
        return float(acceleration),float(steering_angle)