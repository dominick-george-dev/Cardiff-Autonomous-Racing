from enum import Enum


from mission import Mission
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

    def __init__(self, logger, trigger_ebs, timer_period):
        super().__init__(logger=logger, trigger_ebs=trigger_ebs)

        self.timer_start = 0
        self.progress_flag = self.demoFlag.LEFT

        self.time_at_event_start = 0
        self.timer_period = timer_period


    def get_commands(self,current_state:Vehicle_State, desired_path:Path= None):
        acceleration = 0.0
        steering_angle = 0.0
        #steering left and right and return to straight
        if self.progress_flag == self.demoFlag.LEFT:
            self.logger().info("Sub task: steering left")
            steering_angle = 0.5
            if current_state.steering_angle_rad >= 0.41:
                self.progress_flag = self.demoFlag.RIGHT
        if self.progress_flag == self.demoFlag.RIGHT:
            self.logger().info("Sub task: steering right")
            steering_angle = -0.5
            if current_state.steering_angle_rad <= -0.41:
                self.progress_flag = self.demoFlag.CENTRE
        if self.progress_flag == self.demoFlag.CENTRE:
            self.logger().info("Sub task: Steer centre")
            steering_angle = 0.0
            if abs(current_state.steering_angle_rad) <= 0.01:
                self.progress_flag = self.demoFlag.ACCELERATE
        #accellerate for 10m to at least 15kph
        if self.progress_flag == self.demoFlag.ACCELERATE: # THIS DOES NOT CURRENTLY WORK (distance check is always true)
            self.set_time_at_event_start(self.i)
            self.logger().info("Sub task: accelleration for 10m")
            acceleration = 2.0
            #check if 10m have passed using suvat
            if 0.5 * 2.0 * (((self.i-self.time_at_event_start)/self.timer_period)**2) >= 10:
                self.progress_flag = self.demoFlag.BRAKE
        #stop within a furthur 10m
        if self.progress_flag == self.demoFlag.BRAKE:
            self.logger().info("Sub task: break to stop")
            acceleration = -2.0
            if current_state.wheels_rpm == 0.0:
                self.progress_flag = self.demoFlag.REPEAT
                self.time_at_event_start = 0 #ONLY DO THIS IN BITS OF CODE NOT IN A LOOP, otherwise your timer will be continually reset
        #accellerate for a furthur 10m to at least 15kph
        if self.progress_flag == self.demoFlag.REPEAT:
            self.set_time_at_event_start(self.i)
            self.logger().info("Sub task: Accelerate a further 10m")
            acceleration = 2.0
            #check if 10m have passed using suvat
            if 0.5 * 2.0 * (((self.i-self.time_at_event_start)/self.timer_period)**2) >= 10:
                self.progress_flag = self.demoFlag.EMGCBRAKE
                self.time_at_event_start = 0
        #deploy ebs
        if self.progress_flag == self.demoFlag.EMGCBRAKE:
            self.trigger_ebs()
            self.progress_flag = self.demoFlag.LEFT
        
        return float(acceleration),float(steering_angle)