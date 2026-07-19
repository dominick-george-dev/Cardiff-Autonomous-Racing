from eufs_msgs.msg import CanState
from MPC.main import Model_Predictive_Control
from model.vehical_model import Vehicle_State
from nav_msgs.msg import Path
from mission_flag_enums import *
import math

class Mission_Control:
    #static inspection A (IN2.2.2): ramp drivetrain to 200rpm in 10s, stop within 5s
    STATIC_A_TARGET_RPM = 200.0
    STATIC_A_RAMP_TIME_s = 8.0#rules allow 10s, ramp in 8 for margin
    STATIC_A_RAMP_LIMIT_s = 12.0#give up waiting for the target and brake anyway
    #static inspection B (IN2.2.2): spin drivetrain to 50rpm then trigger the EBS
    STATIC_B_TARGET_RPM = 50.0
    #trackdrive (D8): 10 self-counted laps, stop within 30m after the finish line
    TRACK_TOTAL_LAPS = 10
    TRACK_LAP_RADIUS_m = 4.0#how close to the staging point counts as completing a lap
    TRACK_REARM_DIST_m = 15.0#must get this far from the staging point before a lap can count
    TRACK_FINISH_DIST_m = 8.0#staged 6m before the line (D8.2.5), so this is ~2m past it
    STOPPED_RPM = 5.0#the EMA-filtered wheel rpm decays slowly, 0.1 takes too long to reach

    def __init__(self,mpc_unit:Model_Predictive_Control,timer_period:float,logger,trigger_ebs) -> None:
        self.__static_A_flag = AFlag.LEFT#flag that indicates the progress through the static inspection A mission
        self.__static_B_flag = BFlag.ACCELERATE#flag that indicates the progress through the static inspection B mission
        self.__autonomous_demo_flag = demoFlag.LEFT#flag that indicates the progress through the autonomous demonstration mission
        self.__skidpan_flag = SkidpanFlag.StraightToTimekeepingLine # flag that indicates the progress through the skipan mission

        self.__accelFlag = accelFlag.ACCELERATE#Flag that indicates the progress through the accelleration mission
        self.__autocrossFlag = autocrossFlag.START#Flag that indicates the progress through the autocross mission
        self.__track_flag = TrackFlag.START#Flag that indicates the progress through the trackdrive mission
        self.__track_laps = 0#laps completed so far (counted autonomously, D8.2.8)
        self.__track_lap_armed = False#True once far enough from the staging point to count the next lap
        self.__track_start_x = 0.0#staging position, recorded on the first trackdrive tick
        self.__track_start_y = 0.0
        self.__mission_complete = False

        self.__ami_state = CanState.AMI_NOT_SELECTED
        self.__as_state = CanState.AS_OFF

        self.mpc_unit = mpc_unit
        self.timer_period = timer_period

        self.logger = logger#ros2 logger passed through from cmd node
        self.trigger_ebs = trigger_ebs# trigger ebs function passed through from cmd node

        self.i = 0
        self.time_at_event_start = 0
    

    def reset_mission_progress(self):
        self.__static_A_flag = AFlag.LEFT
        self.__static_B_flag = BFlag.ACCELERATE
        self.__autonomous_demo_flag = demoFlag.LEFT
        self.__skidpan_flag = SkidpanFlag.StraightToTimekeepingLine
        self.__accelFlag = accelFlag.ACCELERATE
        self.__autocrossFlag = autocrossFlag.START
        self.__track_flag = TrackFlag.START
        self.__track_laps = 0
        self.__track_lap_armed = False
        self.__mission_complete = False
        self.time_at_event_start = 0

    def __acceleration(self,current_state:Vehicle_State,desired_path:Path) -> tuple[float, float]:
        acceleration = 0.0
        steering_angle = 0.0
        # msg.drive.speed=10.0    
        self.logger().info("AS_Driving")
        # accelerate along path
        if self.__accelFlag == accelFlag.ACCELERATE:
            self.logger().info("Sub task: Accelerate")
            distance_travelled = math.sqrt(current_state.xpos**2 + current_state.ypos**2) #distance to origin (0,0)
            if distance_travelled < 75:
                try:
                    commands = self.mpc_unit.main(initial_state=current_state,required_path=desired_path)#get required path from path planning, not sure where to get initial state from
                    acceleration = commands.acceleration 
                    steering_angle = commands.steering_angle
                except Exception as e:
                    acceleration = 0.0
                    steering_angle = 0.0
            else:
                self.__accelFlag = accelFlag.BRAKE
        #Brake after 75m travelled 
        if self.__accelFlag == accelFlag.BRAKE: 
            self.logger().info("Sub task: Brake")
            try:
                commands = self.mpc_unit.main(initial_state=current_state,required_path=desired_path)#let mpc brake and make any steering adjustments needed
                #cost function will need to now reward braking and massivly penalise accelerating or high speeds
                acceleration = commands.acceleration 
                steering_angle = commands.steering_angle
            except Exception as e:
                acceleration = -3.0 #might need to be increased
                steering_angle = 0.0

            if current_state.wheels_rpm <= 0.1:
                self.__accelFlag = accelFlag.COMPLETE

        if self.__accelFlag == accelFlag.COMPLETE and not self.__mission_complete:
            #set mission complete
            self.__mission_complete = True
            self.__accelFlag = accelFlag.ACCELERATE
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

    def __skidpan(self, current_state:Vehicle_State, desiredPath:Path) -> tuple[float,float]:

        self.logger().info("AS_Skidpan")

        acceleration = 0.0
        steering_angle = 0.0

        if self.__skidpan_flag == SkidpanFlag.StraightToTimekeepingLine:
            # go straight until line crossed
            acceleration = 2.0
            # if line crossed
                #self.__skidpan_flag = SkidpanFlag.Right
        if self.__skidpan_flag == SkidpanFlag.Right:
            pass
            # go around the right loop twice

            # if line crossed twice
                #self.__skidpan_flag = SkidpanFlag.Left
        if self.__skidpan_flag == SkidpanFlag.Left:
            pass
            # go around the left loop twice

            # if timing line crossed twice
                #self.__skidpan_flag = SkidpanFlag.StopInZone
        if self.__skidpan_flag == SkidpanFlag.StopInZone:
            # stop the car in the finish zone

            # rn just slow car down with -acc, in the future have a cost function specifically for stopping at a point
            acceleration = -5

            if current_state.wheels_rpm < 0.1 and not self.__mission_complete:
                self.__mission_complete = True


        return acceleration, steering_angle

    def __track_drive(self,current_state:Vehicle_State,desired_path:Path) -> tuple[float,float]:
        acceleration = 0.0
        steering_angle = 0.0
        self.logger().info("AS_Driving")

        #record the staging position - the car starts 6m before the start line (D8.2.5)
        if self.__track_flag == TrackFlag.START:
            self.__track_start_x = current_state.xpos
            self.__track_start_y = current_state.ypos
            self.__track_laps = 0
            self.__track_lap_armed = False
            self.__track_flag = TrackFlag.LAP

        dist_from_start = math.hypot(current_state.xpos - self.__track_start_x,
                                     current_state.ypos - self.__track_start_y)

        #follow the planned path with MPC while lapping, crossing the line and braking
        if self.__track_flag in (TrackFlag.LAP, TrackFlag.FINISH, TrackFlag.BRAKE):
            try:
                commands = self.mpc_unit.main(initial_state=current_state,required_path=desired_path)
                acceleration = commands.acceleration
                steering_angle = commands.steering_angle
            except Exception as e:
                acceleration = 0.0
                steering_angle = 0.0

        if self.__track_flag == TrackFlag.LAP:
            self.logger().info(f"Sub task: LAP {self.__track_laps + 1}/{self.TRACK_TOTAL_LAPS}")
            #count laps by passing the staging point again - there is no last lap
            #signal, the vehicle must count laps itself (D8.2.8)
            if not self.__track_lap_armed and dist_from_start > self.TRACK_REARM_DIST_m:
                self.__track_lap_armed = True
            if self.__track_lap_armed and dist_from_start < self.TRACK_LAP_RADIUS_m:
                self.__track_laps += 1
                self.__track_lap_armed = False
                self.logger().info(f"Lap {self.__track_laps} complete")
                if self.__track_laps >= self.TRACK_TOTAL_LAPS:
                    self.__track_flag = TrackFlag.FINISH

        #keep driving until the finish line is behind us, timing stops when the
        #vehicle crosses it (D8.2.6), only then brake
        if self.__track_flag == TrackFlag.FINISH:
            self.logger().info("Sub task: crossing finish line")
            if dist_from_start >= self.TRACK_FINISH_DIST_m:
                self.__track_flag = TrackFlag.BRAKE

        #full stop within 30m after the finish line (D8.2.7)
        if self.__track_flag == TrackFlag.BRAKE:
            self.logger().info(f"Sub task: braking to stop ({current_state.wheels_rpm:.0f}rpm)")
            acceleration = -6.0#below ros_can's engine_threshold: 60% mechanical braking
            if current_state.wheels_rpm <= self.STOPPED_RPM:
                self.__track_flag = TrackFlag.COMPLETE

        #enter AS Finished (D8.2.7)
        if self.__track_flag == TrackFlag.COMPLETE and not self.__mission_complete:
            self.__mission_complete = True

        return float(acceleration),float(steering_angle)

    def __autocross(self, current_state:Vehicle_State, desired_path:Path) -> tuple[float,float]:
        acceleration = 0.0
        steering_angle = 0.0
        self.logger().info("AS_Driving")
        #accelerate to start line
        if self.__autocrossFlag == autocrossFlag.START:
            self.logger().info("Sub task: START")
            #go straight until start line crossed

            #if line crossed
            #    self.__autocrossFlag = autocrossFlag.LAP
        
        if self.__autocrossFlag == autocrossFlag.LAP:
            self.logger().info("Sub task: LAP")
            #follow path using MPC inputs

            #if finish line crossed
            #    self.__autocrossFlag = autocrossFlag.BRAKE
            
        if self.__autocrossFlag == autocrossFlag.BRAKE:
            self.logger().info("Sub task: BRAKE")
            #Brake until stopped

            if current_state.wheels_rpm <= 0.1:
                self.__autocrossFlag = autocrossFlag.COMPLETE

        if self.__autocrossFlag == autocrossFlag.COMPLETE:
            #set mission complete
            self.__mission_complete = True
            self.__autocrossFlag = autocrossFlag.START


        return acceleration, steering_angle

    def __static_A(self,current_state:Vehicle_State) -> tuple[float, float]:
        self.logger().info("AS_Driving")

        acceleration = 0.0
        steering_angle = 0.0
        #steer all the way one way
        if self.__static_A_flag == AFlag.LEFT:
            self.logger().info("Sub task: Steer left")

            steering_angle = 0.5
            if current_state.steering_angle_rad <= -0.3316:#0.41:
                self.__static_A_flag = AFlag.RIGHT
        #steer all the way in the opposite direction
        if self.__static_A_flag == AFlag.RIGHT:
            self.logger().info("Sub task: Steer right")
            steering_angle = -0.5
            if current_state.steering_angle_rad >= 0.3316:#-0.41:
                self.__static_A_flag = AFlag.CENTRE
        #steering back to centre
        if self.__static_A_flag == AFlag.CENTRE:
            self.logger().info("Sub task: Steer centre")
            steering_angle = 0.0
            if abs(current_state.steering_angle_rad) <= 0.01:
                self.__static_A_flag = AFlag.ACCELERATE
        #wheels to 200rpm - must ramp up in 10 seconds (IN2.2.2)
        if self.__static_A_flag == AFlag.ACCELERATE:
            self.set_time_at_event_start(self.i)
            elapsed_s = (self.i - self.time_at_event_start) * self.timer_period
            self.logger().info(f"Sub task: Ramp to 200rpm ({elapsed_s:.1f}s, {current_state.wheels_rpm:.0f}rpm)")
            #linear ramp target so 200rpm is reached in ~8s
            target_rpm = min(self.STATIC_A_TARGET_RPM,
                             self.STATIC_A_TARGET_RPM * elapsed_s / self.STATIC_A_RAMP_TIME_s)
            #even the smallest positive acceleration maps to ~19Nm of torque in ros_can,
            #which spins free wheels up fast, so pulse the drive only while below the ramp
            if current_state.wheels_rpm < target_rpm:
                acceleration = 0.3
            else:
                acceleration = 0.0
            if current_state.wheels_rpm >= self.STATIC_A_TARGET_RPM:
                self.__static_A_flag = AFlag.BRAKE
                self.time_at_event_start = 0
            elif elapsed_s > self.STATIC_A_RAMP_LIMIT_s:
                self.logger().warn(f"200rpm not reached after {elapsed_s:.1f}s, braking anyway")
                self.__static_A_flag = AFlag.BRAKE
                self.time_at_event_start = 0
        #stop car - must stop within 5 seconds (IN2.2.2)
        if self.__static_A_flag == AFlag.BRAKE:
            self.logger().info(f"Sub task: Brake to zero ({current_state.wheels_rpm:.0f}rpm)")
            #-4.0 is above ros_can's engine_threshold, giving speed-controlled engine
            #braking to 0rpm, which stops the free wheels well inside 5s
            acceleration = -4.0
            if current_state.wheels_rpm <= self.STOPPED_RPM:
                self.__static_A_flag = AFlag.COMPLETE
        # set AS_FINISHED
        if self.__static_A_flag == AFlag.COMPLETE and not self.__mission_complete:
            #set mission complete
            self.__mission_complete = True
            self.__static_A_flag = AFlag.LEFT
            #self.logger().info("Mission complete published!")
        
        return float(acceleration), float(steering_angle)

    def __static_B(self,current_state:Vehicle_State) -> tuple[float, float]:
        acceleration = 0.0
        steering_angle = 0.0
        #spin the drivetrain up to 50rpm (IN2.2.2)
        if self.__static_B_flag == BFlag.ACCELERATE:
            self.logger().info(f"Sub task: spin up to 50rpm ({current_state.wheels_rpm:.0f}rpm)")
            if current_state.wheels_rpm < self.STATIC_B_TARGET_RPM:
                acceleration = 0.3#gentle: ~30Nm in ros_can, plenty for free wheels
            else:
                self.__static_B_flag = BFlag.EMGCBRAKE
        #trigger the EBS, the VCU then sets the state to AS Emergency (IN2.2.2)
        if self.__static_B_flag == BFlag.EMGCBRAKE:
            self.logger().info("Sub task: triggering EBS")
            self.trigger_ebs()
            self.__static_B_flag = BFlag.WAIT_FOR_STOP

        #once the EBS fires the VCU leaves AS_DRIVING, so this state normally never
        #runs again - it is only a fallback in case the AS state lingers
        if self.__static_B_flag == BFlag.WAIT_FOR_STOP:
            if current_state.wheels_rpm <= self.STOPPED_RPM:
                self.__mission_complete = True
                self.__static_B_flag = BFlag.COMPLETE

        return float(acceleration), float(steering_angle)


    def __autonomous_demo(self,current_state:Vehicle_State) -> tuple[float, float]:
        acceleration = 0.0
        steering_angle = 0.0
        #steering left and right and return to straight
        if self.__autonomous_demo_flag == demoFlag.LEFT:
            self.logger().info("Sub task: steering left")
            steering_angle = 0.5
            if current_state.steering_angle_rad >= 0.41:
                self.__autonomous_demo_flag = demoFlag.RIGHT
        if self.__autonomous_demo_flag == demoFlag.RIGHT:
            self.logger().info("Sub task: steering right")
            steering_angle = -0.5
            if current_state.steering_angle_rad <= -0.41:
                self.__autonomous_demo_flag = demoFlag.CENTRE
        if self.__autonomous_demo_flag == demoFlag.CENTRE:
            self.logger().info("Sub task: Steer centre")
            steering_angle = 0.0
            if abs(current_state.steering_angle_rad) <= 0.01:
                self.__autonomous_demo_flag = demoFlag.ACCELERATE
        #accellerate for 10m to at least 15kph
        if self.__autonomous_demo_flag == demoFlag.ACCELERATE: # THIS DOES NOT CURRENTLY WORK (distance check is always true)
            self.set_time_at_event_start(self.i)
            self.logger().info("Sub task: accelleration for 10m")
            acceleration = 2.0
            #check if 10m have passed using suvat
            if 0.5 * 2.0 * (((self.i-self.time_at_event_start)/self.timer_period)**2) >= 10:
                self.__autonomous_demo_flag = demoFlag.BRAKE
        #stop within a furthur 10m
        if self.__autonomous_demo_flag == demoFlag.BRAKE:
            self.logger().info("Sub task: break to stop")
            acceleration = -2.0
            if current_state.wheels_rpm == 0.0:
                self.__autonomous_demo_flag = demoFlag.REPEAT
                self.time_at_event_start = 0 #ONLY DO THIS IN BITS OF CODE NOT IN A LOOP, otherwise your timer will be continually reset
        #accellerate for a furthur 10m to at least 15kph
        if self.__autonomous_demo_flag == demoFlag.REPEAT:
            self.set_time_at_event_start(self.i)
            self.logger().info("Sub task: Accelerate a further 10m")
            acceleration = 2.0
            #check if 10m have passed using suvat
            if 0.5 * 2.0 * (((self.i-self.time_at_event_start)/self.timer_period)**2) >= 10:
                self.__autonomous_demo_flag = demoFlag.EMGCBRAKE
                self.time_at_event_start = 0
        #deploy ebs
        if self.__autonomous_demo_flag == demoFlag.EMGCBRAKE:
            self.trigger_ebs()
            self.__autonomous_demo_flag = demoFlag.LEFT
        
        return float(acceleration),float(steering_angle)

    def set_can_states(self,AMI:CanState.ami_state,AS:CanState.ami_state):
        #mirror ros_can: start every mission from a clean state when the car
        #drops back to AS_OFF (e.g. officials reset between attempts)
        if AS == CanState.AS_OFF and self.__as_state != CanState.AS_OFF:
            self.reset_mission_progress()
        self.__ami_state = AMI
        self.__as_state = AS

    def get_message(self,current_state:Vehicle_State,desired_path:Path=None) -> tuple[float, float]:
        self.i+=1

        if not self.__mission_complete:
            if self.__ami_state == CanState.AMI_ACCELERATION:
                if self.__as_state == CanState.AS_DRIVING:
                    return self.__acceleration(current_state,desired_path)
            elif self.__ami_state == CanState.AMI_SKIDPAD:
                if self.__as_state == CanState.AS_DRIVING:
                    return self.__skidpan(current_state=current_state, desiredPath=desired_path)
            elif self.__ami_state == CanState.AMI_AUTOCROSS:
                if self.__as_state == CanState.AS_DRIVING:
                    return self.__autocross(current_state=current_state, desired_path=desired_path)
            elif self.__ami_state == CanState.AMI_TRACK_DRIVE:
                if self.__as_state == CanState.AS_DRIVING:
                    return self.__track_drive(current_state,desired_path)
            elif self.__ami_state == CanState.AMI_DDT_INSPECTION_A:
                if self.__as_state == CanState.AS_DRIVING:
                    return self.__static_A(current_state)
            elif self.__ami_state == CanState.AMI_DDT_INSPECTION_B:
                if self.__as_state == CanState.AS_DRIVING:
                    return self.__static_B(current_state)
            elif self.__ami_state == CanState.AMI_AUTONOMOUS_DEMO:
                if self.__as_state == CanState.AS_DRIVING:
                    return self.__autonomous_demo(current_state)
            elif self.__ami_state == CanState.AMI_NOT_SELECTED:
                return 0.0,0.0
            
        

        #fallback in case of bad can state
        return 0.0,0.0
    
    def get_mission_complete(self) -> bool:
        return self.__mission_complete

    def set_time_at_event_start(self,time):
        if self.time_at_event_start == 0:
            self.time_at_event_start = time

if __name__ == "__main__":
    mission_controler = Mission_Control()
    print(mission_controler.get_message())
