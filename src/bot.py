import math
import csv, time # Used for logging 

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3

# from RLUtilities.GameInfo import GameInfo
# from RLUtilities.Simulation import Input
# from RLUtilities.LinearAlgebra import norm

class Maneuver:
    pass

class MyBot(BaseAgent):

    def get_next_csv_name(self):

        # Read in the number we should make it 
        f = open("./MyBots/RLBot-Puddles/src/nextcsvnumber.txt", "r")
        returnValue = int(f.read())
        f.close()

        # Update the file we just read from to increment it 
        with open("./MyBots/RLBot-Puddles/src/nextcsvnumber.txt", "w") as f: 
            f.write(str(int(returnValue) + 1))

        # Actually return the value 
        return returnValue 
    
    # This runs once before the bot starts up
    def initialize_agent(self):
       
        self.controller_state = SimpleControllerState()
        self.maneuver = Maneuver()
        self.maneuver.name = None
        self.maneuver.prevent_goal_properties = Maneuver()
        self.collected_data = []
        self.time_since_last_log = time.time() # Don't want to save a log any more than .1 seconds, but also don't want it to be blocking 
        self.current_csv_name = self.get_next_csv_name()
    
    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.set_maneuver(packet, self.get_ball_prediction_struct())

        self.exec_maneuver(packet)

        my_car = packet.game_cars[self.index]

        draw_debug(self.renderer, my_car, packet.game_ball, self.maneuver.name)

        # Update log if it's been a tenth of a second since we last did 
        if time.time() - self.time_since_last_log > .1:
            self.update_log(packet, self.get_ball_prediction_struct())
            self.time_since_last_log = 0 

        return self.controller_state

    def update_log(self, packet, ball_prediction_struct):

        csv_line = []

        ''' PlayerInfo Object (Bot) '''
        # Position (X, Y, Z) 
        csv_line.insert(0, str(packet.game_cars[0].physics.location.x))
        csv_line.insert(1, str(packet.game_cars[0].physics.location.y))
        csv_line.insert(2, str(packet.game_cars[0].physics.location.z))

        # Rotation (Roll, Pitch, Yaw)
        csv_line.insert(3, str(packet.game_cars[0].physics.rotation.roll))
        csv_line.insert(4, str(packet.game_cars[0].physics.rotation.pitch))
        csv_line.insert(5, str(packet.game_cars[0].physics.rotation.yaw))

        # Velocity (X/s, Y/s, Z/s) 
        csv_line.insert(6, str(packet.game_cars[0].physics.velocity.x))
        csv_line.insert(7, str(packet.game_cars[0].physics.velocity.y))
        csv_line.insert(8, str(packet.game_cars[0].physics.velocity.z))

        # Angular Velocity (Roll/s, Pitch/s, Yaw/s)
        csv_line.insert(9, str(packet.game_cars[0].physics.angular_velocity.x))
        csv_line.insert(10, str(packet.game_cars[0].physics.angular_velocity.y))
        csv_line.insert(11, str(packet.game_cars[0].physics.angular_velocity.z))

        # Bot Score, Goals, Own Goals, Saves, Shots, Demolitions
        csv_line.insert(12, str(packet.game_cars[0].score_info.score))
        csv_line.insert(13, str(packet.game_cars[0].score_info.goals))
        csv_line.insert(14, str(packet.game_cars[0].score_info.own_goals))
        csv_line.insert(15, str(packet.game_cars[0].score_info.saves))
        csv_line.insert(16, str(packet.game_cars[0].score_info.shots))
        csv_line.insert(17, str(packet.game_cars[0].score_info.demolitions))

        # Has Wheel Contact
        csv_line.insert(18, str(packet.game_cars[0].has_wheel_contact))

        # Is Currently Supersonic
        csv_line.insert(19, str(packet.game_cars[0].is_super_sonic))

        # Has Currently Jumped
        csv_line.insert(20, str(packet.game_cars[0].jumped))

        # Has Currently Used Double Jump 
        csv_line.insert(21, str(packet.game_cars[0].double_jumped))

        ''' PlayerInfo Object (Enemy) ''' 
        # Position (X, Y, Z) 
        csv_line.insert(22, str(packet.game_cars[1].physics.location.x))
        csv_line.insert(23, str(packet.game_cars[1].physics.location.y))
        csv_line.insert(24, str(packet.game_cars[1].physics.location.z))

        # Rotation (Roll, Pitch, Yaw)
        csv_line.insert(25, str(packet.game_cars[1].physics.rotation.roll))
        csv_line.insert(26, str(packet.game_cars[1].physics.rotation.pitch))
        csv_line.insert(27, str(packet.game_cars[1].physics.rotation.yaw))

        # Velocity (X/s, Y/s, Z/s) 
        csv_line.insert(28, str(packet.game_cars[1].physics.velocity.x))
        csv_line.insert(29, str(packet.game_cars[1].physics.velocity.y))
        csv_line.insert(30, str(packet.game_cars[1].physics.velocity.z))

        # Angular Velocity (Roll/s, Pitch/s, Yaw/s)
        csv_line.insert(31, str(packet.game_cars[1].physics.angular_velocity.x))
        csv_line.insert(32, str(packet.game_cars[1].physics.angular_velocity.y))
        csv_line.insert(33, str(packet.game_cars[1].physics.angular_velocity.z))

        # Bot Score
        csv_line.insert(34, str(packet.game_cars[1].score_info.score))
        csv_line.insert(35, str(packet.game_cars[1].score_info.goals))
        csv_line.insert(36, str(packet.game_cars[1].score_info.own_goals))
        csv_line.insert(37, str(packet.game_cars[1].score_info.saves))
        csv_line.insert(38, str(packet.game_cars[1].score_info.shots))
        csv_line.insert(39, str(packet.game_cars[1].score_info.demolitions))

        # Has Wheel Contact
        csv_line.insert(40, str(packet.game_cars[1].has_wheel_contact))

        # Is Currently Supersonic
        csv_line.insert(41, str(packet.game_cars[1].is_super_sonic))

        # Has Currently Jumped
        csv_line.insert(42, str(packet.game_cars[1].jumped))

        # Has Currently Used Double Jump 
        csv_line.insert(43, str(packet.game_cars[1].double_jumped))

        ''' BallInfo Object ''' 
        # Ball X, Y, Z
        csv_line.insert(44, str(packet.game_ball.physics.location.x))
        csv_line.insert(45, str(packet.game_ball.physics.location.y))
        csv_line.insert(46, str(packet.game_ball.physics.location.z))

        # Ball Velocity (X/s, Y/s, Z/s)
        csv_line.insert(47, str(packet.game_ball.physics.velocity.x))
        csv_line.insert(48, str(packet.game_ball.physics.velocity.y))
        csv_line.insert(49, str(packet.game_ball.physics.velocity.z))

        # Ball Angular Velocity (Roll/s, Pitch/s, Yaw/s)
        csv_line.insert(50, str(packet.game_ball.physics.angular_velocity.x))
        csv_line.insert(51, str(packet.game_ball.physics.angular_velocity.y))
        csv_line.insert(52, str(packet.game_ball.physics.angular_velocity.z))

        ''' Latest Touch Object '''
        # Last Player To Touch (True if Bot, False Otherwise) 
        csv_line.insert(53, str(packet.game_ball.latest_touch.player_index is self.team))

        # Point of contact for touch X, Y, Z
        csv_line.insert(54, str(packet.game_ball.latest_touch.hit_location.x))
        csv_line.insert(55, str(packet.game_ball.latest_touch.hit_location.y))
        csv_line.insert(56, str(packet.game_ball.latest_touch.hit_location.z))

        # Direction of the touch X, Y, Z
        csv_line.insert(57, str(packet.game_ball.latest_touch.hit_normal.x))
        csv_line.insert(58, str(packet.game_ball.latest_touch.hit_normal.y))
        csv_line.insert(59, str(packet.game_ball.latest_touch.hit_normal.z))

        ''' GameInfo Object '''
        # Total seconds elapsed (seconds, I assume)
        # Estimating high here, will revise once I see the CSV 
        csv_line.insert(60, str(packet.game_info.seconds_elapsed))

        # Total game time remaining (seconds, I assume)
        csv_line.insert(61, str(packet.game_info.game_time_remaining))

        # Is Overtime (True if not, False otherwise)
        csv_line.insert(62, str(packet.game_info.is_overtime))

        ''' Predicted Ball Position ''' 
        # 1-Second Slice Values
        csv_line.insert(63, str(ball_prediction_struct[60].location.x))
        csv_line.insert(64, str(ball_prediction_struct[60].location.y))
        csv_line.insert(65, str(ball_prediction_struct[60].location.z))

        # 2-Second Slice Values
        csv_line.insert(66, str(ball_prediction_struct[120].location.x))
        csv_line.insert(67, str(ball_prediction_struct[120].location.y))
        csv_line.insert(68, str(ball_prediction_struct[120].location.z))

        ''' BoostPadState Object '''
        # Activation state for each of the 34 boost pads 
        index = 69
        for boost_pad in packet.game_boosts:
            csv_line.insert(index, str(boost_pad.is_active))
            csv_line.insert(index + 1, str(boost_pad.timer))
            index += 2
        index = 69

        # If the ball is projected to be in the goal at ANY point within the next six seconds (True if so, False if not)
        # TODO: This

        # Deciding what the "correct state" in this position

        # Append onto the variable that holds all our data 
        self.collected_data.append(csv_line)

        # Writing freshest iteration to file 
        # For WHATEVER F*CKING REASON the file itself is run in the RLBot home 
        # directory and not FROM THIS F*CKING FILE... there's 2.5 hours down the drain
        with open("./MyBots/RLBot-Puddles/src/output/" + str(self.current_csv_name) + ".csv", "w", newline="") as f: 
            writer = csv.writer(f)
            writer.writerows(self.collected_data)

    def set_maneuver(self, packet: GameTickPacket, prediction_slices):
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)
        opponent_goal_y = -7000.0 if self.team == 1 else 7000.0
        ball_loc = packet.game_ball.physics.location
        impending_goal = self.will_enter_goal(packet)

        if self.maneuver.name == None:
            self.maneuver.name = "Attack"
        elif self.maneuver.name is not "PreventGoal" and impending_goal:
            self.maneuver.name = "PreventGoal"
            self.maneuver.prevent_goal_properties.chasing_ball = False
        elif isbetween(car_location.y, ball_loc.y, opponent_goal_y) and not impending_goal and abs(ball_loc.y) < 4000:
            self.maneuver.name = "GetHomeBoost"
        
    # This method calls the correct functions dependant on what maneuver we are executing
    def exec_maneuver(self, packet: GameTickPacket):
        if self.maneuver.name == "PreventGoal":
            self.prevent_goal(packet)
        elif self.maneuver.name == "GetHomeBoost":
            self.get_home_boost(packet)
        else:
            self.go_to_ball(packet)

    def prevent_goal(self, packet: GameTickPacket):
        goalYVal = -5200.0 if self.team == 0 else 5200.0
        car_to_own_goal = Vec3(0.0, goalYVal, 0.0) - packet.game_cars[self.index].physics.location

        if not self.will_enter_goal(packet):
            self.maneuver.name = None
            self.maneuver.prevent_goal_properties.chasing_ball = False
            return

        if car_to_own_goal.length() > 2000.0 and self.maneuver.prevent_goal_properties.chasing_ball is not True:
            self.get_to_goal_post(packet)
        else:
            self.maneuver.prevent_goal_properties.chasing_ball = True
            self.go_to_ball(packet)

    def will_enter_goal(self, packet: GameTickPacket) -> bool:
        goalYVal = -5200.0 if self.team == 0 else 5200.0
        ball_prediction = self.get_ball_prediction_struct()

        if ball_prediction is not None:
            for i in range(0, ball_prediction.num_slices):
                prediction_slice = ball_prediction.slices[i]
                location = prediction_slice.physics.location
                if abs(location.y - goalYVal) < 80 and abs(location.x) < 900.0:
                    return True
        return False

    def get_to_goal_post(self, packet: GameTickPacket):
        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)

        goToX = 900.0 if car_location.x > 0 else -900.0
        goToY = -5000.0 if self.team == 0 else 5000.0
        self.go_to_position(packet, Vec3(goToX, goToY, 0.0))

    def go_to_ball(self, packet: GameTickPacket):
        self.go_to_position(packet, packet.game_ball.physics.location)

    # def hit_ball_to_goal(self, packet: GameTickPacket):
    #     ball_prediction = self.get_ball_prediction_struct()
    #     my_car = packet.game_cars[self.index]
    #     speed = my_car.physics.velocity.length()

    #     starting_time = ball_prediction.slices[0].game_seconds

    #     if ball_prediction is not None:
    #         for i in range(0, ball_prediction.num_slices):
    #             prediction_slice = ball_prediction.slices[i]
    #             location = prediction_slice.physics.location
    #             if abs(location.y - goalYVal) < 80 and abs(location.x) < 900.0:
    #                 return True
    #     self.go_to_ball(packet)

    def go_to_position(self, packet: GameTickPacket, ideal: Vec3):
        my_car = packet.game_cars[self.index]
        car_orientation = Orientation(my_car.physics.rotation)
        car_direction = car_orientation.forward

        steer_correction_radians = find_correction(car_direction, Vec3(ideal) - Vec3(my_car.physics.location))

        # Turn left if steer correction is positive in radians
        turn_direction_multiplier = -1.0 if steer_correction_radians > 0 else 1.0
        abs_correction = abs(steer_correction_radians)
        if abs_correction >= .2:
            turn = 1 * turn_direction_multiplier
        else:
            turn = abs_correction * 5 * turn_direction_multiplier

        # Powerslide if the angle of correction is more than 1 radian
        if abs_correction > 1.3:
            self.controller_state.handbrake = True
        else:
            self.controller_state.handbrake = False

        # TODO: change. always boost
        self.controller_state.boost = not self.controller_state.handbrake

        self.controller_state.throttle = 1.0
        self.controller_state.steer = turn

    def get_boost(self, packet: GameTickPacket):
        info = self.get_field_info()
        # Manually construct a list of all big boost pads
        min_dist = 50000
        min_dist_index = 0
        # info.boost_pads has a fixed size but info.num_boosts is how many pads there actually are
        for i in range(info.num_boosts):
            pad = info.boost_pads[i]
            if pad.is_full_boost:
                dist = abs((pad.location - packet.game_cars[self.index].physics.location).length())
                if min_dist > dist:
                    min_dist=dist
                    min_dist_index = i
        self.go_to_position(packet,info.boost_pads[min_dist_index].location)

    def get_home_boost(self, packet: GameTickPacket):
        boost_Yval = -4100.0 if self.team == 0 else 4100.0
        info = self.get_field_info()
        # Manually construct a list of all big boost pads
        min_dist = 50000
        min_dist_index = 0
        # info.boost_pads has a fixed size but info.num_boosts is how many pads there actually are
        for i in range(info.num_boosts):
            pad = info.boost_pads[i]
            if pad.is_full_boost and abs(pad.location.y - boost_Yval) < 100:
                dist = abs((Vec3(pad.location) - Vec3(packet.game_cars[self.index].physics.location)).length())
                if min_dist > dist:
                    min_dist=dist
                    min_dist_index = i
        self.go_to_position(packet, info.boost_pads[min_dist_index].location)
        if min_dist < 150:
            self.maneuver.name = None


def find_correction(current: Vec3, ideal: Vec3) -> float:
    # Finds the angle from current to ideal vector in the xy-plane. Angle will be between -pi and +pi.

    # The in-game axes are left handed, so use -x
    current_in_radians = math.atan2(current.y, -current.x)
    ideal_in_radians = math.atan2(ideal.y, -ideal.x)

    diff = ideal_in_radians - current_in_radians

    # Make sure that diff is between -pi and +pi.
    if abs(diff) > math.pi:
        if diff < 0:
            diff += 2 * math.pi
        else:
            diff -= 2 * math.pi

    return diff

#used to check position vectors
def isbetween(x: float, y1: float, y2: float) -> bool:
    if  y1 <= x <=y2:
        return True

    elif y1 >= x >=y2:
        return True
    
    return False

def draw_debug(renderer, car, ball, action_display):
    renderer.begin_rendering()
    # draw a line from the car to the ball
    renderer.draw_line_3d(car.physics.location, ball.physics.location, renderer.white())
    # print the action that the bot is taking
    renderer.draw_string_3d(car.physics.location, 2, 2, action_display, renderer.white())
    renderer.end_rendering()
