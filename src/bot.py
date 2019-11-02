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
        f = open("./MyBots/RLBot-Puddles/src/nextcsvnumber.txt", "r"); 
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

    # def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
    #     self.set_maneuver(packet)

    #     my_car = packet.game_cars[self.index]
    #     car_location = Vec3(my_car.physics.location)

    #     # Find the direction of our car using the Orientation class
    #     car_orientation = Orientation(my_car.physics.rotation)
    #     car_direction = car_orientation.forward

    #     steer_correction_radians = find_correction(car_direction, self.get_target(packet) - car_location)

    #     if steer_correction_radians > 0:
    #         # Positive radians in the unit circle is a turn to the left.
    #         turn = -1.0  # Negative value for a turn to the left.
    #         action_display = "turn left"
    #     else:
    #         turn = 1.0
    #         action_display = "turn right"

    #     self.controller_state.throttle = 1.0
    #     self.controller_state.steer = turn

    #     draw_debug(self.renderer, my_car, packet.game_ball, action_display)

    #     return self.controller_state
    
    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.set_maneuver(packet)

        self.exec_maneuver(packet)

        my_car = packet.game_cars[self.index]

        draw_debug(self.renderer, my_car, packet.game_ball, self.maneuver.name)

        # Update log if it's been a tenth of a second since we last did 
        if time.time() - self.time_since_last_log > .1:
            self.update_log(packet)
            self.time_since_last_log = 0 

        return self.controller_state

    def update_log(self, packet):

        csv_line = []
            
        # Bot XYZ
        csv_line.insert(0, str(packet.game_cars[0].physics.location.x))
        csv_line.insert(1, str(packet.game_cars[0].physics.location.y))
        csv_line.insert(2, str(packet.game_cars[0].physics.location.z))

        # Other Player XYZ 
        csv_line.insert(3, str(packet.game_cars[1].physics.location.x))
        csv_line.insert(4, str(packet.game_cars[1].physics.location.y))
        csv_line.insert(5, str(packet.game_cars[1].physics.location.z))

        # Append onto the variable that holds all our data 
        self.collected_data.append(csv_line)

        # Writing freshest iteration to file 
        # For WHATEVER F*CKING REASON the file itself is run in the RLBot home 
        # directory and not FROM THIS F*CKING FILE... there's 2.5 hours down the drain
        with open("./MyBots/RLBot-Puddles/src/output/" + str(self.current_csv_name) + ".csv", "w", newline="") as f: 
            writer = csv.writer(f)
            writer.writerows(self.collected_data)

    # def get_target(self, packet: GameTickPacket) -> Vec3:
    #     if self.maneuver.name == "Attack":
    #         return Vec3(packet.game_ball.physics.location)
    #     else:
    #         # Constant values can be found the the FieldInfo:
    #         info = self.get_field_info()
            
    #         # Manually construct a list of all big boost pads
    #         # info.boost_pads has a fixed size but info.num_boosts is how many pads there actually are
    #         big_pads = []
    #         for i in range(info.num_boosts):
    #             pad = info.boost_pads[i]
    #             if pad.is_full_boost:
    #                 big_pads.append(pad)

    #         return Vec3(big_pads[0].location)

    def set_maneuver(self, packet: GameTickPacket):
        if self.maneuver.name == None:
            self.maneuver.name = "Attack"
        elif self.maneuver.name == "Attack" and self.will_enter_goal(packet):
            self.maneuver.name = "PreventGoal"
            self.maneuver.prevent_goal_properties.chasing_ball = False

    def exec_maneuver(self, packet: GameTickPacket):
        if self.maneuver.name == "PreventGoal":
            self.prevent_goal(packet)
        else:
            self.go_to_ball(packet)

    def prevent_goal(self, packet: GameTickPacket):
        goalYVal = -5200.0 if self.team == 0 else 5200.0
        car_to_own_goal = Vec3(0.0, goalYVal, 0.0) - packet.game_cars[self.index].physics.location

        if not self.will_enter_goal(packet):
            self.maneuver.name = None
            self.maneuver.prevent_goal_properties.chasing_ball = False
            return

        if car_to_own_goal.length() > 1000.0 and self.maneuver.prevent_goal_properties.chasing_ball is not True:
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
                if abs(location.y - goalYVal) < 30.0 and abs(location.x) < 900.0:
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

    def go_to_position(self, packet: GameTickPacket, ideal: Vec3):
        my_car = packet.game_cars[self.index]
        car_orientation = Orientation(my_car.physics.rotation)
        car_direction = car_orientation.forward

        steer_correction_radians = find_correction(car_direction, Vec3(ideal) - Vec3(my_car.physics.location))

        if steer_correction_radians > 0:
            # Positive radians in the unit circle is a turn to the left.
            turn = -1.0  # Negative value for a turn to the left.
            # action_display = "turn left"
        else:
            turn = 1.0
            # action_display = "turn right"

        self.controller_state.throttle = 1.0
        self.controller_state.steer = turn


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

def draw_debug(renderer, car, ball, action_display):
    renderer.begin_rendering()
    # draw a line from the car to the ball
    renderer.draw_line_3d(car.physics.location, ball.physics.location, renderer.white())
    # print the action that the bot is taking
    renderer.draw_string_3d(car.physics.location, 2, 2, action_display, renderer.white())
    renderer.end_rendering()
