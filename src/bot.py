import math

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

from util.orientation import Orientation
from util.vec import Vec3

# from RLUtilities.GameInfo import GameInfo
# from RLUtilities.Simulation import Input
# from RLUtilities.LinearAlgebra import norm

class MyBot(BaseAgent):

    def initialize_agent(self):
        # This runs once before the bot starts up
        self.controller_state = SimpleControllerState()
        self.maneuver = None

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.set_maneuver(packet)

        my_car = packet.game_cars[self.index]
        car_location = Vec3(my_car.physics.location)

        # Find the direction of our car using the Orientation class
        car_orientation = Orientation(my_car.physics.rotation)
        car_direction = car_orientation.forward

        steer_correction_radians = find_correction(car_direction, self.get_target(packet) - car_location)

        if steer_correction_radians > 0:
            # Positive radians in the unit circle is a turn to the left.
            turn = -1.0  # Negative value for a turn to the left.
            action_display = "turn left"
        else:
            turn = 1.0
            action_display = "turn right"

        self.controller_state.throttle = 1.0
        self.controller_state.steer = turn

        draw_debug(self.renderer, my_car, packet.game_ball, action_display)

        return self.controller_state

    def get_target(self, packet: GameTickPacket) -> Vec3:
        if self.maneuver == "Attack":
            return Vec3(packet.game_ball.physics.location)
        else:
            # Constant values can be found the the FieldInfo:
            info = self.get_field_info()
            
            # Manually construct a list of all big boost pads
            # info.boost_pads has a fixed size but info.num_boosts is how many pads there actually are
            big_pads = []
            for i in range(info.num_boosts):
                pad = info.boost_pads[i]
                if pad.is_full_boost:
                    big_pads.append(pad)

            return Vec3(big_pads[0].location)

    def set_maneuver(self, packet: GameTickPacket):
        if self.maneuver == None:
            ball_location = Vec3(packet.game_ball.physics.location)

            my_car = packet.game_cars[self.index]
            car_location = Vec3(my_car.physics.location)

            car_to_ball = ball_location - car_location

            other_car = packet.game_cars[1]
            opp_location = Vec3(other_car.physics.location)

            opp_to_ball = ball_location - opp_location

            if car_to_ball.length() < opp_to_ball.length() or my_car.boost > 60.0:
                self.maneuver = "Attack"
            else:
                self.maneuver = "GetBoostAtBack"


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
