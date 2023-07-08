import math
import rlbot.utils.structures.game_data_struct as game_data_struct
from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
import numpy as np
from util.sequence import *


# This file holds all of the objects used in Cheese utils
# Includes custom vector and matrix objects

class CheeseAgent(BaseAgent):
    # This is the main object of Cheese Utils. It holds/updates information about the game and runs routines
    # All utils rely on information being structured and accessed the same way as configured in this class
    def initialize_agent(self):
        # A list of cars for both teammates and opponents
        self.friends = []
        self.foes = []
        self.cars = []
        # This holds the carobject for our agent
        self.me = car_object(self.index)

        self.ball = ball_object()
        self.game = game_object()
        # A list of boosts
        self.boosts = []
        # goals
        self.friend_goal = goal_object(self.team)
        self.foe_goal = goal_object(not self.team)
        # Where we store the bot's current objective
        self.intent = None
        # Game time
        self.time = 0.0
        self.delta_time = 0.0
        self._last_time = 0.0
        # Whether or not CheesegAgent has run its get_ready() function
        self.ready = False
        # the controller that is returned to the framework after every tick
        self.controller = SimpleControllerState()
        # a flag that tells us when kickoff is happening
        self.kickoff_flag = False
        self.active_sequence: Sequence = None
        self.packet: GameTickPacket = None
        self._dribbler = DribbleDetection()

        self.LT = 0
        self.RLT = 0
        self.FTR = True
        self.ST = 0
        self.DT = 0
        self.CT = 0
        self.FPS = 120

        self.follow_up = False
        self.kickoff_time = 0


    def get_ready(self, packet):
        # Preps all of the objects that will be updated during play
        field_info = self.get_field_info()
        for i in range(field_info.num_boosts):
            boost = field_info.boost_pads[i]
            self.boosts.append(boost_object(i, boost.location))
        self.refresh_player_lists(packet)
        self.ball.update(packet)
        self.ready = True

    def refresh_player_lists(self, packet):
        # makes new freind/foe lists
        # Useful to keep separate from get_ready because humans can join/leave a match
        self.friends = [car_object(i, packet) for i in range(packet.num_cars) if
                        packet.game_cars[i].team == self.team and i != self.index]
        self.foes = [car_object(i, packet) for i in range(packet.num_cars) if packet.game_cars[i].team != self.team]
        self.cars = [car_object(i, packet) for i in range(packet.num_cars)]

    def set_intent(self, routine):
        self.intent = routine

    def get_intent(self):
        return self.intent

    def clear_intent(self):
        self.intent = None

    def push(self, routine):
        # Shorthand for adding a routine to the stack
        self.set_intent(routine=routine)

    def pop(self):
        # Shorthand for removing a routine from the stack, returns the routine
        return self.clear_intent()

    def line(self, start, end, color=None):
        color = color if color != None else [255, 255, 255]
        self.renderer.draw_line_3d(start.copy(), end.copy(), self.renderer.create_color(255, *color))

    def text2d(self, text, pos, sX=1, sY=1, color=None):
        color = color if color != None else [255, 255, 255]
        self.renderer.draw_string_2d(pos[0], pos[1], sX, sY, text, self.renderer.create_color(255, *color))

    def debug_intent(self):
        # Draws the stack on the screen
        white = self.renderer.white()
        text = self.get_intent().__class__.__name__
        self.renderer.draw_string_2d(10, 100, 3, 3, text, white)

    def clear(self):
        # Shorthand for clearing the stack of all routines
        self.clear_intent()

    def preprocess(self, packet):
        # Calling the update functions for all of the objects
        if packet.num_cars != len(self.friends) + len(self.foes) + 1: self.refresh_player_lists(packet)
        for car in self.friends: car.update(packet)
        for car in self.foes: car.update(packet)
        for pad in self.boosts: pad.update(packet)
        self.ball.update(packet)
        self.me.update(packet)
        self.game.update(packet)
        self.time = packet.game_info.seconds_elapsed
        # When a new kickoff begins we empty the stack
        if self.kickoff_flag == False and packet.game_info.is_round_active and packet.game_info.is_kickoff_pause:
            self.clear_intent()
        # Tells us when to go for kickoff
        self.kickoff_flag = packet.game_info.is_round_active and packet.game_info.is_kickoff_pause

    def update_deltatime(self):
        self.delta_time = self.time - self._last_time
        self._last_time = self.time

    def get_output(self, packet):
        self.packet = packet

        # Reset controller
        self.controller.__init__()
        # Get ready, then preprocess
        if not self.ready:
            self.get_ready(packet)
        self.preprocess(packet)

        self.TimeManager()
        self.update_deltatime()

        self.renderer.begin_rendering()
            
        # Run our strategy code
        self.run()
        # run the routine on the end of the stack
        intent = self.get_intent()
        if intent is not None:
            intent.run(self)
        self.renderer.end_rendering()
        # send our updated controller back to rlbot
        return self.controller
    
    def TimeManager(self):
        if not self.LT:
            self.LT = self.time
        else:
            if self.RLT == self.time:
                return

            if int(self.LT) != int(self.time):
                if self.FTR or self.kickoff_flag:
                    self.FTR = False
                self.ST = self.DT = 0

            Tp = round(max(1, (self.time - self.LT) * self.FPS))
            self.LT = min(self.time, self.LT + Tp)
            self.RLT = self.time
            self.CT += Tp
            if Tp > 1:
                self.ST += Tp - 1
            self.DT += 1
    
    def is_in_front_of_ball(self):
        me_to_goal = (self.me.location - self.foe_goal.location).magnitude()
        ball_to_goal = (self.ball.location -
                        self.foe_goal.location).magnitude()
        me_to_own_goal = (self.me.location -
                          self.friend_goal.location).magnitude()
        if (me_to_goal < (ball_to_goal)) and (me_to_own_goal > 1000):
            return True
        return False
    
    def get_closest_large_boost(self):
        available_boosts = [
            boost for boost in self.boosts if boost.large and boost.active]
        closest_boost = None
        closest_distance = 10000
        for boost in available_boosts:
            distance = (self.me.location - boost.location).magnitude()
            if closest_boost is None or distance < closest_distance:
                closest_boost = boost
                closest_distance = distance
        return closest_boost

    def get_boost_while_going_home(self):
        active_boosts = [boost for boost in self.boosts if boost.active and abs(
            self.friend_goal.location.y - boost.location.y) - 1000 < abs(self.friend_goal.location.y - self.me.location.y)]
        closest_distance = 8000
        closest_boost = None
        for boost in active_boosts:
            distance = (self.me.location - boost.location).magnitude()
            if closest_boost is None or distance < closest_distance:
                closest_boost = boost
                closest_distance = distance
        return closest_boost

    def get_closest_boost(self):
        active_boosts = [boost for boost in self.boosts if boost.active]
        closest_boost = None
        closest_distance = 10000
        for boost in active_boosts:
            distance = (self.me.location - boost.location).magnitude()
            if closest_boost is None or distance < closest_distance:
                closest_boost = boost
                closest_distance = distance
        return closest_boost
    
    def run(self):
        # override this with your strategy code
        pass
    

class car_object:
    # The carObject, and kin, convert the gametickpacket in something a little friendlier to use,
    # and are updated by CheeseAgent as the game runs
    def __init__(self, index, packet=None):
        self.location = Vector3(0, 0, 0)
        self.orientation = Matrix3(0, 0, 0)
        self.rotation = [0, 0, 0]
        self.velocity = Vector3(0, 0, 0)
        self.angular_velocity = [0, 0, 0]
        self.demolished = False
        self.airborne = False
        self.supersonic = False
        self.jumped = False
        self.doublejumped = False
        self.boost = 0
        self.index = index
        self.team = 0
        if packet is not None:
            self.update(packet)

    def local(self, value):
        # Shorthand for self.orientation.dot(value)
        return self.orientation.dot(value)

    def update(self, packet):
        car = packet.game_cars[self.index]
        self.location.data = [car.physics.location.x,
                              car.physics.location.y, car.physics.location.z]
        self.velocity.data = [car.physics.velocity.x,
                              car.physics.velocity.y, car.physics.velocity.z]
        self.rotation = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]
        self.orientation = Matrix3(
            car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll)
        self.angular_velocity = self.orientation.dot(
            [car.physics.angular_velocity.x, car.physics.angular_velocity.y, car.physics.angular_velocity.z]).data
        self.demolished = car.is_demolished
        self.airborne = not car.has_wheel_contact
        self.supersonic = car.is_super_sonic
        self.jumped = car.jumped
        self.doublejumped = car.double_jumped
        self.boost = car.boost
        self.team = car.team

    @property
    def forward(self):
        # A vector pointing forwards relative to the cars orientation. Its magnitude is 1
        return self.orientation.forward

    @property
    def left(self):
        # A vector pointing left relative to the cars orientation. Its magnitude is 1
        return self.orientation.left

    @property
    def up(self):
        # A vector pointing up relative to the cars orientation. Its magnitude is 1
        return self.orientation.up


class ball_object:
    def __init__(self):
        self.location = Vector3(0, 0, 0)
        self.velocity = Vector3(0, 0, 0)
        self.angular_velocity = Vector3(0, 0, 0)
        self.latest_touched_time = 0
        self.latest_touched_team = 0

    def update(self, packet):
        ball = packet.game_ball
        self.location.data = [ball.physics.location.x,
                              ball.physics.location.y, ball.physics.location.z]
        self.velocity.data = [ball.physics.velocity.x,
                              ball.physics.velocity.y, ball.physics.velocity.z]
        self.velocity.data = [ball.physics.angular_velocity.x,
                              ball.physics.angular_velocity.y, ball.physics.angular_velocity.z]
        self.latest_touched_time = ball.latest_touch.time_seconds
        self.latest_touched_team = ball.latest_touch.team


class boost_object:
    def __init__(self, index, location):
        self.index = index
        self.location = Vector3(location.x, location.y, location.z)
        self.active = True
        # determine boost size by height (only works for standard maps)
        self.large = self.location.z > 7

    def update(self, packet):
        self.active = packet.game_boosts[self.index].is_active


class goal_object:
    # This is a simple object that creates/holds goalpost locations for a given team (for soccer on standard maps only)
    def __init__(self, team):
        team = 1 if team == 1 else -1
        self.location = Vector3(0, team * 5100, 320)  # center of goal line
        # Posts are closer to x=750, but this allows the bot to be a little more accurate
        self.left_post = Vector3(team * 850, team * 5100, 320)
        self.right_post = Vector3(-team * 850, team * 5100, 320)


class game_object:
    # This object holds information about the current match
    def __init__(self):
        self.time = 0
        self.time_remaining = 0
        self.overtime = False
        self.round_active = False
        self.kickoff = False
        self.match_ended = False

    def update(self, packet):
        game = packet.game_info
        self.time = game.seconds_elapsed
        self.time_remaining = game.game_time_remaining
        self.overtime = game.is_overtime
        self.round_active = game.is_round_active
        self.kickoff = game.is_kickoff_pause
        self.match_ended = game.is_match_ended


class DribbleDetection():
    def __init__(self):
        self.dist_req = 165.0
        self.time_req = 0.25
        self.z_req = 120.0
        self.prev_car: car_object = None
        self.duration = 0.0

    def GetDribbler(self, agent, step):
        dribbler = agent.cars[0]
        for car in agent.cars:
            if dribbler.location.dist(agent.ball.location) >= car.location.dist(agent.ball.location):
                dribbler = car

        if dribbler is None or dribbler.index != (self.prev_car.index if self.prev_car is not None else -1) or agent.ball.location.z < self.z_req or dribbler.location.dist(agent.ball.location) > self.dist_req:
            self.duration = 0
            #agent.text2d("RESET", Vector3(49, 220, 0), 2, 2)

        self.duration += step
        self.prev_car = dribbler

        return dribbler if self.duration >= self.time_req else None

    def Duration(self):
        return self.duration


class Matrix3:
    # The Matrix3's sole purpose is to convert roll, pitch, and yaw data into an orientation matrix
    # An orientation matrix contains 3 Vector3's
    # Matrix3[0] is the "forward" direction of a given car
    # Matrix3[1] is the "left" direction of a given car
    # Matrix3[2] is the "up" direction of a given car
    # If you have a distance between the car and some object, ie ball.location - car.location,
    # you can convert that to local coordinates by dotting it with this matrix
    # ie: local_ball_location = Matrix3.dot(ball.location - car.location)
    def __init__(self, pitch, yaw, roll):
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        cr = math.cos(roll)
        sr = math.sin(roll)
        self.data = (
            Vector3(cp * cy, cp * sy, sp),
            Vector3(cy * sp * sr - cr * sy, sy * sp * sr + cr * cy, -cp * sr),
            Vector3(-cr * cy * sp - sr * sy, -cr * sy * sp + sr * cy, cp * cr))
        self.forward, self.left, self.up = self.data

    def __getitem__(self, key):
        return self.data[key]

    def dot(self, vector):
        return Vector3(self.forward.dot(vector), self.left.dot(vector), self.up.dot(vector))


class Vector3:
    # This is the backbone of Cheese Utils.
    # The Vector3 makes it easy to store positions, velocities, etc and perform vector math
    # A Vector3 can be created with:
    # - Anything that has a __getitem__ (lists, tuples, Vector3's, etc)
    # - 3 numbers
    # - A gametickpacket vector
    def __init__(self, *args):
        if hasattr(args[0], "__getitem__"):
            self.data = list(args[0])
        elif isinstance(args[0], game_data_struct.Vector3):
            self.data = [args[0].x, args[0].y, args[0].z]
        elif isinstance(args[0], game_data_struct.Rotator):
            self.data = [args[0].pitch, args[0].yaw, args[0].roll]
        elif len(args) == 3:
            self.data = list(args)
        else:
            raise TypeError("Vector3 unable to accept %s" % args)

    # Property functions allow you to use `Vector3.x` vs `Vector3[0]`
    @property
    def x(self):
        return self.data[0]

    @x.setter
    def x(self, value):
        self.data[0] = value

    @property
    def y(self):
        return self.data[1]

    @y.setter
    def y(self, value):
        self.data[1] = value

    @property
    def z(self):
        return self.data[2]

    @z.setter
    def z(self, value):
        self.data[2] = value

    def WithX(self, x):
        self.data[0] = x

    def WithY(self, y):
        self.data[1] = y

    def WithZ(self, z):
        self.data[2] = z

    def __getitem__(self, key):
        # To access a single value in a Vector3, treat it like a list
        # ie: to get the first (x) value use: Vector3[0]
        # The same works for setting values
        return self.data[key]

    def __setitem__(self, key, value):
        self.data[key] = value

    def __str__(self):
        # Vector3's can be printed to console
        return str(self.data)

    __repr__ = __str__

    def __eq__(self, value):
        # Vector3's can be compared with:
        # - Another Vector3, in which case True will be returned if they have the same values
        # - A single value, in which case True will be returned if the Vector's length matches the value
        if hasattr(value, "__getitem__"):
            return self.data == value.data
        else:
            return self.magnitude() == value

    # Vector3's support most operators (+-*/)
    # If using an operator with another Vector3, each dimension will be independent
    # ie x+x, y+y, z+z
    # If using an operator with only a value, each dimension will be affected by that value
    # ie x+v, y+v, z+v
    def __add__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(self[0] + value[0], self[1] + value[1], self[2] + value[2])
        return Vector3(self[0] + value, self[1] + value, self[2] + value)

    __radd__ = __add__

    def __sub__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(self[0] - value[0], self[1] - value[1], self[2] - value[2])
        return Vector3(self[0] - value, self[1] - value, self[2] - value)

    __rsub__ = __sub__

    def __neg__(self):
        return Vector3(-self[0], -self[1], -self[2])

    def __mul__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(self[0] * value[0], self[1] * value[1], self[2] * value[2])
        return Vector3(self[0] * value, self[1] * value, self[2] * value)

    __rmul__ = __mul__

    def __truediv__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(self[0] / value[0], self[1] / value[1], self[2] / value[2])
        return Vector3(self[0] / value, self[1] / value, self[2] / value)

    def __rtruediv__(self, value):
        if hasattr(value, "__getitem__"):
            return Vector3(value[0] / self[0], value[1] / self[1], value[2] / self[2])
        raise TypeError("unsupported rtruediv operands")

    def __abs__(self):
        return Vector3(abs(self[0]), abs(self[1]), abs(self[2]))
    
    def atan(self):
        return Vector3(math.atan2(self.y, self.x), math.atan2(self.z, math.sqrt(self.x**2 + self.y**2)), 0)

    def magnitude(self):
        # Magnitude() returns the length of the vector
        return math.sqrt((self[0] * self[0]) + (self[1] * self[1]) + (self[2] * self[2]))

    def normalize(self, return_magnitude=False):
        # Normalize() returns a Vector3 that shares the same direction but has a length of 1.0
        # Normalize(True) can also be used if you'd like the length of this Vector3 (used for optimization)
        magnitude = self.magnitude()
        if magnitude != 0:
            if return_magnitude:
                return Vector3(self[0] / magnitude, self[1] / magnitude, self[2] / magnitude), magnitude
            return Vector3(self[0] / magnitude, self[1] / magnitude, self[2] / magnitude)
        if return_magnitude:
            return Vector3(0, 0, 0), 0
        return Vector3(0, 0, 0)

    # Linear algebra functions
    def dot(self, value):
        return self[0] * value[0] + self[1] * value[1] + self[2] * value[2]

    def cross(self, value):
        # A .cross((0, 0, 1)) will rotate the vector counterclockwise by 90 degrees
        return Vector3((self[1] * value[2]) - (self[2] * value[1]), (self[2] * value[0]) - (self[0] * value[2]),
                       (self[0] * value[1]) - (self[1] * value[0]))

    def flatten(self):
        # Sets Z (Vector3[2]) to 0
        return Vector3(self[0], self[1], 0)

    def render(self):
        # Returns a list with the x and y values, to be used with pygame
        return [self[0], self[1]]

    def copy(self):
        # Returns a copy of this Vector3
        return Vector3(self.data[:])

    def angle(self, value):
        # Returns the angle between this Vector3 and another Vector3
        return math.acos(round(self.flatten().normalize().dot(value.flatten().normalize()), 4))
    
    def get_angle(A, B, C):
            Ax, Ay = A[0]-B[0], A[1]-B[1]
            Cx, Cy = C[0]-B[0], C[1]-B[1]
            a = math.atan2(Ay, Ax)
            c = math.atan2(Cy, Cx)
            if a < 0: a += math.pi*2
            if c < 0: c += math.pi*2
            return (math.pi*2 + c - a) if a > c else (c - a)

    def rotate(self, angle):
        # Rotates this Vector3 by the given angle in radians
        # Note that this is only 2D, in the x and y axis
        return Vector3((math.cos(angle) * self[0]) - (math.sin(angle) * self[1]),
                       (math.sin(angle) * self[0]) + (math.cos(angle) * self[1]), self[2])

    def clamp(self, start, end):
        # Similar to integer clamping, Vector3's clamp() forces the Vector3's direction between a start and end Vector3
        # Such that Start < Vector3 < End in terms of clockwise rotation
        # Note that this is only 2D, in the x and y axis
        s = self.normalize()
        right = s.dot(end.cross((0, 0, -1))) < 0
        left = s.dot(start.cross((0, 0, -1))) > 0
        if (right and left) if end.dot(start.cross((0, 0, -1))) > 0 else (right or left):
            return self
        if start.dot(s) < end.dot(s):
            return end
        return start

    def dist(self, vector):
        #returns the distance between two points.
        return math.sqrt((self[0] - vector[0]) ** 2 + (self[1] - vector[1]) ** 2 + (self[2] - vector[2]) ** 2)

    def flat_dist(self, vector):
        #returns the distance between two points, disregarding the z co-ordinate.
        return math.sqrt((self[0] - vector[0]) ** 2 + (self[1] - vector[1]) ** 2)


class Vec2:
    def __init__(self, x, y):
        self._x = float(x)
        self._y = float(y)

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, new_x):
        self._x = float(new_x)

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, new_y):
        self._y = float(new_y)

    def __add__(self, other):
        types = (int, float)
        if isinstance(self, types):
            return Vec2(self + other.x, self + other.y)
        elif isinstance(other, types):
            return Vec2(self.x + other, self.y + other)
        else:
            return Vec2(self.x + other.x, self.y + other.y)

    def __div__(self, other):
        types = (int, float)
        if isinstance(self, types):
            self = Vec2(self, self)
        elif isinstance(other, types):
            other = Vec2(other, other)
        x = self.x / other.x
        y = self.y / other.y
        return Vec2(x, y)

    def __mul__(self, other):
        types = (int, float)
        if isinstance(self, types):
            return Vec2(self * other.x, self * other.y)
        elif isinstance(other, types):
            return Vec2(self.x * other, self.y * other)
        else:
            return Vec2(self.x * other.x, self.y * other.y)

    def __neg__(self):
        return Vec2(-self.x, -self.y)

    def __radd__(self, other):
        return Vec2(self.x + other, self.y + other)

    def __rdiv__(self, other):
        return Vec2(other/self.x, other/self.y)

    def __rmul__(self, other):
        return Vec2(other * self.x, other * self.y)

    def __rsub__(self, other):
        return Vec2(other - self.x, other - self.y)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Vec2: ({0}, {1})".format(self.x, self.y)

    def __sub__(self, other):
        types = (int, float)
        if isinstance(self, types):
            return Vec2(self - other.x, self - other.y)
        elif isinstance(other, types):
            return Vec2(self.x - other, self.y - other)
        else:
            return Vec2(self.x - other.x, self.y - other.y)

    def ceil(self):
        return Vec2(math.ceil(self.x), math.ceil(self.y))

    def floor(self):
        return Vec2(math.floor(self.x), math.floor(self.y))

    def get_data(self):
        return (self.x, self.y)    

    def inverse(self):
        return Vec2(1.0/self.x, 1.0/self.y)

    def length(self):
        return math.sqrt(self.square_length())

    def normalize(self):
        length = self.length()
        if length == 0.0:
            return Vec2(0, 0)
        return Vec2(self.x/length, self.y/length)

    def round(self):
        return Vec2(round(self.x), round(self.y))

    def square_length(self):
        return (self.x * self.x) + (self.y * self.y)

    """
    def transform(self, matrix):#mat2, mat2d, mat3, mat4
        pass

    @classmethod
    def cross(cls, a, b):
        z = (a.x * b.y) - (a.y * b.x)
        return Vec3(0, 0, z)
    """

    @classmethod
    def distance(cls, a, b):
        c = b - a
        return c.length()

    @classmethod
    def dot(self, a, b):
        return (a.x * b.x) + (a.y * b.y)

    @classmethod
    def equals(cls, a, b, tolerance=0.0):
        diff = a - b
        dx = math.fabs(diff.x)
        dy = math.fabs(diff.y)
        if dx <= tolerance * max(1, math.fabs(a.x), math.fabs(b.x)) and \
           dy <= tolerance * max(1, math.fabs(a.y), math.fabs(b.y)):
            return True
        return False

    @classmethod
    def max(cls, a, b):
        x = max(a.x, b.x)
        y = max(a.y, b.y)
        return Vec2(x, y)

    @classmethod
    def min(cls, a, b):
        x = min(a.x, b.x)
        y = min(a.y, b.y)
        return Vec2(x, y)

    @classmethod
    def mix(cls, a, b, t):
        return a * t + b * (1-t)

    @classmethod
    def random(cls):
        x = math.random.random()
        y = math.random.random()
        return Vec2(x, y)

    @classmethod
    def square_distance(cls, a, b):
        c = b - a
        return c.square_length()