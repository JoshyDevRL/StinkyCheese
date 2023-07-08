from util.common import *
from util.custom import *
import random

# This file holds all of the mechanical tasks, called "routines", that the bot can do


class drive():
    def __init__(self, speed, target=None) -> None:
        self.speed = speed
        self.target = target

    def run(self, agent):
        defaultThrottle(agent, self.speed)
        if self.target is not None:
            relative_target = self.target - agent.me.location
            defaultPD(agent, agent.me.local(relative_target))


class atba():
    # An example routine that just drives towards the ball at max speed
    def run(self, agent):
        relative_target = agent.ball.location - agent.me.location
        local_target = agent.me.local(relative_target)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)


class aerial_shot():
    # Very similar to jump_shot(), but instead designed to hit targets above 300uu
    # ***This routine is a WIP*** It does not currently hit the ball very hard, nor does it like to be accurate above 600uu or so
    def __init__(self, ball_location, intercept_time, shot_vector, ratio, dodge_aerial=False, z_extra=0):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        self.d_aerial = dodge_aerial
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we hit the ball at
        self.intercept = self.ball_location - (self.shot_vector * 110)
        # dictates when (how late) we jump, much later than in jump_shot because we can take advantage of a double jump
        self.jump_threshold = 100
        # what time we began our jump at
        self.jump_time = 0
        # If we need a second jump we have to let go of the jump button for 3 frames, this counts how many frames we have let go for
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.01, 10.0)

        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        car_to_intercept = self.intercept - agent.me.location
        car_to_intercept_perp = car_to_intercept.cross(
            (0, 0, side_of_shot))  # perpendicular
        flat_distance_remaining = car_to_intercept.flatten().magnitude()

        speed_required = flat_distance_remaining / time_remaining
        # When still on the ground we pretend gravity doesn't exist, for better or worse
        acceleration_required = backsolve(
            self.intercept, agent.me, time_remaining-0.05, 325)
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump
        adjustment = car_to_intercept.angle(
            self.shot_vector) * flat_distance_remaining / 1.57  # size of adjustment
        # factoring in how close to jump we are
        adjustment *= (cap(self.jump_threshold -
                       (acceleration_required[2]), 0.0, self.jump_threshold) / self.jump_threshold)
        # we don't adjust the final target if we are already jumping
        final_target = self.intercept + \
            ((car_to_intercept_perp.normalize() * adjustment)
             if self.jump_time == 0 else 0)

        # Some extra adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.intercept)
        agent.line(self.intercept-Vector3(0, 0, 100),
                   self.intercept+Vector3(0, 0, 100), [255, 0, 0])
        agent.line(final_target-Vector3(0, 0, 100),
                   final_target+Vector3(0, 0, 100), [0, 255, 0])

        if time_remaining < 0.75:
            angles = defaultPD(agent, self.ball_location)
        else:
            angles = defaultPD(agent, local_final_target)

        if self.jump_time == 0:
            defaultThrottle(agent, speed_required)
            agent.controller.boost = False if abs(
                angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
            agent.controller.handbrake = True if abs(
                angles[1]) > 2.3 else agent.controller.handbrake

            velocity_required = car_to_intercept / time_remaining
            good_slope = velocity_required[2] / cap(
                abs(velocity_required[0]) + abs(velocity_required[1]), 1, 10000) > 0.15
            if good_slope and (local_acceleration_required[2]) > self.jump_threshold and agent.me.velocity.flatten().normalize().dot(acceleration_required.flatten().normalize()) > 0.8:
                # Switch into the jump when the upward acceleration required reaches our threshold, hopefully we have aligned already...
                self.jump_time = agent.time
        else:
            time_since_jump = agent.time - self.jump_time

            # While airborne we boost if we're within 30 degrees of our local acceleration requirement
            if self.d_aerial and time_remaining < 0.15:
                agent.set_intent(flip(agent.me.local(agent.ball.location - agent.me.location), skip_first_jump=True, no_z=True))

            if agent.me.airborne and local_acceleration_required.magnitude() * time_remaining > (90 if not self.d_aerial else 60):
                angles = defaultPD(agent, local_acceleration_required)
                if abs(angles[0]) + abs(angles[1]) < 0.45:
                    agent.controller.boost = True
            else:
                final_target -= Vector3(0, 0, 45)
                local_final_target = agent.me.local(
                    final_target - agent.me.location)
                angles = defaultPD(agent, local_final_target)

            if self.counter == 0 and (time_since_jump <= 0.2 and local_acceleration_required[2] > 0):
                # hold the jump button up to 0.2 seconds to get the most acceleration from the first jump
                agent.controller.jump = True
            elif time_since_jump > 0.7 and self.counter < 3:
                # Release the jump button for 3 ticks
                agent.controller.jump = False
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.roll = 0
                self.counter += 1
            elif local_acceleration_required[2] > 300 and self.counter == 3 and time_since_jump > 0.7 and not self.d_aerial: 
                # the acceleration from the second jump is instant, so we only do it for 1 frame
                agent.controller.jump = True
                agent.controller.pitch = 0
                agent.controller.yaw = 0
                agent.controller.roll = 0
                self.counter += 1

        if raw_time_remaining < -0.25:
            agent.set_intent(recovery())
        if not shot_valid(agent, self, 90):
            agent.clear_intent()


class flip():
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False, skip_first_jump=False, no_z=False):
        self.no_z = no_z
        self.vector = vector.normalize()
        self.pitch = (abs(self.vector[0]) * -sign(self.vector[0])) if not self.no_z else -1
        self.yaw = abs(self.vector[1]) * sign(self.vector[1])
        self.cancel = cancel
        self.skip_first_jump = skip_first_jump
        
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent):
        if not self.skip_first_jump:
            if self.time == -1:
                elapsed = 0
                self.time = agent.time
            else:
                elapsed = agent.time - self.time
            if elapsed < 0.15:
                agent.controller.jump = True
            elif elapsed >= 0.15 and self.counter < 3:
                agent.controller.jump = False
                self.counter += 1
            elif elapsed < 0.3 or (not self.cancel and elapsed < 0.9):
                agent.controller.jump = True
                agent.controller.pitch = self.pitch
                agent.controller.yaw = self.yaw
            else:
                agent.set_intent(recovery())
        
        else:
            if self.time == -1:
                elapsed = 0
                self.time = agent.time
            else:
                elapsed = agent.time - self.time
            
            if elapsed < 0.2:
                agent.controller.jump = True
                agent.controller.pitch = self.pitch
                agent.controller.yaw = self.yaw
            else:
                agent.set_intent(recovery())


class goto():
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    # TODO - slow down if target is inside our turn radius
    def __init__(self, target, vector=None, direction=1, slow_down=False):
        self.target = target
        self.vector = vector
        self.direction = direction
        self.slow_down = slow_down

    def run(self, agent):
        car_to_target = self.target - agent.me.location
        distance_remaining = car_to_target.flatten().magnitude()

        agent.line(self.target - Vector3(0, 0, 500),
                   self.target + Vector3(0, 0, 500), [255, 0, 255])

        if self.vector != None:
            # See commends for adjustment in jump_shot or aerial for explanation
            side_of_vector = sign(self.vector.cross(
                (0, 0, 1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross(
                (0, 0, side_of_vector)).normalize()
            adjustment = car_to_target.angle(
                self.vector) * distance_remaining / 3.14
            final_target = self.target + (car_to_target_perp * adjustment)
        else:
            final_target = self.target

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target, self.direction)
        defaultThrottle(agent, (2300 if distance_remaining > 500 else -agent.me.local(agent.me.velocity).x) if self.slow_down else 2300, self.direction)

        agent.controller.boost = False
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 else agent.controller.handbrake

        velocity = 1+agent.me.velocity.magnitude()
        if distance_remaining < 350:
            agent.clear_intent()
        elif abs(angles[1]) < 0.05 and velocity > 600 and velocity < 2150 and distance_remaining / velocity > 2.0:
            agent.set_intent(flip(local_target))
        elif abs(angles[1]) > 2.8 and velocity < 200:
            agent.set_intent(flip(local_target, True))
        elif agent.me.airborne:
            agent.set_intent(recovery(self.target))

            
class goto_boost():
    # very similar to goto() but designed for grabbing boost
    # if a target is provided the bot will try to be facing the target as it passes over the boost
    def __init__(self, boost, target=None, use_boost=False):
        self.boost = boost
        self.target = target
        self.use_boost = use_boost

    def run(self, agent):
        car_to_boost = self.boost.location - agent.me.location
        distance_remaining = car_to_boost.flatten().magnitude()

        agent.line(self.boost.location - Vector3(0, 0, 500),
                   self.boost.location + Vector3(0, 0, 500), [0, 255, 0])

        if self.target != None:
            vector = (self.target - self.boost.location).normalize()
            side_of_vector = sign(vector.cross((0, 0, 1)).dot(car_to_boost))
            car_to_boost_perp = car_to_boost.cross(
                (0, 0, side_of_vector)).normalize()
            adjustment = car_to_boost.angle(vector) * distance_remaining / 3.14
            final_target = self.boost.location + \
                (car_to_boost_perp * adjustment)
            car_to_target = (self.target - agent.me.location).magnitude()
        else:
            adjustment = 9999
            car_to_target = 0
            final_target = self.boost.location.copy()

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_target = agent.me.local(final_target - agent.me.location)

        angles = defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)

        agent.controller.boost = (self.boost.large if not self.use_boost else True) if abs(
            angles[1]) < 0.3 else False
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.1 else agent.controller.handbrake

        velocity = 1+agent.me.velocity.magnitude()
        if self.boost.active == False or agent.me.boost >= 99.0 or distance_remaining < 350:
            agent.clear_intent()
        elif agent.me.airborne:
            agent.set_intent(recovery(target=self.target))
        elif abs(angles[1]) < 0.05 and velocity > 600 and velocity < 2150 and (distance_remaining / velocity > 2.0 or (adjustment < 90 and car_to_target/velocity > 2.0)):
            agent.set_intent(flip(local_target))


class jump_shot():
    # Hits a target point at a target time towards a target direction
    # Target must be no higher than 300uu unless you're feeling lucky
    #TODO - speed
    def __init__(self, ball_location, intercept_time, shot_vector, ratio, direction=1, speed=2300, z_extra=0, extra_jump_time=0):
        self.ball_location = ball_location
        self.intercept_time = intercept_time
        # The direction we intend to hit the ball in
        self.shot_vector = shot_vector
        # The point we dodge at
        # 173 is the 93uu ball radius + a bit more to account for the car's hitbox
        self.dodge_point = self.ball_location - (self.shot_vector * 173)
        # Ratio is how aligned the car is. Low ratios (<0.5) aren't likely to be hit properly
        self.ratio = ratio
        # whether the car should attempt this backwards
        self.direction = direction
        # Intercept speed not implemented
        self.speed_desired = speed
        self.z_extra = z_extra
        self.extra_jump_time = extra_jump_time
        # controls how soon car will jump based on acceleration required. max 584
        # bigger = later, which allows more time to align with shot vector
        #smaller = sooner
        self.jump_threshold = 400
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0

    def run(self, agent):
        raw_time_remaining = self.intercept_time - agent.time
        # Capping raw_time_remaining above 0 to prevent division problems
        time_remaining = cap(raw_time_remaining, 0.001, 10.0)
        car_to_ball = self.ball_location - agent.me.location
        # whether we are to the left or right of the shot vector
        side_of_shot = sign(self.shot_vector.cross((0, 0, 1)).dot(car_to_ball))

        car_to_dodge_point = self.dodge_point - agent.me.location
        car_to_dodge_perp = car_to_dodge_point.cross(
            (0, 0, side_of_shot))  # perpendicular
        distance_remaining = car_to_dodge_point.magnitude()

        speed_required = distance_remaining / time_remaining
        acceleration_required = backsolve(
            self.dodge_point, agent.me, time_remaining, 0 if not self.jumping else 650)
        local_acceleration_required = agent.me.local(acceleration_required)

        # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
        # The adjustment slowly decreases to 0 as the bot nears the time to jump
        adjustment = car_to_dodge_point.angle(
            self.shot_vector) * distance_remaining / 2.0  # size of adjustment
        # factoring in how close to jump we are
        adjustment *= (cap(self.jump_threshold -
                       (acceleration_required[2]), 0.0, self.jump_threshold) / self.jump_threshold)
        # we don't adjust the final target if we are already jumping
        final_target = self.dodge_point + \
            ((car_to_dodge_perp.normalize() * adjustment)
             if not self.jumping else 0) + Vector3(0, 0, 50)
        # Ensuring our target isn't too close to the sides of the field, where our car would get messed up by the radius of the curves

        # Some adjustment to the final target to ensure it's inside the field and we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        local_final_target = agent.me.local(final_target - agent.me.location)

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.dodge_point)
        agent.line(self.dodge_point-Vector3(0, 0, 100),
                   self.dodge_point+Vector3(0, 0, 100), [255, 0, 0])
        agent.line(final_target-Vector3(0, 0, 100),
                   final_target+Vector3(0, 0, 100), [0, 255, 0])
        agent.line(agent.ball.location, agent.ball.location +
                   (self.shot_vector * 300))

        # Calling our drive utils to get us going towards the final target
        angles = defaultPD(agent, local_final_target, self.direction)
        defaultThrottle(agent, speed_required, self.direction)

        agent.line(agent.me.location, agent.me.location +
                   (self.shot_vector*200), [255, 255, 255])

        agent.controller.boost = False if abs(
            angles[1]) > 0.3 or agent.me.airborne else agent.controller.boost
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 and self.direction == 1 else agent.controller.handbrake

        if not self.jumping:
            if raw_time_remaining <= 0.0 or (speed_required - 2300) * time_remaining > 60 or not shot_valid(agent, self):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we reset
                agent.clear_intent()
                if agent.me.airborne:
                    agent.set_intent(recovery())
            elif local_acceleration_required[2] > self.jump_threshold and local_acceleration_required[2] > local_acceleration_required.flatten().magnitude():
                # Switch into the jump when the upward acceleration required reaches our threshold, and our lateral acceleration is negligible
                self.jumping = True
        else:
            if (raw_time_remaining > 0.2 and not shot_valid(agent, self, 150)) or raw_time_remaining <= -0.9 or (not agent.me.airborne and self.counter > 0):
                agent.set_intent(recovery())
            elif self.counter == 0 and local_acceleration_required[2] > 0.0 + self.extra_jump_time and raw_time_remaining > 0.083:
                # Initial jump to get airborne + we hold the jump button for extra power as required
                agent.controller.jump = True
            elif self.counter < 3:
                # make sure we aren't jumping for at least 3 frames
                agent.controller.jump = False
                self.counter += 1
            elif raw_time_remaining <= 0.1 and raw_time_remaining > -0.9:
                # dodge in the direction of the shot_vector
                agent.controller.jump = True
                if not self.dodging:
                    vector = agent.me.local(self.shot_vector)
                    self.p = abs(vector[0]) * -sign(vector[0])
                    self.y = abs(vector[1]) * sign(vector[1]) * self.direction
                    self.dodging = True
                # simulating a deadzone so that the dodge is more natural
                agent.controller.pitch = self.p if abs(self.p) > 0.2 else 0
                agent.controller.yaw = self.y if abs(self.y) > 0.3 else 0


class simple_kickoff():
    def run(self, agent):
        target = agent.ball.location + Vector3(0, 300*side(agent.team), 0)
        local_target = agent.me.local(target - agent.me.location)
        defaultPD(agent, local_target)
        defaultThrottle(agent, 2300)
        if local_target.magnitude() < 650:
            agent.set_intent(
                flip(agent.me.local(agent.foe_goal.location - agent.me.location)))


class SpeedKickoff():
    def __init__(self):
        self.diag = False
        self.far_back = False
        self.diag_div = 17.5
        self.LAT = None
        self.KS = 0

    def run(self, agent: CheeseAgent):
        if self.LAT == None:
            self.LAT = agent.CT
        Delta = agent.CT - self.LAT
        self.diag = abs(agent.me.location.y) < 3200
        self.far_back = abs(agent.me.location.y) > 4000
        local_target = agent.me.local(agent.ball.location - agent.me.location)
        div = 1 if agent.team == 1 else -1
        dir = 0.5 * ((1 if agent.me.location.x > 0 else -1) if agent.team == 0 else (1 if agent.me.location.x < 0 else -1))
        dir_c = (1 if dir <= 0 else -1) if not self.far_back else (-1 if dir <= 0 else 1)
        error = 5
        sided = ((1 if agent.me.location.x > 0 else -1) if agent.team == 0 else (1 if agent.me.location.x < 0 else -1))

        dir_s = (0.1 * sided if self.diag == True else 0.3 * sided)
        if self.KS == 0:
            if Delta < 70 - (0 if self.diag == False else self.diag_div) - error:
                agent.controller.throttle = 1
                agent.controller.steer = (dir_s if not self.far_back else 0.2)
                agent.controller.boost = True
            else:
                agent.controller.throttle = 1
                agent.controller.boost = True
                self.KS = 1
        
        elif self.KS == 1:
            if Delta < 73 - (0 if self.diag == False else self.diag_div) - error:
                agent.controller.throttle = 1
                agent.controller.steer = (-(0 if self.diag else (dir_s * 2.2)) if not self.far_back else -1)
                agent.controller.boost = True
                agent.controller.jump = True
            else:
                agent.controller.throttle = 1
                agent.controller.boost = True
                self.KS = 2
        
        elif self.KS == 2:
            if Delta < 82 - (0 if self.diag == False else self.diag_div) - error:
                agent.controller.throttle = 1
                agent.controller.boost = True
                agent.controller.jump = False
            else:
                agent.controller.throttle = 1
                agent.controller.boost = True
                self.KS = 3
 
        elif self.KS == 3:
            if Delta < 87 - (0 if self.diag == False else self.diag_div) - error:
                agent.controller.throttle = 1
                agent.controller.boost = True
                agent.controller.yaw = dir_c
                agent.controller.jump = True
                agent.controller.pitch = -1
            else:
                agent.controller.throttle = 1
                agent.controller.boost = True
                self.KS = 4
        
        elif self.KS == 4:
            if Delta < 195 - (0 if self.diag == False else self.diag_div) - error:
                agent.controller.throttle = 1
                agent.controller.boost = True
                agent.controller.pitch = 1
            else:
                agent.controller.throttle = 1
                agent.controller.boost = True
                self.KS = 5
        
        elif self.KS == 5:
            if Delta < 231 - (0 if self.diag == False else self.diag_div) - error:
                agent.controller.throttle = 1
                agent.controller.boost = True
                agent.controller.yaw = dir_c
                agent.controller.roll = dir_c
            else:
                agent.controller.throttle = 1
                agent.controller.boost = True
                self.KS = 6

        elif self.KS == 6:
            agent.controller.throttle = 1
            agent.controller.boost = True
            local_ball_car = agent.me.local(agent.ball.location - agent.me.location)
            defaultPD(agent, local_ball_car)
            if local_target.magnitude() < 650:
                agent.set_intent(flip(agent.me.local(agent.foe_goal.location - agent.ball.location)))


class recovery():
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None):
        self.target = target

    def run(self, agent):
        if self.target != None:
            local_target = agent.me.local(
                (self.target-agent.me.location).flatten())
        else:
            local_target = agent.me.local(agent.me.velocity.flatten())

        defaultPD(agent, local_target)
        agent.controller.throttle = 1.0

        if not agent.me.airborne:
            agent.clear_intent()


class short_shot():
    # This routine drives towards the ball and attempts to hit it towards a given target
    # It does not require ball prediction and kinda guesses at where the ball will be on its own
    def __init__(self, target):
        self.target = target

    def run(self, agent):
        car_to_ball = (agent.ball.location - agent.me.location).normalize()
        distance = (agent.ball.location - agent.me.location).magnitude()
        ball_to_target = (self.target - agent.ball.location).normalize()

        relative_velocity = car_to_ball.dot(
            agent.me.velocity-agent.ball.velocity)
        if relative_velocity != 0.0:
            eta = cap(distance / cap(relative_velocity, 400, 2300), 0.0, 1.5)
        else:
            eta = 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross((0, 0, 1))
        right_vector = car_to_ball.cross((0, 0, -1))
        target_vector = -ball_to_target.clamp(left_vector, right_vector)
        final_target = agent.ball.location + (target_vector*(distance/2))

        # Some adjustment to the final target to ensure we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        agent.line(final_target-Vector3(0, 0, 100), final_target +
                   Vector3(0, 0, 100), [255, 255, 255])

        angles = defaultPD(agent, agent.me.local(
            final_target-agent.me.location))
        defaultThrottle(agent, 2300 if distance >
                        1600 else 2300-cap(1600*abs(angles[1]), 0, 2050))
        agent.controller.boost = False if agent.me.airborne or abs(
            angles[1]) > 0.3 else agent.controller.boost
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 else agent.controller.handbrake

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.set_intent(flip(agent.me.local(car_to_ball)))


class block_shot():
    def __init__(self, target):
        self.target = target

    def run(self, agent):
        car_to_ball = (agent.ball.location - agent.me.location).normalize()
        distance = (agent.ball.location - agent.me.location).magnitude()
        ball_to_target = (self.target - agent.ball.location).normalize()

        relative_velocity = car_to_ball.dot(
            agent.me.velocity-agent.ball.velocity)
        if relative_velocity != 0.0:
            eta = cap(distance / cap(relative_velocity, 400, 2300), 0.0, 1.5)
        else:
            eta = 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross((0, 0, 1))
        right_vector = car_to_ball.cross((0, 0, -1))
        target_vector = -ball_to_target.clamp(left_vector, right_vector)
        final_target = agent.ball.location + (target_vector*(distance/2))

        # Some adjustment to the final target to ensure we don't try to dirve through any goalposts to reach it
        if abs(agent.me.location[1]) > 5150:
            final_target[0] = cap(final_target[0], -750, 750)

        agent.line(final_target-Vector3(0, 0, 100), final_target +
                   Vector3(0, 0, 100), [255, 255, 255])

        angles = defaultPD(agent, agent.me.local(
            final_target-agent.me.location))
        defaultThrottle(agent, 2300 if distance >
                        1600 else 2300-cap(1600*abs(angles[1]), 0, 2050))
        agent.controller.boost = False if agent.me.airborne or abs(
            angles[1]) > 0.3 else agent.controller.boost
        agent.controller.handbrake = True if abs(
            angles[1]) > 2.3 else agent.controller.handbrake

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.set_intent(flip(agent.me.local(car_to_ball)))