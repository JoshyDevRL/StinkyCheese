from util.objects import *
from util.common import *
import numpy as np


TIME_STEP = 1/120
MAX_THRUST = 1

KP_YAW = 1.0
KI_YAW = 0.0
KD_YAW = 0.0


class PID():
	def __init__(self, KP, KI, KD):
		self.kp = KP
		self.ki = KI
		self.kd = KD
		self.setpoint = 0
		self.error = 0
		self.integral_error = 0
		self.error_last = 0
		self.derivative_error = 0
		self.output = 0

	def compute(self, dir):
		self.error = self.setpoint + dir
		print(self.error)
		self.integral_error += self.error * TIME_STEP
		self.derivative_error = (self.error - self.error_last) / TIME_STEP
		self.error_last = self.error
		self.output = self.kp * self.error + self.ki * self.integral_error + self.kd * self.derivative_error
		if self.output >= MAX_THRUST:
			self.output = MAX_THRUST
		elif self.output <= -MAX_THRUST:
			self.output = -MAX_THRUST
		return self.output
	

# class AerialShot():
#     def __init__(self, target: Vector3):
#         self.target = target
#         self.PID_YAW = PID(KP_YAW, KI_YAW, KD_YAW)
#         self.dirsYaw = np.array([])
#         self.times = np.array([])
#         self.timer = 0 # temp

#     def run(self, agent: CheeseAgent):
# 	    agent.controller.jump = True
#         Yaw = self.PID_YAW.compute(get_flat_angle(agent.me.location, self.target))
#         print(Yaw)
#         agent.controller.yaw = Yaw
#         self.timer += 1
#         self.dirsYaw = np.append(self.dirsYaw, get_flat_angle(agent.me.location, self.target))
#         self.times = np.append(self.times, self.timer)








def get_flat_angle(v: Vector3, other: Vector3):
    return v.flatten().angle(other.flatten())

def norm(v: Vector3):
	return math.sqrt(v.dot(v))