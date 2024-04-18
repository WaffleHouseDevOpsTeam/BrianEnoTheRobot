from XRPLib.defaults import *
from GFULib.defaults import *
from machine import Pin
import time

red = Pin(8, Pin.OUT)
green = Pin(9, Pin.OUT)
blue = Pin(17, Pin.OUT)


def setColor(value):
	if value == 'red':
		red.value(1)
	else:
		red.value(0)
	if value == 'green':
		green.value(1)
	else:
		green.value(0)
	if value == 'blue':
		blue.value(1)
	else:
		blue.value(0)
	if value == 'white':
		red.value(1)
		green.value(1)
		blue.value(1)


class StateMachine:
	# The State variable. Start the state in the RESET state
	state = "RED_SEARCHING"
	# This is used to save the name of the current state
	stateName = ""
	new_turn = True
	turnOkay = False

	prev_head_diff = 1000000
	head_diff = 3000000
	# Initial motor speeds
	target_speed = 30
	turn_around_speed = 40
	targetleft_speed = target_speed
	targetright_speed = 0.8 * target_speed
	newPrint = True
	previousState = ''
	wallCrawlMode = 'right'
	zone1initialturn = True
	zone1_oneeighty = False
	gothisway = 0
	foundFirstHole = False

	# Proximity values for wall following
	proximity_center = 500  # The ideal value we want to read
	proximity_range = 250  # The complete range over which the proximity will be considered "stable"
	# That is - nothing to worry about. So - proximity_center +/- range/2

	# The time in ms between each read of the sensors.
	read_ms = 10

	# The distance readout from each sensor
	distAhead = 0  # Rangefinder
	distRight = 0  # Proximity0
	distLeft = 0  # Proximity1
	heading = 0  # IMU
	dist_trav_left = 0
	dist_trav_right = 0

	# and a dictionary to hold the times when each of the sensors were last updated
	update_time = {}

	def __init__(self):
		# get the current time at the start
		current_time = time.ticks_ms()

		imu.calibrate(2, 0)
		imu.calibrate(2, 1)
		imu.calibrate(2, 2)
		# Note that they are being started using staggered time - so that when we go through the
		# loop, we don't try and read each of them at the same time, but balance it across
		# the read_ms time - which is 10 ms currently.
		self.update_time["rangefinder"] = current_time
		self.update_time["proximity0"] = time.ticks_add(current_time, 2)
		self.update_time["proximity1"] = time.ticks_add(current_time, 4)
		self.update_time["heading"] = time.ticks_add(current_time, 6)
		self.update_time["distance_traveled_left"] = time.ticks_add(current_time, 7)
		self.update_time["distance_traveled_right"] = time.ticks_add(current_time, 7)
		self.update_time["state_interval"] = time.ticks_add(current_time, 8)

	# We want to have a function that prints the state name for debugging
	# But only when we enter it - not every time through - that would be crazy
	def print_state(self, newState):
		if newState != self.stateName:
			print(f"Entering state {newState}")
			self.stateName = newState

	def update_sensors(self):
		# Each time we are called, check the current time ...
		current_time = time.ticks_ms()

		# Then compare that current time against the update_time for each element
		# to see if it is time to read it again.  For this example, I am using
		# the same read_ms for each of them - you don't have to.

		if time.ticks_diff(current_time, self.update_time["rangefinder"]) >= self.read_ms:
			self.update_time["rangefinder"] += self.read_ms
			self.distAhead = GFURangefinder.distance()
		if time.ticks_diff(current_time, self.update_time["proximity0"]) >= self.read_ms:
			self.update_time["proximity0"] += self.read_ms
			self.distRight = proximity0.getProximity()
		if time.ticks_diff(current_time, self.update_time["proximity1"]) >= self.read_ms:
			self.update_time["proximity1"] += self.read_ms
			self.distLeft = proximity1.getProximity()
		if time.ticks_diff(current_time, self.update_time["heading"]) >= self.read_ms:
			self.update_time["heading"] += self.read_ms
			self.heading = imu.get_heading()
		if time.ticks_diff(current_time, self.update_time["distance_traveled_left"]) >= self.read_ms:
			self.update_time["distance_traveled_left"] += self.read_ms
			self.dist_trav_left = drivetrain.get_left_encoder_position()
		if time.ticks_diff(current_time, self.update_time["distance_traveled_right"]) >= self.read_ms:
			self.update_time["distance_traveled_right"] += self.read_ms
			self.dist_trav_right = drivetrain.get_right_encoder_position()

	def define_course(self, accuracy):
		self.course_range = [self.heading - (accuracy * 0.01 * self.heading),
		                     self.heading + (accuracy * 0.01 * self.heading)]

	def stay_straight(self):
		self.define_course(1)
		print(self.heading)
		print(self.course_range)
		if self.heading <= self.course_range[0]:
			drivetrain.set_speed(self.targetleft_speed, 10)
			print('slight left')
		elif self.heading >= self.course_range[1]:
			drivetrain.set_speed(10, self.targetright_speed)
			print('slight right')
		else:
			drivetrain.set_speed(self.targetleft_speed, self.targetright_speed)
			print('staying straight')

	# the problem is that the speeds arent accurate???? but this doesn't seem to fix :(
	# It might be fixed, maybe, we shall see

	def evaluate_state(self):

		current_time = time.ticks_ms()

		if self.state == "FOLLOW_WALL":
			self.right_speed = self.targetright_speed
			self.left_speed = self.targetleft_speed
			print(f'following wall, wallCrawlMode is {self.wallCrawlMode}')
			# If there is something within 10 cm ahead (and we are getting a legit reading that isn't 0):
			if 0.0 < self.distAhead < 10.0:
				self.state = "ENCOUNTER_WALL"
			# If we are getting too close to the left wall (number getting bigger)
			elif (self.distLeft > 20.0) and (self.distRight > 20.0) and (self.zone1initialturn == True):
				self.state = 'ENTERING_ZONE2'
			if self.wallCrawlMode == 'right':
				if (self.distRight > self.proximity_center + self.proximity_range / 2):
					self.state = 'VEER_TOWARD_WALL'
				elif (self.distRight < self.proximity_center + self.proximity_range / 2):
					self.state = 'VEER_AWAY_FROM_WALL'
			if self.wallCrawlMode == 'left':
				if (self.distLeft > self.proximity_center + self.proximity_range / 2):
					self.state = 'VEER_TOWARD_WALL'
				elif (self.distLeft < self.proximity_center + self.proximity_range / 2):
					self.state = 'VEER_AWAY_FROM_WALL'
			if (self.wallCrawlMode == 'right') and (self.distRight > 10):
				foundFirstHole = True
				self.state = 'TURNING'
			# elif (self.distLeft > self.proximity_center + self.proximity_range / 2) and (self.distRight > 10.0):
			# 		self.state = 'VEER_TOWARD_WALL'
			# # If we are too far away from the left wall (number getting smaller)
			# elif (self.distLeft > self.proximity_center - self.proximity_range / 2) and (self.distRight > 10.0):
			# 	self.state = "VEER_TOWARD_WALL"
			# elif (self.distRight > self.proximity_center + self.proximity_range / 2) and (self.distLeft > 10.0):
			# 	self.state = "VEER_AWAY_FROM_WALL"
			# # If we are too far away from the left wall (number getting smaller)
			# elif (self.distRight < self.proximity_center - self.proximity_range / 2) and (self.distLeft > 10.0):
			# 	self.state = "VEER_TOWARD_WALL"

		elif self.state == "VEER_AWAY_FROM_WALL":
			self.print_state("VEER_AWAY_FROM_WALL")
			if 0.0 < self.distAhead < 10.0:
				self.state = "ENCOUNTER_WALL"
			if time.ticks_diff(current_time, self.update_time["state_interval"]) > self.read_ms:
				self.update_time["state_interval"] = current_time
				if self.wallCrawlMode == 'left':
					if self.left_speed < 40:
						self.left_speed = self.left_speed + (
								(abs(self.proximity_center - self.distLeft) / self.proximity_center) * .1)
				if self.wallCrawlMode == 'right':
					if self.right_speed < 40:
						self.right_speed = self.right_speed + (
								(abs(self.proximity_center - self.distRight) / self.proximity_center) * .1)
				drivetrain.set_speed(self.left_speed, self.right_speed)

			# Once we see the value is within tolerance, immediately reset the speed (to keep it straight)
			# And let's return to the state we came from - which is FOLLOW_LEFT
			if (self.distLeft <= self.proximity_center + self.proximity_range / 2) or (
					self.distRight <= self.proximity_center + self.proximity_range / 2):
				self.left_speed = self.targetleft_speed
				self.right_speed = self.targetright_speed
				drivetrain.set_speed(self.targetleft_speed, self.targetright_speed)
				self.state = "FOLLOW_WALL"

		elif self.state == "VEER_TOWARD_WALL":
			self.print_state("VEER_TOWARD_WALL")
			print('dist=', self.distAhead)
			initialHeading = self.heading
			if 0.0 < self.distAhead < 10.0:
				self.state = "ENCOUNTER_WALL"
			if self.wallCrawlMode == 'left':
				if self.right_speed < 40:
					self.right_speed = self.right_speed + (
							(abs(self.proximity_center - self.distLeft) / self.proximity_center) * .1)
					self.left_speed = 10
				if (abs(self.heading - initialHeading) >= 80):
					if self.previousState == 'ZONEFINDER_STRAIGHTAWAY':
						self.state = 'ENTERING_ZONE2'
			if self.wallCrawlMode == 'right':
				if self.left_speed < 40:
					self.left_speed = self.left_speed + (
							(abs(self.proximity_center - self.distRight) / self.proximity_center) * .1)
					self.right_speed = 10
			drivetrain.set_speed(self.left_speed, self.right_speed)

			# Once we see the value is within tolerance, immediately reset the speed (to keep it straight)
			# And let's return to the state we came from - which is FOLLOW_LEFT
			if (self.distLeft >= self.proximity_center - self.proximity_range / 2) or (
					self.distRight >= self.proximity_center - self.proximity_range / 2):
				# Reset the left and right speeds to target for a future veer session, set to target speed
				self.left_speed = self.targetleft_speed
				self.right_speed = self.targetright_speed
				drivetrain.set_speed(self.targetleft_speed, self.targetright_speed)
				self.state = "FOLLOW_WALL"
				print('dist=', self.distAhead)
				drivetrain.set_speed(self.left_speed, self.right_speed)

		elif self.state == "ENCOUNTER_WALL":
			drivetrain.straight(5, -1)
			self.state = "TURNING"

		elif self.state == "TURNING":
			# if crawling using left sensor, it needs to turn to its right (away from wall)
			if self.zone1initialturn == True:
				self.turn_angle = 180
				turning = 'around'
			if self.wallCrawlMode == 'left':
				self.turn_angle = -90
				turning = 'right'
			# if crawling using right sensor along zone 1 wall and detects wall in front of it, needs to turn 180
			# if ((self.wallCrawlMode == 'right') and (self.zone1initialturn == True)) or (self.zone1initialturn == False):
			# 	self.turn_angle = 180
			# 	turning = 'around'
			# if crawling using right sensor, it needs to turn to its left (away from wall)
			if self.wallCrawlMode == 'right':
				self.turn_angle = 90
				turning = 'left'
			if self.foundFirstHole == True:
				self.target_heading = self.gothisway
				self.new_turn = False
			print(f'turning {turning} at an angle of {self.turn_angle} degrees')
			# this ensures target heading is only ever updated once per state
			if self.new_turn == True:
				self.target_heading = abs(self.heading + self.turn_angle) % 360
				self.new_turn = False
			accurcy_perc = 5
			# this ensures print is only called once to keep things from getting too cluttered
			if self.newPrint == True:
				print(
					f'turn angle is {self.turn_angle}, self_heading is {self.heading}, target heading is {self.target_heading}')
				self.newPrint == False
			target_heading_small = self.target_heading - (self.target_heading * 0.01 * accurcy_perc)
			target_heading_big = self.target_heading + (self.target_heading * 0.01 * accurcy_perc)

			print(target_heading_small, '-', target_heading_big)
			print(f'In range is {self.heading in range(int(target_heading_small), int(target_heading_big))}')

			if not int(target_heading_small) <= self.heading <= int(target_heading_big):
				if self.wallCrawlMode == 'left':
					self.right_speed = -1 * self.targetright_speed
					self.left_speed = self.targetleft_speed
				if self.wallCrawlMode == 'right':
					self.right_speed = self.targetright_speed
					self.left_speed = -1 * self.targetleft_speed
			if 0.0 < self.distAhead < 5:
				drivetrain.straight(5, -1)
			if int(target_heading_small) <= self.heading <= int(target_heading_big):
				self.new_turn = True
				# if self.zone1initialturn == True:
				# 	self.zone1_oneeighty = True
				# 	print('scanned the wall, no holes found, turning around now uWu')
				# 	self.wallCrawlMode = 'left'
				if self.previousState == "ZONEFINDER_STRAIGHTAWAY":
					self.zone1initialturn = True
					print('just hit zone 1 wall and it kinda hurt')
				self.state = "FOLLOW_WALL"

			print(
				f'target {self.target_heading}\nbounds {target_heading_small} {target_heading_big}\ncurrent heading {self.heading}')
			drivetrain.set_speed(self.left_speed, self.right_speed)

		elif self.state == "RED_SEARCHING":
			# it starts here but STUPID ENCOUNTER WALL REEE
			self.turnOkay = False
			self.wallCrawlMode = 'right'
			currentCol = color.getColor()
			print(f'current color is {currentCol}')
			if currentCol == 'red':
				setColor(currentCol)
				print("Found red")
				self.turnOkay = False
				self.state = "GREEN_SEARCHING"

		elif self.state == "GREEN_SEARCHING":
			currentCol = color.getColor()
			print(f'current color is {currentCol}')
			if currentCol == 'red':
				self.turnOkay = False
				print("I was born red-y")
				self.state = "GREEN_SEARCHING"
			if currentCol == 'green':
				setColor(currentCol)
				self.turnOkay = True
			if self.turnOkay == True:
				drivetrain.turn(180, -1, 5)
				time.sleep(0.75)
				self.gothisway = self.heading
				self.state = "ZONEFINDER_STRAIGHTAWAY"

		elif self.state == "ZONEFINDER_STRAIGHTAWAY":
			self.previousState = self.state
			# self.define_course(5)
			# self.stay_straight()
			drivetrain.set_speed(self.targetleft_speed, self.targetright_speed)
			if self.distAhead <= 9.0:
				drivetrain.straight(5, -1)
				self.state = "TURNING"

		elif self.state == "HIT_LAST_ZONE1_WALL":
			# might not need, i think we're going to cover this in encounter_wall
			# it hits the wall keeping it from entering zone 2, prompting it to crawl the wall in search of a hole
			# there will be a variable that, depending on whether or not it's using left/right wallcrawler, tells it angle to turn when entering zone 2
			# if it hits an outer wall before finding that hole, it does a 180 turn and notes that it hit an outer wall
			# because this SHOULD mean that the light is on that outer wall
			pass

		elif self.state == "ENTERING_ZONE2":
			if self.wallCrawlMode == 'left':
				drivetrain.turn(-90, -1, 5)
				time.sleep(0.5)
				drivetrain.straight(10, 1)
			if self.wallCrawlMode == 'right':
				drivetrain.turn(90, -1, 5)
				time.sleep(0.5)
				drivetrain.straight(10, 1)

			# using aforementioned variable, turn robot and proceed set distance into zone 2 (use blocking at first, make non-blocking if there's time)
			# turn another 45/-45 (STORE AS VAR) so that it's heading straight towards light. read value of light and then jump to next state
			pass

		elif self.state == "FINDING_ZONE2_LIGHT":
			# will go forward until it consistently (for <0.5 sec maybe?) detects a solid color light and stores that color as a variable (and changes led)
			# if it doesn't, it'll immediately do a 180 and go to above state
			# then it'll enter next state
			pass

		elif self.state == "FINDING_ZONE3_ENTRANCE":
			# turn by stored var from ENTERING_ZONE2, proceed straight until it hits wall or enters zone 3
			# if it hits wall, crawl along it until it finds hole, next state
			# if it hits zone 3 (how do we know this?), next state
			pass


sm = StateMachine()

while True:
	sm.update_sensors()
	# print(sm.right_speed)
	# print(str(sm.heading))
	print(sm.state)
	sm.evaluate_state()
