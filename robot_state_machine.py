from XRPLib.defaults import *
from GFULib.defaults import *
import time


class StateMachine:
	# The State variable. Start the state in the RESET state
	state = "RED_SEARCHING"
	# This is used to save the name of the current state
	stateName = ""
	new_turn = True

	prev_head_diff = 1000000
	head_diff = 3000000
	# Initial motor speeds
	target_speed = 30
	turn_around_speed = 40
	left_speed = target_speed
	right_speed = target_speed
	currentZone = 0
	turn_around_target = 0

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

	def evaluate_state(self):
		# It might behoove you to verify that the self.state variable has been set to a legal
		# state name. That would mean that you would  create a list of the names, and see if
		# the state name is in that list ... that has been left as an exercise for the reader :)
		# But - I guarantee you are going to mistype a state name at some point on this project.

		current_time = time.ticks_ms()
		# This is the start of the state machine part
		if self.state == "RESET":
			self.print_state("RESET")
			drivetrain.set_speed(self.target_speed, self.target_speed)
			self.state = "FOLLOW_LEFT_WALL"

		elif self.state == "FOLLOW_LEFT_WALL":
			self.right_speed = self.target_speed
			self.left_speed = self.target_speed
			print('following left wall')
			self.print_state("FOLLOW_LEFT_WALL")
			# If there is something within 10 cm ahead (and we are getting a legit reading that isn't 0):
			if 0.0 < self.distAhead < 20.0:
				self.state = "ENCOUNTER_WALL"
			# If we are getting too close to the left wall (number getting bigger)
			elif self.distLeft > self.proximity_center + self.proximity_range / 2:
				self.state = "VEER_AWAY_FROM_LEFT_WALL"
			# If we are too far away from the left wall (number getting smaller)
			elif self.distLeft < self.proximity_center - self.proximity_range / 2:
				self.state = "VEER_TOWARD_LEFT_WALL"
		# elif self.distAhead == 0.0:
		#     print('rangefinder broken :\(')
		# Otherwise, we just stay in this happy little state :)

		elif self.state == "VEER_AWAY_FROM_LEFT_WALL":
			self.print_state("VEER_AWAY_FROM_LEFT_WALL")
			if 0.0 < self.distAhead < 20.0:
				self.state = "ENCOUNTER_WALL"
			if time.ticks_diff(current_time, self.update_time["state_interval"]) > self.read_ms:
				self.update_time["state_interval"] = current_time
				if self.left_speed < 40:
					self.left_speed = self.left_speed + (
							(abs(self.proximity_center - self.distLeft) / self.proximity_center) * .1)
				# print(self.left_speed)
				drivetrain.set_speed(self.left_speed, self.right_speed)

			# Once we see the value is within tolerance, immediately reset the speed (to keep it straight)
			# And let's return to the state we came from - which is FOLLOW_LEFT
			if self.distLeft <= self.proximity_center + self.proximity_range / 2:
				# Reset the left and right speeds to target for a future veer session
				self.left_speed = self.target_speed
				self.right_speed = self.target_speed
				# reset the motors to the target speed
				drivetrain.set_speed(self.target_speed, self.target_speed)
				# And head back to FOLLOW_LEFT
				self.state = "FOLLOW_LEFT_WALL"

		elif self.state == "VEER_TOWARD_LEFT_WALL":
			self.print_state("VEER_TOWARD_LEFT_WALL")
			print('dist=', self.distAhead)
			if 0.0 < self.distAhead < 20.0:
				self.state = "ENCOUNTER_WALL"
			if self.right_speed < 40:
				self.right_speed = self.right_speed + (
						(abs(self.proximity_center - self.distLeft) / self.proximity_center) * .1)
			drivetrain.set_speed(self.left_speed, self.right_speed)

			# Once we see the value is within tolerance, immediately reset the speed (to keep it straight)
			# And let's return to the state we came from - which is FOLLOW_LEFT
			if self.distLeft >= self.proximity_center - self.proximity_range / 2:
				# Reset the left and right speeds to target for a future veer session, set to target speed
				self.left_speed = self.target_speed
				self.right_speed = self.target_speed
				drivetrain.set_speed(self.target_speed, self.target_speed)
				self.state = "FOLLOW_LEFT_WALL"
				print('dist=', self.distAhead)
				drivetrain.set_speed(self.left_speed, self.right_speed)

		elif self.state == "ENCOUNTER_WALL":
			self.print_state("Encounter Wall")
			# if self.new_turn == True:
			self.turn_angle = 90
			drivetrain.straight(5, -1)
			# (self.target_heading) = abs(self.heading - 90) % 360
			# self.new_turn = True
			# print(f'{self.heading}/{self.target_heading} ({self.right_speed}, {self.left_speed}) - start a turn')
			self.state = "TURN_LEFT"

		elif self.state == "TURN_LEFT":

			self.target_heading = abs(self.heading - self.turn_angle) % 360
			accurcy_perc = 5

			target_heading_bounds = [self.target_heading + (self.target_heading * 0.01 * accurcy_perc),
			                         self.target_heading - (self.target_heading * 0.01 * accurcy_perc)]

			print(target_heading_bounds[1], '-', target_heading_bounds[0])
			print(f'In range is {self.heading in range(int(target_heading_bounds[1]), int(target_heading_bounds[0]))}')

			if not int(target_heading_bounds[1]) <= self.heading <= int(target_heading_bounds[0]):
				print(f'In range is false skjdh')
				self.right_speed = self.target_speed
				self.left_speed = -1 * self.target_speed
			if 0.0 < self.distAhead < 10:
				self.state = "ENCOUNTER_WALL"

			if int(target_heading_bounds[1]) <= self.heading <= int(target_heading_bounds[0]):
				if self.currentZone == 0:
					print("going straight")
					self.state = "ZONEFINDER_STRAIGHTAWAY"
				elif self.currentZone == 1:
					self.new_turn = True
				self.state = "FOLLOW_LEFT_WALL"
				print('we are so back')

			print(
				f'target {self.target_heading}\nbounds {target_heading_bounds[0]} {target_heading_bounds[1]}\ncurrent heading {self.heading}')
			drivetrain.set_speed(self.left_speed, self.right_speed)
		elif self.state == "RED_SEARCHING":
			currentCol = color.getColor()
			print(f'current color is {currentCol}')
			if currentCol == 'red':
				print("Found red")
				self.state = "GREEN_SEARCHING"

		elif self.state == "GREEN_SEARCHING":
			currentCol = color.getColor()
			print(f'current color is {currentCol}')
			if currentCol == 'red':
				print("I'm red-y")
				self.state = "GREEN_SEARCHING"
			if currentCol == 'green':
				self.state = "GREEN_DETECTED"

		elif self.state == "GREEN_DETECTED":
			self.currentZone = 0
			self.turn_angle = 180
			self.state = "TURN_LEFT"

		elif self.state == "ZONEFINDER_STRAIGHTAWAY":
# 			drivetrain.turn(180, 1, 5)
			dist_trav_left = 0
			dist_trav_right = 0
			current_dist_trav = (dist_trav_right + dist_trav_left) / 2
			print(current_dist_trav)
			self.left_speed = self.target_speed
			self.right_speed = self.target_speed
			drivetrain.set_speed(self.left_speed, self.right_speed)


sm = StateMachine()

while True:
	sm.update_sensors()
	print(sm.right_speed)
	# print(str(sm.heading))
	# print(sm.state)
	sm.evaluate_state()
