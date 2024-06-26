from XRPLib.defaults import *
from GFULib.defaults import *
from machine import Pin
import time

red = Pin(8, Pin.OUT)
green = Pin(9, Pin.OUT)
blue = Pin(17, Pin.OUT)


class robot:
    # Control Variables
    state = 'detect green'
    state_prev = ''
    sub_state = ''

    phase = 0
    init_state = True

    search_color = 'green'
    detected_color = ''

    crawl_mode = 'Right'
    turn_direction = 'Left'

    # Initial motor speeds
    target_speed = 30
    turn_around_speed = 40

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
    dist_traveled_left = 0
    distance_traveled_right = 0

    # Room Variables
    room_width = 0  # replace with measurements
    wall_length = 0  # This will update, no need to edit here

    # misc variables
    dist_stop = 10

    # and a dictionary to hold the times when each of the sensors were last updated
    update_time = {}

    def __init__(self):
        # get the current time at the star
        self.current_time = time.ticks_ms()

        imu.calibrate(2, 0)
        imu.calibrate(2, 1)
        imu.calibrate(2, 2)
        # Note that they are being started using staggered time - so that when we go through the
        # loop, we don't try and read each of them at the same time, but balance it across
        # the read_ms time - which is 10 ms currently.
        self.update_time["rangefinder"] = self.current_time
        self.update_time["proximity0"] = time.ticks_add(self.current_time, 2)
        self.update_time["proximity1"] = time.ticks_add(self.current_time, 4)
        self.update_time["heading"] = time.ticks_add(self.current_time, 6)
        self.update_time["distance_traveled_left"] = time.ticks_add(self.current_time, 7)
        self.update_time["distance_traveled_right"] = time.ticks_add(self.current_time, 7)
        self.update_time["state_interval"] = time.ticks_add(self.current_time, 8)

    def switch_crawl_mode(self):
        if self.crawl_mode == 'Left':
            self.crawl_mode = 'Right'
        else:
            self.crawl_mode = 'Left'

    def setColor(self, value):
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

    def update_sensors(self):
        # Each time we are called, check the current time ...
        self.current_time = time.ticks_ms()

        # Then compare that current time against the update_time for each element
        # to see if it is time to read it again.  For this example, I am using
        # the same read_ms for each of them - you don't have to.

        if time.ticks_diff(self.current_time, self.update_time["rangefinder"]) >= self.read_ms:
            self.update_time["rangefinder"] += self.read_ms
            self.distAhead = GFURangefinder.distance()
        if time.ticks_diff(self.current_time, self.update_time["proximity0"]) >= self.read_ms:
            self.update_time["proximity0"] += self.read_ms
            self.distRight = proximity0.getProximity()
        if time.ticks_diff(self.current_time, self.update_time["proximity1"]) >= self.read_ms:
            self.update_time["proximity1"] += self.read_ms
            self.distLeft = proximity1.getProximity()
        if time.ticks_diff(self.current_time, self.update_time["heading"]) >= self.read_ms:
            self.update_time["heading"] += self.read_ms
            self.heading = imu.get_heading()
        if time.ticks_diff(self.current_time, self.update_time["distance_traveled_left"]) >= self.read_ms:
            self.update_time["distance_traveled_left"] += self.read_ms
            self.dist_trav_left = drivetrain.get_left_encoder_position()
        if time.ticks_diff(self.current_time, self.update_time["distance_traveled_right"]) >= self.read_ms:
            self.update_time["distance_traveled_right"] += self.read_ms
            self.dist_trav_right = drivetrain.get_right_encoder_position()

    def print_debug(self):
        print(
            f'{self.phase} state: {self.state} | crawl_mode {self.crawl_mode} | turn_direction {self.turn_direction} | {self.search_color} {self.detected_color} | {self.sub_state} | {self.current_time}')

    def immigrate(self):
        if self.init_state:
            immi_cond = False
            self.init_state = False

    def emmigrate(self):
        self.init_state = True

    def eval_state(self):
        if self.state == f'detect {self.search_color}':
            self.immigrate()
            self.detected_color = color.getColor()
            if self.detected_color == self.search_color:
                if self.phase == 0:
                    self.phase = 1
                    drivetrain.turn(180, -1, 1 )
                    self.state = 'go_straight'

                elif self.phase == 1:
                    self.phase == 2
                    drivetrain.turn(90, -1, 1)
                    self.state = 'go_straight'

                elif self.phase == 3:
                    self.phase = 'return'
                    self.state = 'return'
                self.emmigrate()

        elif self.state == 'go_straight':
            self.immigrate()
            if self.distAhead <= self.dist_stop:
                self.state = 'encounter_wall'
            else:
                drivetrain.set_speed(self.target_speed, self.target_speed)
            self.emmigrate()

        elif self.state == 'encounter_wall':

            self.immigrate()
            if self.phase == 1:
                self.crawl_mode = 'Right'
                self.turn_direction = 'Left'
            drivetrain.straight(5, -1)
            self.state = f'turn {self.turn_direction}'
            self.emmigrate()

        elif self.state == f'turn {self.turn_direction}':
            self.immigrate()
            if self.turn_direction == 'Left':
                drivetrain.turn(-90, -1, 1)
            elif self.turn_direction == 'Right':
                drivetrain.turn(90, -1, 1)

            if self.phase == 2:
                self.state = 'go_straight'
            elif self.phase == 3:
                self.state = 'detect_move_straight'
            else:
                self.state = f'follow {self.crawl_mode} wall'
            self.emmigrate()

        elif self.state == f'follow {self.crawl_mode} wall':
            if self.init_state:
                print('init')
                if self.crawl_mode == 'Right':
                    self.turn_ang = self.heading + 90
                elif self.crawl_mode == 'Left':
                    self.turn_ang = self.heading - 90
                self.sub_state = 'FOLLOW_WALL'
            self.immigrate()
            
            if (self.distAhead <= self.dist_stop) and (self.phase == 3):
                switch_crawl_mode()
                self.state = f'turn {self.turn_direction}'
            elif self.heading >= self.turn_ang:
                self.phase += 1
                self.state = 'go_center_room'
            else:
                if self.sub_state == "FOLLOW_WALL":
                    self.right_speed = self.target_speed
                    self.left_speed = self.target_speed
                    # If there is something within 10 cm ahead (and we are getting a legit reading that isn't 0):
                    # If we are getting too close to the left wall (number getting bigger)
                    if self.crawl_mode == 'Right':
                        if (self.distRight > self.proximity_center + self.proximity_range / 2):
                            self.sub_state = "VEER_AWAY_FROM_WALL"
                        # If we are too far away from the left wall (number getting smaller)
                        elif (self.distRight < self.proximity_center - self.proximity_range / 2):
                            self.sub_state = "VEER_TOWARD_WALL"
                    elif self.crawl_mode == 'Left':
                        if (self.distLeft > self.proximity_center + self.proximity_range / 2):
                            self.sub_state = "VEER_AWAY_FROM_WALL"
                        # If we are too far away from the left wall (number getting smaller)
                        elif (self.distLeft < self.proximity_center - self.proximity_range / 2):
                            self.sub_state = "VEER_TOWARD_WALL"

                if self.sub_state == "VEER_AWAY_FROM_WALL":
                    if time.ticks_diff(self.current_time, self.update_time["state_interval"]) > self.read_ms:
                        self.update_time["state_interval"] = self.current_time
                        if self.crawl_mode == 'Left' and self.left_speed < 40:
                            self.left_speed = self.left_speed + (
                                (abs(self.proximity_center - self.distLeft) / self.proximity_center) * .1)
                        if self.crawl_mode == 'Right' and self.right_speed < 40:
                            self.right_speed = self.right_speed + (
                                (abs(self.proximity_center - self.distRight) / self.proximity_center) * .1)
                        drivetrain.set_speed(self.left_speed, self.right_speed)
        
                    # Once we see the value is within tolerance, immediately reset the speed (to keep it straight)
                    # And let's return to the state we came from - which is FOLLOW_LEFT
                    if (self.distLeft <= self.proximity_center + self.proximity_range / 2) or (
                            self.distRight <= self.proximity_center + self.proximity_range / 2):
                        self.left_speed = self.target_speed
                        self.right_speed = self.target_speed
                        drivetrain.set_speed(self.target_speed, self.target_speed)
                        self.sub_state = "FOLLOW_WALL"
        
                elif self.sub_state == "VEER_TOWARD_WALL":
                    
                    initialHeading = self.heading
                    if self.crawl_mode == 'Left' and self.right_speed < 40:
                        self.right_speed = self.right_speed + (
                                (abs(self.proximity_center - self.distLeft) / self.proximity_center) * .1)
                    elif self.crawl_mode == 'right' and self.left_speed < 40:
                            self.left_speed = self.left_speed + (
                                (abs(self.proximity_center - self.distRight) / self.proximity_center) * .1)
                    drivetrain.set_speed(self.left_speed, self.right_speed)
        
                    # Once we see the value is within tolerance, immediately reset the speed (to keep it straight)
                    # And let's return to the state we came from - which is FOLLOW_LEFT
                    if (self.distLeft >= self.proximity_center - self.proximity_range / 2) or (
                            self.distRight >= self.proximity_center - self.proximity_range / 2):
                        # Reset the left and right speeds to target for a future veer session, set to target speed
                        self.left_speed = self.target_speed
                        self.right_speed = self.target_speed
                        drivetrain.set_speed(self.target_speed, self.target_speed)
                        self.sub_state = "FOLLOW_WALL"

        elif self.state == 'go_center_room':
            self.start_point = self.dist_traveled_left
            bounds = [self.room_bounds[self.phase - 1] + (0.001 * self.room_bounds[self.phase - 1]),
                      self.room_bounds[self.phase - 1] - (0.001 * self.room_bounds[self.phase - 1])]
            self.immigrate()
            if bounds[1] <= self.dist_traveled_left - self.start_point <= bounds[0]:
                if self.wall_length <= self.true_wall_length:
                    self.crawl_mode = 'Left'
                else:
                    self.crawl_mode = 'Right'
                self.state = f'turn {self.turn_direction}'
                self.emmigrate()
            else:
                drivetrain.set_speed(self.target_speed, self.target_speed)

        elif self.state == 'detect_move_straight':
            pass

        elif self.state == 'return':
            pass


sm = robot()

while True:
    sm.update_sensors()
    sm.print_debug()
    sm.eval_state()
