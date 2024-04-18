

class robot:
    # Control Variables
    state = ''
    state_prev = ''

    phase = 0
    init_state = True

    search_color = 'Green'
    detected_color = ''

    crawl_mode = 'Right'
    turn_mode = 'Left'

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
    dist_trav_left = 0
    dist_trav_right = 0

    # Room Variables
    room_width = 0 # replace with measurements
    wall_length = 0 # This will update, no need to edit here
    
    # misc variables
    dist_stop = 20

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
    
    def immigrate(self):
        if self.init_state:
            immi_cond = False
            self.init_state = False

    def emmigrate(self):
        self.init_state = True
    
    def eval_state(self):
        if self.state = f'detect {self.search_color}':
            immigrate()
            self.detected_color = color.getColor()
            if self.detected_color == self.search_color:
                if self.phase == 0:
                    self.phase = 1
                    drivetrain.turn(180, -1, 5)
                    self.state = 'go_straight'

                elif self.phase == 1:
                    self.phase == 2
                    drivetrain.turn(90, -1, 5)
                    self.state = 'go_straight'

                elif self.phase == 3:
                    self.phase = 'return'
                    self.state = 'return'
                
                emmigrate()

    
        elif self.state = 'go_straight':
            immigrate()
            if self.distAhead <= self.dist_stop:
                self.state = 'encounter_wall'
            else:
                drivetrain.set_speed(self.target_speed, self.target_speed)
            emmigrate()
        
        elif self.state = 'encounter_wall':
            
            immigrate()
            if self.phase == 1:
                self.crawl_mode = 'Right'
                self.turn_direction = 'Left'
            drivetrain.straight(5, -1)
            self.state = f'turn {self.turn_direction}':
            emmigrate()
        
        elif self.state = f'turn {self.turn_direction}':
            immigrate()
            if self.turn_direction == 'Left':
                drivetrain.turn(90, -1, 5)
            elif self.turn_direction == 'Right':
                drivetrain.turn(-90, -1, 5)
            
            if self.phase == 2:
                self.state = 'go_straight'
            elif self.phase == 3:
                self.state = 'detect_move_straight'
            else:
                self.state = f'follow {self.crawl_mode} wall'
            emmigrate()

        elif self.state = f'follow {self.crawl_mode} wall':
            if self.init_state:
                if self.crawl_mode == 'Right': 
                    turn_ang = self.heading + 90
                elif self.crawl_mode == 'Left':
                    turn_ang = self.heading - 90
            sub_state = 'FOLLOW_WALL'
            immigrate()
            if self.distAhead <= dist_stop and self.phase == 3:
                switch_crawl_mode()
                self.state = f'turn {self.turn_direction}'
            elif self.heading >= turn_ang:
                self.phase += 1
                self.state = 'go_center_room'
            else:
                if self.sub_state == "FOLLOW_WALL":
                    self.right_speed = self.target_speed
                    self.left_speed = self.target_speed
                    print(f'following wall, wallCrawlMode is {self.wallCrawlMode}')
                    # If there is something within 10 cm ahead (and we are getting a legit reading that isn't 0):
                    # If we are getting too close to the left wall (number getting bigger)
                    elif (self.distLeft > self.proximity_center + self.proximity_range / 2) and (self.distRight > 10.0):
                        self.sub_state = "VEER_AWAY_FROM_WALL"
                    # If we are too far away from the left wall (number getting smaller)
                    elif (self.distLeft < self.proximity_center - self.proximity_range / 2) and (self.distRight > 10.0):
                        self.sub_state = "VEER_TOWARD_WALL"
                    elif (self.distRight > self.proximity_center + self.proximity_range / 2) and (self.distLeft > 10.0):
                        self.sub_state = "VEER_AWAY_FROM_WALL"
                    # If we are too far away from the left wall (number getting smaller)
                    elif (self.distRight < self.proximity_center - self.proximity_range / 2) and (self.distLeft > 10.0):
                        self.sub_state = "VEER_TOWARD_WALL"

                elif self.sub_state == "VEER_AWAY_FROM_WALL":
                    self.print_state("VEER_AWAY_FROM_WALL")
                    if time.ticks_diff(current_time, self.update_time["state_interval"]) > self.read_ms:
                        self.update_time["state_interval"] = current_time
                        if self.crawl_mode == 'Left':
                            if self.left_speed < 40:
                                self.left_speed = self.left_speed + (
                                        (abs(self.proximity_center - self.distLeft) / self.proximity_center) * .1)
                        if self.crawl_mode == 'Right':
                            if self.right_speed < 40:
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
                    self.print_state("VEER_TOWARD_WALL")
                    print('dist=', self.distAhead)
                    initialHeading = self.heading
                    if 0.0 < self.distAhead < 20.0:
                        self.sub_state = "ENCOUNTER_WALL"
                    if self.crawl_mode == 'Left':
                        if self.right_speed < 40:
                            self.right_speed = self.right_speed + (
                                    (abs(self.proximity_center - self.distLeft) / self.proximity_center) * .1)
                        if (abs(self.heading - initialHeading) >= 80):
                            if self.previousState == 'ZONEFINDER_STRAIGHTAWAY':
                                self.state = 'ENTERING_ZONE2'
                    if self.wallCrawlMode == 'right':
                        if self.left_speed < 40:
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
                        self.state = "FOLLOW_WALL"
                        print('dist=', self.distAhead)
                        drivetrain.set_speed(self.left_speed, self.right_speed)


                
                
        elif self.state = 'go_center_room':
            pass
        
        elif self.state = 'detect_move_straight':
            pass
        
        elif self.state = 'return':
            pass




