#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan   

class ReactiveNavigation():
    def __init__(self):
        # Initializations for robots speed, laser variables.*z
        self.cmd_vel = Twist()
        self.laser_msg = LaserScan()
        # Initialization for algorithm
        self.obstacle_distance = 100
        self.threshold = 0.7
        self.forward_speed = 1 
        self.turn_speed = 0.9
        # Topics
        self._cmd_topic = "cmd_vel"
        self._laser_topic = "base_scan"
        # Correctly subsicribe and publish data by topic(get contact with robot).
        self.rospy_sub_laser = rospy.Subscriber(self._laser_topic, LaserScan, self.laser_cb, queue_size=1)
        self.pub_CMD = rospy.Publisher(self._cmd_topic, Twist, queue_size=1)
        # Determine appropriate hertz(hz)
        self.rate = rospy.Rate(5)

    def laser_cb(self, msg):
        self.laser_msg = msg
    
    def calculate_command(self):
        if type(self.laser_msg.ranges) == tuple:
            # Get minimum distance for all directions*e
            # Get the total range from length of lidar's data
            total_ranges = len(self.laser_msg.ranges)
            # Divide the FOV to three and seperate indices to each direction in order to focus information correctly for each direciton.*k
            one_third = total_ranges // 3

            right_indices = range(0, one_third) # Implement first 80 indices as right
            front_indices = range(one_third, 2 * one_third) # Implement second 80 indices as front
            left_indices = range(2 * one_third, total_ranges) # Implement third(last) 80 indices as left

            # Save all distances to array variables, from indices that took before from all the lidar data.
            right_distances = [self.laser_msg.ranges[i] for i in right_indices]
            front_distances = [self.laser_msg.ranges[i] for i in front_indices]
            left_distances = [self.laser_msg.ranges[i] for i in left_indices]

            # Now get the minimum obstacle distance from each direction in order to detect wall
            right_obstacle_distance = min(right_distances)
            front_obstacle_distance = min(front_distances)
            left_obstacle_distance = min(left_distances)
            # Get the maximum obstacle distance in order to get which one is wider
            max_right_obstacle_distance = max(right_distances)
            max_front_obstacle_distance = max(front_distances)
            max_left_obstacle_distance = max(left_distances)
            #------------------------
            # Algorithm to search map *c=close *o=open f r l
            # -----------------------
            # 1. If there is no obstacle on front and left and right: turn radially to right - ooo 
            # 2. If there is an obstacle on front, no obstacle on right, no obstacle on left : stop and turn the direction that is "wider"-- coo
            # 3. If there is no obstacle on front and there is obstacle in right and no obstacle on left: go straight -- oco
            # 4. If there is no obstacle on front and no obstacle in right there is obstacle in left: go straight-- ooc
            # 5. If there is obstacle on front, no obstacle on right, obstacle on left, stop and turn right(maybe radially; slow linear high angular) -- coc 
            # 6. If there is no obstacle on front and obstacle on right and left : go forward -- occ
            # 7. If there is an obstacle on front and right no obstacle on left: stop and turn left(maybe radially; slow linear high angular) --cco
            # 8. If there is obstacle on right front and left: stop and turn back -- ccc
            #--------code----------- 
            #1. ooo
            if front_obstacle_distance >= self.threshold and right_obstacle_distance >= self.threshold and left_obstacle_distance >= self.threshold:
                self.cmd_vel.linear.x = self.forward_speed/1.5
                self.cmd_vel.angular.z = self.turn_speed*2
            #2. coo
            elif front_obstacle_distance < self.threshold and right_obstacle_distance >= self.threshold and left_obstacle_distance >= self.threshold:
                self.cmd_vel.linear.x = 0.0
                if right_obstacle_distance >= left_obstacle_distance:
                    self.cmd_vel.angular.z = -self.turn_speed
                else:
                    self.cmd_vel.angular.z = self.turn_speed
            #3. oco
            elif front_obstacle_distance >= self.threshold and right_obstacle_distance < self.threshold and left_obstacle_distance >= self.threshold:
                self.cmd_vel.linear.x = self.forward_speed/2
                self.cmd_vel.angular.z = self.turn_speed
            #4. ooc
            elif front_obstacle_distance >= self.threshold and right_obstacle_distance >= self.threshold and left_obstacle_distance < self.threshold:
                self.cmd_vel.linear.x = self.forward_speed
                self.cmd_vel.angular.z = 0.0
            #5. coc
            elif front_obstacle_distance < self.threshold and right_obstacle_distance >= self.threshold and left_obstacle_distance < self.threshold:
                self.cmd_vel.linear.x = self.forward_speed/2
                self.cmd_vel.angular.z = -self.turn_speed*2
            #6. occ
            elif front_obstacle_distance >= self.threshold and right_obstacle_distance < self.threshold and left_obstacle_distance < self.threshold:
                self.cmd_vel.linear.x = self.forward_speed
                self.cmd_vel.angular.z = 0.0
            #7. cco
            elif front_obstacle_distance < self.threshold and right_obstacle_distance < self.threshold and left_obstacle_distance >= self.threshold:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = self.turn_speed
            #8. ccc
            elif front_obstacle_distance < self.threshold and right_obstacle_distance < self.threshold and left_obstacle_distance < self.threshold:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -self.turn_speed*5
            # Publish the command
            self.pub_CMD.publish(self.cmd_vel)
            
    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("reactive_controller_py")

    controller = ReactiveNavigation()
    controller.run()