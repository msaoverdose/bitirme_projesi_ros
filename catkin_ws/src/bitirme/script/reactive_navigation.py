#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
class ReactiveNavigation:
    def __init__(self):  
        rospy.init_node('reactive_navigation', anonymous=True)

        self.cmd_vel = Twist()
        self.robot_stopped = False
        self.front_laser_msg = None
        self.left_laser_msg = None
        self.right_laser_msg = None


        self.laser_front = rospy.Subscriber("base_scan_0", LaserScan, self.front_laser_cb, queue_size=1)
        self.laser_left = rospy.Subscriber("base_scan_1", LaserScan, self.left_laser_cb, queue_size=1)
        self.laser_right = rospy.Subscriber("base_scan_2", LaserScan, self.right_laser_cb, queue_size=1)

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(5)
    
    def front_laser_cb(self, msg):
        self.front_laser_msg = msg

    def left_laser_cb(self, msg):
        self.left_laser_msg = msg

    def right_laser_cb(self, msg):
        self.right_laser_msg = msg

    def calculate_command(self):
        if self.front_laser_msg is None or self.left_laser_msg is None or self.right_laser_msg is None:
            return 
        
        if isinstance(self.front_laser_msg.ranges, list) or isinstance(self.front_laser_msg.ranges, tuple):
            self.front_distance = min(self.front_laser_msg.ranges)
        
        if isinstance(self.left_laser_msg.ranges, list) or isinstance(self.left_laser_msg.ranges, tuple):
            self.left_distance = min(self.left_laser_msg.ranges)

        if isinstance(self.right_laser_msg.ranges, list) or isinstance(self.right_laser_msg.ranges, tuple):
            self.right_distance = min(self.right_laser_msg.ranges)


        
        if self.front_distance < 1:
            
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 1.0

        else:

            if self.right_distance < 3:
                self.cmd_vel.linear.x = 1.2
                self.cmd_vel.angular.z = -0.44
            else:
                self.cmd_vel.linear.x = 1.0
                self.cmd_vel.angular.z = 0.0


            

        self.cmd_vel_pub.publish(self.cmd_vel)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = ReactiveNavigation()
        controller.run()
    except rospy.ROSInterruptException:
        pass
