#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveNavigation:
    def __init__(self):  
        rospy.init_node('reactive_navigation', anonymous=True) 
        self.cmd_vel = Twist()
        self.robot_stopped = False
        self.obstacle_distance = 100.0
        self.laser_msg = None  
        self.laser_front = rospy.Subscriber("base_scan0", LaserScan, self.laser_cb, queue_size=1)
       

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(5)

    def laser_cb(self, msg):
        self.laser_msg = msg
    
    def calculate_command(self):
        if self.laser_msg is None:
            return 
        
        if isinstance(self.laser_msg.ranges, list) or isinstance(self.laser_msg.ranges, tuple):

            self.obstacle_distance = min(self.laser_msg.ranges)
        
        if self.obstacle_distance > 1.0:
            self.cmd_vel.linear.x = 1.0
            self.cmd_vel.angular.z = 0.0

            
        else:
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 1.0

        self.cmd_vel_pub.publish(self.cmd_vel)
        self.obstacle_distance = 100.0  

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