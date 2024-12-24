#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class ReactiveNavigation:
    def __init__(self, robot_0):
        # hız ve lazer için ayarlar
        self.cmd_vel = Twist()
        self.laser_msg = LaserScan()

        # Hareket hız tanımlamaları 1'den yukarı yapma ne kadar yavaşsa o kadar stabil çalışıyor!!
        self.threshold = 0.7
        self.forward_speed = 1.0
        self.turn_speed = 0.9

        # ROS
        #fazla robot varsa topicler değişiyor dikkat
        self._cmd_topic = f"/{robot_0}/cmd_vel"
        self._laser_topic = f"/{robot_0}/base_scan"

        # Yayıncı ve abone tanımlamaları
        rospy.Subscriber(self._laser_topic, LaserScan, self.laser_callback, queue_size=1)
        self.pub_cmd = rospy.Publisher(self._cmd_topic, Twist, queue_size=1)
        self.rate = rospy.Rate(5)

    def laser_callback(self, msg):
        """Lidar verisini güncellemek için callback fonksiyonu."""
        self.laser_msg = msg

    def get_region_distances(self):
        """Lidar verilerini sağ, ön ve sol bölgeler için böler."""
        ranges = self.laser_msg.ranges

        # TEk sensördeki veri 3'e bölünüp 3 ayrı kısım olarak inceleniyor
        one_third = len(ranges) // 3
        right = ranges[:one_third]
        front = ranges[one_third:2 * one_third]
        left = ranges[2 * one_third:]

        return min(right), min(front), min(left)

    def calculate_command(self):
        """Engel algılamaya göre komut hesaplar ve yayar."""
        try:
            right_dist, front_dist, left_dist = self.get_region_distances()
        #RASPERR    
        except ValueError:
            rospy.logwarn("Lidar girdisi alınamadı!")
            return

        # Engel kodları
        if front_dist >= self.threshold and right_dist >= self.threshold and left_dist >= self.threshold:
            self.set_velocity(self.forward_speed / 1.5, self.turn_speed * 2)
        elif front_dist < self.threshold and right_dist >= self.threshold and left_dist >= self.threshold:
            turn_direction = -self.turn_speed if right_dist >= left_dist else self.turn_speed
            self.set_velocity(0, turn_direction)
        elif front_dist >= self.threshold and right_dist < self.threshold and left_dist >= self.threshold:
            self.set_velocity(self.forward_speed / 2, self.turn_speed)
        elif front_dist >= self.threshold and right_dist >= self.threshold and left_dist < self.threshold:
            self.set_velocity(self.forward_speed, 0)
        elif front_dist < self.threshold and right_dist >= self.threshold and left_dist < self.threshold:
            self.set_velocity(self.forward_speed / 2, -self.turn_speed * 2)
        elif front_dist >= self.threshold and right_dist < self.threshold and left_dist < self.threshold:
            self.set_velocity(self.forward_speed, 0)
        elif front_dist < self.threshold and right_dist < self.threshold and left_dist >= self.threshold:
            self.set_velocity(0, self.turn_speed)
        elif front_dist < self.threshold and right_dist < self.threshold and left_dist < self.threshold:
            self.set_velocity(0, -self.turn_speed * 5)

    def set_velocity(self, linear, angular):
        self.cmd_vel.linear.x = linear
        self.cmd_vel.angular.z = angular
        self.pub_cmd.publish(self.cmd_vel)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("reactive_controller_py")
    controller = ReactiveNavigation()
    controller.run()
