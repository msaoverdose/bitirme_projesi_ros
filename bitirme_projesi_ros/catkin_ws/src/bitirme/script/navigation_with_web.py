#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse
import subprocess

def start_navigation(req):
    try:
        subprocess.Popen(["rosrun", "bitirme", "dual_reactive.py", "robot_0"])
        return TriggerResponse(success=True, message="Navigation started.")
    except Exception as e:
        return TriggerResponse(success=False, message=str(e))

if __name__ == '__main__':
    rospy.init_node('navigation_service')
    service = rospy.Service('/start_navigation', Trigger, start_navigation)
    rospy.loginfo("Navigation service ready.")
    rospy.spin()
