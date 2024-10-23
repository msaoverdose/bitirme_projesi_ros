# bitirme_projesi_ros

This project aim to CRoP on Localization and Mapping with using Integrated Server Services

Erzurum Technical University EEE Graduation Project 



If you want to work with 2 robot use this directory:

Config file:
available_robots: ["robot_0", "robot_1"]

house.world:
robot_1 (pose [ 14.773 17.980 0.000 -144.000 ] name "robot")

Robot.inc: 
define robot_1 position(
    odom_error [0.03 0.03 0.00 0.05]
    localization "odom"
    size [0.8 0.7 0.2]
    gui_nose 1
    drive "diff"
    color "white"
    block(
        points 6
        point[0] [0.4 0.0]
        point[1] [0.8 0.35]
        point[2] [0.4 0.7]
        point[3] [-0.4 0.7]
        point[4] [-0.8 0.35]
        point[5] [-0.4 0.0]
        z [0 0.2]
    )

    laser(pose [0.37 0.0 -0.13 0.0])

)

Start the ROS Codes with this steps:

rosrun X dual_reactive.py robot_X

roslaunch X dual_gmapping.launch