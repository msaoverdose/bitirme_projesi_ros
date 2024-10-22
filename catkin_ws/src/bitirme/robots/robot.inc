define laser ranger(
    sensor(
        range [0.0 5.6]
        fov 240.0
        samples 240
    )

    color "red"
    size [0.15 0.15 0.15]

    block(
		points 8
		point[0] [0.3 0.0]   
		point[1] [0.6 -0.3]   
		point[2] [0.3 -0.6]   
		point[3] [0.0 -0.3]   
		point[4] [-0.3 0.0]   
		point[5] [0.0 0.3]   
		point[6] [0.3 0.6]   
		point[7] [0.6 0.3]   
		z [0 0.2]  
	)


)


define robot_0 position(
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