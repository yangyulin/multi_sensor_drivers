source /opt/ros/melodic/setup.bash

rosbag record \
	/camera/accel/sample \
	/camera/gyro/sample \
	/camera/color/image_raw \
	/camera/depth/image_rect_raw \
	/camera_pg/image_raw \
	/imu/data \
	/imu/data_raw \
	/garmin/fix \
	/garmin/nmea_sentence \
	/garmin/time_reference \
	/imu_um7/data \
	/imu_um7/data_raw \
	/jackal_velocity_controller/odom \
	/odometry/filtered \
	/velodyne_packets \
	/velodyne_points
	
	
	

