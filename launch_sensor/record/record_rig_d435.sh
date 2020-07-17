source /opt/ros/melodic/setup.bash

rosbag record \
	/camera/accel/sample \
	/camera/gyro/sample \
	/camera/color/image_raw \
	/camera/aligned_depth_to_color/image_raw \
	/camera/extrinsics/depth_to_color \
	/camera_pg/image_mono \
	/imu/data
	#/camera/depth/color/points \

