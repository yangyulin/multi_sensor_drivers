source /opt/ros/melodic/setup.bash

rosbag record \
	/camera/accel/sample \
	/camera/gyro/sample \
	/camera/color/image_raw \
	/camera/depth/image_rect_raw \
	/camera_pg/image_raw \
	/imu_ms/data 
	
	
	

