source /opt/ros/melodic/setup.bash

rosbag record \
	/t265/accel/sample \
	/t265/gyro/sample \
	/t265/fisheye1/image_raw \
	/t265/fisheye2/image_raw \
	/d435/color/image_raw \
	/d435/infra1/image_rect_raw \
	/d435/infra2/image_rect_raw \
	/d435/depth/image_rect_raw \
	/blackfly/image_raw \
	/gx3_25/data \
	/gx3_35/imu/data \
	/vio/odom \
	/vicon/vio

	
	

