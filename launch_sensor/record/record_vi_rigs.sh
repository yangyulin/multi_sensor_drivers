source /opt/ros/melodic/setup.bash

rosbag record \
	/T265/accel/sample \
	/T265/gyro/sample \
	/T265/fisheye1/image_raw \
	/T265/fisheye2/image_raw \
	/D435/color/image_raw \
	/D435/infra1/image_rect_raw \
	/D435/infra2/image_rect_raw \
	/D435/depth/image_rect_raw \
	/blackfly/image_raw \
	/gx3_25/data \
	/gx3_35/data

	
	

