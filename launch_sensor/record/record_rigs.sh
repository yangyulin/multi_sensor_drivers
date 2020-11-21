source /opt/ros/melodic/setup.bash

rosbag record \
	/camera_pg/image_raw \
	/gx5/imu/data \
	/t265/accel/sample \
	/t265/gyro/sample \
	/t265/fisheye1/image_raw \
	/t265/fisheye2/image_raw \
        /d435i/color/image_raw \
        /d435i/infra1/image_rect_raw \
        /d435i/infra2/image_rect_raw \
        /d435i/accel/sample \
        /d435i/gyro/sample 
	
	
	

