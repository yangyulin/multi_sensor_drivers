source /opt/ros/melodic/setup.bash

rosbag record \
	/t265_03/imu \
	/t265_03/fisheye1/image_raw \
	/t265_03/fisheye2/image_raw \
	/t265_03/gyro/sample \
	/t265_03/gyro/imu_info \
	/t265_03/accel/imu_info \
	/t265_04/imu \
	/t265_04/fisheye1/image_raw \
	/t265_04/fisheye2/image_raw \
	/t265_04/gyro/sample \
	/t265_04/gyro/imu_info \
	/t265_04/accel/imu_info
