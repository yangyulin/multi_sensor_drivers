source /opt/ros/melodic/setup.bash

rosbag record \
      /gx3_25/data \
      /gx3_35/imu/data \
      /imu/data \
      /t265/imu \
      /t265/fisheye1/image_raw \
      /t265/fisheye2/image_raw \
      /blackfly/image_raw \
	/t265/accel/imu_info \
	/t265/accel/sample \
	/t265/gyro/imu_info \
	/t265/gyro/sample


