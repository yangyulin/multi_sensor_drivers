# rs_pointgrey_microstrain
Drivers for real-senser D435i, T256, pointgrey camera and microstrain IMU


## Instruction

1) connect to wifi: 360WiFi

2) Go to 192.168.0.1 to check whether cpr-j100-0403 is connected. copy the IP of cpr0j100-0403

3) open two terminals. 

In Terminal 1: ssh administrator@IP, cd to catkin_ws_sensor, 

`sudo chmod a+rw /dev/ttyACM1`

`source devel/setup.bash`

`roslaunch launch_sensor ms_pg_d435i.launch`

In Terminal 2: ssh administrator@IP, cd to catkin_ws_sensor/src/rs_pointgrey_microstrain/record

`./record.sh`

This will automaticaly start the recording. 

