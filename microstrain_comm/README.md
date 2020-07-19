# microstrain_comm

* IMU driver for the Microstrain 3DM-GX3®-25.
* Converted to run on the ROS framework from LCM
* Original package got from https://github.com/ipab-slmc/pronto-distro
* Built with ROS indigo on Ubuntu 14.04 LTS
* Make sure that you are in the `dialout` group - `sudo adduser $USER dialout`
* Add yourself to the dialout group and restart your computer

### Dependencies:
* GLIB2
* GTHREAD2
* libbot (already included)

### Parameters:
* `"verbose"`: default false, (if true packet bits are printed)
* `"quiet"`: default false, (if true no messages are printed)
* `"com_port"`: default "", (custom serial port, default scans /dev/ttyACM*)
* `"rate"`: default "low", ("high"=1000hz, "medium"=500hz, "low"=100hz)
* `"window"`: default 15, (digital filter window size)
