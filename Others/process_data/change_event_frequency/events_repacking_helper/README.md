# events_repacking_helper
A modified version based on [ESVO/events_repacking_helper/](https://github.com/HKUST-Aerial-Robotics/ESVO/tree/master/events_repacking_helper) to change the frequency of event streams.

This package provides an example of preparing a bag file for ESVO. A bag file recorded by a stereo event camera (e.g., a pair of DAVIS sensors) typically consists of the following topics:

	-- /davis/left/events
	-- /davis/right/events

	-- /davis/left/imu
	-- /davis/right/imu

	-- /davis/left/camera_info
	-- /davis/right/camera_info

	-- /davis/left/image_raw (optional for visualization)
	-- /davis/right/image_raw (optional for visualization)

## Preparation

### 1. Extract event messages from the original bag, and change the streaming rate to the a fixed frequency.
	
Set the input and output paths as arguments in the file `repacking.launch`, and then run   

   `$ roslaunch events_repacking_helper repacking.launch`

This command will return a bag file (e.g., output.bag.events) which only contains the re-packed stereo event messages.

