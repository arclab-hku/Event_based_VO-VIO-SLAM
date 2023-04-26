# DV ROS Capture

This project contains a capture node that can is intended for live camera and aedat4 recording data publishing in ROS.

The node features:
* Publishing data from a live camera connected over USB.
* Playback data from an aedat4 file and publish it into ROS topics.
* Load and save calibration files using DV format.

# Running the node

The node will try to detect and open a camera connected through USB if no settings are provided.

Aedat4 playback will be performed if a path to an aedat4 file is set as the parameter "aedat4FilePath".

It is also possible to specify an exact camera to open using the "cameraName" parameter. Camera name consists
of camera model and a serial number concatenated by a '_' symbol, e.g. "DVXplorer_DXA000001".  In live mode, this
will open only the specified camera, in aedat4 playback it will filter data streams that were recorded from the
specified camera only that are available in the file.

Please see `config/settings.yaml` for a full list of available settings.

# Calibration files

Calibration files are stored under `~/.dv_camera/{cameraName}` directory. The node will try to find
"active calibration" file named `~/.dv_camera/{cameraName}/active_calibration.json` and will publish the calibration
into `~camera_info` topic. Active calibration is saved when a `set_camera_info` service is called.

This calibration can be overridden manually using the `cameraCalibrationFilePath` parameter, this parameter
will not replace the "active calibration" file, it only loads calibration from the given path instead.

If no calibration is provided or detected, the capture node will issue a warning message and publish "ideal"
calibration which assumes no distortion, central point as the center of the image pixel space, and focal length
equal to the width of the image.

# Multiple camera synchronization

Multi-camera setups connected with synchronization cables require synchronization signals to synchronize
the data streams. This can be achieved by setting up the launch files on which cameras need to be synchronized.
This guarantees the synchronization to happen at correct time, since all cameras need to be opened prior
to sending synchronization signal. Please refer `launch/synchronization.launch` file for details on how to
set up the multi-camera synchronization.
