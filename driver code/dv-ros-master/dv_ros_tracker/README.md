# DV ROS Tracker

DV ROS Tracker provides a tracker node that can be used to run Lucas-Kanade tracking algorithm on either events or
images. The tracker supports three main modes of execution:

* **Event-only**: the tracking is performed on event accumulated edge maps at a configurable framerate. The edge maps
  are generated at fixed frequency accumulating a configured amount of events. This results in an overlapping slices of
  events, which is more favourable for tracking.
* **Frame-only**: this mode performs regular Lucas-Kanade tracking on an image input.
* **Combined**: the tracking is performed on regular frames, but it also performs intermediate accumulated frame
  tracking to improve robustness. The result of intermediate tracking on event accumulated frames is used as priors for
  most probable track location for the next iteration of frame tracking.

The modes of tracker operation are configured using parameters, the full list of parameters is available
in `config/settings.yaml`. Sample configurations for each mode can be in the `config` directory, since each mode
operates on different modality of input data, the settings have to be fine-tuned for each case.

The tracker requires a `camera_info` topic and at least on type of supported input data (event or frames). The node
outputs `TimedKeyPointArray` messages which contain the track locations. Preview images are available in `preview/image`
topic to see the visualization of the tracker performance.

### Motion Aware Tracker

To enable the motion awareness the parameter `useMotionCompensation` is set to *true*. A `PoseStamped` topic
indicating the position of the camera must be provided to the node. All the modes listed above support the motion
compensation feature.
