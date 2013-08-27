corobot_pid
========

ROS package to interface the control_pid package to the corobot.

This publishes /control_error and subscribes to /control_correction.


/control_error will be computed from the camera, pulled from /PTZ/image_raw, and will be specified in radians from the bottom of the camera frame

/control_correction will be received as radians, translated to motor command messages, which are then published onto /PhidgetMotor topic

This also subscribes to /target_velocity, of type integer from 0 to 100.  This will be combined with the corrected term and published to /PhidgetMotor
