# A DVS trigger

# Numeric value representing the type of trigger signal received
int8 type
# Time of occurence
time timestamp

# Below are constant numeric values describing eah type of trigger
# A timestamp reset occurred.
int8 TRIGGER_TYPE_TIMESTAMP_RESET=0

# A rising edge was detected (External Input module on device).
int8 TRIGGER_TYPE_EXTERNAL_SIGNAL_RISING_EDGE=1

# A falling edge was detected (External Input module on device).
int8 TRIGGER_TYPE_EXTERNAL_SIGNAL_FALLING_EDGE=2

# A pulse was detected (External Input module on device).
int8 TRIGGER_TYPE_EXTERNAL_SIGNAL_PULSE=3

# A rising edge was generated (External Generator module on device).
int8 TRIGGER_TYPE_EXTERNAL_GENERATOR_RISING_EDGE=4

# A falling edge was generated (External Generator module on device).
int8 TRIGGER_TYPE_EXTERNAL_GENERATOR_FALLING_EDGE=5

# An APS frame capture has started (Frame Event will follow).
int8 TRIGGER_TYPE_APS_FRAME_START=6

# An APS frame capture has completed (Frame Event is contemporary).
int8 TRIGGER_TYPE_APS_FRAME_END=7

# An APS frame exposure has started (Frame Event will follow).
int8 TRIGGER_APS_EXPOSURE_START=8

# An APS frame exposure has completed (Frame Event will follow).
int8 TRIGGER_APS_EXPOSURE_END=9
