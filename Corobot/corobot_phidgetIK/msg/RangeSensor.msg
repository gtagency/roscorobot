#Deprecated. Prefer to use the one in corobot_msgs package, which is exactly the same

# Message used to transmit range data (infrared, ultrasounds...)

Header header     # timestamp in the header is the time the distance was read

# type enums
uint8 ULTRASOUND=0
uint8 INFRARED=1

# The type of sensor represented by these data. The value can be ULTRASOUND or INFRARED.
int8 type 

#The number of sensors
int8 numberSensors

#The data for each sensor. The size of this array is the numberSensors variable.
float32[] range
