# Vehicle Command uORB message
# Used for sending commands to the vehicle.
# Follows the MAVLink COMMAND_LONG message definition

uint32 MESSAGE_VERSION = 0

uint64 timestamp
uint8 ORB_QUEUE_LENGTH = 4

uint16 command
float32 param1
float32 param2
float32 param3
float32 param4
float32 param5
float32 param6
float32 param7
uint8 target_system
uint8 target_component
uint8 source_system
uint8 source_component
bool from_external 