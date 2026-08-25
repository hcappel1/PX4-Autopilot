uint64 timestamp

float32 input_yaw_sp
float32 qpd_yaw_sp

# Get the roll, pitch, and yaw from the vehicle attitude quaternion
float32 roll
float32 pitch
float32 yaw

# Get the roll, pitch, and yaw setpoints from the vehicle attitude setpoint quaternion
float32 roll_sp
float32 pitch_sp
float32 yaw_sp

# Get the roll, pitch, and yaw rates from the vehicle angular velocity
float32 roll_rate
float32 pitch_rate
float32 yaw_rate

float32 yaw_rate_sp

# Get the vehicle moments
float32 roll_torque
float32 pitch_torque
float32 yaw_torque

# Yaw Control Initalization params
bool yaw_constrain
bool yaw_initialized
