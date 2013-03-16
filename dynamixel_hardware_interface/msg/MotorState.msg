# all values are in encoder units unless otherwise specified

float64 timestamp       # motor state is at this time

int32 id                # motor id
int32 target_position   # commanded position
int32 target_velocity   # commanded velocity
int32 position          # current position
int32 velocity          # current velocity
int32 torque_limit      # current torque limit
int32 load              # current load - ratio of applied torque over maximum torque
bool  moving            # whether the motor is currently in motion
bool  alive             # whether the motor is currently giving valid feedback info
int32 voltage           # current voltage (V * 10)
int32 temperature       # current temperature (degrees Celsius)

