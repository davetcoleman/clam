Header header
string name             # joint name
float64 target_position # commanded position (in radians)
float64 target_velocity # commanded position (in radians per second)
float64 position        # current joint position (in radians)
float64 velocity        # current joint speed (in radians per second)
float64 load            # current load
bool    moving          # is joint currently in motion
bool    alive           # whether the motor is currently giving valid feedback info
