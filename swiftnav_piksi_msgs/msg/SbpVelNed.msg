# SBP library message MSG_VEL_NED for Piski GPS

std_msgs/Header header

# GPS Time of Week (ms)
uint32 time_of_week

# Velocity North coordinate (m/s)
float64 vel_north

# Velocity East coordinate (m/s)
float64 vel_east

# Velocity Down coordinate (m/s)
float64 vel_down

# Horizontal velocity accuracy estimate (m/s)
float64 h_accuracy

# Vertical velocity accuracy estimate (m/s)
float64 v_accuracy

# Number of satellites used in solution
uint8 sats

# Flags

# Velocity Mode (0 = Invalid, 1 = Measured Doppler derived, 2 = Computed Doppler derived)
uint8 velocity_mode

string velocity_mode_description