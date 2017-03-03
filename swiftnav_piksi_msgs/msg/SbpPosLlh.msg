# SBP library message MSG_POS_LLH for Piski GPS

std_msgs/Header header

# GPS Time of Week (ms)
uint32 time_of_week

# Latitude [degrees]. Positive is north of equator; negative is south.
float64 latitude

# Longitude [degrees]. Positive is east of prime meridian; negative is west.
float64 longitude

# Altitude [m]. Positive is above the WGS 84 ellipsoid
# (quiet NaN if no altitude is available).
float64 altitude

# Horizontal position accuracy estimate (m)
float64 h_accuracy

# Vertical position accuracy estimate (m)
float64 v_accuracy

# Number of satellites used in solution
uint8 sats

# Flags

#RAIM repair flag (0 = no repair, 1 = solution came from RAIM repair)
bool raim_repair

#Fix Mode (0 = Invalid, 1 = SPP, 2 = DGNSS, 3 = Float RTK, 4 = Fixed RTK)
uint8 fix_type

string fix_description