# SBP library message MSG_DOPS

std_msgs/Header header

# GPS Time of Week (ms)
uint32 time_of_week

# Geometric dilution of precision
float64 gdop

# Position dilution of precision
float64 pdop

# Time dilution of precision
float64 tdop

# Horizontal dilution of precision
float64 hdop

# Vertical dilution of precision
float64 vdop

# Flags

#RAIM repair flag (0 = no repair, 1 = solution came from RAIM repair)
bool raim_repair

#Fix Mode (0 = Invalid, 1 = SPP, 2 = DGNSS, 3 = Float RTK, 4 = Fixed RTK)
uint8 fix_type

string fix_description
