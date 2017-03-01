# SBP library message MSG_GPS_TIME for Piski GPS

std_msgs/Header header

# GPS week number
uint16 weeks

# GPS Time of Week (TOW) rounded to nearest millisecond(ms)
uint32 time_of_week

# Nanosecond residual of millisecond-rounded TOW (range -500000 to 500000)
int32 nano

# Flags
# Time source (0 = None (invalid), 1 = GNSS)
bool time_source

