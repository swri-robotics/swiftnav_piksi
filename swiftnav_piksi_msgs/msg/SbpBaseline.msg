# SBP library message MSG_BASELINE_NED for Piski GPS

std_msgs/Header header

# GPS Time of Week (ms)
uint32 time_of_week

# Baseline North coordinate
int32 north

# Baseline East coordinate
int32 east

# Baseline Down coordinate
int32 down

# Horizontal position accuracy estimate (mm)
uint16 h_accuracy

# Vertical position accuracy estimate (mm)
uint16 v_accuracy

# Number of satellites used in solution
uint8 sats

# Flags

#RAIM repair flag (0 = no repair, 1 = solution came from RAIM repair)
bool raim_repair

#Fix Mode (0 = Invalid, 1 = SPP, 2 = DGNSS, 3 = Float RTK, 4 = Fixed RTK)
uint8 fix_type

