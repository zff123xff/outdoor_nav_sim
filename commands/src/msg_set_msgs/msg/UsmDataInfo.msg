uint8 usm_state
uint8 USM_STATE_NONE = 0
uint8 USM_STATE_MANUAL = 1
uint8 USM_STATE_TAKEOFF = 2
uint8 USM_STATE_HOVER = 3
uint8 USM_STATE_LAND = 4
uint8 USM_STATE_USELESS = 5
uint8 USM_STATE_UNREADY = 6
uint8 USM_STATE_ALGORITHM = 9

uint8      ctrl_state
uint8      algo_state

float64[3] ref_pos        # x,y,z (meters)
float64[3] ref_gps        # Latitude(degrees), Longitude(degrees), Altitude(meters)