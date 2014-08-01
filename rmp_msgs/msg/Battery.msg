#constants
uint32 BATTERY_COUNT = 5
uint32 BATT_FRONT_1_IDX = 0
uint32 BATT_FRONT_2_IDX = 1
uint32 BATT_REAR_1_IDX = 2
uint32 BATT_REAR_2_IDX = 3
uint32 BATT_AUX_IDX = 4
#fields
Header header
float32[5] charge_state #perc.
float32[5] temperature #deg. celsius
float32 min_propulsion_charge_state #perc.
float32 aux_battery_voltage #VDC
float32 aux_battery_current #A (0-peak)
uint32 abb_system_status 
uint32 aux_battery_status