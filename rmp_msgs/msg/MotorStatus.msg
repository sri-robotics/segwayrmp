#constants
uint32 MOTOR_COUNT = 4
uint32 RIGHT_FRONT_IDX = 0
uint32 LEFT_FRONT_IDX = 1
uint32 RIGHT_REAR_IDX = 2
uint32 LEFT_REAR_IDX = 3
#fields
Header header
float32[4] current #A (0-peak)
float32[4] current_limit #A (0-peak)
float32[4] mcu_inst_power #Watts
float32[4] mcu_total_energy #Watt-hours
float32 max_current #A (0-peak)
float32 min_current_limit #A (0-peak)
