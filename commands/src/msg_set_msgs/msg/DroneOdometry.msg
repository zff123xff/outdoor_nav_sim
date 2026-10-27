uint16 state_mask
int8 drone_id
int16 robotmajorstate
int16 task_id # 当前飞机的任务id
builtin_interfaces/Time stamp
float32 x_lat
float32 y_lon
float32 z
# float32 target_x
# float32 target_y
# float32 target_z
float32 vx
float32 vy
float32 vz
uint16 KILL=1
uint16 LOW_HEIGHT=2
uint16 LOSE_SIGNAL=4
uint16 UNABLE_SEARCH=8
uint16 UNABLE_TRACK=16
uint16 UNABLE_DROP=32
uint16 ENROUTE_SUCCESS=256  #暂时没用
uint16 TASK_SUCCESS=128
uint16 RELAY_NODE=512
uint16 RELAY_LOST=1024
uint16 TRAC_MA=2048 #ma liu
uint16 ATTACK_GOAL=4096 # 表示打击目标
uint16 ATTACK_ANY=8192  
