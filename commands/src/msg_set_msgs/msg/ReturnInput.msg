mavros_msgs/WaypointList return_path   #返航路径点
int8 task_type # 任务类型：0起飞集结，1协同搜索，2协同，3弃炸，4返航,6中继 ，7 point
int8 move_mode # 移动模式：0single，1swarm，2move
float32 move_speed # 速度:m/s
float32 high_height # 最高高度:m
float32 low_height # 最低高度:m
