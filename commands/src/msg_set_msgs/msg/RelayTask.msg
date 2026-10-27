int8[] drones_id    # 无人机群id
AreaSample task_area # 区域数据 ，指控中继是点，任务中继是 圆形或者四边形区域
float32 height  # 高度:m
float32 move_speed  #sudu
bool relay_mode # 0 指控， 1 任务
int8 move_mode   # 移动模式：0 single，1 swarm，2 move