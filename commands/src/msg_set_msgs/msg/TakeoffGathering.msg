int32[] drones_id    # 无人机群id
mavros_msgs/WaypointList gather_path # 集结路径
AreaSample task_area # 区域数据
float32 high_height  # 最高高度:m
float32 low_height   # 最低高度:m
float32 move_speed   # 最低高度:m
int8 takeoff_mode    # 起飞模式：0正常起飞，1分布式起飞
int8 gather_mode     # 集结模式: 0圆形分布，2 半圆形分布
int8 move_mode       # 移动模式：0single，1swarm，2move