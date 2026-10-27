#编队运动的起始点和目标点
int8[] drones_id 
mavros_msgs/Waypoint start_point
mavros_msgs/Waypoint end_point
int8   formation_type #编队类型 1 圆形 2 一字形
float32 formation_safe_dis              # 编队间距
int8[] fomration_parameter #编队类型参数
