int8[] drones_id # 无人机群id
AreaSample task_area # 区域数据
mavros_msgs/WaypointList arrive_path # 到该任务的路径
float32 high_height # 最高高度:m
float32 low_height # 最低高度:m
float32 search_duration # 搜索时间:s
float32[] tracking_dis # 跟踪距离 [水平距离，垂直距离]
float32 move_speed # 任务区域速度:m/s
float32 cruise_speed #速度:m/s  前往区域的速度
float32 attack_speed # 速度:m/s
bool auto_attack # 是否自主
int8 search_count # 搜索次数
int8[] target_type # 目标类型
int8 move_mode # 移动模式：0single，1swarm，2move
int8 search_mode # 搜索模式：0平面，1立体，2环绕
int8 tracking_mode # 跟踪模式：0避障，1无避障
int8 attack_mode # 模式：0抛投，1穿窗自杀打击 ，2 仅跟踪， 3 直线自杀打击
int8 attack_flag # 区分是否进入attack, 0 不进，1可进
int8 formation_shape # 任务区域间移动编队形状 0 不使用队形 1 圆形 2 一字形
float32 formation_safe_dis # 编队间距
int8 task_mode # 任务模式（新增）： 0仅搜索，1仅跟踪，2打击
bool remissionflag  #是否重分配
