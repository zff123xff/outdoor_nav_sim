string task_id  #任务编号
int8[] drones_id # 无人机群id
int8 task_type # 任务类型：0起飞集结，1协同搜索，2协同，3弃炸，4返航, 6中继 ，7 point, 8,kgr ,9立体 10环绕 11，巡航, 12,起飞 
AreaSample task_area # 区域数据    
mavros_msgs/WaypointList arrive_path # 到该任务的路径
float32 high_height # 最高高度:m
float32 low_height # 最低高度:m
float32 cruise_height # 巡航高度 m
float32 search_duration # 单个任务的最大时间: min
float32 move_speed # 速度:m/s  区域内速度
float32 cruise_speed #速度:m/s  前往区域的速度
float32 attack_speed # 速度:m/s
float32[] tracking_dis # 跟踪距离 [水平距离，垂直距离]
int8 search_count # 搜索次数   默认给999
bool auto_attack # 是否自主
bool safe_mode # 是否开启保护模式
int8[] target_type # 目标类型
int8 takeoff_mode # 起飞模式：0正常起飞，1分布式起飞  变成：0 日间，1 夜晚，2 干扰；
int8 gather_mode # 集结模式   
int8 move_mode # 移动模式：0single，1swarm，2move。 7定点模式：0抛投，1自杀打击
int8 search_mode # 搜索模式：0平面，1立体，2环绕,3 高空 4 point 5 协打 
int8 tracking_mode # 跟踪模式：0避障，1无避障
int8 attack_mode # 模式：0抛投，1穿窗自杀打击 ，2 仅跟踪， 3 直线自杀打击
int8 formation_shape # 任务区域间移动编队形状 1 圆形 2 一字形
int8 formation_shape_in_task  # 任务区域内编队形状 1 圆形 2 一字形
float32 formation_safe_dis # 编队间距
int8 task_mode # 任务模式（新增）： 0仅搜索，1仅跟踪，2打击    返航： 0 返航到起飞点，1 返航到指定区域
int8 hazard_level # 任务区域的危险等级 0：低   1：中   2：高
