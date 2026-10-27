int8 task_id
int8[] drones_id # 本组的无人机编号
AreaSample explor_area # 探索区域，目前只支持矩形和圆形
AreaSample task_area # 任务区域，目前只支持矩形和圆形
mavros_msgs/WaypointList arrive_path # 前往目标区域的路径
float32 task_low_height # 任务区域下限，目前只支持矩形和圆形
float32 task_high_height # 任务区域上限，目前只支持矩形和圆形
float32 explor_low_height # 探索区域下限，目前只支持矩形和圆形
float32 explor_high_height # 探索区域上限，目前只支持矩形和圆形
float32 search_duration # 最大搜索时间，值为0表示无上限
bool exploration_attack # true为科目五，false为科目四
