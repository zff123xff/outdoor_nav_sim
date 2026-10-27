int8[] drones_id # 无人机群id
float32 height # 高度:m
float32 antijam_speed # 速度:m/s
float32 pitch # 期望前进时的pitch角度，影响速度，取值[5°,25°]
float32 extended_dis # 飞到目标区域后多飞的距离
AreaSample task_area # 区域数据, 圆形或者四边形区域
mavros_msgs/WaypointList arrive_path # 到该任务的路径