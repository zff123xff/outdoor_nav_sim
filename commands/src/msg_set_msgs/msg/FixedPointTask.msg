# int32 task_id                           # 任务id
int8 group_id                           # 组id
int8[] drones_id                        # 无人机群id
mavros_msgs/WaypointList point_path     # 移动路径点
float32 move_speed                      # 移动速度
float32 cruise_speed                    # 巡航速度
int8 move_mode                          # 移动模式：0定点移动，1定点dj
bool safe_mode                          # 是否开启保护模式
bool auto_attack                        # 是否自主
float32 attack_height                   # 打击高度
int8 formation_shape                    # 任务区域间移动编队形状 1 圆形 2 一字形
int8 formation_shape_in_task            # 任务区域内编队形状 1 圆形 2 一字形
float32 formation_safe_dis              # 编队间距
