int8 group_id # 组id
int8 drones_size # 组内无人机的数量  
uint16 mtask_hash # edited_task*4 + task_stream.size*3 + fence.size()*2 + threat_area.size
uint16 jtask_hash # task_id*10 + arrive_path.size