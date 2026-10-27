int8[] drones_id                                                          
int8 formation_shape                                                     
mavros_msgs/WaypointList arrive_path   #Waypoint的param1赋值yaw,param2赋值vel                                   
AreaSample task_area 
float32 high_height       
int8 formation_mode     # 0:formation_control  1:formation_mantain                                      
CoverParam param    #速度和半径的参数                                                    
bool coverparam_update #是否只读参数    
string source     # 标注来源，task_attack,task_search,task_shield                                                   
