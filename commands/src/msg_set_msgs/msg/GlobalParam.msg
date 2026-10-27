int16 lowbattery_failsafeaction   #低电量 0 降落 2 抛弹返航 2 自主执行任务流
int8 battery_warnlevel
int8 battery_failsafelevel
int8 battery_emergencylevel
int16 linkloss_failsafeaction   			#信号丢失 0 自主执行任务流 1 降落 2 抛弹返航 如果选择 
bool auto_attack                            # 如果是自主执行任务流，则需要确认是否自主攻击
int16 linkloss_timeout    # 连接超时 : s
int16 missioncancel_action   # 0 悬停 1 降落 2 抛弹返航