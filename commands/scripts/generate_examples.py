#!/usr/bin/env python3
"""Generate test inputs for the real outdoor_nav interfaces; never publish."""
import argparse
import json
import math
from pathlib import Path
import re

import yaml

ROOT = Path(__file__).resolve().parents[1]
RADIUS = 6378137.0


def enu_to_gps(east, north, up, latitude, longitude, altitude):
    """Inverse of outdoor_nav's PX4 spherical tangent-plane projection."""
    lat0, lon0 = map(math.radians, (latitude, longitude))
    rho = math.hypot(east, north)
    if rho == 0:
        return latitude, longitude, altitude+up
    angle = rho/RADIUS
    lat = math.asin(math.cos(angle)*math.sin(lat0) + north*math.sin(angle)*math.cos(lat0)/rho)
    lon = lon0 + math.atan2(east*math.sin(angle), rho*math.cos(lat0)*math.cos(angle)-north*math.sin(lat0)*math.sin(angle))
    return math.degrees(lat), math.degrees(lon), altitude+up


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='robot_1')
    parser.add_argument('--vehicle-id', type=int, default=1)
    parser.add_argument('--params-file', type=Path, default=ROOT.parent/'simulation/config/nav.yaml')
    parser.add_argument('--data-dir', type=Path, default=ROOT/'examples')
    args = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', args.namespace):
        parser.error('namespace must be one ROS name')
    if not 0 <= args.vehicle_id <= 127:
        parser.error('vehicle-id must fit the hardware int8 drones_id')
    original = yaml.safe_load(args.params_file.read_text())
    reference = {key: float(original['nav2_input_bridge']['ros__parameters'][key])
                 for key in ('reference_latitude','reference_longitude','reference_altitude')}
    if abs(reference['reference_latitude']) < 1e-9 and abs(reference['reference_longitude']) < 1e-9:
        parser.error('The real mission bridge rejects origin (0,0); configure a nonzero shared test origin')
    args.data_dir.mkdir(parents=True, exist_ok=True)
    def save(name, data):
        target = args.data_dir/name
        target.parent.mkdir(parents=True, exist_ok=True)
        comments = {
            'multi_goal.yaml': 'GPS 多航点：经纬度为度，海拔为米；默认目标为 map (1,0) → (2,0)。',
            'execute_task.yaml': 'ExecuteTask 请求：request_json 内为 JSON，agent_id 需匹配命名空间。',
            'gps_goal.yaml': 'NavSatFix 单目标：经纬度为度，海拔为米，默认对应 map (1,0)。',
            'wrong_agent.yaml': '异常测试：错误 agent_id，预期拒绝。',
            'unsupported_task.yaml': '异常测试：不支持的 capability_id，预期拒绝。',
            'local_multi_negative.yaml': '异常测试：本地坐标多航点，当前队列不支持，预期拒绝。',
        }
        comment = comments.get(Path(name).name, '')
        target.write_text(('# '+comment+'\n' if comment else '') + yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    coords = [enu_to_gps(e, 0., 0., *reference.values()) for e in (1.,2.)]
    goals = [{'x_or_lat':lat,'y_or_lon':lon,'z_or_alt':alt,'yaw':0.,'vel':.3} for lat,lon,alt in coords]
    lat,lon,alt = coords[0]
    save('gps_goal.yaml', {'header':{'frame_id':'gps'},'status':{'status':0,'service':1},
                          'latitude':lat,'longitude':lon,'altitude':alt})
    save('multi_goal.yaml', {'multi_goal_points':goals,'is_gps_aid':True,'is_gps_hgt':True,'ctl_mode':0})
    save('negative/local_multi_negative.yaml', {'multi_goal_points':[{'x_or_lat':1.,'y_or_lon':0.,'z_or_alt':0.,'yaw':0.,'vel':.3}],
         'is_gps_aid':False,'is_gps_hgt':False,'ctl_mode':0})
    request = {'mission_id':'sim_acceptance_001','agent_id':args.namespace,'capability_id':'goto_waypoints',
               'params':{'waypoint_frame':'gps','waypoints':[{'lat':a,'lon':b,'alt':c} for a,b,c in coords]}}
    save('execute_task.yaml',{'request_json':json.dumps(request)})
    request['agent_id']='wrong_robot'
    save('negative/wrong_agent.yaml',{'request_json':json.dumps(request)})
    save('negative/unsupported_task.yaml',{'request_json':json.dumps({'agent_id':args.namespace,'capability_id':'unsupported_test'})})
    mqtt_task = {'data': {'multiTask': {'groupId': 'sim_acceptance', 'editTaskId': '',
        'dronesId': [args.vehicle_id], 'fences': [], 'threatAreas': [], 'taskStream': [{
            'taskId': 'sim_mqtt_001', 'taskType': 7, 'dronesId': [args.vehicle_id],
            'lowHeight': reference['reference_altitude'], 'highHeight': reference['reference_altitude'],
            'arrivePath': [{'x_lat': a, 'y_lon': b, 'z_alt': c} for a,b,c in coords],
            'areaType': 0, 'areaData': [], 'trackingDis': [], 'targetTypeArray': []}]}}}
    (args.data_dir/'mqtt_task_stream.json').write_text(json.dumps(mqtt_task, indent=2), encoding='utf-8')
    info={'namespace':args.namespace,'reference':reference,'targets_enu':[[1.,0.,0.],[2.,0.,0.]],
          'targets_gps':coords,'note':'Absolute map targets; not relative to the current robot. No ROS messages sent.'}
    (args.data_dir/'test_info.json').write_text(json.dumps(info,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(info,ensure_ascii=False,indent=2))
    print('Shared navigation configuration (unchanged): '+str(args.params_file))
    print('Test inputs: '+str(args.data_dir)+'. No ROS messages sent.')


if __name__ == '__main__':
    main()
