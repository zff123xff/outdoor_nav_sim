#!/usr/bin/env python3
"""Send the hardware taskStream MQTT payload, with MQTT 5 request properties."""
import argparse
import json
from pathlib import Path
import threading
import time
from urllib.parse import urlparse
import uuid

import paho.mqtt.client as mqtt
from paho.mqtt.properties import Properties
from paho.mqtt.packettypes import PacketTypes
import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/mqtt/client.yaml')
    parser.add_argument('--file', type=Path, default=ROOT/'examples/mqtt_task_stream.json')
    parser.add_argument('--legacy', action='store_true', help='Use /mqtt/web/task_stream/{id} without application ACK')
    parser.add_argument('--timeout', type=float, default=10.)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    url = urlparse(config['address'])
    if url.scheme not in ('tcp', 'ssl', 'mqtt', 'mqtts') or not url.hostname:
        parser.error('address must be a tcp:// or ssl:// broker URL')
    tls = url.scheme in ('ssl', 'mqtts')
    payload = args.file.read_text(encoding='utf-8')
    json.loads(payload)
    identifier = int(config.get('vehicle_id', 1))
    request_topic = ('/mqtt/web/task_stream/' if args.legacy else '/mqtt/request/task_deploy/nx/task_stream/')+str(identifier)
    response_topic = '/mqtt/response/task_deploy/nx/task_stream/'+str(identifier)
    correlation = uuid.uuid4().hex.encode()
    connected, subscribed, answered = threading.Event(), threading.Event(), threading.Event()
    received = []
    client = mqtt.Client(client_id='sim_task_'+uuid.uuid4().hex[:10], protocol=mqtt.MQTTv5)
    if config.get('mqtt_username'):
        client.username_pw_set(config['mqtt_username'], config.get('mqtt_passwd', ''))
    if tls: client.tls_set()
    def on_connect(client, userdata, flags, code, properties=None):
        if getattr(code, 'value', code) == 0:
            connected.set()
            if not args.legacy: client.subscribe(response_topic, qos=1)
    def on_message(client, userdata, message):
        if getattr(message.properties, 'CorrelationData', None) == correlation:
            received.append(json.loads(message.payload))
            answered.set()
    client.on_connect = on_connect
    client.on_subscribe = lambda *unused: subscribed.set()
    client.on_message = on_message
    try:
        client.connect(url.hostname, url.port or (8883 if tls else 1883), keepalive=30)
        client.loop_start()
        if not connected.wait(args.timeout): raise RuntimeError('MQTT connection failed or timed out')
        properties = None
        if not args.legacy:
            if not subscribed.wait(args.timeout): raise RuntimeError('Response subscription timed out')
            properties = Properties(PacketTypes.PUBLISH)
            properties.ResponseTopic = response_topic
            properties.CorrelationData = correlation
        info = client.publish(request_topic, payload, qos=1, retain=False, properties=properties)
        deadline = time.monotonic()+args.timeout
        while not info.is_published() and time.monotonic()<deadline:
            time.sleep(.01)
        if not info.is_published(): raise RuntimeError('MQTT publish timed out')
        print('Published hardware task JSON to '+request_topic)
        if not args.legacy:
            if not answered.wait(args.timeout): raise RuntimeError('Hardware MQTT response timed out')
            print('Protocol ACK: '+json.dumps(received[0]))
            if received[0].get('data', {}).get('code') != 0: raise RuntimeError('Hardware MQTT gateway rejected request')
        print('Check ROS task/goal messages and actual arrival separately; this is not a mission completion ACK.')
    except OSError as error:
        raise SystemExit(
            f'MQTT 连接失败：{url.hostname}:{url.port or (8883 if tls else 1883)} — {error}\n'
            '本地仿真请先在另一个终端运行并保持：bash commands/start.sh mqtt\n'
            '本次未确认任务发布成功。') from None
    finally:
        client.disconnect()
        client.loop_stop()


if __name__ == '__main__':
    main()
