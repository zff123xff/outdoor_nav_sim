#!/usr/bin/env python3
"""Run the local test broker and MQTT task stack in one terminal."""
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.parse import urlparse

import yaml

ROOT = Path(__file__).resolve().parents[1]


def main():
    arguments = sys.argv[1:]
    config = ROOT/'config/mqtt/client.yaml'
    for argument in arguments:
        if argument.startswith('mqtt_config:='):
            config = Path(argument.split(':=', 1)[1])
    address = urlparse(yaml.safe_load(config.read_text())['address'])
    host, port = address.hostname, address.port or 1883
    local = host in ('127.0.0.1', 'localhost') and port == 1883
    broker = None
    children = []

    def reachable():
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return True
        except OSError:
            return False

    try:
        if local:
            if reachable():
                print('Reusing listener on local port 1883; MQTT gateway will verify connection.', flush=True)
            else:
                broker = subprocess.Popen(['bash', str(ROOT/'start.sh'), 'broker'])
                children.append(broker)
                deadline = time.monotonic()+10
                while not reachable():
                    if broker.poll() is not None:
                        raise RuntimeError('Local MQTT broker exited')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Local MQTT broker did not start in 10 seconds')
                    time.sleep(0.1)
        else:
            print('Using configured external broker; no local broker started.', flush=True)
        stack = subprocess.Popen(['bash', str(ROOT/'start.sh'), 'mqtt-gateway', *arguments])
        children.append(stack)
        next_check = time.monotonic()+2
        while stack.poll() is None:
            if broker is not None and broker.poll() is not None:
                raise RuntimeError('Local MQTT broker exited')
            if local and broker is None and time.monotonic() >= next_check:
                next_check = time.monotonic()+2
                if not reachable():
                    print('Reused local broker disappeared; starting a managed test broker.', flush=True)
                    broker = subprocess.Popen(['bash', str(ROOT/'start.sh'), 'broker'])
                    children.insert(0, broker)
            time.sleep(0.2)
        if stack.returncode:
            raise RuntimeError(f'MQTT task stack exited with {stack.returncode}')
    except KeyboardInterrupt:
        pass
    finally:
        for process in reversed(children):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=5)


if __name__ == '__main__':
    main()
