#!/usr/bin/env python
import time

import paho.mqtt.subscribe as subscribe

# setup mqtt
port = 1883  # default mqtt port: 1883
broker = 'localhost'  # address of the MQTT broker

topics = ['hololens/log', 'celidon/pos', 'celidon/iloc']

def on_message(_, __, m):
    print('{} [{}]: {}'.format(time.time(), m.topic, m.payload.decode()))

subscribe.callback(on_message, topics=topics, hostname=broker, port=port)
