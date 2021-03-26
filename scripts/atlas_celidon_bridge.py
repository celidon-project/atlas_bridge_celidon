#!/usr/bin/python

import json
import signal
import sys

import paho.mqtt.client as mqtt

import rospy
import tf
from geometry_msgs.msg import PointStamped

import numpy as np
import collections

def signal_handler(sig, frame):
    print('You pressed Ctrl+C')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def ros_mqtt_bridge():
    rospy.init_node('ros_mqtt_bridge', anonymous=True)

    euis = []
    alias = []

    if rospy.has_param('/atlas/fast'):
        objects = rospy.get_param('/atlas/fast')
        keys = objects.keys()
        for key in keys: 
            if rospy.has_param('/atlas/fast/' + key + '/enable'):
                euis.append(key)
                if rospy.has_param('/atlas/fast/' + key + '/alias'):
                    a = rospy.get_param('/atlas/fast/' + key + '/alias')
                    alias.append(a)
                else:
                    alias.append('No Alias')
    else:
        rospy.logerr("No EUIs in tags.yaml")

    mqtt_prefix = 'celidon/iloc'
    ros_topic = '/atlas/loc/'

    tracked = []
    ros_subs = []
    timeout = 0.5
    mw = 10 # moving window length
    multi = 0.5 # stddev multiplier
    off = 200
    rate = rospy.Rate(30)

    def callback(data, i):
        tracked[i]['ts'] = rospy.get_rostime()
        tracked[i]['point'] = data.point
        tracked[i]['eui'] = euis[i]
        tracked[i]['alias'] = alias[i]
        tracked[i]['xbuf'].append(data.point.x)
        tracked[i]['ybuf'].append(data.point.y)

    def on_connect(client, userdata, flags, rc):
        print('Connected with result code ' + str(rc))
        
    
    def publish():
        now = rospy.get_rostime()
        for i, obj in enumerate(tracked):
            diff = now - obj['ts']
            if diff.to_sec() < timeout and obj['point'] is not None:

                position_mm = [int(obj['point'].x * 1000),
                               int(obj['point'].y * 1000),
                               int(obj['point'].z * 1000)]
                
                std_dev = np.mean(np.std(np.asarray(obj['xbuf'])) +
                                  np.std(np.asarray(obj['ybuf'])))
                std_dev = int((std_dev) * 1000) + off

                mqtt_dict = {
                    obj['eui']: {
                        'ts': int(obj['ts'].to_sec()*1000),
                        'pos': position_mm,
                        'std_deviation': std_dev,
                        'alias': obj['alias']
                    }
                }

                print(mqtt_dict)
                client.publish(mqtt_prefix, json.dumps(mqtt_dict))

    client = mqtt.Client()
    client.on_connect = on_connect

    client.connect("localhost", 1883, 60)

    for i, obj in enumerate(euis):
        ros_subs.append(rospy.Subscriber(ros_topic + obj + '/point', PointStamped, callback, i))
        tracked.append({'point': None, 
            'ts': rospy.get_rostime(), 
            'xbuf': collections.deque(maxlen=mw),
            'ybuf': collections.deque(maxlen=mw)})

    while not rospy.is_shutdown():
        publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        ros_mqtt_bridge()
    except rospy.ROSInterruptException:
        pass
