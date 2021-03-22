#!/usr/bin/python

import json

import paho.mqtt.client as mqtt

import rospy


def extract_subset(subset, d):
    """ Returns a subset of dict d with all keys mapped only to the subkeys
    listed in subset

    Example:
        extract_positions(["pos"], {"foo": {"mode": 0, "pos": [1, 2, 3]}
                                    "bar": {"port": 9999, "pos": [4, 5, 6]}})
        {"foo": {"pos": [1, 2, 3]}, "bar": {"pos": [4, 5, 6]}}
    """
    return {k: {s: v[s] for s in subset} for k, v in d.items()}


def ros_mqtt_parameter_bridge():
    """ Publishes relevant parameter from the ROS parameter server to MQTT.
    """
    rospy.init_node('ros_mqtt_parameter_bridge', anonymous=True)

    pos = '{}'

    client = mqtt.Client()
    client.connect("localhost", 1883, 60)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        # Track changes in parameter server and publish new message whenever
        # something has changed
        params = rospy.get_param('/atlas')
        anchor = extract_subset(['pos'], params['anchor'])
        ref = extract_subset(['pos', 'yaw'], params['ref'])
        pos_new = json.dumps(dict(anchor=anchor, ref=ref))
        if pos_new != pos:
            pos = pos_new
            rospy.loginfo('MQTT parameters updated')
            client.publish('celidon/pos', pos, qos=1, retain=True)
        rate.sleep()


if __name__ == '__main__':
    ros_mqtt_parameter_bridge()
