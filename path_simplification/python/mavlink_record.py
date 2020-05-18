#!/usr/bin/python2.7

import rosbag

bag_file = "../bags/manual_flight_rosbag.bag"

bag = rosbag.Bag(bag_file)

with open("../diverse/data.txt", "w+") as f:
    last_time = None
    homes_alt = None
    for topic, msg, t in bag.read_messages(topics=["/mavlink_interface/home_position"]):
        homes_alt = msg.altitude/1000
        break

    for topic, msg, t in bag.read_messages(topics=["/mavlink_pos"]):
        dt = 0
        if last_time != None:
            dt = t - last_time
        last_time = t

        f.write(str(dt) + " " + str(msg.lat) + " " + str(msg.lon) + " " + str(msg.alt - homes_alt) + "\n")

bag.close()
