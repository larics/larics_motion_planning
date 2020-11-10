#!/usr/bin/env python
# Print total cumulative serialized msg size per topic
import rosbag
import sys
topic_size_dict = {}
for topic, msg, time in rosbag.Bag(sys.argv[1], 'r').read_messages(raw=True):
  topic_size_dict[topic] = topic_size_dict.get(topic, 0) + len(msg[1])
topic_size = list(topic_size_dict.items())
topic_size.sort(key=lambda x: x[1])
for topic, size in topic_size:
  print(topic, size/1000000.0)