#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
import copy, os, psutil
from std_msgs.msg import Float64MultiArray

class MonitorCpu:

  def __init__(self):
    self.cpu_memory_pub = rospy.Publisher('cpu_memory', Float64MultiArray, 
      queue_size=1)
    self.cpu_freq_pub = rospy.Publisher('cpu_freq', Float64MultiArray,
      queue_size=1)
    self.rate = rospy.get_param('~rate', 50)

  def run(self):
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()

      # Get cpu and memory info
      cpu_load = psutil.cpu_percent(interval=0.95/self.rate, percpu=True)
      memory_load = psutil.virtual_memory()
      swap_load = psutil.swap_memory()

      multi_array = Float64MultiArray()
      multi_array.data.append(len(cpu_load))

      for i in range(len(cpu_load)):
        multi_array.data.append(cpu_load[i])
      #cpu_load_all = psutil.cpu_percent(interval=0.5/self.rate)
      #multi_array.data.append(cpu_load_all)

      multi_array.data.append(memory_load.percent)
      multi_array.data.append(swap_load.percent)

      self.cpu_memory_pub.publish(multi_array)


      # Get cpu frequency
      cpu_freq = psutil.cpu_freq(percpu=True)
      freq_multi_array = Float64MultiArray()
      for i in range(len(cpu_freq)):
        freq_multi_array.data.append(cpu_freq[i][0])

      self.cpu_freq_pub.publish(freq_multi_array)


if __name__ == '__main__':

  rospy.init_node('cpu_monitor')
  monitor = MonitorCpu()
  monitor.run()
