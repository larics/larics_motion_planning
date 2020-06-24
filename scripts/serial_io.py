#!/usr/bin/env python

__author__ = 'aivanovic'

import copy, time, sys
import serial

import rospy
from std_msgs.msg import Int32

class SerialIO:

  def __init__(self):
    # Serial io params
    self.port = rospy.get_param('port', '/dev/ttyUSB0')
    self.baudrate = rospy.get_param('baudrate', int(9600))
    # Define serial port
    self.serial_port = serial.Serial(
      port=self.port,
      baudrate=self.baudrate)
    self.serial_port.isOpen()

    # Ros loop params
    self.rate = rospy.get_param('~rate', 10000)

    # Initial message is 0
    self.io_msg = b'\x00'
    # Subscriber to io topic
    self.serial_io_sub = rospy.Subscriber('serial_io', Int32,
      self.serialIoCallback, queue_size=1)

  def run(self):
    print("Starting loop.")
    rate = rospy.Rate(self.rate)
    while not rospy.is_shutdown():
      rate.sleep()
      
      self.serial_port.write(self.io_msg)

    # Close the serial port
    print("Closing serial port.")
    self.serial_port.close()

  def serialIoCallback(self, msg):
    if msg.data == 0:
      self.io_msg = b'\x00'
    else:
      self.io_msg = b'\xff'

if __name__ == '__main__':
  rospy.init_node('serial_io')
  serial_io = SerialIO()
  serial_io.run()