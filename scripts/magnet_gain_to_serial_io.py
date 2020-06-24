#!/usr/bin/env python

import rospy, math
from std_msgs.msg import Float64, Int32


class MagnetGainToSerialIo:

  def __init__(self):
    # Publish to serial io
    self.serial_io_pub = rospy.Publisher('serial_io', Int32, queue_size=1)
    self.magnet_on_off = Int32()

    self.magnet_gain_old = 0.0
    rospy.Subscriber('magnet/gain', Float32, self.magnetGainCallback, 
      queue_size=1)


  def run(self):
    rospy.spin()

  def magnetGainCallback(self, msg):
    # If rising or falling edge was detected publish
    if (abs(self.magnet_gain_old - msg.data) > 0.1):
      self.magnet_on_off.data = int(msg.data)
      self.serial_io_pub.publish(self.magnet_on_off)

      self.magnet_gain_old = msg.data

if __name__ == "__main__":
  rospy.init_node('magnet_gain_to_serial_io')
    magnet_gain_to_serial_io = MagnetGainToSerialIo()
    magnet_gain_to_serial_io.run()