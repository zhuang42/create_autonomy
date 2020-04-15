#! /usr/bin/python

#***********************************************************
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Author: Wim Meeussen

import math
import os
import PyKDL
import rospy
import subprocess
import threading
import yaml

from ca_msgs.msg import Float64Stamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

import dynamic_reconfigure.client

def quat_to_angle(quat):
  rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
  return rot.GetRPY()[2]

def normalize_angle(angle):
  angle = math.fmod(angle + math.pi, 2*math.pi)
  if angle < 0:
    angle += 2*math.pi
  return angle - math.pi

sign = lambda a: (a>0) - (a<0)

class CalibrateRobot:
  def __init__(self):
    self.lock = threading.Lock()

    # TODO: WHERE DO I GET THIS?
    self.has_gyro = rospy.get_param("turtlebot_node/has_gyro")
    rospy.loginfo("has_gyro: {}".format(self.has_gyro))
    if self.has_gyro:
      self.sub_imu  = rospy.Subscriber('imu', Imu, self.imu_cb)

    self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_cb)
    self.sub_scan = rospy.Subscriber('scan_angle', Float64Stamped, self.scan_cb)
    self.cmd_pub = rospy.Publisher('cmd_vel', Twist)

    # params
    self.initial_wall_angle = rospy.get_param("initial_wall_angle", 0.1)
    self.imu_calibrate_time = rospy.get_param("imu_calibrate_time", 10.0)
    self.buffer_time = rospy.get_param("buffer_time", 0.5)
    self.imu_angle = 0
    self.imu_time = rospy.Time.now()
    self.scan_angle = 0
    self.scan_time = rospy.Time.now()
    self.odom_angle = 0
    self.odom_time = rospy.Time.now()

  def calibrate(self, speed, imu_drift=0):
      # Rotate 360 degrees
      (imu_start_angle, odom_start_angle, scan_start_angle,
          imu_start_time, _, _) = self.sync_timestamps()
      last_angle = odom_start_angle
      turn_angle = 0
      while turn_angle < 2*math.pi:
        if rospy.is_shutdown():
          return
        cmd = Twist()
        cmd.angular.z = speed
        self.cmd_pub.publish(cmd)
        rospy.sleep(0.1)
        with self.lock:
          delta_angle = normalize_angle(self.odom_angle - last_angle)
        turn_angle += delta_angle
        last_angle = self.odom_angle
      # Stop the robot
      self.cmd_pub.publish(Twist())
      # Compare rotations obtained by laser and wheel odometry
      (imu_end_angle, odom_end_angle, scan_end_angle,
          imu_end_time, _, _) = self.sync_timestamps()

      scan_delta = 2*math.pi + normalize_angle(scan_end_angle - scan_start_angle)

      odom_delta = 2*math.pi + normalize_angle(odom_end_angle - odom_start_angle)

      odom_result = odom_delta/scan_delta
      rospy.loginfo("Odom error: {0:.2f}%".format(100.*(odom_result-1.)))
      # Check gyro drift now
      if self.has_gyro:
        imu_delta = 2*math.pi + normalize_angle(imu_end_angle - imu_start_angle) \
                   - imu_drift*(imu_end_time - imu_start_time).to_sec()

        imu_result = imu_delta/scan_delta
        rospy.loginfo("IMU error: {0:.2f}%".format(100.*(imu_result-1.)))
      else:
        imu_result = None

      return (imu_result, odom_result)


  def imu_drift(self):
    if not self.has_gyro:
      return 0
    # Estimate imu drift
    rospy.loginfo('Estimating IMU drift')
    (imu_start_angle, _, _,
        imu_start_time, _, _) = self.sync_timestamps()
    rospy.sleep(self.imu_calibrate_time)
    (imu_end_angle, _, _,
        imu_end_time, _, _) = self.sync_timestamps()
    # Drift is calculated with dAngle / dTime
    imu_drift = normalize_angle(imu_end_angle - imu_start_angle) / ((imu_end_time - imu_start_time).to_sec())
    rospy.loginfo("IMU drift is {} deg/s".format(imu_drift*180./math.pi))
    return imu_drift


  def align(self):
    self.sync_timestamps()
    rospy.loginfo("Aligning base with wall")
    with self.lock:
      angle = self.scan_angle
    cmd = Twist()
    # The initial angle must be less than "initial_wall_angle"
    while abs(angle) > self.initial_wall_angle:
      if rospy.is_shutdown():
        exit(0)
      # Positive angle ==> rotate left
      cmd.angular.z = -0.3*sign(angle)
      self.cmd_pub.publish(cmd)
      rospy.sleep(0.05)
      with self.lock:
        angle = self.scan_angle


  def sync_timestamps(self, start_time=None):
    if not start_time:
      start_time = rospy.Time.now() + rospy.Duration(self.buffer_time)
    while not rospy.is_shutdown():
      rospy.sleep(0.3)
      with self.lock:
        if self.imu_time < start_time and self.has_gyro:
          rospy.loginfo("Still waiting for imu")
        elif self.odom_time < start_time:
          rospy.loginfo("Still waiting for odom")
        elif self.scan_time < start_time:
          rospy.loginfo("Still waiting for scan")
        else:
          return (self.imu_angle, self.odom_angle, self.scan_angle,
                  self.imu_time, self.odom_time, self.scan_time)
    exit(0)


  def imu_cb(self, msg):
    with self.lock:
      self.imu_angle = quat_to_angle(msg.orientation)
      self.imu_time = msg.header.stamp

  def odom_cb(self, msg):
    with self.lock:
      self.odom_angle = quat_to_angle(msg.pose.pose.orientation)
      self.odom_time = msg.header.stamp

  def scan_cb(self, msg):
    with self.lock:
      self.scan_angle = msg.data
      self.scan_time = msg.header.stamp

def get_usb_to_serial_id():
  usbpath = subprocess.check_output("readlink -f /sys/class/tty/ttyUSB0", shell=True)
  usbpath = usbpath.strip()
  if len(usbpath) == 0:
    return None
  serialid = ""
  try:
    f = open(usbpath + "/../../../../serial", "r")
    serialid = f.read().strip()
    f.close()
  except:
    pass
  try:
    f = open(usbpath + "/../../../../idVendor", "r")
    serialid += f.read().strip()
    f.close()
    f = open(usbpath + "/../../../../idProduct", "r")
    serialid += f.read().strip()
    f.close()
  except:
    pass
  if len(serialid.strip()) == 0:
    return None
  return serialid

def get_kinect_serial():
  ret = subprocess.check_output("lsusb -v -d 045e:02ae | grep Serial | awk '{print $3}'", shell=True)
  if len(ret) > 0:
    return ret.strip()
  return None

def getCurrentParams(drclient):
  cfg = drclient.get_configuration()
  return (cfg['gyro_scale_correction'],
          cfg['odom_angular_scale_correction'],
          cfg['gyro_measurement_range'])

def writeParams(drclient, newparams):
  drclient.update_configuration(newparams)
  rospy.loginfo("Automatically updated the params in the current running instance of ROS, no need to restart.")

def writeParamsToCalibrationFile(newparams):
  kinect_serial = get_kinect_serial()
  if kinect_serial is None:
    kinect_serial =  get_usb_to_serial_id()  # can't find a kinect, attempt to use the usb to serial convert's id as a backup
    if kinect_serial is None:
      return
  ros_home = os.environ.get('ROS_HOME')
  if ros_home is None:
    ros_home = "~/.ros"
  calib_dir = os.path.expanduser(ros_home +"/turtlebot_create/")
  calib_file = calib_dir +str(kinect_serial) + ".yaml"
  # if the file exists, load into a dict, update the new params, and then save
  if os.path.isfile(calib_file):
    f = open(calib_file, 'r')
    docs = yaml.load_all(f)
    d = docs.next()
    for k,v in newparams.iteritems():
      d[k] = v
    newparams = d
    f.close()
  try:
    os.makedirs(calib_dir)
  except:
    pass
  with open(calib_file, 'w') as outfile:
    outfile.write( yaml.dump(newparams, default_flow_style=False) )
  rospy.loginfo("Saved the params to the calibration file: %s" % calib_file)

def writeParamsToLaunchFile(gyro, odom, gyro_range):
  # try:
  #     f = open("/etc/ros/distro/turtlebot.launch", "r")
  #     # this is totally NOT the best way to solve this problem.
  #     foo = []
  #     for lines in f:
  #         if "turtlebot_node/gyro_scale_correction" in lines:
  #             foo.append("  <param name=\"turtlebot_node/gyro_scale_correction\" value=\"%f\"/>\n" % gyro)
  #         elif "turtlebot_node/odom_angular_scale_correction" in lines:
  #             foo.append("  <param name=\"turtlebot_node/odom_angular_scale_correction\" value=\"%f\"/>\n" % odom)
  #         elif "turtlebot_node/gyro_measurement_range" in lines:
  #             foo.append("  <param name=\"turtlebot_node/gyro_measurement_range\" value=\"%f\"/>\n" % gyro_range)
  #         else:  f = open(calib_file, 'r')
  #   docs = yaml.load_all(f)
  #   d = docs.next()
  #   for k,v in newparams.iteritems():
  #       d[k] = v
  #   newparams = d

  #             foo.append(lines)
  #     f.close()

  #     # and... write!
  #     f = open("/etc/ros/distro/turtlebot.launch", "w")
  #     for i in foo:
  #         f.write(i)
  #     f.close()
  #     rospy.loginfo("Automatically updated turtlebot.launch, please restart the turtlebot service.")
  # except:
  #     rospy.loginfo("Could not automatically update turtlebot.launch, please manually update it.")

def warnAboutGyroRange(drclient):
  params = getCurrentParams(drclient)
  rospy.logwarn("***** If you have not manually set the gyro range parameter you must do so before running calibration.  Cancel this run and see http://wiki.ros.org/turtlebot_calibration/Tutorials/Calibrate%20Odometry%20and%20Gyro")
  rospy.logwarn("******* turtlebot_node/gyro_measurement_range is currently set to: %d ******" % params[2])


def main():
  rospy.init_node('robot_calibration')
  robot = CalibrateRobot()
  imu_res = 1.0
  # Start dynamic reconfigure client
  # TODO: ENABLE DYNAMIC RECONFIGURE
  # https://github.com/Arkapravo/turtlebot/blob/master/turtlebot_node/nodes/turtlebot_node.py#L109
  drclient = dynamic_reconfigure.client.Client("turtlebot_node")
  warnAboutGyroRange(drclient)

  imu_corr = []
  odom_corr = []
  for speed in (0.3, 0.7, 1.0, 1.5):
    robot.align()
    (imu, odom) = robot.calibrate(speed, robot.imu_drift())
    if imu:
        imu_corr.append(imu)
    odom_corr.append(odom)
  # TODO: Enable gyro parameters
  # https://github.com/Arkapravo/turtlebot/blob/88e910d252adfa3b8da145ec63fda436bcb2ecc0/turtlebot_node/src/turtlebot_node/gyro.py#L51-L52
  (prev_gyro, prev_odom, gyro_range) = getCurrentParams(drclient)
  if len(imu_corr)>0:
    imu_res = prev_gyro * (1.0/(sum(imu_corr)/len(imu_corr)))
    rospy.loginfo("Set the 'turtlebot_node/gyro_scale_correction' parameter to %f"%imu_res)

  odom_res = prev_odom * (1.0/(sum(odom_corr)/len(odom_corr)))
  rospy.loginfo("Set the 'turtlebot_node/odom_angular_scale_correction' parameter to %f"%odom_res)

  # Write parameters
  # TODO: Make it work
  # writeParamsToLaunchFile(imu_res, odom_res, gyro_range)

  newparams = {
    'gyro_scale_correction' : imu_res,
    'odom_angular_scale_correction' : odom_res,
    'gyro_measurement_range' : gyro_range
  }
  writeParamsToCalibrationFile(newparams)
  writeParams(drclient, newparams)

if __name__ == '__main__':
    main()
