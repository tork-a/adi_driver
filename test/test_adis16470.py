#!/usr/bin/env python

# Copyright (c) 2017, Tokyo Opensource Robotics Kyokai Association
# All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Ryosuke Tajima

import math
import unittest
import rospy
import rostest
import time
from sensor_msgs.msg import Imu

def imu_get_min(imu, a, b):
    imu.angular_velocity.x = min(a.angular_velocity.x, b.angular_velocity.x)
    imu.angular_velocity.y = min(a.angular_velocity.y, b.angular_velocity.y)
    imu.angular_velocity.z = min(a.angular_velocity.z, b.angular_velocity.z)
    imu.linear_acceleration.x = min(a.linear_acceleration.x, b.linear_acceleration.x)
    imu.linear_acceleration.y = min(a.linear_acceleration.y, b.linear_acceleration.y)
    imu.linear_acceleration.z = min(a.linear_acceleration.z, b.linear_acceleration.z)

def imu_get_max(imu, a, b):
    imu.angular_velocity.x = max(a.angular_velocity.x, b.angular_velocity.x)
    imu.angular_velocity.y = max(a.angular_velocity.y, b.angular_velocity.y)
    imu.angular_velocity.z = max(a.angular_velocity.z, b.angular_velocity.z)
    imu.linear_acceleration.x = max(a.linear_acceleration.x, b.linear_acceleration.x)
    imu.linear_acceleration.y = max(a.linear_acceleration.y, b.linear_acceleration.y)
    imu.linear_acceleration.z = max(a.linear_acceleration.z, b.linear_acceleration.z)

class TestImu(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_imu')

    def setUp(self):
        self.imu_raw_count = 0
        self.imu_count = 0
        self.imu_raw_min = Imu()
        self.imu_raw_max = Imu()
        self.imu_min = Imu()
        self.imu_max = Imu()
        rospy.Subscriber('/imu/data_raw', Imu, self.cb_imu_raw, queue_size=1000)
        rospy.Subscriber('/imu/data', Imu, self.cb_imu, queue_size=1000)
        
    def cb_imu_raw(self, msg):
        self.imu_raw_count += 1
        imu_get_min(self.imu_raw_min, self.imu_raw_min, msg)
        imu_get_max(self.imu_raw_max, self.imu_raw_max, msg)

    def cb_imu(self, msg):
        self.imu_count += 1
        imu_get_min(self.imu_raw_min, self.imu_raw_min, msg)
        imu_get_max(self.imu_raw_max, self.imu_raw_max, msg)

    def test_imu_raw(self):
        time.sleep(0.1)
        self.assertTrue(self.imu_raw_count>0, 'No data received from /imu/data_raw')
        self.assertFalse(math.isnan(self.imu_raw_min.linear_acceleration.x));
        self.assertFalse(math.isnan(self.imu_raw_min.linear_acceleration.y));
        self.assertFalse(math.isnan(self.imu_raw_min.linear_acceleration.z));
        self.assertFalse(math.isnan(self.imu_raw_max.linear_acceleration.x));
        self.assertFalse(math.isnan(self.imu_raw_max.linear_acceleration.y));
        self.assertFalse(math.isnan(self.imu_raw_max.linear_acceleration.z));

    def test_imu(self):
        time.sleep(0.1)
        self.assertTrue(self.imu_raw_count>0, 'No data received from /imu/data')
        self.assertFalse(math.isnan(self.imu_min.linear_acceleration.x));
        self.assertFalse(math.isnan(self.imu_min.linear_acceleration.y));
        self.assertFalse(math.isnan(self.imu_min.linear_acceleration.z));
        self.assertFalse(math.isnan(self.imu_max.linear_acceleration.x));
        self.assertFalse(math.isnan(self.imu_max.linear_acceleration.y));
        self.assertFalse(math.isnan(self.imu_max.linear_acceleration.z));

if __name__ == '__main__':
    rostest.rosrun('adi_driver', 'test_adi_driver', TestImu)
        
