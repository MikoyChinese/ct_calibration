#!/usr/bin/env python
#
# Copyright 2019 Mikoy Chinese.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Mikoy Chinese [mikoychinese@gmail.com]
# Github: https://github.com/MikoyChinese
# Reference: [https://github.com/OpenPTrack/open_ptrack_v2/tree/master/opt_calibration]
#==============================================================================

import os
import roslib; roslib.load_manifest('ct_calibration')
import rospy, rospkg
from ct_calibration.srv import *


class Listener:

    def __init__(self):
        # Check dir existed.
        self.sensor_launchers_dir = rospy.get_param('~sensor_launchers_dir')
        self.camera_poses_dir = rospy.get_param('~camera_poses_dir')
        _exist_flag = True
        for _dir in [self.sensor_launchers_dir, self.camera_poses_dir]:
            _exist = os.path.exists(_dir)
            if not _exist:
                rospy.logerr('[Error]: %s is not existed.' % _dir)
                _exist_flag = False
        if not _exist_flag:
            exit(1)

        # Create sensor launch  and camera_poses ervice.
        self.create_sensor_launch_srv = rospy.Service('create_sensor_launch',
                                    CTSensor, self._handle_create_sensor_launch)
        self.create_camera_poses_srv = rospy.Service('create_camera_poses',
                                    CTTransform, self._handle_create_camera_poses)


    def _handle_create_sensor_launch(self, request):
        sensor_file_name = os.path.join(self.sensor_launchers_dir,
                                        'sensor_%s.launch' % request.id)
        sensor_file = open(sensor_file_name, 'w')
        sensor_file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        sensor_file.write('<!-- SESSION ID: %s -->\n' % str(request.session_id))
        sensor_file.write('<!-- SERIAL: %s -->\n' % str(request.serial))
        sensor_file.write('<launch>\n\n')

        # Sensor parameters
        sensor_file.write('<!-- Sensor parameters -->\n')
        if request.type == CTSensorRequest.TYPE_KINECT2:
            sensor_file.write('  <arg name="sensor_name"    default="%s" />\n'
                              % request.id)
            if request.serial != '':
                sensor_file.write('  <arg name="sensor_serial"    default="%s" />\n\n'
                                  % request.serial)
            sensor_file.write('  <!-- Launch sensor -->\n')
            sensor_file.write('  <include file="$(find kinect2_bridge)/launch/kinect2_bridge_ir.launch">\n')
            if request.serial != '':
                sensor_file.write('    <arg name="sensor_id"    value="$(arg sensor_serial) />\n')
            sensor_file.write('    <arg name="sensor_name"    value="$(arg sensor_name)" />\n')
            sensor_file.write('    <arg name="publish_frame"    value="false" />\n')
            sensor_file.write('  </include>\n\n')

            sensor_file.write('  <!-- Publish a further transform -->\n')
            sensor_file.write('  <node pkg="tf" type="static_transform_publisher" name="$(arg sensor_name)_broadcaster" args="0 0 0 1.57079 -1.57079 0 /$(arg sensor_name) /$(arg sensor_name)_link  100" />\n\n')
        elif request.type == CTSensorRequest.TYPE_REALSENSE:
            sensor_file.write('  <arg name="sensor_name"    default="%s" />\n'
                              % request.id)
            if request.serial != '':
                sensor_file.write('  <arg name="sensor_serial"    default="%s" />\n\n'
                                  % request.serial)
            sensor_file.write('  <!-- Launch sensor -->\n')
            sensor_file.write('  <include file="$(find realsense2_camera)/launch/rs_rgbd.launch">\n')
            sensor_file.write('    <arg name="camera"    value="$(arg sensor_name)" />\n')
            if request.serial != '':
                sensor_file.write('    <arg name="serial_no"    value="$(arg sensor_serial)" />\n')
            sensor_file.write('  </include>')

        sensor_file.write('</launch>')
        sensor_file.close()
        rospy.loginfo('[Info]: %s created!' % sensor_file_name)

    def _handle_create_camera_poses(self, request):
        camera_poses_file_name = os.path.join(self.camera_poses_dir, 'camera_poses.txt')
        _file = open(camera_poses_file_name, 'w')
        _file.write('# Auto-generated file.\n')
        _file.write('# CALIBRATION ID: %s\n' % str(request.calibration_id))

        data = zip(request.child_id, request.transform)
        for item in data:
            t = item[1].translation
            r = item[1].rotation
            _file.write(item[0] + ': ' + str(t.x) + ' ' + str(t.y) + ' ' + str(t.z) + ' ')
            _file.write(str(r.x) + ' ' + str(r.y) + ' ' + str(r.z) + ' ' + str(r.w) + '\n')
        _file.close()
        rospy.loginfo(camera_poses_file_name + ' created!')
        return (CTTransformResponse.STATUS_OK, camera_poses_file_name + ' created!')


if __name__ == '__main__':

    rospy.init_node('listener')

    try:
        listener = Listener()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
