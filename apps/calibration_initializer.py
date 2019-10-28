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

import roslib; roslib.load_manifest('ct_calibration')
import rospy, rospkg
from ct_calibration.srv import *

class CalibrationInitializer:

    def __init__(self):

        network = rospy.get_param('~network')
        self.checkerboard = rospy.get_param('~checkerboard')
        self.aprilTag = rospy.get_param('~aprilTag')
        self.master_filename = rospy.get_param('~master_calibration_launch_file')
        self.frame_filename = rospy.get_param('~frame_calibration_launch_file')
        self.sensor_list = []
        self.sensor_map = {}
        for item in network:
            pc = item['pc']
            sensors = item['sensors']
            self.sensor_map[pc] = sensors
            for sensor in sensors:
                self.sensor_list.append(sensor)
        self.session_id = rospy.Time.now().secs

    def createMaster(self):

        # createMaster launch file.
        launch_file = open(self.master_filename, 'w')
        launch_file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        launch_file.write('<!-- SESSION ID: %s -->\n' % str(self.session_id))
        launch_file.write('<launch>\n\n')

        launch_file.write('  <!-- Calibration parameters -->\n')
        launch_file.write('  <arg name="num_sensors"   default="%s" />\n' % (str(len(self.sensor_list))))
        # Calibration Type: aprilTag or checkerboard.
        launch_file.write('  <arg name="calibration_type" default="aprilTag" />\n\n')
        index = 0
        for sensor in self.sensor_list:
            launch_file.write('  <arg name="sensor_%d_name"    default="%s" />\n' \
            % (str(index), sensor['id']))
            index += 1

        # Opening Rviz for visualization.
        launch_file.write('  <!-- Opening Rviz for visualization -->\n')
        file.write('  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find ct_calibration)/config/ct_calibration.rviz" />\n\n')
        launch_file.write('  <!-- Plot calibration status -->\n')
        launch_file.write('  <node name="ct_calibration_status_plot" pkg="ct_calibration" type="status_plot.py" output="screen">\n')
        launch_file.write('    <remap from="~calibration_status" to="/ct_calibration/status" />\n')
        launch_file.write('  </node>\n\n')

        # Checkerboard calibration.
        launch_file.write('  <group if="$(eval calibration_type == "aprilTag")">\n')
        ########################################################
        ###                 Waitting coding                  ###
        ########################################################
        launch_file.write('  </group>\n\n')

        launch_file.write('  <group if="$(eval calibration_type == "checkerboard")">\n')
        launch_file.write('    <arg name="rows"    default="%s" />\n' % str(self.checkerboard['rows']))
        launch_file.write('    <arg name="cols"    default="%s" />\n' % str(self.checkerboard['cols']))
        launch_file.write('    <arg name="cell_width"    default="%s" />\n' % str(self.checkerboard['cell_width']))
        launch_file.write('    <arg name="cell_height"    default="%s" />\n\n' % str(self.checkerboard['cell_height']))

        # Launching calibration.
        launch_file.write('    <!-- Launching calibration -->\n')
        launch_file.write('    <node pkg="ct_calibration" type="ct_calibration" name="ct_calibration" output="screen">\n\n')
        launch_file.write('      <param name="num_sensors"           value="$(arg num_sensors)" />\n')
        launch_file.write('      <param name="rows"           value="$(arg rows)" />\n')
        launch_file.write('      <param name="cols"           value="$(arg cols)" />\n')
        launch_file.write('      <param name="cell_width"           value="$(arg cell_width)" />\n')
        launch_file.write('      <param name="cell_height"           value="$(arg cell_height)" />\n')
        index = 0
        for sensor in self.sensor_list:
            i = str(index)
            launch_file.write('      <param name="sensor_%d/name"    value="/$(arg sensor_%d_name)" />\n' % (i, i))
            if sensor['type'] == 'kinect2':
                launch_file.write('      <param name=sensor_%d/type    value="pinhole_rgb" />\n' %i)
                launch_file.write('      <remap from="~sensor_%d/image"    to="/$(arg sensor_%d_name)/hd/image_color_rect" />\n' % (i, i))
                launch_file.write('      <remap from="~sensor_%d/camera_info"    to="/$(arg sensor_%d_name)/hd/camera_info" />\n\n' % (i, i))
            elif sensor['type'] == 'realsense':
                launch_file.write('      <param name=sensor_%d/type    value="pinhole_rgb" />\n' %i)
                launch_file.write('      <remap from="~sensor_%d/image"    to="/$(arg sensor_%d_name)/color/image_rect_color" />\n' % (i, i))
                launch_file.write('      <remap from="~sensor_%d/camera_info"    to="/$(arg sensor_%d_name)/color/camera_info" />\n\n' % (i, i))
            else:
                rospy.logfatal('[Error]: %s is not supported yet!' % sensor['type'])
            index += 1
        launch_file.write('    </node>\n\n')
        launch_file.write('  </group>\n\n')
        launch_file.write('</launch>\n')
        launch_file.close()


        # Create launch file for defining user reference frame:

        frame_file = open(self.frame_filename, 'w')
        frame_file.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        frame_file.write('<!-- SESSION ID: %s -->\n' % str(self.session_id))
        frame_file.write('<launch>\n\n')

        # Calibration parameters
        frame_file.write('  <!-- Calibration parameters -->\n')
        frame_file.write('  <rosparam command="load" file="$(find ct_calibration)/config/camera_poses.yaml" />\n\n')

        # Network parameters
        frame_file.write('  <!-- Network parameters -->\n')
        frame_file.write('  <arg name="num_sensors"    default="%s" />\n\n' % str(len(self.sensor_list)))

        index = 0
        for sensor in self.sensor_list:
            frame_file.write('  <arg name="sensor_%d_name"    default="%s" />\n' % (str(index), sensor['id']))
            index += 1

        # Calibration node
        frame_file.write('  <!-- Launching calibration -->\n')
        frame_file.write('  <node pkg="ct_calibration" type="ct_define_reference_frame" name="ct_define_reference_frame" output="screen">\n')
        frame_file.write('    <rosparam command="load" file="$(find ct_calibration)/config/camera_network.yaml" />\n\n')
        frame_file.write('    <param name="num_sensors"    value="$(arg num_sensors)" />\n\n')

        index = 0
        for sensor in self.sensor_list:
            i = str(index)
            frame_file.write('    <param name="sensor_%d/name"    value="/$(arg sensor_%d_name)" />\n' % (i, i))
            if sensor['type'] == 'kinect2':
                frame_file.write('      <param name=sensor_%d/type    value="pinhole_rgb" />\n' %i)
                frame_file.write('      <remap from="~sensor_%d/image"    to="/$(arg sensor_%d_name)/hd/image_color_rect" />\n' % (i, i))
                frame_file.write('      <remap from="~sensor_%d/camera_info"    to="/$(arg sensor_%d_name)/hd/camera_info" />\n\n' % (i, i))
            elif sensor['type'] == 'realsense':
                frame_file.write('      <param name=sensor_%d/type    value="pinhole_rgb" />\n' %i)
                frame_file.write('      <remap from="~sensor_%d/image"    to="/$(arg sensor_%d_name)/color/image_rect_color" />\n' % (i, i))
                frame_file.write('      <remap from="~sensor_%d/camera_info"    to="/$(arg sensor_%d_name)/color/camera_info" />\n\n' % (i, i))
            else:
                rospy.logfatal('[Error]: %s is not supported yet!' % sensor['type'])
            index += 1

        frame_file.write('  </node>\n\n')
        frame_file.write('</launch>\n')
        frame_file.close()

    def fileName(self):
        return self.master_filename

    def frameFileName(self):
        return self.frame_filename

    def _invokeService(self, service_name):

        # For each PC
        for pc in self.sensor_map:
            service = pc + '/' + service_name
            rospy.loginfo('[Info]: Waiting for %s service...' % service)
            rospy.wait_for_service(service)

            for sensor_item in self.sensor_map[pc]:
                # Create an CTSensor message. [Which have CTSensor, CTSensorRequest, CTSensorResponse]
                sensor_msg = CTSensorRequest()
                sensor_msg.session_id = self.session_id
                sensor_msg.id = sensor_item['id']
                if sensor_item['type'] == 'kinect2':
                    sensor_msg.type = CTSensorRequest.TYPE_KINECT2
                    if 'serial' in sensor_item:
                        sensor_msg.serial = sensor_item['serial']
                elif sensor_item['type'] == 'realsense':
                    sensor_msg.type = CTSensorRequest.TYPE_REALSENSE
                    if 'serial' in sensor_item['serial']:
                        sensor_msg.serial = sensor_item['serial']
                else:
                    rospy.logfatal('[Error]: %s is not support yet!' % sensor_item['type'])

                # Invoke service
                try:
                    add_sensor = rospy.ServiceProxy(service, CTSensor)
                    response = add_sensor(sensor_msg)
                    if response.status == CTSensorResponse.STATUS_OK:
                        rospy.loginfo('[Info]: <%s>' % pc + response.message)
                    else:
                        rospy.logerr('[Error]: <%s>' % pc + response.message)
                except rospy.ServiceException, e:
                    rospy.logerr('[Error]: Service call failed: %s' % e)

    def createSensorLaunch(self):
        self._invokeService('create_sensor_launch')

if __name__ == '__main__':

    rospy.init_node('calibration_initializer')

    try:
        init = CalibrationInitializer()
        init.createMaster()
        rospy.loginfo('[Info]: %s created' % init.fileName())
        rospy.loginfo('[Info]: %s created' % init.frameFileName())

        init.createSensorLaunch()
        rospy.loginfo('[Info]: Initialization completed. Press [ctrl+c] to quit.')

        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('[Error]: ROSInterruptException.')
        pass
