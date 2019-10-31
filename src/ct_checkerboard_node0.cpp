/*
 *  Copyright (c) 2013- Filippo Basso, Riccardo Levorato, Matteo Munaro
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *     1. Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder(s) nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Filippo Basso [bassofil@dei.unipd.it]
 *          Riccardo Levorato [levorato@dei.unipd.it]
 *          Matteo Munaro [matteo.munaro@dei.unipd.it]
 *          Mikoy Chinese [mikoychinese@gmail.com]
 */

#include <fstream>
#include <omp.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>

#include <ceres/ceres.h>

#include <pcl/filters/filter_indices.h>
#include <pcl/filters/random_sample.h>

#include <calibration_common/pcl/utils.h>

#include <ct_calibration/ct_checkerboard_node.h>
#include <ct_calibration/CalibrationStatus.h>

namespace ct_calibration
{

CTCheckerboardNode::CTCheckerboardNode(const ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle),
    fixed_sensor_pose_(Pose::Identity())
{
  action_sub_ = node_handle_.subscribe("action", 1, &CTCheckerboardNode::actionCallback, this);
  status_pub_ = node_handle_.advertise<ct_calibration::CalibrationStatus>("status", 1);
  node_handle_.param("num_sensors", num_sensors_, 0);

  double cell_width, cell_height;
  int rows, cols;

  bool cb_ok = true;
  cb_ok = cb_ok and node_handle_.getParam("cell_width", cell_width);
  cb_ok = cb_ok and node_handle_.getParam("cell_height", cell_height);
  cb_ok = cb_ok and node_handle_.getParam("rows", rows);
  cb_ok = cb_ok and node_handle_.getParam("cols", cols);
  if (not cb_ok)
    ROS_FATAL("Checkerboard parameter missing! Please set \"rows\", \"cols\", \"cell_width\" and \"cell_height\".");

  checkerboard_ = boost::make_shared<Checkerboard>(cols, rows, cell_width, cell_height);
  checkerboard_->setFrameId("/checkerboard");

  for (int i = 0; i < num_sensors_; ++i)
  {
    std::stringstream ss;

    std::string type_s;
    ss.str("");
    ss << "sensor_" << i << "/type";
    if (not node_handle_.getParam(ss.str(), type_s))
      ROS_FATAL_STREAM("No \"" << ss.str() << "\" parameter found!!");

    ss.str("");
    ss << "/sensor_" << i;
    std::string frame_id = ss.str();

    ss.str("");
    ss << "sensor_" << i << "/name";
    node_handle_.param(ss.str(), frame_id, frame_id);

    ROSDevice::Ptr ros_device;

    if (type_s == "pinhole_rgb")
    {
      PinholeRGBDevice::Ptr device = boost::make_shared<PinholeRGBDevice>(frame_id);
      pinhole_vec_.push_back(device);
      ros_device = device;
      ROS_INFO_STREAM(device->frameId() << " added.");
    }
    else
    {
      ROS_FATAL_STREAM("\"" << ss.str() << "\" parameter value not valid. Please use pinhole_rgb.");
    }

    ss.str("");
    ss << "sensor_" << i;
    ros_device->createSubscribers(node_handle_, image_transport_, ss.str());
  }
}

bool CTCheckerboardNode::initialize()
{
  bool all_messages_received = false;
  ros::Rate rate(1.0);
  while (ros::ok() and not all_messages_received)
  {
    ros::spinOnce();
    all_messages_received = true;
    for (size_t i = 0; all_messages_received and i < pinhole_vec_.size(); ++i)
      all_messages_received = pinhole_vec_[i]->hasNewMessages();
    if (not all_messages_received)
      ROS_WARN_THROTTLE(5, "Not all messages received. Waiting...");
    rate.sleep();
  }

  ROS_INFO("All sensors connected.");

  calibration_ = boost::make_shared<CTCalibration>(node_handle_);
  calibration_->setCheckerboard(checkerboard_);

  for (size_t i = 0; i < pinhole_vec_.size(); ++i)
  {
    const PinholeRGBDevice::Ptr & device = pinhole_vec_[i];
    calibration_->addSensor(device->sensor(), true);
    sensor_vec_.push_back(device->sensor());
    images_acquired_map_[device->frameId()] = 0;
    status_msg_.sensor_ids.push_back(device->frameId());
  }

  status_msg_.images_acquired.resize(status_msg_.sensor_ids.size(), 0);
  status_msg_.header.stamp = ros::Time::now();
  status_msg_.header.seq = 0;
  status_pub_.publish(status_msg_);

  return true;
}

void CTCheckerboardNode::actionCallback(const std_msgs::String::ConstPtr & msg)
{
  if (msg->data == "save" or msg->data == "saveExtrinsicCalibration")
  {
    ROS_INFO("Saving calibration results...");
    calibration_->optimize();
    save();
  }
  else if (msg->data == "start floor")
  {
    if (calibration_->floorAcquisition())
    {
      ROS_WARN("Floor acquisition already started.");
    }
    else
    {
      ROS_INFO("Floor acquisition started.");
      calibration_->startFloorAcquisition();
    }
  }
  else if (msg->data == "stop floor")
  {
    if (not calibration_->floorAcquisition())
      ROS_WARN("Floor acquisition not started.");
    else
      calibration_->stopFloorAcquisition();
  }
  else
  {
    ROS_ERROR_STREAM("Unknown action: \"" << msg->data << "\"!");
  }
}

void CTCheckerboardNode::spin()
{
  ros::Rate rate(5.0);

  while (ros::ok())
  {
    ros::spinOnce();
    calibration_->nextAcquisition();

    int count = 0;

    try
    {
#pragma omp parallel for
      for (size_t i = 0; i < pinhole_vec_.size(); ++i)
      {
        const PinholeRGBDevice::Ptr & device = pinhole_vec_[i];
        if (device->hasNewMessages())
        {
          device->convertLastMessages();
          PinholeRGBDevice::Data::Ptr data = device->lastData();
          CTCalibration::CheckerboardView::Ptr cb_view;
          ROS_DEBUG_STREAM("[" << device->frameId() << "] analysing image generated at: " << device->lastMessages().image_msg->header.stamp);
#pragma omp critical
          if (calibration_->analyzeData(device->sensor(), data->image, cb_view))
          {
            calibration_->addData(device->sensor(), cb_view);
            images_acquired_map_[device->frameId()]++;
            ROS_INFO_STREAM("[" << device->frameId() << "] checkerboard detected");
            ++count;
          }
        }
      }
    }
    catch (cv_bridge::Exception & ex)
    {
      ROS_ERROR_STREAM("cv_bridge exception: " << ex.what());
      return;
    }
    catch (std::runtime_error & ex)
    {
      ROS_ERROR_STREAM("exception: " << ex.what());
      return;
    }

    status_msg_.header.stamp = ros::Time::now();
    status_msg_.header.seq++;
    for (size_t i = 0; i < status_msg_.sensor_ids.size(); ++i)
    {
      status_msg_.images_acquired[i] = images_acquired_map_[status_msg_.sensor_ids[i]];
    }
    status_pub_.publish(status_msg_);

    calibration_->perform();
    calibration_->publish();

    if (count > 0)
      ROS_INFO("-----------------------------------------------");

    rate.sleep();
  }
}

bool CTCheckerboardNode::save()
{
  // Save tfs between sensors and world coordinate system (last checherboard) to file
  std::string file_name = ros::package::getPath("ct_calibration") + "/config/camera_poses.yaml";
  std::ofstream file;
  file.open(file_name.c_str());

  if (file.is_open())
  {
    ros::Time time = ros::Time::now();
    file << "# Auto generated file." << std::endl;
    file << "calibration_id: " << time.sec << std::endl << std::endl;

    Pose new_world_pose = Pose::Identity();

    // Write TF transforms between cameras and world frame
    file << "# Poses w.r.t. the \"world\" reference frame" << std::endl;
    file << "poses:" << std::endl;
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      const Sensor::Ptr & sensor = sensor_vec_[i];

      Pose pose = new_world_pose * sensor->pose();

      file << "  " << sensor->frameId().substr(1) << ":" << std::endl;

      file << "    translation:" << std::endl
           << "      x: " << pose.translation().x() << std::endl
           << "      y: " << pose.translation().y() << std::endl
           << "      z: " << pose.translation().z() << std::endl;

      Quaternion rotation(pose.rotation());
      file << "    rotation:" << std::endl
           << "      x: " << rotation.x() << std::endl
           << "      y: " << rotation.y() << std::endl
           << "      z: " << rotation.z() << std::endl
           << "      w: " << rotation.w() << std::endl;

    }

    file << std::endl << "# Inverse poses" << std::endl;
    file << "inverse_poses:" << std::endl;
    for (size_t i = 0; i < sensor_vec_.size(); ++i)
    {
      const Sensor::Ptr & sensor = sensor_vec_[i];

      Pose pose = new_world_pose * sensor->pose();
      pose = pose.inverse();

      file << "  " << sensor->frameId().substr(1) << ":" << std::endl;

      file << "    translation:" << std::endl
           << "      x: " << pose.translation().x() << std::endl
           << "      y: " << pose.translation().y() << std::endl
           << "      z: " << pose.translation().z() << std::endl;

      Quaternion rotation(pose.rotation());
      file << "    rotation:" << std::endl
           << "      x: " << rotation.x() << std::endl
           << "      y: " << rotation.y() << std::endl
           << "      z: " << rotation.z() << std::endl
           << "      w: " << rotation.w() << std::endl;

    }
  }
  file.close();

  ROS_INFO_STREAM(file_name << " created!");

  return true;

}

} /* namespace ct_calibration */
