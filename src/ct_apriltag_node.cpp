// Copyright 2019 Mikoy Chinese.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Mikoy Chinese [mikoychinese@gmail.com]
// Github: https://github.com/MikoyChinese
// Reference: [https://github.com/OpenPTrack/open_ptrack_v2/tree/master/opt_calibration]
/////////////////////////////////////////////////////////////////
#include <fstream>
#include <omp.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <ceres/ceres.h>
#include <pcl/filters/filter_indices.h>
#include <pcl/filters/random_sample.h>

#include "apriltag_ros/common_functions.h"
#include "common/homography.h"
#include "tagStandard52h13.h"
#include "tagStandard41h12.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"

#include <ct_calibration/ct_apriltag_node.h>

namespace ct_calibration
{

CTApriltagNode::CTApriltagNode(cont ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle)
{
    action_sub_ = node_handle_.subscribe("action", 1, &CTApriltagNode::actionCallback, this);
    status_pub_ = node_handle_.advertise<ct_calibration::CalibrationStatus>("status", 1);
    marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("markers", 0);
    node_handle_.param("num_sensors", num_sensors_, 0);

    // AprilTag parameters.
    bool is_ok = true;
    is_ok = is_ok and node_handle_.param("tag_rows", tag_rows_);
    is_ok = is_ok and node_handle_.param("tag_cols", tag_cols_);
    is_ok = is_ok and node_handle_.param("tag_size", tag_size_);
    is_ok = is_ok and node_handle_.param("tag_space", tag_space_);
    if (not is_ok)
    {
        ROS_FATAL("[Error]: AprilTag paramters are missing! Check tag_rows, tag_cols, tag_size, tag_space.");
    }

    apriltag_ = boost::make_shared<Checkerboard>(tag_cols_, tag_rows_, tag_size_, tag_size_);
    apriltag_->setFrameId("/apriltag");

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

    // Define the tag family whose tags should be searched for in the camera images.
    if (tag_family_ == "tagStandard52h13")
    {
      tf_ = tagStandard52h13_create();
    }
    else if (tag_family_ == "tagStandard41h12")
    {
      tf_ = tagStandard41h12_create();
    }
    else if (tag_family_ == "tag36h11")
    {
      tf_ = tag36h11_create();
    }
    else if (tag_family_ == "tag25h9")
    {
      tf_ = tag25h9_create();
    }
    else if (tag_family_ == "tag16h5")
    {
      tf_ = tag16h5_create();
    }
    else
    {
      ROS_WARN("[Error]: Invalid tag family specified! Aborting");
      exit(1);
    }

    // Create the AprilTag 2 detector
    td_ = apriltag_detector_create();
    // tf_>black_border(2);
    apriltag_detector_add_family(td_, tf_);
    td_->quad_decimate = (float)tag_decimate_;
    td_->quad_sigma = (float)tag_blur_;
    td_->nthreads = tag_threads_;
    td_->debug = tag_debug_;
    td_->refine_edges = tag_refine_edges_;

    detections_ = NULL;
}

// Destructor
CTApriltagNode::~CTApriltagNode()
{
    // free memory associated with tag detector
    apriltag_detector_destroy(td_);

    // Free memory associated with the array of tag detections
    apriltag_detections_destroy(detections_);

    // free memory associated with tag family
    if (family_ == "tagStandard52h13")
    {
      tagStandard52h13_destroy(tf_);
    }
    else if (family_ == "tagStandard41h12")
    {
      tagStandard41h12_destroy(tf_);
    }
    else if (family_ == "tag36h11")
    {
      tag36h11_destroy(tf_);
    }
    else if (family_ == "tag25h9")
    {
      tag25h9_destroy(tf_);
    }
    else if (family_ == "tag16h5")
    {
      tag16h5_destroy(tf_);
    }
}


bool CTApriltagNode::initialize()
{
    bool all_messages_received = false;
    ros::Rate rate(1.0);
    while (ros::ok() and not all_messages_received)
    {
        ros::spinOnce();
        all_messages_received = true;
        for (size_t i = 0; all_messages_received and i < pinhole_vec_.size(); ++ i)
            all_messages_received = pinhole_vec_[i]->hasNewMessages();
        if (not all_messages_received)
            ROS_WARN_THROTTLE(5, "[Warn]: Not all sensors connected. Waiting...")
        rate.sleep();
    }

    ROS_INFO("[Info] All sensors connected!");

    calibration_ = boost::make_shared<CTCalibration>(node_handle_);

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

void CTApriltagNode::actionCallback(const std_msgs::String::ConstPtr & msg)
{
    if (msg->data == "save" or msg->data == "saveExtrinsicCalibration")
    {
        ROS_INFO("[Info] Saving calibration results...");
        calibration_->optimize();
        save();
    }
    else
    {
        ROS_ERROR_STREAM("[Error]: Unknown action: " << msg->data);
    }
}

void CTApriltagNode::spin()
{
    ros::Rate rate(5.0);

    while (ros::ok())
    {
        ros::spinOnce();
        calibration_.nextAcquisition();

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
                    cv::Mat image;
                    cv::cvtColor(data->image, image, cv::CV_BGR2GRAY);
                    if (analyzeData(device->sensor(), data->image, cb_view))
                    {
                        calibration_.addData(device->sensor(), cb_view);
                        images_acquired_map_[device->frameId]++;
                        ROS_INFO_STREAM("[" << device->frameId() << "] apriltag detected");
                        ++count;
                    }

                }
            }
        }
        catch (cv_bridge::Exception & ex)
        {
            ROS_ERROR_STREAM("[Error]: cv_bridge exception: " << ex.what());
            return;
        }
        catch (std::runtime_error & ex)
        {
            ROS_ERROR_STREAM("[Error]: Runtime exception: " << ex.what());
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
            ROS_INFO("-----------------------------------------------")
        rate.sleep();
    }
}

bool CTApriltagNode::save()
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


void CTApriltagNode::getTagParams(cont ros::NodeHandle & node_handle)
{
    // Get all parameters of AprilTag.
    tag_family_ = getAprilTagOption<std::string>(node_handle_, "tag_family", "tag36h11");
    tag_threads_ = getAprilTagOption<int>(node_handle_, "tag_threads", 4);
    tag_decimate_ = getAprilTagOption<double>(node_handle_, "tag_decimate", 1.0);
    tag_blur_ = getAprilTagOption<double>(node_handle_, "tag_blur", 0.0);
    tag_refine_edges_ = getAprilTagOption<int>(node_handle_, "tag_refine_edges", 1);
    tag_debug_ = getAprilTagOption<int>(node_handle_, "tag_debug", 0);
}

bool CTApriltagNode::analyzeData(const cb::PinholeView::Ptr & color_sensor,
                                 const cv::Mat & gray_image,
                                 CheckerboardView::Ptr & color_cb_view)
{
    cb::PinholeView<cb::Checkboard>::Ptr color_view;
    cb::Checkerboard::Ptr extracted_apriltag;
    if (detectTags(gray_image, color_sensor, color_view, extracted_apriltag))
    {
        visualization_msgs::Marker apriltag_marker;
        apriltag_marker.ns = "apriltag";
        apriltag_marker.id = node_map_[color_sensor]->id();
        extracted_apriltag->toMarker(apriltag_marker);
        marker_pub_.publish(apriltag_marker);

        geometry_msgs::TransformStamped transform_msg;
        extracted_apriltag->toTF(transform_msg);
        tf_pub_.sendTransform(transform_msg);

        color_cb_view = boost::make_shared<CheckerboardView>(color_view, extracted_apriltag, extracted_apriltag->center(), false);

        return true;

    }
    return false;
}

bool CTApriltagNode::detectTags(cv::Mat gray_image,
                                cb::PinholeSensor::Ptr & color_sensor,
                                boost::shared_ptr<cb::PinholeView<cb::Checkerboard> > & color_view,
                                boost::shared_ptr<cb::Checkerboard> & extracted_apriltag) const
{
    if (detections_)
    {
        apriltag_detections_destroy(detections_);
        detections_ = NULL;
    }

    image_u8_t apriltag_image = { .width = gray_image.cols,
                                  .height = gray_image.rows,
                                  .stride = gray_image.cols,
                                  .buf = gray_image.data};
    detections_ = apriltag_detector_detect(td_, &apriltag_image);

    // Only use first detection;s corners.
    if (detections_.size() > 0)
    {
        std::vector<cv::Point2f> corners;
        corners = detections_[0]->p;

        std::stringstream ss;
        ss << "view_" << color_sensor_->frameId().substr(1);

        color_view = boost::make_shared<cb::PinholeView<cb::Checkerboard> >();
        color_view->setId(ss.str());
        color_view->setObject(apriltag_);
        color_view->setPoints(corners);
        color_view->setSensor(color_sensor);

        extracted_apriltag = boost::make_shared<cb::Checkboard>(*color_view);
        return true;
    }
    return false;
}

}
