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

#include <ct_calibration/ct_apriltag_node.h>

namespace ct_calibration
{

CTApriltagNode::CTApriltagNode(cont ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle)
{
    action_sub_ = node_handle_.subscribe("action", 1, &CTApriltagNode::actionCallback, this);
    status_pub_ = node_handle_.advertise<ct_calibration::CalibrationStatus>("status", 1);
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
            ROS_WARN_THROTTLE(5, "Not all sensors connected. Waiting...")
        rate.sleep();
    }

    ROS_INFO("All sensors connected!");

    calibration_ = boost::make_shared<CTCalibration>(node_handle_);

    for (size_t i = 0; i < pinhole_vec_.size(); ++i)
    {
        const PinholeRGBDevice::Ptr & device = pinhole_vec_[i];
        calibration_->addSensor(device->sensor(), true);
        sensor_vec_.push_back(device->sensor());
        
    }
}


void CTApriltagNode::getTagParams(cont ros::NodeHandle & node_handle)
{
    // status_flag
    bool is_ok = true;
    // Get all parameters of AprilTag.
    tag_family_ = getAprilTagOption<std::string>(node_handle_, "tag_family", "tag36h11");
    tag_threads_ = getAprilTagOption<int>(node_handle_, "tag_threads", 4);
    tag_decimate_ = getAprilTagOption<double>(node_handle_, "tag_decimate", 1.0);
    tag_blur_ = getAprilTagOption<double>(node_handle_, "tag_blur", 0.0);
    tag_refine_edges_ = getAprilTagOption<int>(node_handle_, "tag_refine_edges", 1);
    tag_debug_ = getAprilTagOption<int>(node_handle_, "tag_debug", 0);



}

}
