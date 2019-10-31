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

#ifndef CT_APRILTAG_NODE_H
#define CT_APRILTAG_NODE_H

#include <string>
#include <sstream>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/package.h>
#include <XmlRpcException.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <apriltag.h>

namespace ct_calibration
{

template<typename T>
T getAprilTagOption(ros::NodeHandle& pnh,
                    const std::string& param_name, const T & default_val)
{
    T param_val;
    pnh.param<T>(param_name, param_val, default_val);
    return param_val;
}

class CTApriltagNode
{
public:

    /**
     *  @brief Constructor
     */
    CTApriltagNode(const ros::NodeHandle & node_handle);

    /**
     *  @brief Apriltag initialization.
     */
    bool initialize();

private:

    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    // AprilTag 2 code's attributes.
    std::string tag_family_;
    int tag_threads_;
    double tag_decimate_;
    double tag_blur_;
    int tag_refine_edges_;
    int tag_debug_;

    // AprilTag 2 objects
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    zarray_t *detections_;
}

class StandaloneTagDescription
{
 public:
  StandaloneTagDescription() {};
  StandaloneTagDescription(int id, double size,
                           std::string &frame_name) :
      id_(id),
      size_(size),
      frame_name_(frame_name) {}

  double size() { return size_; }
  int id() { return id_; }
  std::string& frame_name() { return frame_name_; }

 private:
  // Tag description
  int id_;
  double size_;
  std::string frame_name_;
};

class TagBundleDescription
{
 public:
  std::map<int, int > id2idx_; // (id2idx_[<tag ID>]=<index in tags_>) mapping

  TagBundleDescription(std::string name) :
      name_(name) {}

  void addMemberTag(int id, double size, cv::Matx44d T_oi) {
    TagBundleMember member;
    member.id = id;
    member.size = size;
    member.T_oi = T_oi;
    tags_.push_back(member);
    id2idx_[id] = tags_.size()-1;
  }

  std::string name () const { return name_; }
  // Get IDs of bundle member tags
  std::vector<int> bundleIds () {
    std::vector<int> ids;
    for (unsigned int i = 0; i < tags_.size(); i++) {
      ids.push_back(tags_[i].id);
    }
    return ids;
  }
  // Get sizes of bundle member tags
  std::vector<double> bundleSizes () {
    std::vector<double> sizes;
    for (unsigned int i = 0; i < tags_.size(); i++) {
      sizes.push_back(tags_[i].size);
    }
    return sizes;
  }
  int memberID (int tagID) { return tags_[id2idx_[tagID]].id; }
  double memberSize (int tagID) { return tags_[id2idx_[tagID]].size; }
  cv::Matx44d memberT_oi (int tagID) { return tags_[id2idx_[tagID]].T_oi; }

 private:
  // Bundle description
  std::string name_;
  std::vector<TagBundleMember > tags_;
};

}

#endif /* CT_APRILTAG_NODE_H */
