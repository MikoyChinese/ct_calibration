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

#ifndef CT_APRILTAG_VISUAL_H
#define CT_APRILTAG_VISUAL_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz/helpers/color.h>
#include <ct_calibration/Apriltag.h>
#include <ct_calibration/Apriltags.h>

namespace ct_calibration
{

const rviz::Color RED = {1, 0, 0};
const rviz::Color GREEN = {0, 1, 0};
const rviz::Color BLUE = {0, 0, 1};
const rviz::Color CYAN = {0, 1, 1};
const rviz::Color MAGENTA = {1, 0, 1};
const rviz::Color YELLOW = {1, 1, 0};

class ApriltagVisualizer {
 public:
  ApriltagVisualizer(const ros::NodeHandle& nh, const std::string& topic)
      : nh_(nh),
        pub_markers_(nh_.advertise<visualization_msgs::MarkerArray>(topic, 1)) {
  }

  void SetColor(const rviz::Color& color) {
    color_.r = color.r_;
    color_.g = color.g_;
    color_.b = color.b_;
  }
  void SetAlpha(double alpha) { color_.a = alpha; }

  void PublishApriltagsMarker(const aprilslam::Apriltags& apriltags);
  void PublishApriltagsMarker(const std::vector<aprilslam::Apriltag>& tags,
                              const std::string& frame_id,
                              const ros::Time& stamp);

 private:
  ros::NodeHandle nh_;
  ros::Publisher pub_markers_;
  std_msgs::ColorRGBA color_;
};

}  // namespace aprilslam

#endif /* CT_APRILTAG_VISUAL_H */
