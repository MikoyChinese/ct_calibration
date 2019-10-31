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

#include <ct_calibration/ct_apriltag_detector.h>

namespace ct_calibration
{

CTApriltagNode::CTApriltagNode(cont ros::NodeHandle & node_handle)
  : node_handle_(node_handle),
    image_transport_(node_handle)
{
    // Get all parameters of AprilTag.
    tag_family_ = getAprilTagOption<std::string>(node_handle, "tag_family", "tag36h11");
    tag_threads_ = getAprilTagOption<int>(node_handle, "tag_threads", 4);
    tag_decimate_ = getAprilTagOption<double>(node_handle, "tag_decimate", 1.0);
    tag_blur_ = getAprilTagOption<double>(node_handle, "tag_blur", 0.0);
    tag_refine_edges_ = getAprilTagOption<int>(node_handle, "tag_refine_edges", 1);
    tag_debug_ = getAprilTagOption<int>(node_handle, "tag_debug", 0);

    // Parse standalone tag descriptions specified by user.
    XmlRpc::XmlRpcValue standalone_tag_descriptions;
    if(!pnh.getParam("standalone_tags", standalone_tag_descriptions))
    {
      ROS_WARN("[Error]: No april tags specified");
    }
    else
    {
      try
      {
        standalone_tag_descriptions_ =
            parseStandaloneTags(standalone_tag_descriptions);
      }
      catch(XmlRpc::XmlRpcException e)
      {
        // in case any of the asserts in parseStandaloneTags() fail
        ROS_ERROR_STREAM("[Error]: loading standalone tag descriptions: " <<
                         e.getMessage().c_str());
      }
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
    
}

}
