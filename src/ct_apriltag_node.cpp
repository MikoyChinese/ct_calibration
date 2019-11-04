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
                    cv::Mat image;
                    cv::cvtColor(data->image, image, cv::CV_BGR2GRAY);


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

AprilTagDetectionArray CTApriltagNode::detectTags(const cv::Mat image)
{
    if (detections_)
    {
        apriltag_detections_destroy(detections_);
        detections_ = NULL;
    }

    image_u8_t apriltag_image = { .width = image.cols,
                                  .height = image.rows,
                                  .stride = image.cols,
                                  .buf = image.data};
    detections_ = apriltag_detector_detect(td_, &apriltag_image);
    // Compute the estimated translation and rotation individually for each
    // detected tag
    AprilTagDetectionArray tag_detection_array;
    std::vector<std::string > detection_names;
    tag_detection_array.header = device->lastMessages().image_msg->header;
    std::map<std::string, std::vector<cv::Point3d > > bundleObjectPoints;
    std::map<std::string, std::vector<cv::Point2d > > bundleImagePoints;

    for (int i=0; i < zarray_size(detections_); i++)
    {
        apriltag_detection_t *detection;
        zarray_get(detections_, i, &detection);
        int tagID = detection->id;
        bool is_part_of_bundle = false;
        for (unsigned int j=0; j<tag_bundle_descriptions_.size(); j++)
        {
          // Iterate over the registered bundles
          TagBundleDescription bundle = tag_bundle_descriptions_[j];

          if (bundle.id2idx_.find(tagID) != bundle.id2idx_.end())
          {
            // This detected tag belongs to the j-th tag bundle (its ID was found in
            // the bundle description)
            is_part_of_bundle = true;
            std::string bundleName = bundle.name();

            //===== Corner points in the world frame coordinates
            double s = bundle.memberSize(tagID)/2;
            addObjectPoints(s, bundle.memberT_oi(tagID),
                            bundleObjectPoints[bundleName]);

            //===== Corner points in the image frame coordinates
            addImagePoints(detection, bundleImagePoints[bundleName]);
          }
        }

        StandaloneTagDescription* standaloneDescription;
        if (!findStandaloneTagDescription(tagID, standaloneDescription, !is_part_of_bundle)
            continue;

        double tag_size = standaloneDescription->size();
        // Get estimated tag pose in the camera frame.
        //
        // Note on frames:
        // The raw AprilTag 2 uses the following frames:
        //   - camera frame: looking from behind the camera (like a
        //     photographer), x is right, y is up and z is towards you
        //     (i.e. the back of camera)
        //   - tag frame: looking straight at the tag (oriented correctly),
        //     x is right, y is down and z is away from you (into the tag).
        // But we want:
        //   - camera frame: looking from behind the camera (like a
        //     photographer), x is right, y is down and z is straight
        //     ahead
        //   - tag frame: looking straight at the tag (oriented correctly),
        //     x is right, y is up and z is towards you (out of the tag).
        // Using these frames together with cv::solvePnP directly avoids
        // AprilTag 2's frames altogether.
        // TODO solvePnP[Ransac] better?

        std::vector<cv::Point3d > standaloneTagObjectPoints;
        std::vector<cv::Point2d > standaloneTagImagePoints;
        addObjectPoints(tag_size/2, cv::Matx44d::eye(), standaloneTagObjectPoints);
        addImagePoints(detection, standaloneTagImagePoints);
    }
}

}
