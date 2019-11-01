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

#include <ct_calibration/CalibrationStatus.h>
#include <ct_calibration/ros_device.h>

#include <apriltag.h>

using namespace calibration;

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
     * @brief Callback for string messages which enables saving options.
     * @param[in] msg Message containing the command as a string.
     */
     void actionCallback(const std_msgs::String::ConstPtr & msg);

    /**
     *  @brief Apriltag initialization.
     */
    bool initialize();

private:

    ros::NodeHandle node_handle_;
    image_transport::ImageTransport image_transport_;

    ros::Subscriber action_sub_;
    ros::Publisher status_sub_;
    std::map<std::string, int> images_acquired_map_;
    ct_calibration::CalibrationStatus status_msg_;

    std::vector<PinholeRGBDevice::Ptr> pinhole_vec_;
    std::vector<Sensor::Ptr> sensor_vec_;
    int num_sensors_;    // @brief Number of sensors.

    // AprilTag 2 code's attributes.
    std::string tag_family_;
    int tag_threads_;
    double tag_decimate_;
    double tag_blur_;
    int tag_refine_edges_;
    int tag_debug_;

    int tag_cols_;
    int tag_rows_;
    double tag_size_;
    double tag_space_;

    // AprilTag 2 objects
    apriltag_family_t *tf_;
    apriltag_detector_t *td_;
    zarray_t *detections_;

    void getTagParams(const ros::NodeHandle & node_handle);
} /* class CTApriltagNode */

} /* namespace ct_calibration */

#endif /* CT_APRILTAG_NODE_H */
