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

#include <ros/ros.h>
#include <ct_calibration/ct_checkerboard_node.h>
#include <tagslam/tag_slam.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ct_calibration_multi");
    ros::NodeHandle node_handle("~");

    try
    {
        std::string calibration_type;
        node_handle.param("calibration_type", calibration_type, std::string("apriltag"));

        if (calibration_type == "apriltag")
        {
            tagslam::TagSlam apriltag_node(node_handle);
            if (apriltag_node.initialize())
            {
                if (apriltag_node.runOnline())
                {
                    apriltag_node.subscribe();
                    ros::spin();
                }
            }
            else
            {
                ROS_ERROR_STREAM("[Error]: Initialize apriltag_node failed." );
            }
        }
        else if (calibration_type == "checkerboard")
        {
            ct_calibration::CTCheckerboardNode checkerboard_node(node_handle);
            if (not checkerboard_node.initialize())
                return 1;
            checkerboard_node.spin();
        }
        else
        {
            ROS_FATAL_STREAM(calibration_type << " is not supported yet. " << "Calibration type supports apriltag, checkerboard.");
        }

    }
    catch(std::runtime_error & error)
    {
        ROS_FATAL("[Error]: Calibration error: %s", error.what());
        return 2;
    }

    return 0;
}
