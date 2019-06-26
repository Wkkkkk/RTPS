/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 1/17/19.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef AUTOPLANNING_RTPSNODE_H
#define AUTOPLANNING_RTPSNODE_H

#include <string>
#include <ros/ros.h>

#include <std_msgs/UInt64.h>
#include <automsgs/lane.h>
#include <automsgs/LaneArray.h>
#include <automsgs/V2V_ego_info.h>
#include <automsgs/V2V_nearby_info.h>
#include <automsgs/ControlCommandStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <vector_map/vector_map.h>
#include <vector_map_msgs/PointArray.h>
#include <vector_map_msgs/LaneArray.h>
#include <vector_map_msgs/NodeArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include "messages/common.h"
#include "include/UserDefinedMessageNode.h"

using eprosima::fastrtps::rtps::GUID_t;

namespace AutoAI {
    namespace communication {
        class RTPSNode {
        public:
            RTPSNode();
            ~RTPSNode();

            void run();
        private:
            ros::NodeHandle nodeHandle;
            StatusMessageNode statusMessageNode;
            FinalWayPointsMessageNode finalWayPointsMessageNode;
            EgoInfoMessageNode egoInfoMessageNode;

            // ----------------------------------------- Subscriber ------------------------------------- //
            ros::Subscriber way_planner_sub;        // "/lane_waypoints_array"
            ros::Subscriber final_waypoints_sub;    // "/final_waypoints"
            ros::Subscriber twist_sub;              // "twist_raw"
            ros::Subscriber ctrl_sub;               // "ctrl_cmd"
            ros::Subscriber current_pos_sub;       // "current_pose"
            ros::Subscriber current_velocity_sub;  // "current_velocity"
            ros::Subscriber local_traj_sub;        // "local_trajectories"
            ros::Subscriber state_sub;
            ros::Subscriber target_point_sub;
            ros::Subscriber target_point_sub2;
            ros::Subscriber ego_info_sub;

            ros::Publisher nearby_info_pub;

            void wayPlannerCallback(const automsgs::LaneArrayConstPtr &msg);

            void finalWaypointsCallback(const automsgs::laneConstPtr &msg);

            void callbackGetCurrentPos(const geometry_msgs::PoseStampedConstPtr &msg);

            void callbackLocalTrajs(const visualization_msgs::MarkerArrayConstPtr &msg);

            void callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr &msg);

            void callbackGetCtrlCmd(const automsgs::ControlCommandStampedConstPtr &msg);

            void callbackGetState(const visualization_msgs::Marker &msg);

            void callbackGetTargetPoint2(const visualization_msgs::Marker &msg);

            void callbackGetTargetPoint(const geometry_msgs::Point &msg);

            void callbackGetEgoInfo(const automsgs::V2V_ego_info &msg);

        private:
            MapStatus car_status;
            bool status_has_pos;
            bool status_has_twist;
            bool status_has_ctrl;
        };
    }
}

#endif //AUTOPLANNING_RTPSNODE_H
