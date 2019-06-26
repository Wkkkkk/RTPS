/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by Weicheng Xiong on 9/28/18.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdlib>

#include <tf/tf.h>
#include <std_msgs/Bool.h>
#include <automsgs/lane.h>
#include <automsgs/SimCar.h>
#include <automsgs/CloudClusterArray.h>
#include <automsgs/ConfigWaypointFollower.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <include/RTPSNode.h>


#include "include/DataStructure.h"
#include "include/UserDefinedMessageNode.h"

using namespace AutoAI::communication;

RTPSNode::RTPSNode() :
        status_has_pos(false),
        status_has_twist(false),
        status_has_ctrl(false) {

    statusMessageNode.init();
    finalWayPointsMessageNode.init();
    egoInfoMessageNode.init();

    way_planner_sub = nodeHandle.subscribe("/lane_waypoints_array", 10, &RTPSNode::wayPlannerCallback, this);
    final_waypoints_sub = nodeHandle.subscribe("/final_waypoints", 10, &RTPSNode::finalWaypointsCallback, this);

    current_pos_sub = nodeHandle.subscribe("current_pose", 10, &RTPSNode::callbackGetCurrentPos, this);
    local_traj_sub = nodeHandle.subscribe("local_trajectories", 1, &RTPSNode::callbackLocalTrajs, this);

    twist_sub = nodeHandle.subscribe("twist_raw", 10, &RTPSNode::callbackGetTwistRaw, this);
    ctrl_sub = nodeHandle.subscribe("ctrl_cmd", 10, &RTPSNode::callbackGetCtrlCmd, this);

    state_sub = nodeHandle.subscribe("behavior_state", 1, &RTPSNode::callbackGetState, this);

    target_point_sub = nodeHandle.subscribe("next_target_point", 1, &RTPSNode::callbackGetTargetPoint, this);
    target_point_sub2 = nodeHandle.subscribe("follow_pose", 1, &RTPSNode::callbackGetTargetPoint2, this);

    ego_info_sub = nodeHandle.subscribe("V2V_ego_info", 1, &RTPSNode::callbackGetEgoInfo, this);
    nearby_info_pub  =nodeHandle.advertise<automsgs::V2V_nearby_info>("V2V_nearby_info", 1);
}

RTPSNode::~RTPSNode() { }

void RTPSNode::run() {
    ros::Rate loop_rate(50);

    while (ros::ok()) {
        ros::spinOnce();

        //do something here
        Singleton<MapEgoInfo>::getInstance()->print();

        std::vector<MapEgoInfo> infos = Singleton<MapEgoInfo>::getInstance()->findByFilter([](const MapEgoInfo& mapEgoInfo) { return true;});
        if(!infos.empty()) {
            //print debug

            std::vector<automsgs::V2V_ego_info> ego_infos;
            for(const auto& info : infos) {
                automsgs::V2V_ego_info ego_info;
                ego_info.current_lane_id = info.current_lane_id;
                ego_info.current_node_id = info.current_node_id;
                ego_info.current_car_id = info.current_car_id;
                //position
                ego_info.pose.pose.position.x = info.pose.position.x;
                ego_info.pose.pose.position.y = info.pose.position.y;
                ego_info.pose.pose.position.z = info.pose.position.z;
                //orientation(quaternion)
                ego_info.pose.pose.orientation.x = info.pose.orientation.x;
                ego_info.pose.pose.orientation.y = info.pose.orientation.y;
                ego_info.pose.pose.orientation.z = info.pose.orientation.z;
                ego_info.pose.pose.orientation.w = info.pose.orientation.w;

                ego_info.velocity = info.velocity;

                ego_info.acceleration = info.acceleration;

                ego_infos.emplace_back(std::move(ego_info));
            }

            automsgs::V2V_nearby_info nearby_info;
            nearby_info.infos = ego_infos;

            //publish here
            nearby_info_pub.publish(nearby_info);
        }

        //
        loop_rate.sleep();
    }
}

void RTPSNode::wayPlannerCallback(const automsgs::LaneArrayConstPtr &msg) {
    MapLanes lanes;
    for (auto lane : msg->lanes) {
        MapLane mapLane;
        mapLane.lane_id = lane.lane_id;
        for (auto waypoint : lane.waypoints) {
            MapPose mapPose;
            mapPose.orientation = MapVec4(waypoint.pose.pose.orientation.x, waypoint.pose.pose.orientation.y,
                                          waypoint.pose.pose.orientation.z, waypoint.pose.pose.orientation.w);
            mapPose.position = MapVec3(waypoint.pose.pose.position.x, waypoint.pose.pose.position.y,
                                       waypoint.pose.pose.position.z);
            mapLane.points.push_back(mapPose);
        }
        lanes.lanes.push_back(mapLane);
    }
}

void RTPSNode::finalWaypointsCallback(const automsgs::laneConstPtr &msg) {
    MapLane finalWaypoint;
    finalWaypoint.lane_id = msg->lane_id;
    for (const auto &waypoint : msg->waypoints) {
        MapPose vPose;
        vPose.position.x = waypoint.pose.pose.position.x;
        vPose.position.y = waypoint.pose.pose.position.y;
        vPose.position.z = waypoint.pose.pose.position.z;
        vPose.orientation.x = waypoint.twist.twist.linear.x; // velocity
        vPose.orientation.y = waypoint.twist.twist.linear.y;
        vPose.orientation.z = waypoint.twist.twist.linear.z;
        finalWaypoint.points.push_back(vPose);
    }

    finalWayPointsMessageNode.publishMessage(finalWaypoint);
}

void RTPSNode::callbackGetCurrentPos(const geometry_msgs::PoseStampedConstPtr &msg) {
    MapPose current_pose;
    current_pose.position = MapPosition(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    current_pose.orientation = MapOrientation(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
                                              msg->pose.orientation.w);
//    Q_EMIT emitCurrentPose(current_pose);

    car_status.pose = current_pose;
    status_has_pos = true;
    if (status_has_pos && status_has_twist && status_has_ctrl) {
        statusMessageNode.publishMessage(car_status);

        status_has_pos = false;
        status_has_twist = false;
        status_has_ctrl = false;
    }
}

void RTPSNode::callbackLocalTrajs(const visualization_msgs::MarkerArrayConstPtr &msg) {
    MapMarkers markers;
    for (auto marker : msg->markers) {
        MapMarker mk;
        mk.colors = MapColor(marker.color.r, marker.color.g, marker.color.b);
        for (auto point : marker.points) {
            mk.points.push_back(MapPosition(point.x, point.y, point.z));
        }
        markers.markers.push_back(mk);
    }
}

void RTPSNode::callbackGetCtrlCmd(const automsgs::ControlCommandStampedConstPtr &msg) {
    car_status.steering_angle = msg->cmd.steering_angle;
    car_status.linear_velocity = msg->cmd.linear_velocity;
    car_status.linear_acceleration = msg->cmd.linear_acceleration;
    status_has_twist = true;
}

void RTPSNode::callbackGetTwistRaw(const geometry_msgs::TwistStampedConstPtr &msg) {
    car_status.linear_velocity = msg->twist.linear.x;
    car_status.angular_velocity = msg->twist.angular.z;
    status_has_ctrl = true;
}

void RTPSNode::callbackGetState(const visualization_msgs::Marker &msg) {
}

void RTPSNode::callbackGetTargetPoint2(const visualization_msgs::Marker &msg) {
    MapPosition pos;
    pos.x = msg.pose.position.x;
    pos.y = msg.pose.position.y;
    pos.z = msg.pose.position.z;
}

void RTPSNode::callbackGetTargetPoint(const geometry_msgs::Point &msg) {
    MapPosition pos;
    pos.x = msg.x;
    pos.y = msg.y;
    pos.z = msg.z;
}

void RTPSNode::callbackGetEgoInfo(const automsgs::V2V_ego_info &msg) {
    MapEgoInfo egoInfo;
    egoInfo.current_lane_id = msg.current_lane_id;
    egoInfo.current_node_id = msg.current_node_id;
    egoInfo.current_car_id  = msg.current_car_id;
    auto& position = msg.pose.pose.position;
    egoInfo.pose.position = MapVec3(position.x, position.y, position.z);
    auto& orientation = msg.pose.pose.orientation;
    egoInfo.pose.orientation = MapVec4(orientation.x, orientation.y, orientation.z, orientation.w);
    egoInfo.acceleration = msg.acceleration;
    egoInfo.velocity = msg.velocity;

    egoInfoMessageNode.publishMessage(egoInfo);
}
