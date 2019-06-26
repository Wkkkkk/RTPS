/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 1/9/19.
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

#ifndef FAST_RTPS_TEST_USERDEFINEDMESSAGENODE_H
#define FAST_RTPS_TEST_USERDEFINEDMESSAGENODE_H

#include "MessageNode.h"
#include "messages/common.h"

using AutoAI::interface::MapStatus;

class StatusMessageNode : public MessageNode {
public:
    StatusMessageNode() = default;

    ~StatusMessageNode() final = default;

    bool init() final;

public:
    void receiveMessage(GUID_t guid, MapStatus mapStatus);
    void publishMessage(MapStatus mapStatus);

private:
    PubListener m_pub_listener;
    SubListener<MapStatus> m_sub_listener;

    MapStatusPubSubType myType;
};

class FinalWayPointsMessageNode : public MessageNode {
public:
    FinalWayPointsMessageNode() = default;

    ~FinalWayPointsMessageNode() final = default;

    bool init() final;

public:
    void receiveMessage(GUID_t guid, MapLane mapLane);
    void publishMessage(MapLane mapLane);

private:
    PubListener m_pub_listener;
    SubListener<MapLane> m_sub_listener;

    MapLanePubSubType myType;
};


class EgoInfoMessageNode : public MessageNode {
public:
    EgoInfoMessageNode() = default;

    ~EgoInfoMessageNode() final = default;

    bool init() final;

public:
    void receiveMessage(GUID_t guid, MapEgoInfo mapEgoInfo);
    void publishMessage(MapEgoInfo mapEgoInfo);

private:
    PubListener m_pub_listener;
    SubListener<MapEgoInfo> m_sub_listener;

    MapEgoInfoPubSubType myType;
};

#endif //FAST_RTPS_TEST_USERDEFINEDMESSAGENODE_H
