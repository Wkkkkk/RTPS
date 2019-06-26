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
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/Subscriber.h>

#include <fastrtps/Domain.h>

#include "include/UserDefinedMessageNode.h"
#include "include/DataStructure.h"

///StatusMessageNode
bool StatusMessageNode::init() {
    // Create RTPSParticipant
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 80;
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.setName("status_publisher");
    mp_participant = Domain::createParticipant(PParam);
    if (mp_participant == nullptr) {
        return false;
    }

    // Register the type
    Domain::registerType(mp_participant, (TopicDataType *) &myType);

    // Register callback before they are called!!
    std::function<void(GUID_t, MapStatus)> new_message_callback = std::bind(&StatusMessageNode::receiveMessage, this,
                                                                            std::placeholders::_1,
                                                                            std::placeholders::_2);
    m_sub_listener.setNewDataMessageCallback(new_message_callback);

    std::function<void(GUID_t, bool)> match_callback = std::bind(&MessageNode::connectPartner, this,
                                                                 std::placeholders::_1, std::placeholders::_2);
    m_sub_listener.setSubscriptionMatchCallback(match_callback);

    // Create Publisher
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = myType.getName();  //This type MUST be registered
    Wparam.topic.topicName = "status_topic";
    Wparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Wparam.topic.historyQos.depth = 20;
    Wparam.historyMemoryPolicy = DYNAMIC_RESERVE_MEMORY_MODE;
    Wparam.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;

    /**
     * Publisher bind with pub_listener
     * */
    mp_publisher = Domain::createPublisher(mp_participant, Wparam, (PublisherListener *) &m_pub_listener);
    if (mp_publisher == nullptr) return false;
//    std::cout << "Publisher created, waiting for Subscribers." << std::endl;

    // Create Subscriber
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = myType.getName();  //This type MUST be registered
    Rparam.topic.topicName = "status_topic";
    Rparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Rparam.topic.historyQos.depth = 20;
    Rparam.historyMemoryPolicy = DYNAMIC_RESERVE_MEMORY_MODE;
    Rparam.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;

    /**
     * Subscriber bind with sub_listener
     * */
    mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, (SubscriberListener *) &m_sub_listener);
    if (mp_subscriber == nullptr) return false;
//    std::cout << "Subscriber created, waiting for Publishers." << std::endl;
    return true;
}

void StatusMessageNode::publishMessage(MapStatus mapStatus) {
    static int msgsent = 0;

    mp_publisher->write(&mapStatus);
    ++msgsent;
//    std::cout << "StatusMessageNode sending sample, count=" << msgsent << std::endl;
}

void StatusMessageNode::receiveMessage(GUID_t guid, MapStatus mapStatus) {
    Singleton<MapStatus>::getInstance()->update(Key<MapStatus>(guid), mapStatus);
}

///FinalWayPointsMessageNode
bool FinalWayPointsMessageNode::init() {
    // Create RTPSParticipant
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 80;
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.setName("final_waypoints_publisher");
    mp_participant = Domain::createParticipant(PParam);
    if (mp_participant == nullptr) {
        return false;
    }

    // Register the type
    Domain::registerType(mp_participant, (TopicDataType *) &myType);

    // Register callback before they are called!!
    std::function<void(GUID_t, MapLane)> new_message_callback = std::bind(&FinalWayPointsMessageNode::receiveMessage,
                                                                          this,
                                                                          std::placeholders::_1, std::placeholders::_2);
    m_sub_listener.setNewDataMessageCallback(new_message_callback);

    std::function<void(GUID_t, bool)> match_callback = std::bind(&MessageNode::connectPartner, this,
                                                                 std::placeholders::_1, std::placeholders::_2);
    m_sub_listener.setSubscriptionMatchCallback(match_callback);

    // Create Publisher
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = myType.getName();  //This type MUST be registered
    Wparam.topic.topicName = "final_waypoints_topic";
    Wparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Wparam.topic.historyQos.depth = 20;
    Wparam.historyMemoryPolicy = DYNAMIC_RESERVE_MEMORY_MODE;
    Wparam.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;

    mp_publisher = Domain::createPublisher(mp_participant, Wparam, (PublisherListener *) &m_pub_listener);
    if (mp_publisher == nullptr) return false;
//    std::cout << "Publisher created, waiting for Subscribers." << std::endl;

    // Create Subscriber
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = myType.getName();  //This type MUST be registered
    Rparam.topic.topicName = "final_waypoints_topic";
    Rparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Rparam.topic.historyQos.depth = 20;
    Rparam.historyMemoryPolicy = DYNAMIC_RESERVE_MEMORY_MODE;
    Rparam.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;

    mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, (SubscriberListener *) &m_sub_listener);
    if (mp_subscriber == nullptr) return false;
//    std::cout << "Subscriber created, waiting for Publishers." << std::endl;

    return true;
}

void FinalWayPointsMessageNode::publishMessage(MapLane mapLane) {
    static int msgsent = 0;

    mp_publisher->write(&mapLane);
    ++msgsent;
}

void FinalWayPointsMessageNode::receiveMessage(GUID_t guid, MapLane mapLane) {
    Singleton<MapLane>::getInstance()->update(Key<MapLane>(guid), mapLane);
}

///EgoInfoMessageNode
bool EgoInfoMessageNode::init() {
    // Create RTPSParticipant
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 80;
    PParam.rtps.builtin.leaseDuration = c_TimeInfinite;
    PParam.rtps.setName("ego_info_publisher");
    mp_participant = Domain::createParticipant(PParam);
    if (mp_participant == nullptr) {
        return false;
    }

    // Register the type
    Domain::registerType(mp_participant, (TopicDataType *) &myType);

    // Register callback before they are called!!
    std::function<void(GUID_t, MapEgoInfo)> new_message_callback = std::bind(&EgoInfoMessageNode::receiveMessage,
                                                                          this,
                                                                          std::placeholders::_1, std::placeholders::_2);
    m_sub_listener.setNewDataMessageCallback(new_message_callback);

    std::function<void(GUID_t, bool)> match_callback = std::bind(&MessageNode::connectPartner, this,
                                                                 std::placeholders::_1, std::placeholders::_2);
    m_sub_listener.setSubscriptionMatchCallback(match_callback);

    // Create Publisher
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = myType.getName();  //This type MUST be registered
    Wparam.topic.topicName = "ego_info_topic";
    Wparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Wparam.topic.historyQos.depth = 20;
    Wparam.historyMemoryPolicy = DYNAMIC_RESERVE_MEMORY_MODE;
    Wparam.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;

    mp_publisher = Domain::createPublisher(mp_participant, Wparam, (PublisherListener *) &m_pub_listener);
    if (mp_publisher == nullptr) return false;
//    std::cout << "Publisher created, waiting for Subscribers." << std::endl;

    // Create Subscriber
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = myType.getName();  //This type MUST be registered
    Rparam.topic.topicName = "ego_info_topic";
    Rparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Rparam.topic.historyQos.depth = 20;
    Rparam.historyMemoryPolicy = DYNAMIC_RESERVE_MEMORY_MODE;
    Rparam.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;

    mp_subscriber = Domain::createSubscriber(mp_participant, Rparam, (SubscriberListener *) &m_sub_listener);
    if (mp_subscriber == nullptr) return false;
//    std::cout << "Subscriber created, waiting for Publishers." << std::endl;

    return true;
}

void EgoInfoMessageNode::publishMessage(MapEgoInfo mapEgoInfo) {
    static int msgsent = 0;

    mp_publisher->write(&mapEgoInfo);
    ++msgsent;
}

void EgoInfoMessageNode::receiveMessage(GUID_t guid, MapEgoInfo mapEgoInfo) {
    if(guid == getGUID())
    {
        return;
    }

    Singleton<MapEgoInfo>::getInstance()->update(Key<MapEgoInfo>(guid), mapEgoInfo);
}
