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

#ifndef FAST_RTPS_TEST_MESSAGENODE_H
#define FAST_RTPS_TEST_MESSAGENODE_H
#include <memory>
#include <functional>

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

template<class T>
class SubListener : public eprosima::fastrtps::SubscriberListener {
public:
    SubListener() : n_matched(0), n_msg(0) {};

    ~SubListener() final = default;

    /**
     * Match Paticipate (When has connection or connection's interrupte )  (Base Function)
     * */
    void onSubscriptionMatched(eprosima::fastrtps::Subscriber *sub, eprosima::fastrtps::rtps::MatchingInfo &info) final;

    /**
     * New Data (When new data comming) (Base Function)
     * */
    void onNewDataMessage(eprosima::fastrtps::Subscriber *sub) final;

    inline void
    setNewDataMessageCallback(std::function<void(GUID_t, T)> callback) { m_new_data_message_cb = std::move(callback); }

    inline void
    setSubscriptionMatchCallback(std::function<void(GUID_t, bool)> callback) { m_subscription_cb = std::move(callback); }

    eprosima::fastrtps::SampleInfo_t m_info;
    int n_matched;
    int n_msg;
    std::function<void(GUID_t, T)> m_new_data_message_cb;
    std::function<void(GUID_t, bool)> m_subscription_cb;
};

class PubListener : public eprosima::fastrtps::PublisherListener {
public:
    PubListener() : n_matched(0) {};

    ~PubListener() final = default;

    void onPublicationMatched(eprosima::fastrtps::Publisher *pub, eprosima::fastrtps::rtps::MatchingInfo &info) final;

    inline void
    setPublicationMatchCallback(std::function<void(GUID_t, bool)> callback) { m_publication_cb = std::move(callback); }

    int n_matched;
    std::function<void(GUID_t, bool)> m_publication_cb;
};

class MessageNode  {
public:
    MessageNode();
    virtual ~MessageNode();

    virtual bool init() {}
    GUID_t getGUID();

    void connectPartner(GUID_t guid, bool connect);
protected:
    eprosima::fastrtps::Participant *mp_participant;
    eprosima::fastrtps::Publisher *mp_publisher;
    eprosima::fastrtps::Subscriber *mp_subscriber;

private:
    PubListener m_pub_listener;
};

#endif //FAST_RTPS_TEST_MESSAGENODE_H
