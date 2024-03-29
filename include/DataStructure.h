/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 1/8/19.
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

#ifndef FAST_RTPS_TEST_DATASTRUCTURE_H
#define FAST_RTPS_TEST_DATASTRUCTURE_H

#include <map>
#include <mutex>
#include <memory>
#include <vector>
#include <functional>
#include <thread>
#include <algorithm>
#include <iostream>

#include <fastrtps/rtps/common/Guid.h>

using eprosima::fastrtps::rtps::GUID_t;

template<class T>
class Key {
private:
    GUID_t id_;

public:
    Key() = default;

    explicit Key(GUID_t id)
            : id_(std::move(id)) {
    }

    void setId(GUID_t id) {
        id_ = id;
    }

    GUID_t getId() const {
        return id_;
    }

    bool operator<(const Key<T> &right) const {
        return id_ < right.getId();
    }

    bool operator==(const Key<T> &right) const {
        return id_ == right.getId();
    }
};

template<class T>
using Filter = std::function<bool(const T &)>;

template<class T>
class Handle {
private:
    std::map<Key<T>, T> map_;

public:
    Handle() = default;

    void update(const Key<T> &key, const T &t) {
        map_[key] = t;

        //C++17
        //auto [it, inserted] = map_.insert_or_assign(key, t);
    }

    void remove(const Key<T> &key) {
        map_.erase(key);
    }

    T findByKey(const Key<T> &key) const {
        auto it = map_.find(key);
        if (it == map_.end())
            return T();
        return it->second;
    }

    std::vector<T> findByFilter(const Filter<T> &filter) const {
        std::vector<T> vector;
        for (const auto &pair : map_) {
            if (filter(pair.second))
                vector.push_back(pair.second);
        }
        return vector;
    }

    bool empty() const {
        return map_.empty();
    }

    size_t size() const {
        return map_.size();
    }

    void print() const {
        for (const auto &pair : map_) {
            std::cout << "key: " << pair.first.getId() << " value: " << pair.second << std::endl;
        }
    }
};

template<class T>
class Singleton {
public:
    Singleton(const Singleton &) = delete;

    Singleton &operator=(const Singleton &) = delete;

    static std::shared_ptr<Singleton> getInstance() { return instance; }

    void update(const Key<T> &key, const T &obj) {
        handle.update(key, obj);
    }

    T findByID(const Key<T> &key) const {
        return handle.findByKey(key);
    }

    std::vector<T> findByFilter(const Filter<T> &filter) const {
        return handle.findByFilter(filter);
    }

    void remove(const Key<T> &key) {
        handle.remove(key);
    }

    void print() {
        handle.print();
    }

private:
    Singleton() = default;

    static std::shared_ptr<Singleton<T>> instance;

    Handle<T> handle;
};

#endif //FAST_RTPS_TEST_DATASTRUCTURE_H
