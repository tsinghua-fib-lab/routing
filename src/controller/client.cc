/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-23 8:33:57 pm
 */

#include "controller/client.h"
#include <functional>
#include <string>
#include <utility>
#include <etcd/Response.hpp>

namespace routing {

namespace controller {

Client::Client(const std::string& etcd_uri) : etcd_(etcd_uri) {}

void Client::Listen(
    const std::string& key,
    std::function<void(std::string data, int64_t version)> callback) {
  ListenImpl(key, callback, 1);
}

void Client::ListenImpl(
    std::string key,
    std::function<void(std::string data, int64_t version)> callback,
    int64_t revision) {
  etcd_.watch(key, revision, false)
      .then([this, key, callback](etcd::Response res) {
        for (const auto& e : res.events()) {
          if (e.has_kv()) {
            callback(e.kv().value(), e.kv().mod_revision());
          }
        }
        ListenImpl(std::move(key), callback, res.index() + 1);
      });
}

}  // namespace controller

}  // namespace routing
