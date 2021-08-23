/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-23 8:33:48 pm
 */
#ifndef SRC_CONTROLLER_CLIENT_H_
#define SRC_CONTROLLER_CLIENT_H_

#include <functional>
#include <string>
#include <etcd/Client.hpp>

namespace routing {

namespace controller {

class Client {
 public:
  explicit Client(const std::string& etcd_uri);

  void Listen(const std::string& key,
              std::function<void(std::string data, int64_t revision)> callback);

 private:
  void ListenImpl(
      std::string key,
      std::function<void(std::string data, int64_t revision)> callback,
      int64_t revision);

  etcd::Client etcd_;
};

}  // namespace controller

}  // namespace routing

#endif  // SRC_CONTROLLER_CLIENT_H_
