/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-14 12:29:51 am
 */

#include <absl/flags/flag.h>
#include <absl/flags/internal/flag.h>
#include <absl/flags/parse.h>
#include <fmt/format.h>
#include <grpcpp/channel.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/credentials.h>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include "simulet/route/v1/route.pb.h"
#include "simulet/route/v1/route_api.grpc.pb.h"
#include "simulet/route/v1/route_api.pb.h"

namespace routing {

void PrintRoutePart(const simulet::proto::route::v1::LaneSet& part) {
  bool first = true;
  std::cout << "[";
  for (auto id : part.lanes()) {
    if (first) {
      std::cout << id;
      first = false;
    } else {
      std::cout << " " << id;
    }
  }
  std::cout << "]";
}

void PrintRoute(const simulet::proto::route::v1::DrivingTripBody& route) {
  bool first = true;
  for (auto& set : route.route()) {
    if (first) {
      PrintRoutePart(set);
      first = false;
    } else {
      std::cout << " -> ";
      PrintRoutePart(set);
    }
  }
  std::cout << std::endl;
}

class RouteAPIClient {
 public:
  explicit RouteAPIClient(std::shared_ptr<grpc::Channel> channel)
      : stub_(simulet::proto::route::v1::RouteAPI::NewStub(channel)) {}

  simulet::proto::route::v1::RouteResponse GetRoute(
      simulet::proto::route::v1::RouteRequest req);

 private:
  std::unique_ptr<simulet::proto::route::v1::RouteAPI::Stub> stub_;
};

simulet::proto::route::v1::RouteResponse RouteAPIClient::GetRoute(
    simulet::proto::route::v1::RouteRequest req) {
  grpc::ClientContext context;
  simulet::proto::route::v1::RouteResponse res;
  auto status = stub_->GetRoute(&context, req, &res);
  if (status.ok()) {
    return res;
  } else {
    throw std::runtime_error(
        fmt::format("{}: {}", status.error_code(), status.error_message()));
  }
}

}  // namespace routing

ABSL_FLAG(std::string, grpc_target, "localhost:20218",
          "Target Grpc routing server");

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  routing::RouteAPIClient client(grpc::CreateChannel(
      absl::GetFlag(FLAGS_grpc_target), grpc::InsecureChannelCredentials()));

  uint32_t poi_min = 4'0000'0000, poi_max = 4'0004'7505;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> distrib(poi_min, poi_max);
  auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < 1'000; ++i) {
    simulet::proto::route::v1::RouteRequest req;
    req.set_agent_id(0);
    req.set_agent_request_id(i);
    req.set_type(simulet::proto::route::v1::RouteType::ROUTE_TYPE_DRIVING);
    auto start = req.mutable_start();
    auto end = req.mutable_end();
    start->mutable_area_position()->set_poi_id(distrib(gen));
    end->mutable_area_position()->set_poi_id(distrib(gen));
    // start.mutable_street_position()->set_lane_id(94829);
    // end.mutable_street_position()->set_lane_id(152183);
    auto res = client.GetRoute(std::move(req));
    // routing::PrintRoute(res.trips().at(0).driving());
  }
  auto time_cost = std::chrono::duration_cast<std::chrono::duration<float>>(
                       std::chrono::steady_clock::now() - start)
                       .count();
  std::cout << " time: " << time_cost << "s\a" << std::endl;

  return 0;
}
