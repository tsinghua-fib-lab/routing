/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-14 12:29:51 am
 */

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <fmt/core.h>
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
#include "wolong/routing/v1/routing.pb.h"
#include "wolong/routing/v1/routing_api.grpc.pb.h"
#include "wolong/routing/v1/routing_api.pb.h"

namespace routing {

class RoutingServiceClient {
 public:
  explicit RoutingServiceClient(std::shared_ptr<grpc::Channel> channel)
      : stub_(wolong::routing::v1::RoutingService::NewStub(channel)) {}

  wolong::routing::v1::GetRouteResponse GetRoute(
      wolong::routing::v1::GetRouteRequest req);
  wolong::routing::v1::GetRouteByBatchResponse GetRouteByBatch(
      wolong::routing::v1::GetRouteByBatchRequest requests);

 private:
  std::unique_ptr<wolong::routing::v1::RoutingService::Stub> stub_;
};

wolong::routing::v1::GetRouteResponse RoutingServiceClient::GetRoute(
    wolong::routing::v1::GetRouteRequest req) {
  grpc::ClientContext context;
  wolong::routing::v1::GetRouteResponse res;
  auto status = stub_->GetRoute(&context, req, &res);
  if (status.ok()) {
    return res;
  } else {
    throw std::runtime_error(
        fmt::format("{}: {}", status.error_code(), status.error_message()));
  }
}

wolong::routing::v1::GetRouteByBatchResponse
RoutingServiceClient::GetRouteByBatch(
    wolong::routing::v1::GetRouteByBatchRequest requests) {
  grpc::ClientContext context;
  wolong::routing::v1::GetRouteByBatchResponse responses;
  auto status = stub_->GetRouteByBatch(&context, requests, &responses);
  if (status.ok()) {
    return responses;
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

  routing::RoutingServiceClient client(grpc::CreateChannel(
      absl::GetFlag(FLAGS_grpc_target), grpc::InsecureChannelCredentials()));

  uint32_t poi_min = 4'0000'0000, poi_max = 4'0000'0010;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> distrib(poi_min, poi_max);
  auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < 1; ++i) {
    wolong::routing::v1::GetRouteRequest req;
    req.set_agent_id(0);
    req.set_agent_request_id(i);
    req.set_type(wolong::routing::v1::RouteType::ROUTE_TYPE_DRIVING);
    uint32_t start_poi_id = distrib(gen);
    uint32_t end_poi_id = distrib(gen);
    auto start = req.mutable_start();
    auto end = req.mutable_end();
    start->mutable_poi_position()->set_poi_id(400000825);
    end->mutable_poi_position()->set_poi_id(400000840);
    // start.mutable_lane_position()->set_lane_id(94829);
    // end.mutable_lane_position()->set_lane_id(152183);
    req.set_access_revision(100);
    auto res = client.GetRoute(std::move(req));
    std::cout << "from: " << start_poi_id << " to: " << end_poi_id << " "
              << res.ShortDebugString();
  }
  auto time_cost = std::chrono::duration_cast<std::chrono::duration<float>>(
                       std::chrono::steady_clock::now() - start)
                       .count();
  std::cout << "time: " << time_cost << "s\a" << std::endl;

  // batch model
  start = std::chrono::steady_clock::now();
  wolong::routing::v1::GetRouteByBatchRequest reqs;
  for (size_t i = 0; i < 1'000; ++i) {
    wolong::routing::v1::GetRouteRequest req;
    req.set_agent_id(0);
    req.set_agent_request_id(i);
    req.set_type(wolong::routing::v1::RouteType::ROUTE_TYPE_DRIVING);
    auto start = req.mutable_start();
    auto end = req.mutable_end();
    start->mutable_poi_position()->set_poi_id(distrib(gen));
    end->mutable_poi_position()->set_poi_id(distrib(gen));
    req.set_access_revision(100);
    *reqs.add_requests() = std::move(req);
  }
  client.GetRouteByBatch(std::move(reqs));
  time_cost = std::chrono::duration_cast<std::chrono::duration<float>>(
                  std::chrono::steady_clock::now() - start)
                  .count();
  std::cout << "batch time: " << time_cost << "s\a" << std::endl;

  return 0;
}
