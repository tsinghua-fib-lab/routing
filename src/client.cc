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
#include <chrono>  // NOLINT
#include <cstdint>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <thread>  // NOLINT
#include "wolong/routing/v1/routing.pb.h"
#include "wolong/routing/v1/routing_service.grpc.pb.h"
#include "wolong/routing/v1/routing_service.pb.h"

namespace routing {

class RoutingServiceClient {
 public:
  explicit RoutingServiceClient(std::shared_ptr<grpc::Channel> channel)
      : stub_(wolong::routing::v1::RoutingService::NewStub(channel)) {}

  void GetRoute(const wolong::routing::v1::GetRouteRequest& req,
                wolong::routing::v1::GetRouteResponse* res);
  void AsyncCompleteRpc(int cnt);

 private:
  std::unique_ptr<wolong::routing::v1::RoutingService::Stub> stub_;
  grpc::CompletionQueue cq_;
  struct AsyncClientCall {
    /* to contain the reply*/
    wolong::routing::v1::GetRouteResponse* res;
    wolong::routing::v1::GetRouteResponse response;
    grpc::ClientContext context;
    grpc::Status status;
    std::unique_ptr<
        grpc::ClientAsyncResponseReader<wolong::routing::v1::GetRouteResponse>>
        response_reader;
  };
};

void RoutingServiceClient::GetRoute(
    const wolong::routing::v1::GetRouteRequest& req,
    wolong::routing::v1::GetRouteResponse* res) {
  AsyncClientCall* call = new AsyncClientCall;
  call->res = res;
  call->response_reader =
      stub_->PrepareAsyncGetRoute(&call->context, req, &cq_);

  call->response_reader->StartCall();
  call->response_reader->Finish(&call->response, &call->status,
                                reinterpret_cast<void*>(call));
}

void RoutingServiceClient::AsyncCompleteRpc(int cnt) {
  void* got_tag;
  bool ok = false;
  int cnt_res = 0;
  while (cq_.Next(&got_tag, &ok)) {
    AsyncClientCall* call = static_cast<AsyncClientCall*>(got_tag);
    GPR_ASSERT(ok);
    if (call->status.ok())
      *(call->res) = call->response;
    else
      std::cout << "RPC failed" << std::endl;
    delete call;
    if (++cnt_res == cnt) {
      std::cout << "finish processing res " << cnt_res << std::endl;
      break;
    }
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

    wolong::routing::v1::GetRouteResponse res;
    client.GetRoute(req, &res);
    int cnt = 1;
    std::thread worker = std::thread(
        &routing::RoutingServiceClient::AsyncCompleteRpc, &client, cnt);

    worker.join();
    std::cout << "from: " << start_poi_id << " to: " << end_poi_id << " "
              << res.ShortDebugString();
  }
  auto time_cost = std::chrono::duration_cast<std::chrono::duration<float>>(
                       std::chrono::steady_clock::now() - start)
                       .count();
  std::cout << "time: " << time_cost << "s\a" << std::endl;

  // batch model
  size_t cnt = 1'000;
  wolong::routing::v1::GetRouteResponse res;
  start = std::chrono::steady_clock::now();
  std::thread worker = std::thread(
      &routing::RoutingServiceClient::AsyncCompleteRpc, &client, cnt);
  for (size_t i = 0; i < cnt; ++i) {
    wolong::routing::v1::GetRouteRequest req;
    req.set_agent_id(0);
    req.set_agent_request_id(i);
    req.set_type(wolong::routing::v1::RouteType::ROUTE_TYPE_DRIVING);
    auto start = req.mutable_start();
    auto end = req.mutable_end();
    start->mutable_poi_position()->set_poi_id(distrib(gen));
    end->mutable_poi_position()->set_poi_id(distrib(gen));
    req.set_access_revision(100);
    client.GetRoute(req, &res);
  }
  worker.join();
  time_cost = std::chrono::duration_cast<std::chrono::duration<float>>(
                  std::chrono::steady_clock::now() - start)
                  .count();
  std::cout << "batch time: " << time_cost << "s\a" << std::endl;

  return 0;
}
