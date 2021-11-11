/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-21 4:35:50 pm
 */

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <fmt/core.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/resource_quota.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server_builder.h>
#include <map_loader/map_loader.h>
#include <map_loader/mongo_loader.h>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <wolong/map/v1/map.pb.h>
#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include "graph/lane_graph.h"
#include "wolong/routing/v1/routing.pb.h"
#include "wolong/routing/v1/routing_api.grpc.pb.h"
#include "wolong/routing/v1/routing_api.pb.h"

namespace routing {

using GrpcRoutingService = wolong::routing::v1::RoutingService;
using PbGetRouteRequest = wolong::routing::v1::GetRouteRequest;
using PbGetRouteResponse = wolong::routing::v1::GetRouteResponse;
using PbGetRouteByBatchRequest = wolong::routing::v1::GetRouteByBatchRequest;
using PbGetRouteByBatchResponse = wolong::routing::v1::GetRouteByBatchResponse;
using PbRouteType = wolong::routing::v1::RouteType;
using PbMap = wolong::map::v1::Map;
using PbJourneyType = wolong::routing::v1::JourneyType;

class RoutingServiceImpl final : public GrpcRoutingService::Service {
 public:
  explicit RoutingServiceImpl(std::shared_ptr<graph::LaneGraph> graph);
  grpc::Status GetRoute(grpc::ServerContext* context,
                        const PbGetRouteRequest* request,
                        PbGetRouteResponse* response) override;
  grpc::Status GetRouteByBatch(grpc::ServerContext* context,
                               const PbGetRouteByBatchRequest* requests,
                               PbGetRouteByBatchResponse* responses) override;

 private:
  std::shared_ptr<graph::LaneGraph> graph_;
};

RoutingServiceImpl::RoutingServiceImpl(std::shared_ptr<graph::LaneGraph> graph)
    : graph_(std::move(graph)) {}

grpc::Status RoutingServiceImpl::GetRoute(grpc::ServerContext* context,
                                          const PbGetRouteRequest* request,
                                          PbGetRouteResponse* response) {
  (void)context;
  // TODO(zhangjun): error code
  assert(request->type() == PbRouteType::ROUTE_TYPE_DRIVING);
  auto journey = response->add_journeys();
  *journey->mutable_driving() = graph_->Search(request->start(), request->end(),
                                               request->access_revision());
  journey->set_type(PbJourneyType::JOURNEY_TYPE_DRIVING);
  response->set_agent_id(request->agent_id());
  response->set_agent_request_id(request->agent_request_id());
  return grpc::Status::OK;
}

grpc::Status RoutingServiceImpl::GetRouteByBatch(
    grpc::ServerContext* context, const PbGetRouteByBatchRequest* requests,
    PbGetRouteByBatchResponse* responses) {
  for (const auto& req : requests->requests()) {
    PbGetRouteResponse res;
    auto status = GetRoute(context, &req, &res);
    // res.PrintDebugString();
    if (status.ok()) {
      *responses->add_responses() = std::move(res);
    } else {
      return status;
    }
  }
  return grpc::Status::OK;
}

}  // namespace routing

ABSL_FLAG(std::string, mongo_uri, "mongodb://localhost:27017/", "mongodb uri");
ABSL_FLAG(std::string, mongo_db_map, "db", "map db name");
ABSL_FLAG(std::string, mongo_col_map, "col", "map collection name");
ABSL_FLAG(std::string, grpc_listen, "0.0.0.0:20218", "grpc listening address");
ABSL_FLAG(std::string, routing_cost_type, "time",
          "choose routing cost type, choice: [time, distance]");

int main(int argc, char** argv) {
#ifndef NDEBUG
  spdlog::set_level(spdlog::level::debug);
#else
  spdlog::set_level(spdlog::level::info);
#endif
  absl::ParseCommandLine(argc, argv);
  wolong::map::v1::Map map;
  map = map_loader::LoadMapFromMongo(absl::GetFlag(FLAGS_mongo_uri),
                                     absl::GetFlag(FLAGS_mongo_db_map),
                                     absl::GetFlag(FLAGS_mongo_col_map));

  routing::graph::CostType type = routing::graph::ParseStringToCostType(
      absl::GetFlag(FLAGS_routing_cost_type));
  auto graph = std::make_shared<routing::graph::LaneGraph>(map, type);

  routing::RoutingServiceImpl service(graph);
  grpc::ServerBuilder builder;
  // grpc::ResourceQuota resource_quota;
  // resource_quota.SetMaxThreads(12);
  // builder.SetResourceQuota(resource_quota);
  builder.AddListeningPort(absl::GetFlag(FLAGS_grpc_listen),
                           grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  auto server = builder.BuildAndStart();
  spdlog::info("Routing Server is listening on {}",
               absl::GetFlag(FLAGS_grpc_listen));
  server->Wait();

  return 0;
}
