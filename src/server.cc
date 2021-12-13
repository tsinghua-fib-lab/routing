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
#include <proto_loader/map_loader.h>
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
#include "wolong/routing/v1/routing_service.grpc.pb.h"
#include "wolong/routing/v1/routing_service.pb.h"

namespace routing {

using GrpcRoutingService = wolong::routing::v1::RoutingService;
using PbGetRouteRequest = wolong::routing::v1::GetRouteRequest;
using PbGetRouteResponse = wolong::routing::v1::GetRouteResponse;
using PbRouteType = wolong::routing::v1::RouteType;
using PbMap = wolong::map::v1::Map;
using PbJourneyType = wolong::routing::v1::JourneyType;

class RoutingServiceImpl final : public GrpcRoutingService::Service {
 public:
  explicit RoutingServiceImpl(std::shared_ptr<graph::LaneGraph> graph)
      : graph_(std::move(graph)) {}
  grpc::Status GetRoute(grpc::ServerContext*, const PbGetRouteRequest* request,
                        PbGetRouteResponse* response) override {
    switch (request->type()) {
      case PbRouteType::ROUTE_TYPE_DRIVING: {
        auto journey = response->add_journeys();
        *journey->mutable_driving() = graph_->SearchDriving(
            request->start(), request->end(), request->access_revision());
        journey->set_type(PbJourneyType::JOURNEY_TYPE_DRIVING);
        return grpc::Status::OK;
      }
      case PbRouteType::ROUTE_TYPE_WALKING: {
        auto journey = response->add_journeys();
        *journey->mutable_walking() = graph_->SearchWalking(
            request->start(), request->end(), request->access_revision());
        journey->set_type(PbJourneyType::JOURNEY_TYPE_WALKING);
        return grpc::Status::OK;
      }
      default:
        assert(false);
        return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT,
                            "request type must be either driving or walking");
    }
  }

 private:
  std::shared_ptr<graph::LaneGraph> graph_;
};

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
  map = proto_loader::LoadMapFromMongo(absl::GetFlag(FLAGS_mongo_uri),
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
