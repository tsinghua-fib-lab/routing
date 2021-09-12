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
#include <simulet/map/v1/map.pb.h>
#include <spdlog/spdlog.h>
#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include "controller/client.h"
#include "graph/road_graph.h"
#include "simulet/route/v1/route.pb.h"
#include "simulet/route/v1/route_api.grpc.pb.h"
#include "simulet/route/v1/route_api.pb.h"

namespace routing {

using GrpcRouteAPI = simulet::proto::route::v1::RouteAPI;
using PbRouteRequest = simulet::proto::route::v1::RouteRequest;
using PbRouteResponse = simulet::proto::route::v1::RouteResponse;
using PbRouteBatchRequest = simulet::proto::route::v1::RouteBatchRequest;
using PbRouteBatchResponse = simulet::proto::route::v1::RouteBatchResponse;
using PbRouteType = simulet::proto::route::v1::RouteType;
using PbMap = simulet::proto::map::v1::Map;
using PbTripType = simulet::proto::route::v1::TripType;

class RouteAPIImpl final : public GrpcRouteAPI::Service {
 public:
  explicit RouteAPIImpl(std::shared_ptr<graph::RoadGraph> graph);
  grpc::Status GetRoute(grpc::ServerContext* context,
                        const PbRouteRequest* request,
                        PbRouteResponse* response) override;
  grpc::Status GetRouteByBatch(grpc::ServerContext* context,
                               const PbRouteBatchRequest* requests,
                               PbRouteBatchResponse* responses) override;

 private:
  std::shared_ptr<graph::RoadGraph> graph_;
};

RouteAPIImpl::RouteAPIImpl(std::shared_ptr<graph::RoadGraph> graph)
    : graph_(std::move(graph)) {}

grpc::Status RouteAPIImpl::GetRoute(grpc::ServerContext* context,
                                    const PbRouteRequest* request,
                                    PbRouteResponse* response) {
  (void)context;
  // TODO(zhangjun): error code
  assert(request->type() == PbRouteType::ROUTE_TYPE_DRIVING);
  auto trip = response->add_trips();
  *trip->mutable_driving() = graph_->Search(request->start(), request->end(),
                                            request->access_revision());
  trip->set_type(PbTripType::TRIP_TYPE_DRIVING);
  response->set_agent_id(request->agent_id());
  response->set_agent_request_id(request->agent_request_id());
  return grpc::Status::OK;
}

grpc::Status RouteAPIImpl::GetRouteByBatch(grpc::ServerContext* context,
                                           const PbRouteBatchRequest* requests,
                                           PbRouteBatchResponse* responses) {
  for (const auto& req : requests->requests()) {
    PbRouteResponse res;
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

ABSL_FLAG(std::string, mongo_uri, "mongodb://127.0.0.1:27017/", "mongodb uri");
ABSL_FLAG(std::string, mongo_db, "dev_t", "db name");
ABSL_FLAG(std::string, mongo_col_map, "map", "map collection name");
ABSL_FLAG(std::string, mongo_setid, "simple-x-junction", "map setid");
ABSL_FLAG(std::string, map_cache_dir, "./data/protobuf/",
          "map cache directory");
ABSL_FLAG(std::string, grpc_listen, "0.0.0.0:20218", "grpc listening address");
ABSL_FLAG(std::string, etcd_uri, "127.0.0.1:2379", "control-plane etcd uri");
ABSL_FLAG(std::string, etcd_access_key, "/access",
          "the key of access information");
ABSL_FLAG(std::string, routing_cost_type, "time",
          "choose routing cost type, choice: [time, distance]");

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  simulet::proto::map::v1::Map map = map_loader::LoadMapFromMongoWithLocalCache(
      absl::GetFlag(FLAGS_mongo_uri), absl::GetFlag(FLAGS_mongo_db),
      absl::GetFlag(FLAGS_mongo_col_map), absl::GetFlag(FLAGS_mongo_setid),
      absl::GetFlag(FLAGS_map_cache_dir));
  routing::graph::CostType type;
  auto type_string = absl::GetFlag(FLAGS_routing_cost_type);
  if (type_string == "time") {
    type = routing::graph::CostType::kTime;
  } else if (type_string == "distance") {
    type = routing::graph::CostType::kDistance;
  } else {
    spdlog::error("Invalid cost type. Choose one in [time, distance].");
    exit(EXIT_FAILURE);
  }
  auto graph =
      std::make_shared<routing::graph::RoadGraph>(std::move(map), type);

  routing::RouteAPIImpl service(graph);
  grpc::ServerBuilder builder;
  // grpc::ResourceQuota resource_quota;
  // resource_quota.SetMaxThreads(12);
  // builder.SetResourceQuota(resource_quota);
  builder.AddListeningPort(absl::GetFlag(FLAGS_grpc_listen),
                           grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  routing::controller::Client controller(absl::GetFlag(FLAGS_etcd_uri));
  controller.Listen(absl::GetFlag(FLAGS_etcd_access_key),
                    [graph](std::string data, int64_t revision) {
                      graph->ParseAndSetLaneAccess(std::move(data), revision);
                    });
  spdlog::info("Controller Client is watching on {} {}",
               absl::GetFlag(FLAGS_etcd_uri),
               absl::GetFlag(FLAGS_etcd_access_key));

  auto server = builder.BuildAndStart();
  spdlog::info("Routing Server is listening on {}",
               absl::GetFlag(FLAGS_grpc_listen));
  server->Wait();

  return 0;
}
