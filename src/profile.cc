/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-14 12:29:51 am
 */

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <gperftools/profiler.h>
#include <proto_loader/map_loader.h>
#include <wolong/map/v1/map.pb.h>
#include <chrono>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include "graph/lane_graph.h"

ABSL_FLAG(std::string, mongo_uri, "mongodb://localhost:27017/", "mongodb uri");
ABSL_FLAG(std::string, mongo_db_map, "db", "db name");
ABSL_FLAG(std::string, mongo_col_map, "col", "map collection name");
ABSL_FLAG(std::string, routing_cost_type, "time",
          "choose routing cost type, choice: [time, distance]");

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  wolong::map::v1::Map map;
  map = proto_loader::LoadMapFromMongo(absl::GetFlag(FLAGS_mongo_uri),
                                       absl::GetFlag(FLAGS_mongo_db_map),
                                       absl::GetFlag(FLAGS_mongo_col_map));

  routing::graph::CostType type = routing::graph::ParseStringToCostType(
      absl::GetFlag(FLAGS_routing_cost_type));
  routing::graph::LaneGraph graph(map, type);

  // 测试步行导航
  // auto rs = graph.SearchWalking(16535, 0.1, 35883, 30, 0);
  // std::cout << rs.ShortDebugString() << std::endl;
  // rs = graph.SearchWalking(35883, 1, 35879, 1, 0);
  // std::cout << rs.ShortDebugString() << std::endl;
  // rs = graph.SearchWalking(35883, 0.1, 35881, 30, 0);
  // std::cout << rs.ShortDebugString() << std::endl;
  // rs = graph.SearchWalking(35883, 20, 35881, 0, 0);
  // std::cout << rs.ShortDebugString() << std::endl;

  // 测试车辆导航
  // auto rs = graph.SearchDriving(6567, 9053, 0);
  // std::cout << rs.ShortDebugString() << std::endl;
  // rs = graph.SearchDriving(11238, 11238, 0);
  // std::cout << rs.ShortDebugString() << std::endl;
  // rs = graph.SearchDriving(11238, 8742, 0);
  // std::cout << rs.ShortDebugString() << std::endl;
  // return 0;

  uint32_t poi_min = std::numeric_limits<uint32_t>::max();
  uint32_t poi_max = std::numeric_limits<uint32_t>::min();
  for (const auto& poi : map.pois()) {
    poi_max = std::max(poi.id(), poi_max);
    poi_min = std::min(poi.id(), poi_min);
  }
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> distrib(poi_min, poi_max);
#ifndef NDEBUG
  ProfilerStart("profile.log");
#endif
  auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < 1'000; ++i) {
    routing::graph::PbPosition start, end;
    start.mutable_poi_position()->set_poi_id(distrib(gen));
    end.mutable_poi_position()->set_poi_id(distrib(gen));
    auto rs = graph.SearchDriving(start, end, 0);
  }
#ifndef NDEBUG
  ProfilerStop();
#endif

  auto time_cost = std::chrono::duration_cast<std::chrono::duration<float>>(
                       std::chrono::steady_clock::now() - start)
                       .count();
  std::cout << " time: " << time_cost << "s\a" << std::endl;

  return 0;
}
