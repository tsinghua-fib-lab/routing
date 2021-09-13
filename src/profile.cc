/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-14 12:29:51 am
 */

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <gperftools/profiler.h>
#include <map_loader/mongo_loader.h>
#include <simulet/map/v1/map.pb.h>
#include <chrono>
#include <cstdint>
#include <random>
#include <string>
#include "graph/road_graph.h"

ABSL_FLAG(std::string, mongo_uri, "mongodb://localhost:27017/", "mongodb uri");
ABSL_FLAG(std::string, mongo_db_map, "db", "db name");
ABSL_FLAG(std::string, mongo_col_map, "col", "map collection name");
ABSL_FLAG(std::string, mongo_setid_map, "setid", "map setid");
ABSL_FLAG(std::string, routing_cost_type, "time",
          "choose routing cost type, choice: [time, distance]");

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  simulet::proto::map::v1::Map map = map_loader::LoadMapFromMongo(
      absl::GetFlag(FLAGS_mongo_uri), absl::GetFlag(FLAGS_mongo_db_map),
      absl::GetFlag(FLAGS_mongo_col_map), absl::GetFlag(FLAGS_mongo_setid_map));
  routing::graph::CostType type = routing::graph::ParseStringToCostType(
      absl::GetFlag(FLAGS_routing_cost_type));
  routing::graph::RoadGraph graph(std::move(map), type);

  uint32_t poi_min = 4'0000'0000, poi_max = 4'0001'8923;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> distrib(poi_min, poi_max);
#ifndef NDEBUG
  ProfilerStart("profile.log");
#endif
  auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < 1'000; ++i) {
    routing::graph::PbMapPosition start, end;
    start.mutable_area_position()->set_poi_id(distrib(gen));
    end.mutable_area_position()->set_poi_id(distrib(gen));
    auto rs = graph.Search(start, end, 0);
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
