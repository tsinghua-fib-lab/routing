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
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iostream>
#include <set>
#include <vector>
#include "graph/road_graph.h"
#include "map_loader/file_loader.h"
#include "map_loader/mongo_loader.h"
#include "simulet/map/v1/map.pb.h"

void PrintRoutePart(const std::set<uint32_t> part) {
  bool first = true;
  std::cout << "[";
  for (auto id : part) {
    if (first) {
      std::cout << id;
      first = false;
    } else {
      std::cout << " " << id;
    }
  }
  std::cout << "]";
}

void PrintRoute(const std::vector<std::set<uint32_t>>& route) {
  bool first = true;
  for (auto& set : route) {
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

ABSL_FLAG(std::string, mongo_uri, "mongodb://127.0.0.1:27017/", "mongodb uri");
ABSL_FLAG(std::string, mongo_db, "dev_t", "db name");
ABSL_FLAG(std::string, mongo_col_map, "map", "map collection name");
ABSL_FLAG(std::string, mongo_setid, "simple-x-junction", "map setid");
ABSL_FLAG(std::string, map_cache_dir, "./data/protobuf/",
          "map cache directory");

ABSL_FLAG(uint32_t, start, 0U, "start lane id");
ABSL_FLAG(uint32_t, end, 0U, "start lane id");

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);
  std::filesystem::path cache_dir(absl::GetFlag(FLAGS_map_cache_dir));
  auto cache_path =
      cache_dir / fmt::format("{}.{}.{}.pb", absl::GetFlag(FLAGS_mongo_db),
                              absl::GetFlag(FLAGS_mongo_col_map),
                              absl::GetFlag(FLAGS_mongo_setid));
  simulet::proto::map::v1::Map map;
  if (std::filesystem::exists(cache_path)) {
    map = routing::map_loader::LoadMapFromFile(cache_path);
  } else {
    map = routing::map_loader::LoadMapFromMongo(
        absl::GetFlag(FLAGS_mongo_uri), absl::GetFlag(FLAGS_mongo_db),
        absl::GetFlag(FLAGS_mongo_col_map), absl::GetFlag(FLAGS_mongo_setid));
    std::ofstream output(cache_path,
                         std::ios_base::binary | std::ios_base::out);
    map.SerializeToOstream(&output);
  }

  routing::graph::RoadGraph graph(std::move(map));
  // graph.Print();
  routing::graph::PbMapPosition start, end;
  start.mutable_area_position()->set_poi_id(4'0000'0001);
  end.mutable_area_position()->set_poi_id(4'0000'0002);
  PrintRoute(graph.Search(start, end));
  return 0;
}
