/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-15 11:00:32 pm
 */

#include "map_loader/mongo_loader.h"
#include <absl/flags/flag.h>
#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/document/view.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/cursor.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/instance.hpp>
#include "simulet/geo/v1/geo.pb.h"
#include "simulet/map/v1/map.pb.h"

using bsoncxx::builder::stream::document;
using bsoncxx::builder::stream::finalize;

using PbLane = simulet::proto::map::v1::Lane;
using PbLaneType = simulet::proto::map::v1::LaneType;
using PbRoad = simulet::proto::map::v1::Road;
using PbJunction = simulet::proto::map::v1::Junction;
using PbPoi = simulet::proto::map::v1::Poi;
using PbAoi = simulet::proto::map::v1::Aoi;
using PbPolyline = simulet::proto::map::v1::Polyline;
using PbMapHeader = simulet::proto::map::v1::Header;
using PbStreetPosition = simulet::proto::geo::v1::StreetPosition;

namespace routing {

namespace map_loader {

template <typename PbEnum>
static PbEnum LoadEnumFromBson(bool (*parser)(const std::string& name,
                                              PbEnum* value),
                               const bsoncxx::document::element& data) {
  PbEnum e;
  std::string e_str(data.get_utf8().value);
  if (!parser(e_str, &e)) {
    throw std::runtime_error(fmt::format("mongo_loader: Fail to parse {} to {}",
                                         e_str, typeid(PbEnum).name()));
  }
  return e;
}

static PbPolyline LoadPolylineFromBson(const bsoncxx::document::element& data) {
  PbPolyline pb;
  for (const auto& doc : data["nodes"].get_array().value) {
    auto pb_node = pb.add_nodes();
    pb_node->set_x(static_cast<float>(doc["x"].get_double()));
    pb_node->set_y(static_cast<float>(doc["y"].get_double()));
  }
  return pb;
}

static PbStreetPosition LoadStreetPositionFromBson(
    const bsoncxx::document::element& data) {
  PbStreetPosition pb;
  pb.set_lane_id(static_cast<uint32_t>(data["lane_id"].get_int32()));
  pb.set_s(static_cast<float>(data["s"].get_double()));
  return pb;
}

static PbLane LoadLaneFromBson(const bsoncxx::document::element& data) {
  PbLane pb;
  pb.set_id(static_cast<uint32_t>(data["id"].get_int32()));
  pb.set_type(
      LoadEnumFromBson(simulet::proto::map::v1::LaneType_Parse, data["type"]));
  pb.set_turn(
      LoadEnumFromBson(simulet::proto::map::v1::LaneTurn_Parse, data["turn"]));
  pb.set_priority(data["priority"].get_int32());
  pb.set_max_speed(static_cast<float>(data["max_speed"].get_double()));
  pb.set_length(static_cast<float>(data["length"].get_double()));
  *pb.mutable_center_line() = LoadPolylineFromBson(data["center_line"]);
  *pb.mutable_left_border_line() =
      LoadPolylineFromBson(data["left_border_line"]);
  *pb.mutable_right_border_line() =
      LoadPolylineFromBson(data["right_border_line"]);
  for (const auto& id : data["predecessor_ids"].get_array().value) {
    pb.add_predecessor_ids(static_cast<uint32_t>(id.get_int32()));
  }
  for (const auto& id : data["successor_ids"].get_array().value) {
    pb.add_successor_ids(static_cast<uint32_t>(id.get_int32()));
  }
  for (const auto& id : data["left_lane_ids"].get_array().value) {
    pb.add_left_lane_ids(static_cast<uint32_t>(id.get_int32()));
  }
  for (const auto& id : data["right_lane_ids"].get_array().value) {
    pb.add_right_lane_ids(static_cast<uint32_t>(id.get_int32()));
  }
  pb.set_parent_id(static_cast<uint32_t>(data["parent_id"].get_int32()));
  // TODO(zhangjun): reverse_lane_id is not implemented in map-tools:converter
  // pb.set_reverse_lane_id(
  //     static_cast<uint32_t>(data["reverse_lane_id"].get_int32()));
  // TODO(zhangjun): LaneOverlap
  return pb;
}

static PbRoad LoadRoadFromBson(const bsoncxx::document::element& data) {
  PbRoad pb;
  pb.set_id(static_cast<uint32_t>(data["id"].get_int32()));
  for (const auto& id : data["lane_ids"].get_array().value) {
    pb.add_lane_ids(static_cast<uint32_t>(id.get_int32()));
  }
  for (const auto& id : data["poi_ids"].get_array().value) {
    pb.add_poi_ids(static_cast<uint32_t>(id.get_int32()));
  }
  return pb;
}

static PbJunction LoadJunctionFromBson(const bsoncxx::document::element& data) {
  PbJunction pb;
  pb.set_id(static_cast<uint32_t>(data["id"].get_int32()));
  for (auto&& id : data["lane_ids"].get_array().value) {
    pb.add_lane_ids(static_cast<uint32_t>(id.get_int32()));
  }
  return pb;
}

static PbPoi LoadPoiFromBson(const bsoncxx::document::element& data) {
  PbPoi pb;
  pb.set_id(static_cast<uint32_t>(data["id"].get_int32()));
  pb.set_type(
      LoadEnumFromBson(simulet::proto::map::v1::PoiType_Parse, data["type"]));
  *pb.mutable_driving_position() =
      LoadStreetPositionFromBson(data["driving_position"]);
  // TODO(zhangjun): walking_position is not implemented in map-tools:converter
  // *pb.mutable_walking_position() =
  //     LoadStreetPositionFromBson(data["walking_position"]);
  return pb;
}

static PbAoi LoadAoiFromBson(const bsoncxx::document::element& data) {
  PbAoi pb;
  pb.set_id(static_cast<uint32_t>(data["id"].get_int32()));
  // TODO(zhangjun): poi_ids is not implemented in map-tools:converter
  // for (const auto& id : data["poi_ids"].get_array().value) {
  //   pb.add_poi_ids(static_cast<uint32_t>(id.get_int32()));
  // }
  for (const auto& id : data["gate_poi_ids"].get_array().value) {
    pb.add_gate_poi_ids(static_cast<uint32_t>(id.get_int32()));
  }
  return pb;
}

static PbMapHeader LoadHeaderFromBson(const bsoncxx::document::element& data) {
  PbMapHeader pb;
  pb.set_name(std::string(data["name"].get_utf8().value));
  pb.set_date(std::string(data["date"].get_utf8().value));
  pb.set_north(static_cast<float>(data["north"].get_double()));
  pb.set_south(static_cast<float>(data["south"].get_double()));
  pb.set_east(static_cast<float>(data["east"].get_double()));
  pb.set_west(static_cast<float>(data["west"].get_double()));
  pb.set_projection(std::string(data["projection"].get_utf8().value));
  return pb;
}

simulet::proto::map::v1::Map LoadMapFromMongo(const std::string& uri,
                                              const std::string& dbname,
                                              const std::string& colname,
                                              const std::string& setid) {
  static mongocxx::instance _;
  spdlog::info("Load map from {}.{}.{} at {}", dbname, colname, setid, uri);
  mongocxx::client client(mongocxx::uri{uri});

  // check link
  // client.list_database_names({});

  auto col = client[dbname][colname];
  auto cursor = col.find(document{} << "setid" << setid << finalize);

  simulet::proto::map::v1::Map map;

  for (auto&& doc : cursor) {
    std::string_view doc_class = doc["class"].get_utf8();
    bsoncxx::document::element data = doc["data"];
    if (doc_class == "lane") {
      auto pb = LoadLaneFromBson(data);
      (*map.mutable_lanes())[pb.id()] = std::move(pb);
    } else if (doc_class == "road") {
      auto pb = LoadRoadFromBson(data);
      (*map.mutable_roads())[pb.id()] = std::move(pb);
    } else if (doc_class == "junction") {
      auto pb = LoadJunctionFromBson(data);
      (*map.mutable_junctions())[pb.id()] = std::move(pb);
    } else if (doc_class == "poi") {
      auto pb = LoadPoiFromBson(data);
      (*map.mutable_pois())[pb.id()] = std::move(pb);
    } else if (doc_class == "aoi") {
      auto pb = LoadAoiFromBson(data);
      (*map.mutable_aois())[pb.id()] = std::move(pb);
    } else if (doc_class == "header") {
      *map.mutable_header() = LoadHeaderFromBson(data);
    } else {
      throw std::runtime_error(fmt::format("Unknown doc class {}", doc_class));
    }
  }

  return map;
}

}  // namespace map_loader

}  // namespace routing
