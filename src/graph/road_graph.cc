/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-16 9:55:17 pm
 */

#include "graph/road_graph.h"
#include <fmt/format.h>
#include <spdlog/spdlog.h>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <map>
#include <optional>
#include <ostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace routing {

namespace graph {

RoadNode::RoadNode(std::vector<PbLane> lanes) : point_({0.0f, 0.0f}) {
  assert(!lanes.empty());
  std::optional<uint32_t> parent;
  for (const auto& lane : lanes) {
    lanes_.emplace(lane.id(), std::move(lane));
    // check
    if (!parent) {
      parent = lane.parent_id();
    } else {
      assert(parent == lane.parent_id());
    }
    // calculate point_ and cost_
    base_cost_ += lane.length();
    auto&& end = lane.center_line().nodes().rbegin();
    point_.x += end->x();
    point_.y += end->y();
  }
  base_cost_ /= lanes.size();
  point_.x /= lanes.size();
  point_.y /= lanes.size();
}

std::string RoadNode::String() const {
  std::string s;
  bool first = true;
  for (const auto& [id, _] : lanes_) {
    if (first) {
      s += fmt::format("{}", id);
      first = false;
    } else {
      s += fmt::format(" {}", id);
    }
  }
  return fmt::format("[{}]", s);
}

RoadGraph::RoadGraph(PbMap map) {
  CreateNodes(std::move(map));
  LinkEdges();
  // TODO(zhangjun): POI to lane
}

void RoadGraph::Print() const {
  for (const auto& node : memory_) {
    std::cout << node.String() << " -> ";
    for (auto node_ptr : node.next_) {
      std::cout << node_ptr->String();
    }
    std::cout << "\n";
  }
  std::cout << std::flush;
}

void RoadGraph::CreateNodes(PbMap map) {
  for (const auto& [_, road] : map.roads()) {
    std::vector<PbLane> lanes;
    for (auto lane_id : road.lane_ids()) {
      lanes.push_back(map.lanes().at(lane_id));
    }
    memory_.emplace_back(std::move(lanes));
    auto& node = memory_.back();
    for (const auto& [id, _] : node.lanes_) {
      table_.emplace(id, &node);
    }
  }
  for (const auto& [_, junction] : map.junctions()) {
    // <start road -> end road> -> lanes
    std::map<std::pair<uint32_t, uint32_t>, std::vector<PbLane>> group;
    for (auto lane_id : junction.lane_ids()) {
      PbLane lane = map.lanes().at(lane_id);
      if (lane.predecessor_ids_size() == 0 || lane.successor_ids_size() == 0) {
        spdlog::warn(
            "road_graph: Lane {} in junction {} has no predecessor or no "
            "successor. Ignore it.");
        continue;
      }
      assert(lane.predecessor_ids_size() == 1);
      assert(lane.successor_ids_size() == 1);
      uint32_t start_road_id =
          map.lanes().at(lane.predecessor_ids().at(0)).parent_id();
      // TODO(zhangjun): support road->[lane->lane]->road in junction
      assert(map.roads().contains(start_road_id));
      uint32_t end_road_id =
          map.lanes().at(lane.successor_ids().at(0)).parent_id();
      assert(map.roads().contains(end_road_id));
      group[std::make_pair(start_road_id, end_road_id)].push_back(
          std::move(lane));
    }
    for (auto&& [_, lanes] : group) {
      memory_.emplace_back(std::move(lanes));
      auto& node = memory_.back();
      for (const auto& [id, _] : node.lanes_) {
        table_.emplace(id, &node);
      }
    }
  }
}

void RoadGraph::LinkEdges() {
  for (auto& node : memory_) {
    for (const auto& [_, lane] : node.lanes_) {
      for (auto successor_lane_id : lane.successor_ids()) {
        node.next_.push_back(table_.at(successor_lane_id));
      }
    }
  }
}

}  // namespace graph

}  // namespace routing
