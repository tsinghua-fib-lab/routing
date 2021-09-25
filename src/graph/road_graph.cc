/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-16 9:55:17 pm
 */

#include "graph/road_graph.h"
#include <fmt/core.h>
#include <spdlog/spdlog.h>
#include <unistd.h>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <set>
#include <stack>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "simulet/map_runtime/v1/map_runtime.pb.h"

#define IGNORE_MAP_REVISION

namespace routing {

namespace graph {

namespace {

const double kAverageSpeedInManhattenDistance = 60.0 / 3.6;

}

CostType ParseStringToCostType(const std::string& s) {
  if (s == "time") {
    return CostType::kTime;
  } else if (s == "distance") {
    return CostType::kDistance;
  } else {
    throw std::runtime_error(
        "Invalid cost type. Choose one in [time, distance].");
  }
}

RoadNode::RoadNode() = default;

RoadNode::RoadNode(int id, std::vector<PbLane> lanes, CostType type)
    : type_(type), id_(id), point_({0.0f, 0.0f}) {
  assert(!lanes.empty());
  std::optional<uint32_t> parent;
  for (const auto& lane : lanes) {
    lanes_.emplace(lane.id(), std::move(lane));
    lanes_runtime_.emplace(lane.id(), LaneRuntime{});
    // check
    if (!parent) {
      parent = lane.parent_id();
    } else {
      assert(parent == lane.parent_id());
    }
    switch (type) {
      case CostType::kDistance:
        base_cost_ += lane.length();
        break;
      case CostType::kTime:
        base_cost_ += lane.length() / lane.max_speed();
        break;
      default:
        assert(false);
    }
    // calculate point_ and cost_
    auto end = lane.center_line().nodes().rbegin();
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

float RoadNode::GetCost(const RoadNode& end) const {
  switch (type_) {
    case CostType::kDistance:
      return base_cost_ + point_.GetManhattanDistance(end.point_);
    case CostType::kTime:
      return base_cost_ + point_.GetManhattanDistance(end.point_) /
                              kAverageSpeedInManhattenDistance;
    default:
      assert(false);
  }
}

RoadGraph::RoadGraph(PbMap map, CostType type) {
  CreateNodes(map, type);
  LinkEdges();
  CreatePoiMapper(map);
}

void RoadNode::SetLaneAccess(const PbLaneAccessSetting& setting) {
  using Type = simulet::proto::map_runtime::v1::LaneAccessType;
  auto lane_id = setting.lane_id();
  assert(lanes_.find(lane_id) != lanes_.cend());
  assert(lanes_runtime_.find(lane_id) != lanes_runtime_.cend());
  auto& runtime = lanes_runtime_.at(lane_id);
  bool ok_in_setting = setting.type() != Type::LANE_ACCESS_TYPE_NO_ENTRY;
  if (runtime.ok != ok_in_setting) {
    runtime.ok = ok_in_setting;
    // if the lane access has changed, insert/erase lane_id from
    // next_[next_node]
    for (auto next_node : runtime.next_nodes) {
      if (runtime.ok) {
        // insert
        next_.at(next_node).insert(lane_id);
      } else {
        // erase
        next_.at(next_node).erase(lane_id);
      }
    }
  }
}

void RoadGraph::Print() const {
  std::cout
      << "[node with lane ids] -> [next node]:(lanes connected to the node)\n";
  for (const auto& node : memory_) {
    std::cout << node.String() << " -> ";
    for (const auto& [node_ptr, v] : node.next_) {
      std::cout << node_ptr->String() << ":(";
      bool first = true;
      for (auto id : v) {
        if (first) {
          std::cout << id;
          first = false;
        } else {
          std::cout << "," << id;
        }
      }
      std::cout << ") ";
    }
    std::cout << "\n";
  }
  std::cout << std::flush;
}

// node, parent, priority
struct NodeTuple {
  const RoadNode* node;
  const RoadNode* parent;
  float priority;
  NodeTuple(const RoadNode* node, const RoadNode* parent, float priority)
      : node(node), parent(parent), priority(priority) {}
};

PbDrivingTripBody RoadGraph::Search(uint32_t start_lane, uint32_t end_lane,
                                    int64_t revision, bool loopback) {
#ifdef IGNORE_MAP_REVISION
  revision = revision_;
#endif
  {
    std::unique_lock<std::mutex> lk(search_mtx_);
    search_cv_.wait(lk, [this, revision] {
      return allow_search_ == true && revision_ >= revision;
    });
  }
  ++num_running_search_;
  spdlog::debug(
      "road_graph: Start searching, start={}, end={}, revision={}, loopback={}",
      start_lane, end_lane, revision, loopback);

  if (loopback) {
    assert(start_lane == end_lane);
  }
  // reference: https://zhuanlan.zhihu.com/p/54510444

  auto cmp = [](const NodeTuple& a, const NodeTuple& b) {
    return a.priority > b.priority;
  };
  // nodes waiting for searching
  std::priority_queue<NodeTuple, std::vector<NodeTuple>, decltype(cmp)>
      open_queue(cmp);
  // node id -> visited, in open_queue now or has been in open_queue
  std::vector<bool> visited_nodes;
  visited_nodes.resize(node_size_, false);
  // node id -> parent, nodes searched (not nullptr)
  std::vector<const RoadNode*> close_set;
  close_set.resize(node_size_, nullptr);
  PbDrivingTripBody result;

  const RoadNode* start_node = table_.at(start_lane);
  const RoadNode* end_node = table_.at(end_lane);
  if (loopback) {
    // skip the start_node and push start_node's next nodes into open_set
    for (const auto& [next_node, _] : start_node->next_) {
      open_queue.emplace(next_node, start_node, next_node->GetCost(*end_node));
      visited_nodes[next_node->id_] = true;
    }
  } else {
    open_queue.emplace(start_node, &null_node, 0.0f);
    visited_nodes[start_node->id_] = true;
  }
  while (!open_queue.empty()) {
    // get the node with the least priority from open_set to search
    NodeTuple tuple = open_queue.top();
    open_queue.pop();
    if (tuple.node == end_node) {
      // finish
      // node, next
      // use stack to reverse node-link
      const RoadNode* parent = tuple.parent;
      const RoadNode* node = tuple.node;
      std::stack<std::pair<const RoadNode*, const RoadNode*>> stack;
      stack.emplace(node, nullptr);
      while (parent != &null_node) {
        stack.emplace(parent, node);
        node = parent;
        if (loopback && node == start_node) {
          break;
        } else {
          parent = close_set[node->id_];
          assert(parent != nullptr);
        }
      }
      // find avaliable lanes from node data
      assert(!stack.empty());
      while (!stack.empty()) {
        auto [node, next] = stack.top();
        stack.pop();
        auto lane_set = result.add_route();
        if (next == nullptr) {
          lane_set->add_lanes(end_lane);
        } else {
          for (auto lane_id : node->next_.at(next)) {
            lane_set->add_lanes(lane_id);
          }
        }
      }
      goto RETURN;
    }
    // searched node -> close_set
    // node->next -> open_set
    assert(close_set[tuple.node->id_] == nullptr);
    close_set[tuple.node->id_] = tuple.parent;
    for (const auto& [next_node, lanes] : tuple.node->next_) {
      if (lanes.empty()) {
        continue;
      } else if (visited_nodes[next_node->id_]) {
        continue;
      } else {
        open_queue.emplace(next_node, tuple.node,
                           tuple.priority + next_node->GetCost(*end_node));
        visited_nodes[next_node->id_] = true;
      }
    }
  }

RETURN:
  spdlog::debug(
      "road_graph: Finish searching, start={}, end={}, revision={}, "
      "loopback={}",
      start_lane, end_lane, revision, loopback);
  // return
  --num_running_search_;
  {
    std::lock_guard<std::mutex> lg(set_access_mtx_);
    set_access_cv_.notify_one();
  }
  return result;
}

PbDrivingTripBody RoadGraph::Search(const PbMapPosition& start,
                                    const PbMapPosition& end,
                                    int64_t revision) {
  // convert MapPosition into StreetPosition
  PbStreetPosition start_street, end_street;
  if (start.has_area_position()) {
    start_street = poi_mapper_.at(start.area_position().poi_id());
  } else {
    start_street = start.street_position();
  }
  if (end.has_area_position()) {
    end_street = poi_mapper_.at(end.area_position().poi_id());
  } else {
    end_street = end.street_position();
  }
  // consider start.s and end.s
  if (start_street.lane_id() == end_street.lane_id() &&
      start_street.s() > end_street.s()) {
    return Search(start_street.lane_id(), start_street.lane_id(), revision,
                  true);
  } else {
    return Search(start_street.lane_id(), end_street.lane_id(), revision,
                  false);
  }
}

void RoadGraph::ParseAndSetLanesAccess(std::string data, int64_t revision) {
  if (revision <= revision_) {
    spdlog::warn(
        "road_graph: received access revision {} <= the last revision {}. "
        "Ignore "
        "it",
        revision, revision_);
    return;
  }
  PbBatchAccessSetting settings;
  if (!settings.ParseFromString(std::move(data))) {
    spdlog::warn("road_graph: cannot parse lane access. Ignore it");
    return;
  }
  SetLanesAccess(std::move(settings), revision);
}

void RoadGraph::SetLanesAccess(PbBatchAccessSetting settings,
                               int64_t revision) {
  if (revision <= revision_) {
    spdlog::warn(
        "road_graph: received access revision {} <= the last revision {}. "
        "Ignore "
        "it",
        revision, revision_);
    return;
  }
  spdlog::debug("road_graph: waiting for the running searching (revision={})",
                revision);
  allow_search_ = false;
  {
    std::unique_lock<std::mutex> lk(set_access_mtx_);
    set_access_cv_.wait(lk, [this] { return num_running_search_ == 0; });
  }
  spdlog::debug("road_graph: apply {} with revision {}",
                settings.ShortDebugString(), revision);
  for (const auto& one : settings.lanes()) {
    table_.at(one.lane_id())->SetLaneAccess(one);
  }
  revision_ = revision;
  allow_search_ = true;
  {
    std::lock_guard<std::mutex> lg(search_mtx_);
    search_cv_.notify_all();
  }
}

void RoadGraph::CreateNodes(const PbMap& map, CostType type) {
  for (const auto& [_, road] : map.roads()) {
    std::vector<PbLane> lanes;
    for (auto lane_id : road.lane_ids()) {
      lanes.push_back(map.lanes().at(lane_id));
    }
    memory_.emplace_back(node_size_++, std::move(lanes), type);
    auto& node = memory_.back();
    for (const auto& [id, _] : node.lanes_) {
      table_.emplace(id, &node);
    }
  }
  for (const auto& [junction_id, junction] : map.junctions()) {
    // <start road -> end road> -> lanes
    std::map<std::pair<uint32_t, uint32_t>, std::vector<PbLane>> group;
    for (auto lane_id : junction.lane_ids()) {
      PbLane lane = map.lanes().at(lane_id);
      if (lane.predecessor_ids_size() == 0 || lane.successor_ids_size() == 0) {
        spdlog::warn(
            "road_graph: Lane {} in junction {} has no predecessor or no "
            "successor. Ignore it.",
            lane_id, junction_id);
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
    for (auto& [_, lanes] : group) {
      memory_.emplace_back(node_size_++, std::move(lanes), type);
      auto& node = memory_.back();
      for (const auto& [id, _] : node.lanes_) {
        table_.emplace(id, &node);
      }
    }
  }
}

void RoadGraph::LinkEdges() {
  for (auto& node : memory_) {
    for (const auto& [lane_id, lane] : node.lanes_) {
      for (auto successor_lane_id : lane.successor_ids()) {
        const RoadNode* next_node = table_.at(successor_lane_id);
        node.next_[next_node].insert(lane_id);
        node.lanes_runtime_[lane_id].next_nodes.push_back(next_node);
      }
    }
  }
}

void RoadGraph::CreatePoiMapper(const PbMap& map) {
  for (const auto& [id, poi] : map.pois()) {
    poi_mapper_.emplace(id, poi.driving_position());
  }
}

}  // namespace graph

}  // namespace routing
