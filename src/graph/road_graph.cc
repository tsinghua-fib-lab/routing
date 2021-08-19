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
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#include "simulet/map_runtime/v1/map_runtime.pb.h"

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

float RoadNode::GetCost(const Point& endpoint) const {
  return base_cost_ + point_.GetDistance(endpoint);
}

RoadGraph::RoadGraph(PbMap map) {
  CreateNodes(map);
  LinkEdges();
  CreatePoiMapper(map);
}

void RoadNode::SetLaneAccess(PbLaneAccessSetting setting) {
  using Type = simulet::proto::map_runtime::v1::LaneAccessType;
  auto lane_id = setting.lane_id();
  assert(lanes_.find(lane_id) != lanes_.cend());
  assert(lanes_runtime_.find(lane_id) != lanes_runtime_.cend());
  auto&& runtime = lanes_runtime_.at(lane_id);
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

std::vector<std::set<uint32_t>> RoadGraph::Search(uint32_t start_lane,
                                                  uint32_t end_lane,
                                                  bool loopback) const {
  if (loopback) {
    assert(start_lane == end_lane);
  }
  // reference: https://zhuanlan.zhihu.com/p/54510444

  // node, parent, priority
  using NodeTuple = std::tuple<const RoadNode*, const RoadNode*, float>;
  auto cmp = [](const NodeTuple& a, const NodeTuple& b) {
    return std::get<2>(a) > std::get<2>(b);
  };
  // nodes waiting for searching
  std::priority_queue<NodeTuple, std::vector<NodeTuple>, decltype(cmp)>
      open_set(cmp);
  // node, parent, nodes searched
  std::unordered_map<const RoadNode*, const RoadNode*> close_set;
  const RoadNode* start_node = table_.at(start_lane);
  const RoadNode* end_node = table_.at(end_lane);
  if (loopback) {
    // skip the start_node and push start_node next into open_set
    for (const auto& [next_node, _] : start_node->next_) {
      open_set.emplace(next_node, start_node,
                       next_node->GetCost(end_node->point_));
    }
  } else {
    open_set.emplace(start_node, nullptr, 0.0f);
  }
  while (!open_set.empty()) {
    // get the node with the least priority from open_set to search
    auto [node, parent, priority] = open_set.top();
    open_set.pop();
    if (node == end_node) {
      // finish
      // node, next
      // use stack to reverse node-link
      std::stack<std::pair<const RoadNode*, const RoadNode*>> stack;
      stack.emplace(node, nullptr);
      while (parent != nullptr) {
        stack.emplace(parent, node);
        node = parent;
        if (loopback && node == start_node) {
          parent = nullptr;
        } else {
          parent = close_set.at(node);
        }
      }
      // find avaliable lanes from node data
      std::vector<std::set<uint32_t>> result;
      assert(!stack.empty());
      while (!stack.empty()) {
        auto [node, next] = stack.top();
        stack.pop();
        if (next == nullptr) {
          result.push_back({end_lane});
        } else {
          result.push_back(node->next_.at(next));
        }
      }
      return result;
    }
    // searched node -> close_set
    // node->next -> open_set
    close_set.emplace(node, parent);
    for (const auto& [next_node, lanes] : node->next_) {
      if (lanes.empty() || close_set.find(next_node) != close_set.cend()) {
        continue;
      } else {
        open_set.emplace(next_node, node,
                         priority + next_node->GetCost(end_node->point_));
      }
    }
  }
  // fail to find the route
  return {};
}

std::vector<std::set<uint32_t>> RoadGraph::Search(
    const PbMapPosition start, const PbMapPosition end) const {
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
    return Search(start_street.lane_id(), start_street.lane_id(), true);
  } else {
    return Search(start_street.lane_id(), end_street.lane_id());
  }
}

void RoadGraph::SetLaneAccess(PbLaneAccessSetting setting) {
  auto id = setting.lane_id();
  table_.at(id)->SetLaneAccess(std::move(setting));
}

void RoadGraph::CreateNodes(const PbMap& map) {
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
    for (const auto& [lane_id, lane] : node.lanes_) {
      for (auto successor_lane_id : lane.successor_ids()) {
        const RoadNode* next_node = (table_.at(successor_lane_id));
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
