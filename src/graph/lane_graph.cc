/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-09-25 7:52:42 pm
 */

#include "graph/lane_graph.h"
#include <spdlog/spdlog.h>
#include <algorithm>
#include <cassert>
#include <condition_variable>
#include <cstdint>
#include <list>
#include <mutex>
#include <queue>
#include <set>
#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>
#include "wolong/map/v1/map.pb.h"

#define IGNORE_MAP_REVISION

namespace routing {

namespace graph {

namespace {

// TODO(zhangjun): absl flag
const float kAverageSpeedInManhattenDistance = 60.0 / 3.6;
const float kLaneChangePenaltyForTime = 0.0;
const float kLaneChangePenaltyForDistance = 0.0;

bool IsLaneChange(PbNextLaneType relation) {
  return relation == PbNextLaneType::NEXT_LANE_TYPE_LEFT ||
         relation == PbNextLaneType::NEXT_LANE_TYPE_RIGHT;
}

}  // namespace

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

LaneNode::LaneNode() = default;

LaneNode::LaneNode(int index, const PbLane* base, CostType type)
    : index_(index), id_(base->id()), base_(base) {
  auto start = base->center_line().nodes().begin();
  start_point_.x = start->x();
  start_point_.y = start->y();
  auto end = base->center_line().nodes().rbegin();
  end_point_.x = end->x();
  end_point_.y = end->y();
  length_ = base->length();
  switch (type) {
    case CostType::kDistance:
      base_cost_ = base->length();
      break;
    case CostType::kTime:
      assert(base->max_speed() > 0.0f);
      base_cost_ = base->length() / base->max_speed();
      break;
    default:
      throw std::runtime_error("lane_graph: not implemented CostType");
  }
}

int LaneNode::index() const { return index_; }

bool LaneNode::ok() const { return ok_; }

uint32_t LaneNode::id() const { return id_; }

const PbLane* LaneNode::base() const { return base_; }

const LaneNode::NextNodesDriving& LaneNode::next_nodes_driving() const {
  return next_nodes_driving_;
}

const LaneNode::NextNodesWalking& LaneNode::next_nodes_walking(
    PbMovingDirection direction) const {
  switch (direction) {
    case PbMovingDirection::MOVING_DIRECTION_FORWARD:
      return next_nodes_walking_forward_;
    case PbMovingDirection::MOVING_DIRECTION_BACKWARD:
      return next_nodes_walking_backward_;
    default:
      assert(false);
  }
}

float LaneNode::GetCostDriving(const LaneNode& end, CostType type,
                               PbNextLaneType relation) const {
  float base_cost = base_cost_;
  if (IsLaneChange(relation)) {
    base_cost = (type == CostType::kTime ? kLaneChangePenaltyForTime
                                         : kLaneChangePenaltyForDistance);
  }
  switch (type) {
    case CostType::kDistance:
      return base_cost + end_point_.GetEuclideanDistance(end.end_point_);
    case CostType::kTime:
      return base_cost + end_point_.GetEuclideanDistance(end.end_point_) /
                             kAverageSpeedInManhattenDistance;
    default:
      throw std::runtime_error("lane_graph: not implemented CostType");
  }
}

float LaneNode::GetCostWalking(PbMovingDirection direction,
                               const LaneNode& target_lane,
                               float target_s) const {
  assert(direction != PbMovingDirection::MOVING_DIRECTION_UNSPECIFIED);
  const auto& p = direction == PbMovingDirection::MOVING_DIRECTION_FORWARD
                      ? end_point_
                      : start_point_;
  return std::min(p.GetEuclideanDistance(target_lane.start_point_) + target_s,
                  p.GetEuclideanDistance(target_lane.end_point_) +
                      target_lane.length_ - target_s);
}

float LaneNode::length() const { return length_; }

// void LaneNode::SetLaneAccess(const PbLaneAccessSetting& setting) {
//   using Type = wolong::map_runtime::v1::LaneAccessType;
//   assert(setting.lane_id() == id());
//   ok_ = setting.type() != Type::LANE_ACCESS_TYPE_NO_ENTRY;
// }

void LaneNode::AddNextNodeDriving(const LaneNode* next_node,
                                  PbNextLaneType type) {
  next_nodes_driving_.emplace_back(next_node, type);
}

void LaneNode::AddNextNodeWalkingForward(const LaneNode* next_node,
                                         PbMovingDirection direction) {
  next_nodes_walking_forward_.emplace_back(next_node, direction);
}

void LaneNode::AddNextNodeWalkingBackward(const LaneNode* next_node,
                                          PbMovingDirection direction) {
  next_nodes_walking_backward_.emplace_back(next_node, direction);
}

LaneNode LaneNode::CloneAndClearNextNode(int new_index) const {
  LaneNode o = *this;
  o.index_ = new_index;
  o.next_nodes_driving_.clear();
  return o;
}

LaneGraph::LaneGraph(PbMap map, CostType type)
    : map_(std::move(map)), type_(type) {
  // add all nodes
  uint32_t index = 0;
  for (const auto& lane : map_.lanes()) {
    // lookup_table_: id -> lane_node_ptr
    lookup_table_.emplace(lane.id(),
                          &memory_.emplace_back(index++, &lane, type_));
  }
  // link edges
  for (auto& node : memory_) {
    const auto& base = *node.base();
    switch (base.type()) {
      case PbLaneType::LANE_TYPE_WALKING: {
        for (const auto& conn : base.successors()) {
          assert(conn.type() !=
                 PbLaneConnectionType::LANE_CONNECTION_TYPE_UNSPECIFIED);
          node.AddNextNodeWalkingForward(
              lookup_table_.at(conn.id()),
              conn.type() == PbLaneConnectionType::LANE_CONNECTION_TYPE_HEAD
                  ? PbMovingDirection::MOVING_DIRECTION_FORWARD
                  : PbMovingDirection::MOVING_DIRECTION_BACKWARD);
        }
        for (const auto& conn : base.predecessors()) {
          assert(conn.type() !=
                 PbLaneConnectionType::LANE_CONNECTION_TYPE_UNSPECIFIED);
          node.AddNextNodeWalkingBackward(
              lookup_table_.at(conn.id()),
              conn.type() == PbLaneConnectionType::LANE_CONNECTION_TYPE_HEAD
                  ? PbMovingDirection::MOVING_DIRECTION_FORWARD
                  : PbMovingDirection::MOVING_DIRECTION_BACKWARD);
        }
      } break;
      case PbLaneType::LANE_TYPE_DRIVING: {
        for (const auto& conn : base.successors()) {
          assert(conn.type() ==
                 PbLaneConnectionType::LANE_CONNECTION_TYPE_HEAD);
          node.AddNextNodeDriving(lookup_table_.at(conn.id()),
                                  PbNextLaneType::NEXT_LANE_TYPE_FORWARD);
        }
        if (base.left_lane_ids().size() > 0) {
          node.AddNextNodeDriving(lookup_table_.at(base.left_lane_ids().at(0)),
                                  PbNextLaneType::NEXT_LANE_TYPE_LEFT);
        }
        if (base.right_lane_ids().size() > 0) {
          node.AddNextNodeDriving(lookup_table_.at(base.right_lane_ids().at(0)),
                                  PbNextLaneType::NEXT_LANE_TYPE_RIGHT);
        }
      } break;
      default:
        assert(false);
    }
  }
  // create poi mapper
  for (const auto& poi : map_.pois()) {
    poi_driving_mapper_.emplace(poi.id(), poi.driving_position());
    poi_walking_mapper_.emplace(poi.id(), poi.walking_position());
  }
}

PbDrivingJourneyBody LaneGraph::SearchDriving(const PbPosition& start,
                                              const PbPosition& end,
                                              int64_t revision) {
  // convert Position into LanePosition
  PbLanePosition start_street, end_street;
  if (start.has_poi_position()) {
    start_street = poi_driving_mapper_.at(start.poi_position().poi_id());
  } else {
    start_street = start.lane_position();
  }
  if (end.has_poi_position()) {
    end_street = poi_driving_mapper_.at(end.poi_position().poi_id());
  } else {
    end_street = end.lane_position();
  }
  // consider start.s and end.s
  if (start_street.lane_id() == end_street.lane_id() &&
      start_street.s() <= end_street.s()) {
    PbDrivingJourneyBody result;
    auto* route = result.add_route();
    route->set_lane_id(start_street.lane_id());
    route->set_next_lane_type(PbNextLaneType::NEXT_LANE_TYPE_LAST);
    return result;
  }
  return SearchDriving(start_street.lane_id(), end_street.lane_id(), revision);
}

PbDrivingJourneyBody LaneGraph::SearchDriving(uint32_t start_lane,
                                              uint32_t end_lane,
                                              int64_t revision) {
  // #ifdef IGNORE_MAP_REVISION
  //   revision = revision_;
  // #endif
  //   {
  //     std::unique_lock<std::mutex> lk(search_mtx_);
  //     search_cv_.wait(lk, [this, revision] {
  //       return allow_search_ == true && revision_ >= revision;
  //     });
  //   }
  //   ++num_running_search_;
  spdlog::debug(
      "lane_graph: Start searching driving route, start={}, end={}, "
      "revision={}",
      start_lane, end_lane, revision);

  PbDrivingJourneyBody result;
  if (start_lane == end_lane) {
    // create temporary nodes for the start lane and its neighbors
    // insight: the pointers of such new nodes are different from
    // those of the nodes with the same ids in this->memory_.
    // For example, if we set start_lane to 100, the new node's id can be seen
    // as 100_1.
    // Thus, we separate the start_lane (and its neighbors) and the end_lane
    // (and its neighbors), although they have the same id
    // and then SearchImpl(...) without loopback will work well.
    int node_size = lookup_table_.size();
    std::set<uint32_t> extra_ids;
    const auto& pb = map_.lanes().at(start_lane);
    extra_ids.insert(pb.id());
    extra_ids.insert(pb.left_lane_ids().cbegin(), pb.left_lane_ids().cend());
    extra_ids.insert(pb.right_lane_ids().cbegin(), pb.right_lane_ids().cend());
    std::list<LaneNode> extra_memory;
    std::unordered_map<uint32_t, LaneNode*> extra_lookup_table;
    for (auto id : extra_ids) {
      auto& ref = extra_memory.emplace_back(
          lookup_table_.at(id)->CloneAndClearNextNode(node_size++));
      extra_lookup_table.emplace(id, &ref);
    }
    for (auto& node : extra_memory) {
      const auto& base = map_.lanes().at(node.id());
      for (const auto& conn : base.successors()) {
        assert(conn.type() == PbLaneConnectionType::LANE_CONNECTION_TYPE_HEAD);
        node.AddNextNodeDriving(lookup_table_.at(conn.id()),
                                PbNextLaneType::NEXT_LANE_TYPE_FORWARD);
      }
      if (base.left_lane_ids().size() > 0) {
        node.AddNextNodeDriving(
            extra_lookup_table.at(base.left_lane_ids().at(0)),
            PbNextLaneType::NEXT_LANE_TYPE_LEFT);
      }
      if (base.right_lane_ids().size() > 0) {
        node.AddNextNodeDriving(
            extra_lookup_table.at(base.right_lane_ids().at(0)),
            PbNextLaneType::NEXT_LANE_TYPE_RIGHT);
      }
    }
    result = SearchImpl(extra_lookup_table.at(start_lane),
                        lookup_table_.at(end_lane), node_size);
  } else {
    result = SearchImpl(lookup_table_.at(start_lane),
                        lookup_table_.at(end_lane), lookup_table_.size());
  }

  spdlog::debug("lane_graph: Finish searching, start={}, end={}, revision={}",
                start_lane, end_lane, revision);
  // return
  // --num_running_search_;
  // {
  //   std::lock_guard<std::mutex> lg(set_access_mtx_);
  //   set_access_cv_.notify_one();
  // }
  return result;
}

PbDrivingJourneyBody LaneGraph::SearchImpl(const LaneNode* start,
                                           const LaneNode* end, int node_size) {
  assert(start != end);
  struct NodeTuple {
    const LaneNode* node;
    const LaneNode* parent;
    // parent通过parent_next_type到达node
    PbNextLaneType parent_next_type;
    float priority;
    // 已经走过的距离
    float passed_distance;
    NodeTuple() = delete;
    NodeTuple(const LaneNode* node, const LaneNode* parent,
              PbNextLaneType parent_next_type, float priority,
              float passed_distance)
        : node(node),
          parent(parent),
          parent_next_type(parent_next_type),
          priority(priority),
          passed_distance(passed_distance) {}
  };
  struct CompareNodeTuple {
    bool operator()(const NodeTuple& a, const NodeTuple& b) {
      return a.priority > b.priority;
    }
  };
  // 距离转优先级
  auto d2p = [this](float distance) {
    return type_ == CostType::kDistance
               ? distance
               : distance / kAverageSpeedInManhattenDistance;
  };
  // 待搜索队列
  std::priority_queue<NodeTuple, std::vector<NodeTuple>, CompareNodeTuple>
      queue;
  // 记录结点是否搜索过
  std::vector<bool> visited_nodes(node_size, false);
  // 记录结点的父结点
  std::vector<std::pair<const LaneNode*, PbNextLaneType>> path(
      node_size, {nullptr, PbNextLaneType::NEXT_LANE_TYPE_UNSPECIFIED});
  PbDrivingJourneyBody result;
  // 记录最佳距离
  std::vector<float> best_distance(lookup_table_.size() + 1, INFINITY);

  queue.emplace(start, nullptr, PbNextLaneType::NEXT_LANE_TYPE_UNSPECIFIED, 0,
                0);
  visited_nodes[start->index()] = true;
  best_distance[start->index()] = 0;

  while (!queue.empty()) {
    auto tuple = queue.top();
    queue.pop();
    if (tuple.node == end) {
      std::vector<wolong::routing::v1::DrivingRouteSegment*> segments;
      auto* node = tuple.parent;
      assert(node);
      auto type = tuple.parent_next_type;
      do {
        auto* seg = new wolong::routing::v1::DrivingRouteSegment();
        seg->set_lane_id(node->id());
        seg->set_next_lane_type(type);
        segments.push_back(seg);
        std::tie(node, type) = path[node->index()];
      } while (node);
      for (auto i = segments.rbegin(), j = segments.rend(); i != j; ++i) {
        result.mutable_route()->AddAllocated(*i);
      }
      auto* seg = result.add_route();
      seg->set_lane_id(end->id());
      seg->set_next_lane_type(PbNextLaneType::NEXT_LANE_TYPE_LAST);
      return result;
    }
    for (auto [next_node, next_type] : tuple.node->next_nodes_driving()) {
      if (!next_node->ok()) {
        continue;
      }
      if (visited_nodes[next_node->index()] &&
          tuple.passed_distance >= best_distance[next_node->index()]) {
        continue;
      }
      queue.emplace(next_node, tuple.node, next_type,
                    d2p(tuple.passed_distance) +
                        next_node->GetCostDriving(*end, type_, next_type),
                    tuple.passed_distance + next_node->length());
      path[next_node->index()] = {tuple.node, next_type};
      best_distance[next_node->index()] = tuple.passed_distance;
      visited_nodes[next_node->index()] = true;
    }
  }
  return result;
}

PbWalkingJourneyBody LaneGraph::SearchWalking(const PbPosition& start,
                                              const PbPosition& end,
                                              int64_t revision) {
  PbLanePosition start_street, end_street;
  if (start.has_poi_position()) {
    start_street = poi_walking_mapper_.at(start.poi_position().poi_id());
  } else {
    start_street = start.lane_position();
  }
  if (end.has_poi_position()) {
    end_street = poi_walking_mapper_.at(end.poi_position().poi_id());
  } else {
    end_street = end.lane_position();
  }
  if (start_street.lane_id() == end_street.lane_id()) {
    PbWalkingJourneyBody result;
    auto* segment = result.add_route();
    segment->set_lane_id(start_street.lane_id());
    segment->set_moving_direction(
        start_street.s() < end_street.s()
            ? PbMovingDirection::MOVING_DIRECTION_FORWARD
            : PbMovingDirection::MOVING_DIRECTION_BACKWARD);
    return result;
  } else {
    return SearchWalking(start_street.lane_id(), start_street.s(),
                         end_street.lane_id(), end_street.s(), revision);
  }
}

PbWalkingJourneyBody LaneGraph::SearchWalking(uint32_t start_lane,
                                              float start_s, uint32_t end_lane,
                                              float end_s, int64_t revision) {
  assert(start_lane != end_lane);
  struct NodeTuple {
    const LaneNode* node;
    const LaneNode* parent;
    PbMovingDirection self_direction;
    float priority;
    // 已经走过的距离
    float passed_distance;
    NodeTuple() = delete;
    NodeTuple(const LaneNode* node, const LaneNode* parent,
              PbMovingDirection self_direction, float priority,
              float passed_distance)
        : node(node),
          parent(parent),
          self_direction(self_direction),
          priority(priority),
          passed_distance(passed_distance) {}
  };
  struct CompareNodeTuple {
    bool operator()(const NodeTuple& a, const NodeTuple& b) {
      return a.priority > b.priority;
    }
  };

  // 方向转0/1
  auto d2i = [](PbMovingDirection direction) {
    assert(direction != PbMovingDirection::MOVING_DIRECTION_UNSPECIFIED);
    return direction == PbMovingDirection::MOVING_DIRECTION_FORWARD ? 0 : 1;
  };

  spdlog::debug(
      "lane_graph: Start searching walking route, start={}, end={}, "
      "revision={}",
      start_lane, end_lane, revision);
  const auto* start = lookup_table_.at(start_lane);
  const auto* end = lookup_table_.at(end_lane);
  // 待搜索队列
  std::priority_queue<NodeTuple, std::vector<NodeTuple>, CompareNodeTuple>
      queue;
  // 记录结点是否搜索过
  std::vector<bool> visited_nodes[2] = {
      std::vector<bool>(lookup_table_.size() + 1, false),
      std::vector<bool>(lookup_table_.size() + 1, false)};
  // 记录结点的父结点
  std::vector<std::pair<const LaneNode*, PbMovingDirection>> path[2] = {
      std::vector<std::pair<const LaneNode*, PbMovingDirection>>(
          lookup_table_.size() + 1,
          {nullptr, PbMovingDirection::MOVING_DIRECTION_UNSPECIFIED}),
      std::vector<std::pair<const LaneNode*, PbMovingDirection>>(
          lookup_table_.size() + 1,
          {nullptr, PbMovingDirection::MOVING_DIRECTION_UNSPECIFIED})};
  // 记录最佳距离
  std::vector<float> best_distance[2] = {
      std::vector<float>(lookup_table_.size() + 1, INFINITY),
      std::vector<float>(lookup_table_.size() + 1, INFINITY)};
  PbWalkingJourneyBody result;
  // 创建反向结点
  auto start_backward = start->CloneAndClearNextNode(lookup_table_.size());
  queue.emplace(start, nullptr, PbMovingDirection::MOVING_DIRECTION_FORWARD,
                start->length() - start_s, start->length() - start_s);
  queue.emplace(&start_backward, nullptr,
                PbMovingDirection::MOVING_DIRECTION_BACKWARD, start_s, start_s);
  visited_nodes[0][start->index()] = true;
  visited_nodes[1][start->index()] = true;
  best_distance[0][start->index()] = 0;
  best_distance[1][start->index()] = 0;
  while (!queue.empty()) {
    auto tuple = queue.top();
    queue.pop();
    if (tuple.node == end) {
      std::vector<wolong::routing::v1::WalkingRouteSegment*> segments;
      auto* node = tuple.node;
      auto direction = tuple.self_direction;
      do {
        auto* seg = new wolong::routing::v1::WalkingRouteSegment();
        seg->set_lane_id(node->id());
        seg->set_moving_direction(direction);
        segments.push_back(seg);
        std::tie(node, direction) = path[d2i(direction)][node->index()];
      } while (node);
      for (auto i = segments.rbegin(), j = segments.rend(); i != j; ++i) {
        result.mutable_route()->AddAllocated(*i);
      }
      return result;
    }
    for (auto [next_node, next_direction] :
         tuple.node->next_nodes_walking(tuple.self_direction)) {
      if (!next_node->ok()) {
        continue;
      }
      if (visited_nodes[d2i(next_direction)][next_node->index()] &&
          tuple.passed_distance >=
              best_distance[d2i(next_direction)][next_node->index()]) {
        continue;
      }
      auto passed_distance = tuple.passed_distance;
      float priority = 0;
      if (next_node == end) {
        switch (next_direction) {
          case PbMovingDirection::MOVING_DIRECTION_FORWARD:
            priority = passed_distance += end_s;
            break;
          case PbMovingDirection::MOVING_DIRECTION_BACKWARD:
            priority = passed_distance += next_node->length() - end_s;
            break;
          default:
            assert(false);
        }
      } else {
        passed_distance += next_node->length();
        priority = passed_distance +
                   next_node->GetCostWalking(next_direction, *end, end_s);
      }
      queue.emplace(next_node, tuple.node, next_direction, priority,
                    passed_distance);
      path[d2i(next_direction)][next_node->index()] = {tuple.node,
                                                       tuple.self_direction};
      best_distance[d2i(next_direction)][next_node->index()] =
          tuple.passed_distance;
      visited_nodes[d2i(next_direction)][next_node->index()] = true;
    }
  }
  return result;
}

// void LaneGraph::ParseAndSetLanesAccess(std::string data, int64_t revision) {
//   if (revision <= revision_) {
//     spdlog::warn(
//         "lane_graph: received access revision {} <= the last revision {}. "
//         "Ignore it",
//         revision, revision_);
//     return;
//   }
//   PbBatchAccessSetting settings;
//   if (!settings.ParseFromString(std::move(data))) {
//     spdlog::warn("lane_graph: cannot parse lane access. Ignore it");
//     return;
//   }
//   SetLanesAccess(std::move(settings), revision);
// }

// void LaneGraph::SetLanesAccess(PbBatchAccessSetting settings,
//                                int64_t revision) {
//   spdlog::debug("lane_graph: waiting for the running searching
//   (revision={})",
//                 revision);
//   {
//     allow_search_ = false;
//     std::unique_lock<std::mutex> lk(set_access_mtx_);
//     set_access_cv_.wait(lk, [this] { return num_running_search_ == 0; });
//   }
//   if (revision <= revision_) {
//     spdlog::warn(
//         "lane_graph: received access revision {} <= the last revision {}. "
//         "Ignore it",
//         revision, revision_);
//     return;
//   }
//   spdlog::debug("lane_graph: apply {} with revision {}",
//                 settings.ShortDebugString(), revision);
//   for (const auto& one : settings.lanes()) {
//     lookup_table_.at(one.lane_id())->SetLaneAccess(one);
//   }
//   revision_ = revision;
//   {
//     allow_search_ = true;
//     std::lock_guard<std::mutex> lg(search_mtx_);
//     search_cv_.notify_all();
//   }
// }
}  // namespace graph

}  // namespace routing
