/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-09-25 7:52:42 pm
 */

#include "graph/lane_graph.h"
#include <spdlog/spdlog.h>
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
#include "simulet/map/v1/map.pb.h"
#include "simulet/map_runtime/v1/map_runtime.pb.h"

#define IGNORE_MAP_REVISION

namespace routing {

namespace graph {

namespace {

// TODO(zhangjun): absl flag
const float kAverageSpeedInManhattenDistance = 60.0 / 3.6;
const float kLaneChangePenaltyForTime = 0.0;
const float kLaneChangePenaltyForDistance = 0.0;

// node, parent, priority
struct NodeTuple {
  const LaneNode* node;
  const LaneNode* parent;
  // the relation between parent and node: parent -> parent_next_type -> node
  PbNextLaneType parent_next_type;
  float priority;
  NodeTuple() = delete;
  NodeTuple(const LaneNode* node, const LaneNode* parent,
            PbNextLaneType parent_next_type, float priority)
      : node(node),
        parent(parent),
        parent_next_type(parent_next_type),
        priority(priority) {}
};

bool CompareNodeTuple(const NodeTuple& a, const NodeTuple& b) {
  return a.priority > b.priority;
}

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

LaneNode::LaneNode(int index, const PbLane& base, CostType type)
    : index_(index), id_(base.id()) {
  auto end = base.center_line().nodes().rbegin();
  point_.x = end->x();
  point_.y = end->y();
  switch (type) {
    case CostType::kDistance:
      base_cost_ = base.length();
      break;
    case CostType::kTime:
      assert(base.max_speed() > 0.0f);
      base_cost_ = base.length() / base.max_speed();
      break;
    default:
      throw std::runtime_error("lane_graph: not implemented CostType");
  }
}

int LaneNode::index() const { return index_; }

bool LaneNode::ok() const { return ok_; }

uint32_t LaneNode::id() const { return id_; }

const LaneNode::NextNodes& LaneNode::next_nodes() const { return next_nodes_; }

float LaneNode::GetCost(const LaneNode& end, CostType type,
                        PbNextLaneType relation) const {
  float base_cost = base_cost_;
  if (IsLaneChange(relation)) {
    base_cost = (type == CostType::kTime ? kLaneChangePenaltyForTime
                                         : kLaneChangePenaltyForDistance);
  }
  switch (type) {
    case CostType::kDistance:
      return base_cost + point_.GetManhattanDistance(end.point_);
    case CostType::kTime:
      return base_cost + point_.GetManhattanDistance(end.point_) /
                             kAverageSpeedInManhattenDistance;
    default:
      throw std::runtime_error("lane_graph: not implemented CostType");
  }
}

void LaneNode::SetLaneAccess(const PbLaneAccessSetting& setting) {
  using Type = simulet::proto::map_runtime::v1::LaneAccessType;
  assert(setting.lane_id() == id());
  ok_ = setting.type() != Type::LANE_ACCESS_TYPE_NO_ENTRY;
}

void LaneNode::AddNextNode(const LaneNode* next_node, PbNextLaneType type) {
  next_nodes_.emplace_back(next_node, type);
}

LaneNode LaneNode::CloneAndClearNextNode(int new_index) {
  LaneNode o = *this;
  o.index_ = new_index;
  o.next_nodes_.clear();
  return o;
}

LaneGraph::LaneGraph(PbMap map, CostType type)
    : map_(std::move(map)), type_(type) {
  // add all nodes
  for (const auto& [id, lane] : map_.lanes()) {
    // lookup_table_: id -> lane_node_ptr
    lookup_table_.emplace(id, &memory_.emplace_back(node_size_++, lane, type_));
  }
  // link edges
  for (auto& node : memory_) {
    const auto& base = map_.lanes().at(node.id());
    for (auto id : base.successor_ids()) {
      node.AddNextNode(lookup_table_.at(id),
                       PbNextLaneType::NEXT_LANE_TYPE_FORWARD);
    }
    if (base.left_lane_ids().size() > 0) {
      node.AddNextNode(lookup_table_.at(base.left_lane_ids().at(0)),
                       PbNextLaneType::NEXT_LANE_TYPE_LEFT);
    }
    if (base.right_lane_ids().size() > 0) {
      node.AddNextNode(lookup_table_.at(base.right_lane_ids().at(0)),
                       PbNextLaneType::NEXT_LANE_TYPE_RIGHT);
    }
  }
  // create poi mapper
  for (const auto& [id, poi] : map_.pois()) {
    poi_mapper_.emplace(id, poi.driving_position());
  }
}

PbDrivingTripBody LaneGraph::Search(uint32_t start_lane, uint32_t end_lane,
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
      "lane_graph: Start searching, start={}, end={}, revision={}, loopback={}",
      start_lane, end_lane, revision, loopback);

  PbDrivingTripBody result;
  if (loopback) {
    // create temporary nodes for the start lane and its neighbors
    // insight: the pointers of such new nodes are different from
    // those of the nodes with the same ids in this->memory_.
    // For example, if we set start_lane to 100, the new node's id can be seen
    // as 100_1.
    // Thus, we separate the start_lane (and its neighbors) and the end_lane
    // (and its neighbors), although they have the same id
    // and then SearchImpl(...) without loopback will work well.
    int node_size = node_size_;
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
      for (auto id : base.successor_ids()) {
        node.AddNextNode(lookup_table_.at(id),
                         PbNextLaneType::NEXT_LANE_TYPE_FORWARD);
      }
      if (base.left_lane_ids().size() > 0) {
        node.AddNextNode(extra_lookup_table.at(base.left_lane_ids().at(0)),
                         PbNextLaneType::NEXT_LANE_TYPE_LEFT);
      }
      if (base.right_lane_ids().size() > 0) {
        node.AddNextNode(extra_lookup_table.at(base.right_lane_ids().at(0)),
                         PbNextLaneType::NEXT_LANE_TYPE_RIGHT);
      }
    }
    result = SearchImpl(extra_lookup_table.at(start_lane),
                        lookup_table_.at(end_lane), node_size);
  } else {
    result = SearchImpl(lookup_table_.at(start_lane),
                        lookup_table_.at(end_lane), node_size_);
  }

  spdlog::debug(
      "lane_graph: Finish searching, start={}, end={}, revision={}, "
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

PbDrivingTripBody LaneGraph::Search(const PbMapPosition& start,
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
  bool loopback = start_street.lane_id() == end_street.lane_id() &&
                  start_street.s() > end_street.s();
  return Search(start_street.lane_id(), end_street.lane_id(), revision,
                loopback);
}

void LaneGraph::ParseAndSetLanesAccess(std::string data, int64_t revision) {
  if (revision <= revision_) {
    spdlog::warn(
        "lane_graph: received access revision {} <= the last revision {}. "
        "Ignore it",
        revision, revision_);
    return;
  }
  PbBatchAccessSetting settings;
  if (!settings.ParseFromString(std::move(data))) {
    spdlog::warn("lane_graph: cannot parse lane access. Ignore it");
    return;
  }
  SetLanesAccess(std::move(settings), revision);
}

void LaneGraph::SetLanesAccess(PbBatchAccessSetting settings,
                               int64_t revision) {
  spdlog::debug("lane_graph: waiting for the running searching (revision={})",
                revision);
  {
    allow_search_ = false;
    std::unique_lock<std::mutex> lk(set_access_mtx_);
    set_access_cv_.wait(lk, [this] { return num_running_search_ == 0; });
  }
  if (revision <= revision_) {
    spdlog::warn(
        "lane_graph: received access revision {} <= the last revision {}. "
        "Ignore it",
        revision, revision_);
    return;
  }
  spdlog::debug("lane_graph: apply {} with revision {}",
                settings.ShortDebugString(), revision);
  for (const auto& one : settings.lanes()) {
    lookup_table_.at(one.lane_id())->SetLaneAccess(one);
  }
  revision_ = revision;
  {
    allow_search_ = true;
    std::lock_guard<std::mutex> lg(search_mtx_);
    search_cv_.notify_all();
  }
}

PbDrivingTripBody LaneGraph::SearchImpl(const LaneNode* start,
                                        const LaneNode* end, int node_size) {
  // reference: https://zhuanlan.zhihu.com/p/54510444

  // nodes waiting for searching
  std::priority_queue<NodeTuple, std::vector<NodeTuple>,
                      decltype(&CompareNodeTuple)>
      open_queue(&CompareNodeTuple);
  // node index -> visited, in open_queue now or has been in open_queue
  std::vector<bool> visited_nodes;
  visited_nodes.resize(node_size, false);
  // node index -> parent (not nullptr), parent_next_type
  std::vector<std::tuple<const LaneNode*, PbNextLaneType>> close_set;
  close_set.resize(node_size,
                   {nullptr, PbNextLaneType::NEXT_LANE_TYPE_UNKNOWN});
  PbDrivingTripBody result;

  // node, parent, parent_next_type, priority
  open_queue.emplace(start, &kNullNode, PbNextLaneType::NEXT_LANE_TYPE_UNKNOWN,
                     0.0f);
  visited_nodes[start->index()] = true;

  while (!open_queue.empty()) {
    // get the node with the least priority from open_set to search
    NodeTuple tuple = open_queue.top();
    open_queue.pop();
    if (tuple.node == end) {
      // finish, use stack to reverse node-link
      const LaneNode* parent = tuple.parent;
      PbNextLaneType parent_next_type = tuple.parent_next_type;
      const LaneNode* node = tuple.node;
      PbNextLaneType node_next_type = PbNextLaneType::NEXT_LANE_TYPE_LAST;
      // node, next_lane_type: node -> next_lane_type [ -> next_node ]
      std::stack<std::tuple<const LaneNode*, PbNextLaneType>> stack;
      while (true) {
        stack.emplace(node, node_next_type);
        if (parent == &kNullNode) {
          break;
        }
        node = parent;
        node_next_type = parent_next_type;
        std::tie(parent, parent_next_type) = close_set[node->index()];
        assert(parent != nullptr);
      }
      // find avaliable lanes from node data
      assert(!stack.empty());
      while (!stack.empty()) {
        auto [node, node_next_type] = stack.top();
        stack.pop();
        auto* segment = result.add_route();
        segment->set_lane_id(node->id());
        segment->set_next_lane_type(node_next_type);
      }
      return result;
    }
    // searched node -> close_set
    // node->next -> open_set
    assert(std::get<0>(close_set[tuple.node->index()]) == nullptr);
    close_set[tuple.node->index()] =
        std::make_tuple(tuple.parent, tuple.parent_next_type);
    for (auto [next_node, next_type] : tuple.node->next_nodes()) {
      if (visited_nodes[next_node->index()]) {
        continue;
      }
      if (!next_node->ok()) {
        continue;
      }
      // node, parent, parent_next_type, priority
      open_queue.emplace(
          next_node, tuple.node, next_type,
          tuple.priority + next_node->GetCost(*end, type_, next_type));
      visited_nodes[next_node->index()] = true;
    }
  }
  return result;
}

}  // namespace graph

}  // namespace routing
