/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-09-25 7:52:35 pm
 */

#ifndef SRC_GRAPH_LANE_GRAPH_H_
#define SRC_GRAPH_LANE_GRAPH_H_

#include <cmath>
#include <condition_variable>
#include <list>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>
#include "wolong/geo/v1/geo.pb.h"
#include "wolong/map/v1/map.pb.h"
#include "wolong/routing/v1/routing.pb.h"

namespace routing {

namespace graph {

using PbDrivingJourneyBody = wolong::routing::v1::DrivingJourneyBody;
using PbLane = wolong::map::v1::Lane;
using PbMap = wolong::map::v1::Map;
using PbMapPosition = wolong::geo::v1::MapPosition;
using PbNextLaneType = wolong::routing::v1::NextLaneType;
using PbLanePosition = wolong::geo::v1::LanePosition;

enum class CostType {
  // the cost is distance, which is used to find the shortest path
  kDistance,
  // the cost is time, which is used to find the fastest path
  kTime
};

CostType ParseStringToCostType(const std::string& s);

struct Point {
  float x;
  float y;

  float GetManhattanDistance(const Point& other) const {
    return std::abs(x - other.x) + std::abs(y - other.y);
  }
};

// the endpoint of the lane
class LaneNode {
  // reference: https://zhuanlan.zhihu.com/p/54510444
  // f(n) = g(n) + h(n)
  // g(n): cost from the starting node
  // h(n): estimated cost to the end node
 public:
  using NextNodes = std::vector<std::tuple<const LaneNode*, PbNextLaneType>>;

  LaneNode();
  LaneNode(int index, const PbLane& base, CostType type);

  int index() const;
  bool ok() const;
  uint32_t id() const;
  const NextNodes& next_nodes() const;

  // TODO(zhangjun): lane-change cost
  float GetCost(const LaneNode& end, CostType type,
                PbNextLaneType relation) const;
  // TODO(张钧): 迁移到新的接口上
  // void SetLaneAccess(const PbLaneAccessSetting& setting);
  void AddNextNode(const LaneNode* next_node, PbNextLaneType type);
  LaneNode CloneAndClearNextNode(int new_index);

 private:
  // for vector index
  int index_;
  // if the lane is set to NOENTRY, ok_ = false
  bool ok_ = true;
  // lane id
  uint32_t id_;
  Point point_;
  float base_cost_;
  NextNodes next_nodes_;
};

class LaneGraph {
 public:
  LaneGraph(PbMap map, CostType type);
  // search route from the END of start_lane to the END of end_lane
  // loopback=true, search route from the END of start_lane to the START of
  // start_lane and to the END of start_lane.
  // return vector of the lanes set whose ends should be passed
  PbDrivingJourneyBody Search(uint32_t start_lane, uint32_t end_lane,
                              int64_t revision, bool loopback);
  PbDrivingJourneyBody Search(const PbMapPosition& start,
                              const PbMapPosition& end, int64_t revision);

  // revision: the etcd access revision used to sync the map access status
  // TODO(张钧): 迁移到新的接口上
  // void ParseAndSetLanesAccess(std::string data, int64_t revision);
  // void SetLanesAccess(PbBatchAccessSetting settings, int64_t revision);

 private:
  PbDrivingJourneyBody SearchImpl(const LaneNode* start, const LaneNode* end,
                                  int node_size);

  // pseudo null node
  inline static const LaneNode kNullNode;

  PbMap map_;
  int node_size_ = 0;
  // do not use vector because resizing will break the pointer
  std::list<LaneNode> memory_;
  // lane id -> lane node
  std::unordered_map<uint32_t, LaneNode*> lookup_table_;
  // poi id -> lane id, lane s
  std::unordered_map<uint32_t, PbLanePosition> poi_mapper_;
  CostType type_;

  // access update components

  // current map access revision
  int revision_ = 0;
  std::mutex search_mtx_;
  std::condition_variable search_cv_;
  std::atomic_bool allow_search_ = true;

  std::mutex set_access_mtx_;
  std::condition_variable set_access_cv_;
  std::atomic_int num_running_search_ = 0;
};

}  // namespace graph

}  // namespace routing

#endif  // SRC_GRAPH_LANE_GRAPH_H_
