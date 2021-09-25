/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-16 9:51:13 pm
 */

#ifndef SRC_GRAPH_ROAD_GRAPH_H_
#define SRC_GRAPH_ROAD_GRAPH_H_

#include <cmath>
#include <simulet/geo/v1/geo.pb.h>
#include <simulet/map/v1/map.pb.h>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <list>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "simulet/map_runtime/v1/map_runtime.pb.h"
#include "simulet/route/v1/route.pb.h"

namespace routing {

namespace graph {

using PbLane = simulet::proto::map::v1::Lane;
using PbMap = simulet::proto::map::v1::Map;
using PbStreetPosition = simulet::proto::geo::v1::StreetPosition;
using PbMapPosition = simulet::proto::geo::v1::MapPosition;
using PbLaneAccessSetting = simulet::proto::map_runtime::v1::LaneAccessSetting;
using PbBatchAccessSetting =
    simulet::proto::map_runtime::v1::BatchAccessSetting;
using PbLaneSet = simulet::proto::route::v1::LaneSet;
using PbDrivingTripBody = simulet::proto::route::v1::DrivingTripBody;

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

class RoadGraph;

// the endpoint of the road/lane
// PS: lanes in junction which have the same predecessor and the same
// successor will be seen as one road
class RoadNode {
  // reference: https://zhuanlan.zhihu.com/p/54510444
  // f(n) = g(n) + h(n)
  // g(n): cost from the starting node
  // h(n): estimated cost to the end node
 public:
  RoadNode();
  RoadNode(int id, std::vector<PbLane> lanes, CostType type);
  std::string String() const;
  float GetCost(const RoadNode& end) const;
  void SetLaneAccess(const PbLaneAccessSetting& setting);

 private:
  friend RoadGraph;
  struct LaneRuntime {
    bool ok = true;
    std::vector<const RoadNode*> next_nodes;
  };

  CostType type_;

  // auto-increasing id or array offset
  int id_;
  std::map<uint32_t, PbLane> lanes_;
  // the average length of the lanes
  float base_cost_ = 0;
  Point point_;

  // next node -> valid lanes
  std::map<const RoadNode*, std::set<uint32_t>> next_;
  // lane id -> is ok, related next nodes
  std::map<uint32_t, LaneRuntime> lanes_runtime_;
};

class RoadGraph {
 public:
  RoadGraph(PbMap map, CostType type);
  void Print() const;
  // search route from the END of start_lane to the END of end_lane
  // loopback=true, search route from the END of start_lane to the START of
  // start_lane and to the END of start_lane.
  // return vector of the lanes set whose ends should be passed
  PbDrivingTripBody Search(uint32_t start_lane, uint32_t end_lane,
                           int64_t revision, bool loopback);
  PbDrivingTripBody Search(const PbMapPosition& start, const PbMapPosition& end,
                           int64_t revision);

  // revision: the etcd access revision used to sync the map access status
  void ParseAndSetLanesAccess(std::string data, int64_t revision);
  void SetLanesAccess(PbBatchAccessSetting settings, int64_t revision);

 private:
  void CreateNodes(const PbMap& map, CostType type);
  void LinkEdges();
  void CreatePoiMapper(const PbMap& map);

  int node_size_ = 0;

  // pseudo null node
  RoadNode null_node;
  // do not use vector because resizing will break the pointer
  std::list<RoadNode> memory_;
  // lane id -> road node
  // it is possible that different keys map into the same node
  std::unordered_map<uint32_t, RoadNode*> table_;
  // poi id -> lane id, lane s
  std::unordered_map<uint32_t, PbStreetPosition> poi_mapper_;

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

#endif  // SRC_GRAPH_ROAD_GRAPH_H_
