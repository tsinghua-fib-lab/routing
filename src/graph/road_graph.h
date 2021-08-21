/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-16 9:51:13 pm
 */

#ifndef SRC_GRAPH_ROAD_GRAPH_H_
#define SRC_GRAPH_ROAD_GRAPH_H_

#include <cmath>
#include <cstdint>
#include <list>
#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "simulet/geo/v1/geo.pb.h"
#include "simulet/map/v1/map.pb.h"
#include "simulet/map_runtime/v1/map_runtime.pb.h"
#include "simulet/route/v1/route.pb.h"

namespace routing {

namespace graph {

using PbLane = simulet::proto::map::v1::Lane;
using PbMap = simulet::proto::map::v1::Map;
using PbStreetPosition = simulet::proto::geo::v1::StreetPosition;
using PbMapPosition = simulet::proto::geo::v1::MapPosition;
using PbLaneAccessSetting = simulet::proto::map_runtime::v1::LaneAccessSetting;
using PbLaneSet = simulet::proto::route::v1::LaneSet;
using PbDrivingTripBody = simulet::proto::route::v1::DrivingTripBody;

struct Point {
  float x;
  float y;

  float GetDistance(const Point& other) const {
    // https://stackoverflow.com/questions/2940367/what-is-more-efficient-using-pow-to-square-or-just-multiply-it-with-itself
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
  explicit RoadNode(std::vector<PbLane> lanes);
  std::string String() const;
  float GetCost(const RoadNode& end) const;
  void SetLaneAccess(PbLaneAccessSetting setting);

 private:
  friend RoadGraph;
  struct LaneRuntime {
    bool ok = true;
    std::vector<const RoadNode*> next_nodes;
  };

  std::map<uint32_t, PbLane> lanes_;
  // the average length of the lanes
  float base_cost_ = 0;
  Point point_;

  // next node -> valid lanes
  std::map<const RoadNode*, std::set<uint32_t>> next_;
  // lane id -> is ok, related next nodes
  std::map<uint32_t, LaneRuntime> lanes_runtime_;

  std::unordered_map<const RoadNode*, float> cost_cache_;
};

class RoadGraph {
 public:
  explicit RoadGraph(PbMap map);
  void Print() const;
  // search route from the END of start_lane to the END of end_lane
  // loopback=true, search route from the END of start_lane to the START of
  // start_lane.
  // return vector of the lanes set whose ends should be passed
  PbDrivingTripBody Search(uint32_t start_lane, uint32_t end_lane,
                           bool loopback = false) const;
  PbDrivingTripBody Search(const PbMapPosition start,
                           const PbMapPosition end) const;
  void SetLaneAccess(PbLaneAccessSetting setting);

 private:
  void CreateNodes(const PbMap& map);
  void LinkEdges();
  void CreatePoiMapper(const PbMap& map);

  // do not use vector because resizing will break the pointer
  std::list<RoadNode> memory_;
  // lane id -> road node
  // it is possible that different keys map into the same node
  std::unordered_map<uint32_t, RoadNode*> table_;
  // poi id -> lane id, lane s
  std::unordered_map<uint32_t, PbStreetPosition> poi_mapper_;
};

}  // namespace graph

}  // namespace routing

#endif  // SRC_GRAPH_ROAD_GRAPH_H_
