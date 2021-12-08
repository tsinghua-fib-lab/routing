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
#include <utility>
#include <vector>
#include "wolong/geo/v1/geo.pb.h"
#include "wolong/map/v1/map.pb.h"
#include "wolong/routing/v1/routing.pb.h"

namespace routing {

namespace graph {

using PbDrivingJourneyBody = wolong::routing::v1::DrivingJourneyBody;
using PbWalkingJourneyBody = wolong::routing::v1::WalkingJourneyBody;
using PbLane = wolong::map::v1::Lane;
using PbMap = wolong::map::v1::Map;
using PbPosition = wolong::geo::v1::Position;
using PbLaneType = wolong::map::v1::LaneType;
using PbNextLaneType = wolong::routing::v1::NextLaneType;
using PbMovingDirection = wolong::routing::v1::MovingDirection;
using PbLanePosition = wolong::geo::v1::LanePosition;
using PbLaneConnectionType = wolong::map::v1::LaneConnectionType;

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
  float GetEuclideanDistance(const Point& other) const {
    float dx = x - other.x, dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }
};

// the endpoint of the lane
class LaneNode {
 public:
  using NextNodesDriving =
      std::vector<std::pair<const LaneNode*, PbNextLaneType>>;
  using NextNodesWalking =
      std::vector<std::pair<const LaneNode*, PbMovingDirection>>;

  LaneNode();
  LaneNode(int index, const PbLane* base, CostType type);
  // 在memory_中的索引
  int index() const;
  // 如果设置为NOENTRY则不ok
  bool ok() const;
  // lane_id
  uint32_t id() const;
  const PbLane* base() const;
  const NextNodesDriving& next_nodes_driving() const;
  const NextNodesWalking& next_nodes_walking(PbMovingDirection direction) const;

  // TODO(zhangjun): lane-change cost
  float GetCostDriving(const LaneNode& end, CostType type,
                       PbNextLaneType relation) const;
  float GetCostWalking(PbMovingDirection direction, const LaneNode& target_lane,
                       float target_s) const;
  float length() const;
  // TODO(张钧): 迁移到新的接口上
  // void SetLaneAccess(const PbLaneAccessSetting& setting);
  void AddNextNodeDriving(const LaneNode* next_node, PbNextLaneType type);
  void AddNextNodeWalkingForward(const LaneNode* next_node,
                                 PbMovingDirection direction);
  void AddNextNodeWalkingBackward(const LaneNode* next_node,
                                  PbMovingDirection direction);
  LaneNode CloneAndClearNextNode(int new_index) const;

 private:
  int index_;
  bool ok_ = true;
  uint32_t id_;
  Point start_point_, end_point_;
  float base_cost_;
  NextNodesDriving next_nodes_driving_;
  NextNodesWalking next_nodes_walking_forward_, next_nodes_walking_backward_;
  float length_;
  const PbLane* base_;
};

class LaneGraph {
 public:
  LaneGraph(PbMap map, CostType type);
  // 搜索从start_lane到end_lane的路径，返回值包含start_lane和end_lane
  PbDrivingJourneyBody SearchDriving(uint32_t start_lane, uint32_t end_lane,
                                     int64_t revision);
  PbDrivingJourneyBody SearchDriving(const PbPosition& start,
                                     const PbPosition& end, int64_t revision);
  PbWalkingJourneyBody SearchWalking(uint32_t start_lane, float start_s,
                                     uint32_t end_lane, float end_s,
                                     int64_t revision);
  PbWalkingJourneyBody SearchWalking(const PbPosition& start,
                                     const PbPosition& end, int64_t revision);

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
  // 存储所有的LaneNode
  std::list<LaneNode> memory_;
  // lane id -> lane node
  std::unordered_map<uint32_t, LaneNode*> lookup_table_;
  // poi id -> lane id, lane s
  std::unordered_map<uint32_t, PbLanePosition> poi_driving_mapper_,
      poi_walking_mapper_;
  CostType type_;

  // access update components

  // current map access revision
  // int revision_ = 0;
  // std::mutex search_mtx_;
  // std::condition_variable search_cv_;
  // std::atomic_bool allow_search_ = true;

  // std::mutex set_access_mtx_;
  // std::condition_variable set_access_cv_;
  // std::atomic_int num_running_search_ = 0;
};

}  // namespace graph

}  // namespace routing

#endif  // SRC_GRAPH_LANE_GRAPH_H_
