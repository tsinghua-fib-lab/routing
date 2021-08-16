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
#include <string>
#include <unordered_map>
#include <vector>
#include "simulet/map/v1/map.pb.h"

namespace routing {

namespace graph {

using PbLane = simulet::proto::map::v1::Lane;
using PbMap = simulet::proto::map::v1::Map;

struct Point {
  float x;
  float y;

  float GetDistance(const Point& other) const {
    // https://stackoverflow.com/questions/2940367/what-is-more-efficient-using-pow-to-square-or-just-multiply-it-with-itself
    return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
  }
};

class RoadGraph;

// the endpoint of the road
// PS: lanes in junction which have the same predecessor and the same
// successor will be seen as one road
class RoadNode {
  // reference: https://zhuanlan.zhihu.com/p/54510444
  // f(n) = g(n) + h(n)
  // g(n): cost from the starting node
  // h(n): estimated cose to the end node
 public:
  explicit RoadNode(std::vector<PbLane> lanes);
  std::string String() const;

 private:
  friend RoadGraph;

  std::unordered_map<uint32_t, PbLane> lanes_;
  // the average length of the lanes
  float base_cost_ = 0;
  Point point_;
  std::vector<const RoadNode*> next_;
};

class RoadGraph {
 public:
  explicit RoadGraph(PbMap map);
  void Print() const;

 private:
  void CreateNodes(PbMap map);
  void LinkEdges();

  // do not use vector because resizing will break the pointer
  std::list<RoadNode> memory_;
  // lane id -> road node
  // it is possible that different keys map into the same node
  std::unordered_map<uint32_t, RoadNode*> table_;
};

}  // namespace graph

}  // namespace routing

#endif  // SRC_GRAPH_ROAD_GRAPH_H_
