package router

import (
	"fmt"
	"math"
	"sort"

	"git.fiblab.net/general/common/v2/geometry"
	"git.fiblab.net/general/common/v2/protoutil"
	geov2 "git.fiblab.net/sim/protos/v2/go/city/geo/v2"
	mapv2 "git.fiblab.net/sim/protos/v2/go/city/map/v2"
	routingv2 "git.fiblab.net/sim/protos/v2/go/city/routing/v2"
	"git.fiblab.net/sim/routing/v2/router/algo"
)

const (
	PERSON_SPEED = 1.1
)

type WalkHeuristics struct {
}

func (h WalkHeuristics) HeuristicEuclidean(p1 geometry.Point, p2 geometry.Point) float64 {
	return geometry.Distance(p1, p2) / PERSON_SPEED
}
func (h WalkHeuristics) HeuristicBus(attr algo.WalkNodeAttr, fromEdgeAttrs []*routingv2.WalkingRouteSegment, pEnd geometry.Point, time float64) float64 {
	return math.Inf(0)
}
func (r *Router) buildWalkGraph() {
	walkGraph := algo.NewSearchGraph[algo.WalkNodeAttr, *routingv2.WalkingRouteSegment](
		false,
		WalkHeuristics{},
	)
	// 处理walking lane的nodeId，进行合并
	walkingLanes := make(map[int32]*Lane)
	for _, lane := range r.lanes {
		if lane.Type == mapv2.LaneType_LANE_TYPE_WALKING {
			walkingLanes[lane.Id] = lane
		}
	}
	// 将connection连接关系加入mergeGraph
	for _, lane := range walkingLanes {
		// 遍历所有前驱，如果对接的点已经分配了NodeId，则采用之
		// 否则，新建一个nodeId，并分配给所有对接的点
		if _, headOk := lane.WalkNodeIds[algo.HEAD]; !headOk {
			for _, c := range lane.Predecessors {
				if c.Type == mapv2.LaneConnectionType_LANE_CONNECTION_TYPE_HEAD {
					if v, ok := walkingLanes[c.Id].WalkNodeIds[algo.HEAD]; ok {
						lane.WalkNodeIds[algo.HEAD] = v
						headOk = true
						break
					}
				} else {
					if v, ok := walkingLanes[c.Id].WalkNodeIds[algo.TAIL]; ok {
						lane.WalkNodeIds[algo.HEAD] = v
						headOk = true
						break
					}
				}
			}
			if !headOk {
				lane.WalkNodeIds[algo.HEAD] = walkGraph.InitNode(
					lane.line[0],
					algo.WalkNodeAttr{ID: -1},
					false,
				)
				// 给所有对接的lane分配nodeId
				for _, c := range lane.Predecessors {
					if c.Type == mapv2.LaneConnectionType_LANE_CONNECTION_TYPE_HEAD {
						walkingLanes[c.Id].WalkNodeIds[algo.HEAD] = lane.WalkNodeIds[algo.HEAD]
					} else {
						walkingLanes[c.Id].WalkNodeIds[algo.TAIL] = lane.WalkNodeIds[algo.HEAD]
					}
				}
			}
		}
		if _, tailOk := lane.WalkNodeIds[algo.TAIL]; !tailOk {
			// 遍历所有后继，如果对接的点已经分配了NodeId，则采用之
			// 否则，新建一个nodeId，并分配给所有对接的点
			for _, c := range lane.Successors {
				if c.Type == mapv2.LaneConnectionType_LANE_CONNECTION_TYPE_HEAD {
					if v, ok := walkingLanes[c.Id].WalkNodeIds[algo.HEAD]; ok {
						lane.WalkNodeIds[algo.TAIL] = v
						tailOk = true
						break
					}
				} else {
					if v, ok := walkingLanes[c.Id].WalkNodeIds[algo.TAIL]; ok {
						lane.WalkNodeIds[algo.TAIL] = v
						tailOk = true
						break
					}
				}
			}
			if !tailOk {
				lane.WalkNodeIds[algo.TAIL] = walkGraph.InitNode(
					lane.line[len(lane.line)-1],
					algo.WalkNodeAttr{ID: -1},
					false,
				)
				// 给所有对接的lane分配nodeId
				for _, c := range lane.Successors {
					if c.Type == mapv2.LaneConnectionType_LANE_CONNECTION_TYPE_HEAD {
						walkingLanes[c.Id].WalkNodeIds[algo.HEAD] = lane.WalkNodeIds[algo.TAIL]
					} else {
						walkingLanes[c.Id].WalkNodeIds[algo.TAIL] = lane.WalkNodeIds[algo.TAIL]
					}
				}
			}
		}
	}
	// 将aoi加入graph (node & edge)
	// lane id -> []node{S, NodeId}
	for _, lane := range walkingLanes {
		lane.Nodes = []algo.WalkLaneNode{
			{S: 0, NodeId: lane.WalkNodeIds[algo.HEAD]},
			{S: lane.Length, NodeId: lane.WalkNodeIds[algo.TAIL]},
		}
	}
	for _, aoi := range r.aois {
		aoi.WalkOutNodeId = walkGraph.InitNode(
			aoi.CenterPoint,
			algo.WalkNodeAttr{ID: aoi.Id, IsAoi: true},
			false,
		)
		aoi.WalkInNodeId = walkGraph.InitNode(
			aoi.CenterPoint,
			algo.WalkNodeAttr{ID: aoi.Id, IsAoi: true},
			true,
		)
		for _, p := range aoi.WalkingPositions {
			lane := r.lanes[p.LaneId]
			nodeId := walkGraph.InitNode(
				lane.GetPositionByS(p.S),
				algo.WalkNodeAttr{ID: lane.Id},
				false,
			)
			lane.Nodes = append(lane.Nodes,
				algo.WalkLaneNode{S: p.S, NodeId: nodeId},
			)
			// 连接walkOutNode和walkInNode
			walkGraph.InitEdge(
				aoi.WalkOutNodeId, nodeId,
				[]float64{0.1},
				nil,
			)
			walkGraph.InitEdge(
				nodeId, aoi.WalkInNodeId,
				[]float64{0.1},
				nil,
			)
		}
	}
	// 对Lane中的node进行排序
	for _, lane := range walkingLanes {
		sort.Slice(lane.Nodes, func(i, j int) bool {
			return lane.Nodes[i].S < lane.Nodes[j].S
		})
		// 在node间建立无向图
		for i := 0; i < len(lane.Nodes)-1; i++ {
			cost := (lane.Nodes[i+1].S - lane.Nodes[i].S) / PERSON_SPEED
			walkGraph.InitEdge(
				lane.Nodes[i].NodeId, lane.Nodes[i+1].NodeId,
				[]float64{cost},
				&routingv2.WalkingRouteSegment{
					LaneId:          lane.Id,
					MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_FORWARD,
				},
			)
			walkGraph.InitEdge(
				lane.Nodes[i+1].NodeId, lane.Nodes[i].NodeId,
				[]float64{cost},
				&routingv2.WalkingRouteSegment{
					LaneId:          lane.Id,
					MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_BACKWARD,
				},
			)
		}
	}

	r.walkGraph = walkGraph
}

type walkSearchNode struct {
	nodeId        int
	extraEdgeAttr *routingv2.WalkingRouteSegment
	extraCost     float64
}

func (r *Router) toWalkStartSearchNode(pb *geov2.Position) []walkSearchNode {
	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
		aoiNode := r.aois[aoiPosition.GetAoiId()].WalkOutNodeId
		return []walkSearchNode{
			{nodeId: aoiNode, extraEdgeAttr: nil, extraCost: 0},
		}
	} else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
		lane := r.lanes[lanePosition.GetLaneId()]
		s := lanePosition.GetS()
		// 在lane的.Nodes中找到s所在位置前后的两个node
		ind := sort.Search(len(lane.Nodes), func(i int) bool {
			return lane.Nodes[i].S >= s
		})
		if ind == 0 {
			ind = 1
		}
		return []walkSearchNode{
			{
				nodeId: lane.Nodes[ind-1].NodeId,
				extraEdgeAttr: &routingv2.WalkingRouteSegment{
					LaneId:          lane.Id,
					MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_BACKWARD,
				},
				extraCost: (s - lane.Nodes[ind-1].S) / PERSON_SPEED,
			},
			{
				nodeId: lane.Nodes[ind].NodeId,
				extraEdgeAttr: &routingv2.WalkingRouteSegment{
					LaneId:          lane.Id,
					MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_FORWARD,
				},
				extraCost: (lane.Nodes[ind].S - s) / PERSON_SPEED,
			},
		}
	} else {
		panic("wrong type")
	}
}

func (r *Router) toWalkEndSearchNode(pb *geov2.Position) []walkSearchNode {
	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
		aoiNode := r.aois[aoiPosition.GetAoiId()].WalkInNodeId
		return []walkSearchNode{
			{nodeId: aoiNode, extraEdgeAttr: nil, extraCost: 0},
		}
	} else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
		lane := r.lanes[lanePosition.GetLaneId()]
		s := lanePosition.GetS()
		// 在lane的.Nodes中找到s所在位置前后的两个node
		ind := sort.Search(len(lane.Nodes), func(i int) bool {
			return lane.Nodes[i].S >= s
		})
		if ind == 0 {
			ind = 1
		}
		return []walkSearchNode{
			{
				nodeId: lane.Nodes[ind-1].NodeId,
				extraEdgeAttr: &routingv2.WalkingRouteSegment{
					LaneId:          lane.Id,
					MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_FORWARD,
				},
				extraCost: (s - lane.Nodes[ind-1].S) / PERSON_SPEED,
			},
			{
				nodeId: lane.Nodes[ind].NodeId,
				extraEdgeAttr: &routingv2.WalkingRouteSegment{
					LaneId:          lane.Id,
					MovingDirection: routingv2.MovingDirection_MOVING_DIRECTION_BACKWARD,
				},
				extraCost: (lane.Nodes[ind].S - s) / PERSON_SPEED,
			},
		}
	} else {
		panic("wrong type")
	}
}

type SearchLane struct {
	Head int
	Tail int
	S    float64
}

func (r *Router) SearchWalking(
	start, end *geov2.Position, time float64,
) (segments []*routingv2.WalkingRouteSegment, cost float64, err error) {
	// panic recover
	defer func() {
		if e := recover(); e != nil {
			segments = []*routingv2.WalkingRouteSegment{}
			cost = math.Inf(0)
			err = fmt.Errorf("panic: SearchWalking %v with input start=%v, end=%v, time=%v", e, start, end, time)
			log.Errorln(err)
		}
	}()

	// 转换为搜索图中结点
	startNodes := r.toWalkStartSearchNode(start)
	endNodes := r.toWalkEndSearchNode(end)
	// startLength := r.walkGraph.EdgesLength()[r.walkGraph.EdgesLookup()[startNode]][0]
	// endLength := r.walkGraph.EdgesLength()[r.walkGraph.EdgesLookup()[endNode]][0]
	var bestPath []algo.PathItem[algo.WalkNodeAttr, *routingv2.WalkingRouteSegment]
	bestCost := math.Inf(0)
	var bestStartNode, bestEndNode walkSearchNode

	for _, startNode := range startNodes {
		for _, endNode := range endNodes {
			pt, cost := r.walkGraph.ShortestPath(startNode.nodeId, endNode.nodeId, time)
			// 跳过正无穷
			if cost == math.Inf(0) {
				continue
			}
			if len(pt) > 1 { // 不是原地不动
				if startNode.extraEdgeAttr != nil {
					cost += startNode.extraCost
				}
				// 如果终点节点处额外增加的cost与pt[len(pt)-2].edge方向相反，则为减少量
				if endNode.extraEdgeAttr != nil {
					cost += endNode.extraCost
				}
			}
			if cost < bestCost {
				bestCost = cost
				bestPath = pt
				bestStartNode = startNode
				bestEndNode = endNode
			}
		}
	}

	if bestCost == math.Inf(0) {
		log.Debugf(
			"routing failed, no path between %v and %v",
			start, end,
		)
		return nil, bestCost, fmt.Errorf("routing failed: no path")
	}
	// 将搜索结果转换为路由结果
	// 检查所有的edge，取其中具有合法的segment的edge
	ret := make([]*routingv2.WalkingRouteSegment, 0, len(bestPath)+1)
	if bestStartNode.extraEdgeAttr != nil {
		ret = append(ret, protoutil.Clone(bestStartNode.extraEdgeAttr))
	}
	for _, attrs := range bestPath[:len(bestPath)-1] {
		edgeAttr := attrs.EdgeAttr
		if edgeAttr == nil {
			continue
		}
		if len(ret) > 0 && edgeAttr.LaneId == ret[len(ret)-1].LaneId {
			ret[len(ret)-1].MovingDirection = edgeAttr.MovingDirection
			continue
		}
		ret = append(ret, protoutil.Clone(edgeAttr))
	}
	if bestEndNode.extraEdgeAttr != nil {
		ret = append(ret, protoutil.Clone(bestEndNode.extraEdgeAttr))
	}
	return ret, bestCost, nil
}
