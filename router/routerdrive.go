package router

import (
	"fmt"
	"math"

	"git.fiblab.net/general/common/geometry"
	geov2 "git.fiblab.net/sim/protos/go/city/geo/v2"
	mapv2 "git.fiblab.net/sim/protos/go/city/map/v2"
	"git.fiblab.net/sim/routing/router/algo"
)

const (
	// 路口带来的额外用时（单位：秒）除了左转其他数字都是拍脑袋定的
	TURN_STRAIGHT_TIME_PENALTY = 15
	TURN_LEFT_TIME_PENALTY     = 60 // Stamatiadis, Nikiforos, Kenneth R. Agent, and Apostolos Bizakis. "Guidelines for left-turn phasing treatment." Transportation Research Record 1605.1 (1997): 1-7. 有保护左转相对来说会带来60-70秒的延迟
	TURN_RIGHT_TIME_PENALTY    = 5
	TURN_AROUND_TIME_PENALTY   = 1.5 * TURN_LEFT_TIME_PENALTY

	// road id与junction id的切分点
	ROAD_JUNCTION_SPLIT = 3_0000_0000

	// 车辆速度
	VEHICLE_SPEED = 60 / 3.6
)

type DriveHeuristics struct {
}

func (h DriveHeuristics) HeuristicEuclidean(p1 geometry.Point, p2 geometry.Point) float64 {
	return geometry.Distance(p1, p2) / VEHICLE_SPEED
}
func (h DriveHeuristics) HeuristicBus(nodeAttr algo.DriveNodeAttr, fromEdgeAttrs []algo.DriveEdgeAttr, pEnd geometry.Point, time float64) float64 {
	return math.Inf(0)
}
func (r *Router) buildDriveGraph() {

	driveGraph := algo.NewSearchGraph[algo.DriveNodeAttr, algo.DriveEdgeAttr](true,
		DriveHeuristics{})
	// 将road加入graph
	for _, road := range r.roads {
		timeCosts := make([][]float64, len(road.DrivingLanes))
		startP, endP := geometry.Point{}, geometry.Point{}
		for id, lane := range road.DrivingLanes {
			startNode := lane.CenterLine.Nodes[0]
			startP.X += startNode.X
			startP.Y += startNode.X
			endNode := lane.CenterLine.Nodes[len(lane.CenterLine.Nodes)-1]
			endP.X += endNode.X
			endP.Y += endNode.Y
			timeCosts[id] = make([]float64, algo.TIME_SLICE_LENGTH)
			if len(lane.Speeds) == algo.TIME_SLICE_LENGTH {
				for i := 0; i < algo.TIME_SLICE_LENGTH; i++ {
					timeCosts[id][i] = lane.Length / lane.Speeds[i]
				}
			} else {
				for i := 0; i < algo.TIME_SLICE_LENGTH; i++ {
					timeCosts[id][i] = lane.Length / lane.MaxSpeed
				}
			}
		}
		count := float64(len(road.DrivingLanes))
		aveTimeCosts := make([]float64, algo.TIME_SLICE_LENGTH)
		startP.X /= count
		startP.Y /= count
		endP.X /= count
		endP.Y /= count
		for i := 0; i < algo.TIME_SLICE_LENGTH; i++ {
			for j := 0; j < len(road.DrivingLanes); j++ {
				aveTimeCosts[i] += timeCosts[j][i] / count
			}
		}
		road.DriveHeadNodeId = driveGraph.InitNode(startP, algo.DriveNodeAttr{ID: road.Id}, false)
		road.DriveTailNodeId = driveGraph.InitNode(endP, algo.DriveNodeAttr{ID: road.Id}, false)
		driveGraph.InitEdge(
			road.DriveHeadNodeId, road.DriveTailNodeId,
			aveTimeCosts,
			algo.DriveEdgeAttr{ID: road.Id},
		)
	}

	// 将junction lane加入graph
	for _, lane := range r.lanes {
		if lane.ParentId < ROAD_JUNCTION_SPLIT {
			// 不是junction lane
			continue
		}
		if lane.Type != mapv2.LaneType_LANE_TYPE_DRIVING {
			// 不是driving lane
			continue
		}
		// junction中的车道只有一个前驱后继
		startRoad := r.roads[r.lanes[lane.Predecessors[0].Id].ParentId]
		endRoad := r.roads[r.lanes[lane.Successors[0].Id].ParentId]
		costs := make([]float64, algo.TIME_SLICE_LENGTH)
		penaltyCost := .0
		switch lane.Turn {
		case mapv2.LaneTurn_LANE_TURN_STRAIGHT:
			penaltyCost = TURN_STRAIGHT_TIME_PENALTY
		case mapv2.LaneTurn_LANE_TURN_LEFT:
			penaltyCost = TURN_LEFT_TIME_PENALTY
		case mapv2.LaneTurn_LANE_TURN_RIGHT:
			penaltyCost = TURN_RIGHT_TIME_PENALTY
		case mapv2.LaneTurn_LANE_TURN_AROUND:
			penaltyCost = TURN_AROUND_TIME_PENALTY
		default:
			log.Fatalf("unknown lane(id=%d) turn type %v", lane.Id, lane.Turn)
		}
		// junction lane不考虑路况输入
		for i := 0; i < len(costs); i++ {
			costs[i] = lane.Length/lane.MaxSpeed + penaltyCost
		}
		// TODO: 多个lane如果在同一个lane group内的话，这里会相互覆盖
		driveGraph.InitEdge(
			startRoad.DriveTailNodeId, endRoad.DriveHeadNodeId,
			costs,
			algo.DriveEdgeAttr{ID: lane.ParentId},
		)
	}

	// 将aoi加入graph
	for _, aoi := range r.aois {
		aoi.DriveInNodeId = driveGraph.InitNode(aoi.CenterPoint, algo.DriveNodeAttr{ID: aoi.Id, IsAoi: true}, true)
		aoi.DriveOutNodeId = driveGraph.InitNode(aoi.CenterPoint, algo.DriveNodeAttr{ID: aoi.Id, IsAoi: true}, false)
		for _, p := range aoi.DrivingPositions {
			lane := r.lanes[p.LaneId]
			road := r.roads[lane.ParentId]
			// o---->----AOI---->----o
			//    head        tail
			headTimeCosts := make([]float64, algo.TIME_SLICE_LENGTH)
			tailTimeCosts := make([]float64, algo.TIME_SLICE_LENGTH)
			if len(lane.Speeds) > 0 {
				for i := 0; i < algo.TIME_SLICE_LENGTH; i++ {
					headTimeCosts[i] = p.S / lane.Speeds[i]
					tailTimeCosts[i] = (lane.Length - p.S) / lane.Speeds[i]
				}
			} else {
				for i := 0; i < algo.TIME_SLICE_LENGTH; i++ {
					headTimeCosts[i] = p.S / lane.MaxSpeed
					tailTimeCosts[i] = (lane.Length - p.S) / lane.MaxSpeed
				}
			}
			// road head -> aoi in node
			driveGraph.InitEdge(
				road.DriveHeadNodeId, aoi.DriveInNodeId,
				headTimeCosts,
				algo.DriveEdgeAttr{ID: road.Id},
			)
			// aoi out node -> road tail
			driveGraph.InitEdge(
				aoi.DriveOutNodeId, road.DriveTailNodeId,
				tailTimeCosts,
				algo.DriveEdgeAttr{ID: road.Id},
			)
		}
	}

	r.driveGraph = driveGraph
}

func (r *Router) toDriveStartNode(pb *geov2.Position) (nodeId int) {
	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
		return r.aois[aoiPosition.GetAoiId()].DriveOutNodeId
	} else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
		roadId := r.lanes[lanePosition.GetLaneId()].ParentId
		return r.roads[roadId].DriveTailNodeId
	} else {
		panic("wrong type")
	}
}

func (r *Router) toDriveEndNode(pb *geov2.Position) (nodeId int) {
	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
		return r.aois[aoiPosition.GetAoiId()].DriveInNodeId
	} else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
		roadId := r.lanes[lanePosition.GetLaneId()].ParentId
		return r.roads[roadId].DriveHeadNodeId
	} else {
		panic("wrong type")
	}
}

func (r *Router) SearchDriving(
	start, end *geov2.Position, time float64,
) (roadIDs []int32, cost float64, err error) {
	// panic recover
	defer func() {
		if e := recover(); e != nil {
			roadIDs = []int32{}
			cost = math.Inf(0)
			err = fmt.Errorf("panic: SearchDriving %v with input start=%v, end=%v, time=%v", e, start, end, time)
			log.Errorln(err)
		}
	}()

	// 如果起终点在同一Road上，且坐标S满足start<end，则不需要计算
	if start.LanePosition != nil && end.LanePosition != nil {
		startRoadId := r.lanes[start.LanePosition.LaneId].ParentId
		endRoadId := r.lanes[end.LanePosition.LaneId].ParentId
		startS := start.LanePosition.S
		endS := end.LanePosition.S
		if startRoadId == endRoadId && startS < endS {
			return []int32{startRoadId}, (endS - startS) / VEHICLE_SPEED, nil
		}
	}

	// 转换为搜索图中结点
	startNode := r.toDriveStartNode(start)
	endNode := r.toDriveEndNode(end)

	// 使用图搜索算法
	// nodes为根据【图搜索算法】得到的DriveGraph上nodeId的序列
	path, cost := r.driveGraph.ShortestPath(startNode, endNode, time)
	if cost == math.Inf(0) {
		log.Debugf("routing failed, no path between %v and %v", start, end)
		return []int32{}, cost, fmt.Errorf("routing failed: no path")
	}
	// node.Attr.ID: aoiId -> roadId(tail) -> roadId(head) -> roadId(tail) -> ...
	// 注意：roadId(head)和roadId(tail)是同一个road的两个端点
	// edge.Attr.ID: roadId -> juncId -> roadId -> ...
	if !path[0].NodeAttr.IsAoi {
		roadIDs = append(roadIDs, path[0].NodeAttr.ID)
	}
	for _, attrs := range path[:len(path)-1] {
		eID := attrs.EdgeAttr.ID
		if eID < ROAD_JUNCTION_SPLIT {
			roadIDs = append(roadIDs, eID)
		}
	}
	if !path[len(path)-1].NodeAttr.IsAoi {
		roadIDs = append(roadIDs, path[len(path)-1].NodeAttr.ID)
	}
	return roadIDs, cost, nil
}

func (r *Router) GetRoadCost(roadId int32, time *float64) (float64, error) {
	road, ok := r.roads[roadId]
	if !ok {
		return 0, fmt.Errorf("road(id=%d) not found", roadId)
	}
	if time == nil {
		t := 0.0
		time = &t
	}
	return r.driveGraph.GetEdgeLength(
		road.DriveHeadNodeId, road.DriveTailNodeId,
		algo.TimeToIndex(*time),
	), nil
}

func (r *Router) SetRoadCost(roadId int32, cost float64, time *float64) error {
	road, ok := r.roads[roadId]
	if !ok {
		return fmt.Errorf("road(id=%d) not found", roadId)
	}
	// TODO: 暂时不支持修改aoi<->road head/tail的边权重
	if time != nil {
		return r.driveGraph.SetEdgeLength(road.DriveHeadNodeId, road.DriveTailNodeId, algo.TimeToIndex(*time), cost)
	} else {
		costs := make([]float64, algo.TIME_SLICE_LENGTH)
		for i := 0; i < algo.TIME_SLICE_LENGTH; i++ {
			costs[i] = cost
		}
		return r.driveGraph.SetEdgeLengths(road.DriveHeadNodeId, road.DriveTailNodeId, costs)
	}
}
