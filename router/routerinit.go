package router

import (
	"git.fiblab.net/general/common/geometry"
	geov2 "git.fiblab.net/sim/protos/go/city/geo/v2"
	mapv2 "git.fiblab.net/sim/protos/go/city/map/v2"
	routingv2 "git.fiblab.net/sim/protos/go/city/routing/v2"
	"github.com/samber/lo"
)

// 将Map中的id转换为指针连接关系
func initMap(mapData *mapv2.Map, roadStatus *routingv2.RoadStatuses) (
	lanes map[int32]*Lane,
	aois map[int32]*Aoi,
	roads map[int32]*Road,
) {
	// 将array转换为map
	lanes = make(map[int32]*Lane)
	for _, lane := range mapData.Lanes {
		l := &Lane{
			Lane: lane,
			line: lo.Map(lane.CenterLine.Nodes, func(node *geov2.XYPosition, _ int) geometry.Point {
				return geometry.NewPointFromPb(node)
			}),
			WalkNodeIds: make(map[int]int),
		}
		l.lineLengths = geometry.GetPolylineLengths2D(l.line)
		lanes[lane.Id] = l
	}
	aois = make(map[int32]*Aoi)
	for _, aoi := range mapData.Aois {
		aois[aoi.Id] = &Aoi{Aoi: aoi}
	}
	roads = make(map[int32]*Road)
	for _, road := range mapData.Roads {
		roads[road.Id] = &Road{Road: road}
	}
	// 计算road的DrivingLanes
	for _, road := range roads {
		road.DrivingLanes = make([]*Lane, 0)
		for _, laneID := range road.LaneIds {
			lane := lanes[laneID]
			if lane.Type == mapv2.LaneType_LANE_TYPE_DRIVING {
				road.DrivingLanes = append(road.DrivingLanes, lanes[laneID])
			}
		}
	}
	// 处理lane的Speeds
	for _, rs := range roadStatus.GetRoadStatuses() {
		lane := lanes[rs.Id]
		lane.Speeds = rs.Speed
	}
	// 计算AOI的中心点
	for _, aoi := range aois {
		aoi.CenterPoint = geometry.Point{}
		for _, p := range aoi.Positions {
			aoi.CenterPoint.X += p.X
			aoi.CenterPoint.Y += p.Y
		}
		count := float64(len(aoi.Positions))
		aoi.CenterPoint.X /= count
		aoi.CenterPoint.Y /= count
		if aoi.Type == mapv2.AoiType_AOI_TYPE_BUS_STATION {
			// aoi.StationNodeId = nodeId
			// nodeId++
		} else {
			aoi.StationNodeId = -1
		}
	}
	return
}
