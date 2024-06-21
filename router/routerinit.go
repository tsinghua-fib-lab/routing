package router

import (
	"math"
	"sort"

	"git.fiblab.net/general/common/geometry"
	geov2 "git.fiblab.net/sim/protos/go/city/geo/v2"
	mapv2 "git.fiblab.net/sim/protos/go/city/map/v2"
	routingv2 "git.fiblab.net/sim/protos/go/city/routing/v2"
	"git.fiblab.net/sim/routing/router/algo"
	"github.com/samber/lo"
)

// 将Map中的id转换为指针连接关系
func initMap(mapData *mapv2.Map, roadStatus *routingv2.RoadStatuses) (
	lanes map[int32]*Lane,
	aois map[int32]*Aoi,
	roads map[int32]*Road,
	sublines map[int32]*PublicSubline,
	tazs map[algo.TazPair]*TransportationAnalysisZone,
	tazInfo TransportationAnalysisZoneInfo,
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
	// 获得TAZ info
	disableTaz := mapData.Header.TazXStep == nil || mapData.Header.TazYStep == nil
	if !disableTaz {
		tazInfo = TransportationAnalysisZoneInfo{
			xStep: *mapData.Header.TazXStep,
			yStep: *mapData.Header.TazYStep,
			xMin:  mapData.Header.West,
			yMin:  mapData.Header.South,
		}
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
		// if aoi.Type == mapv2.AoiType_AOI_TYPE_BUS_STATION {
		// 	// aoi.StationNodeId = nodeId
		// 	// nodeId++
		// } else {
		// 	aoi.StationNodeId = -1
		// }
		if !disableTaz {
			aoi.StationTazCosts = make(map[algo.TazPair][]algo.TazCost)
			aoi.SublineTazCosts = make(map[int32][]algo.TazCost)
			aoi.StationTaz = algo.PointToTaz(aoi.CenterPoint, tazInfo.xStep, tazInfo.yStep, tazInfo.xMin, tazInfo.yMin)
		}
	}
	// 处理公交线路
	tazs = make(map[algo.TazPair]*TransportationAnalysisZone)
	sublines = make(map[int32]*PublicSubline)
	sublineTAZCost := make(map[int32][]algo.TazCost)
	sublineNameToIDs := make(map[string][]int32, 0)
	for _, subline := range mapData.Sublines {
		sublineNameToIDs[subline.ParentName] = append(sublineNameToIDs[subline.ParentName], subline.Id)
		sublines[subline.Id] = &PublicSubline{PublicTransportSubline: subline}
		for _, hcost := range subline.TazCosts {
			aoi := aois[hcost.AoiId]
			tazPair := algo.TazPair{
				X: hcost.TazXId,
				Y: hcost.TazYId,
			}
			// TAZ添加站点id
			departureTimes := subline.Schedules.DepartureTimes
			curTAZCost := algo.TazCost{
				Cost:             hcost.Cost,
				SublineID:        subline.Id,
				SublineStartTime: departureTimes[0],
				SublineEndTime:   departureTimes[len(departureTimes)-1],
				TazPair:          tazPair,
			}
			aoi.StationTazCosts[tazPair] = append(aoi.StationTazCosts[tazPair], curTAZCost)
			sublineTAZCost[subline.Id] = append(sublineTAZCost[subline.Id], curTAZCost)
		}
	}
	// 车站AOI添加所有经过路线的TAZ cost
	for _, subline := range sublines {
		for index, hcost := range subline.TazCosts {
			aoi := aois[hcost.AoiId]
			aoi.IsStation = true
			subline.SameLineID = math.MaxInt32
			if otherIds, ok := sublineNameToIDs[subline.ParentName]; ok {
				for _, otherID := range otherIds {
					if otherID == subline.Id {
						continue
					} else {
						subline.SameLineID = otherID
						break
					}
				}
			}
			// TAZ添加站点id
			tazPair := algo.PointToTaz(aoi.CenterPoint, tazInfo.xStep, tazInfo.yStep, tazInfo.xMin, tazInfo.yMin)
			if _, ok := tazs[tazPair]; !ok {
				tazs[tazPair] = &TransportationAnalysisZone{StationIds: make([]int32, 0)}
			}
			taz := tazs[tazPair]
			taz.StationIds = append(taz.StationIds, aoi.Id)
			afterTAZcosts := sublineTAZCost[subline.Id][index:]
			if len(afterTAZcosts) > 0 {
				aoi.SublineTazCosts[subline.Id] = afterTAZcosts
			}
		}
	}
	// 删除taz里面重复的站点id
	for _, taz := range tazs {
		seen := make(map[int32]bool)
		result := []int32{}
		for _, value := range taz.StationIds {
			if _, ok := seen[value]; !ok {
				seen[value] = true
				result = append(result, value)
			}
		}
		sort.Slice(result, func(i, j int) bool {
			return result[i] < result[j]
		})
		taz.StationIds = result

	}
	return
}
