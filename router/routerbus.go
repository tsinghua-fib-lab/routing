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
	// 换乘的惩罚时间（单位：秒）拍脑袋定的
	STATION_TRANSFER_PENALTY      = 1500
	SUBWAY_STATION_TRANSFER_RATIO = 1.0
	BUS_STATION_TRANSFER_RATIO    = 1.3
	// 途径车站的停车时间
	STATION_PASSBY_TIME = 15
	// 去另一个车站换乘的额外用时（单位：秒）拍脑袋定的
	WALK_LANE_TRANSFER_PENALTY = 300
	// 公交速度
	BUS_SPEED = 40 / 3.6
	// 地铁速度
	SUBWAY_SPEED = 100 / 3.6
	// 车站内换乘边的id
	TRANSFER_SUBLINE_ID = int32(math.MaxInt32)
	// 在同一个TAZ之内选取的公交站起点数量
	SAME_TAZ_STATION_NODES = 10
	// 行人乘坐公交车可忍受的最大步行时长（s）
	MAX_WALKING_TOLERATE_TIME = 60 * 35
	// 行人乘坐公共交通可忍受的最大时长（s）
	MAX_BUS_TOLERATE_TIME = 3600 * 6
	// 行人换乘最大距离（m）
	MAX_TRANSFER_WALKING_DISTANCE = 200
)

var (
	xStep              float64
	yStep              float64
	xMin               float64
	yMin               float64
	SAME_TAZ_DISTANCES = [7]float64{
		600, 840, 1080, 1320, 1560, 1800,
	}
)

type BusHeuristics struct {
}

func (h BusHeuristics) HeuristicEuclidean(p1 geometry.Point, p2 geometry.Point) float64 {
	return math.Inf(0)
}
func (h BusHeuristics) HeuristicBus(attrNode algo.BusNodeAttr, fromEdgeAttrs []*algo.BusEdgeAttr, pEnd geometry.Point, time float64) float64 {
	costs := attrNode.StationTAZCosts
	sublineCosts := attrNode.SublineTazCosts
	heuristicCost := 0.0
	transferRatio := 1.0
	// fromEdgeAttrs长度小于2不可能存在多个subline换乘
	if len(fromEdgeAttrs) >= 2 {
		passBySublineIDs := map[int32]bool{}
		// 换乘边
		passBySublineIDs[TRANSFER_SUBLINE_ID] = true
		for _, attr := range fromEdgeAttrs[:len(fromEdgeAttrs)-1] {
			passBySublineIDs[attr.SublineID] = true
		}
		lastEdgeAttr := fromEdgeAttrs[len(fromEdgeAttrs)-1]
		// 添加了新的subline id, 则加上换乘罚时
		if _, ok := passBySublineIDs[lastEdgeAttr.SublineID]; !ok {
			heuristicCost += STATION_TRANSFER_PENALTY
			if lastEdgeAttr.SublineType == int(mapv2.SublineType_SUBLINE_TYPE_BUS) {
				transferRatio = BUS_STATION_TRANSFER_RATIO
			} else if lastEdgeAttr.SublineType == int(mapv2.SublineType_SUBLINE_TYPE_SUBWAY) {
				transferRatio = SUBWAY_STATION_TRANSFER_RATIO
			}
		}
	}
	tazPair := algo.PointToTaz(pEnd, xStep, yStep, xMin, yMin)
	// p1: 目前的坐标
	// p2: 目标点坐标
	// 可直接乘车抵达指定TAZ
	minCost := math.Inf(0)
	if tazCosts, ok := costs[tazPair]; ok {
		for _, tazCost := range tazCosts {
			if !algo.InServiceTime(tazCost, time) {
				continue
			}
			if tazCost.Cost < minCost {
				minCost = tazCost.Cost
			}
		}
		if minCost != math.Inf(0) {
			return (heuristicCost + minCost) * transferRatio
		}
	}
	// 不可乘车抵达指定TAZ 则计算在离目标点最近的TAZ下车 + 步行到目标TAZ的cost
	for _, sublineTazCosts := range sublineCosts {
		{
			// minDistance := math.Inf(0)
			// closestTazCost := math.Inf(0)
			minCost := math.Inf(0)
			for _, sublineTazCost := range sublineTazCosts {
				if !algo.InServiceTime(sublineTazCost, time) {
					continue
				}
				distance := algo.TazDistance(sublineTazCost.TazPair, tazPair, xStep, yStep)
				rideWalkCost := sublineTazCost.Cost + distance/BUS_SPEED
				if minCost < rideWalkCost {
					minCost = rideWalkCost
				}
			}
			if minCost != math.Inf(0) {
				return (heuristicCost + minCost) * transferRatio
			}

		}
	}
	return (heuristicCost + geometry.Distance(attrNode.CenterPoint, pEnd)/PERSON_SPEED) * transferRatio
}

// BusGraph 从车站->车站的导航
func (r *Router) buildBusGraph() {
	xStep = r.tazInfo.xStep
	yStep = r.tazInfo.yStep
	xMin = r.tazInfo.xMin
	yMin = r.tazInfo.yMin
	busGraph := algo.NewSearchGraph(true, BusHeuristics{})
	for stationID, station := range r.aois {
		if !station.IsStation {
			continue
		}
		nodeStationTazCosts1 := make(map[algo.TazPair][]algo.TazCost, 0)
		nodeStationTazCosts2 := make(map[algo.TazPair][]algo.TazCost, 0)
		for key, value := range station.StationTazCosts {
			values1 := make([]algo.TazCost, 0)
			values2 := make([]algo.TazCost, 0)
			for _, d := range value {
				values1 = append(values1, d)
				values2 = append(values2, d)
			}
			nodeStationTazCosts1[key] = values1
			nodeStationTazCosts2[key] = values2
		}
		nodeSublineTazCosts1 := make(map[int32][]algo.TazCost, 0)
		nodeSublineTazCosts2 := make(map[int32][]algo.TazCost, 0)
		for key, value := range station.SublineTazCosts {
			values1 := make([]algo.TazCost, 0)
			values2 := make([]algo.TazCost, 0)
			for _, d := range value {
				values1 = append(values1, d)
				values2 = append(values2, d)
			}
			nodeSublineTazCosts1[key] = values1
			nodeSublineTazCosts2[key] = values2
		}
		station.StationInNodeId = busGraph.InitNode(station.CenterPoint, algo.BusNodeAttr{ID: stationID, StationTAZCosts: nodeStationTazCosts1, SublineTazCosts: nodeSublineTazCosts1, CenterPoint: station.CenterPoint}, false)
		station.StationOutNodeId = busGraph.InitNode(station.CenterPoint, algo.BusNodeAttr{ID: stationID, StationTAZCosts: nodeStationTazCosts2, SublineTazCosts: nodeSublineTazCosts2, CenterPoint: station.CenterPoint}, false)
		costs := make([]float64, algo.TIME_SLICE_LENGTH)
		// 车站入口到车站出口的边
		for index := range costs {
			costs[index] = STATION_PASSBY_TIME
		}
		busGraph.InitEdge(
			station.StationInNodeId,
			station.StationOutNodeId,
			costs,
			&algo.BusEdgeAttr{FromID: stationID, ToID: stationID, SublineID: TRANSFER_SUBLINE_ID, SublineType: int(mapv2.SublineType_SUBLINE_TYPE_UNSPECIFIED)},
		)
	}
	// 将车站之间的连接边加入graph
	for _, subline := range r.sublines {
		stationIDs := subline.AoiIds
		allStationPairs := make(map[StationIdPairs]bool, 0)
		inStationIds := make(map[int32]int, 0)
		stationCosts := subline.Schedules.OffsetTimes
		departureTimes := subline.Schedules.DepartureTimes
		fromStationID := stationIDs[0]
		inStationIds[fromStationID] = 0
		for index, toStationID := range stationIDs[1:] {
			allStationPairs[StationIdPairs{FromID: fromStationID, ToID: toStationID}] = true
			inStationIds[toStationID] = index + 1
			fromStation := r.aois[fromStationID]
			toStation := r.aois[toStationID]
			stationCost := stationCosts[index]
			costs := algo.StationTimeDependentCosts(stationCost, departureTimes)
			busGraph.InitEdge(
				fromStation.StationOutNodeId,
				toStation.StationInNodeId,
				costs,
				&algo.BusEdgeAttr{FromID: fromStationID, ToID: toStationID, SublineID: subline.Id, SublineType: int(subline.PublicTransportSubline.Type)},
			)
			fromStationID = toStationID
		}
		subline.StationPairs = allStationPairs
		subline.InStationIds = inStationIds
		// // 地铁车站到换乘站之间加边
		// if subline.Type == mapv2.SublineType_SUBLINE_TYPE_SUBWAY {
		// 	transferStationIDs := make([]int32, 0)
		// 	betweenTransferStationIDs := make([][]int32, 0)
		// 	nonTransferStationIDs := make([]int32, 0)
		// 	for _, stationID := range stationIDs {
		// 		station := r.aois[stationID]
		// 		if len(station.SublineIds) > 1 {
		// 			transferStationIDs = append(transferStationIDs, stationID)
		// 			betweenTransferStationIDs = append(betweenTransferStationIDs, nonTransferStationIDs)
		// 			nonTransferStationIDs = make([]int32, 0)
		// 		} else {
		// 			nonTransferStationIDs = append(nonTransferStationIDs, stationID)
		// 		}
		// 	}
		// 	for index, toStationID := range transferStationIDs {
		// 		nonTransferStationIDs = betweenTransferStationIDs[index]
		// 		toStation := r.aois[toStationID]
		// 		for _, fromStationID := range nonTransferStationIDs {
		// 			fromStation := r.aois[fromStationID]
		// 			stationCost := geometry.Distance(fromStation.CenterPoint, toStation.CenterPoint) / SUBWAY_SPEED
		// 			costs := algo.StationTimeDependentCosts(stationCost, departureTimes)
		// 			busGraph.InitEdge(
		// 				fromStation.StationOutNodeId,
		// 				toStation.StationInNodeId,
		// 				costs,
		// 				&algo.BusEdgeAttr{FromID: fromStationID, ToID: toStationID, SublineID: subline.Id, SublineType: int(subline.PublicTransportSubline.Type)},
		// 			)
		// 		}
		// 	}
		// }
	}
	// 将附近的车站连接加入graph
	for startStationID, station := range r.aois {
		if !station.IsStation {
			continue
		}
		if _, ok := r.tazs[station.StationTaz]; !ok {
			continue
		}
		tazStationIDs := r.tazs[station.StationTaz].StationIds
		for _, endStationID := range tazStationIDs {
			if startStationID == endStationID {
				continue
			}
			startStation := r.aois[startStationID]
			endStation := r.aois[endStationID]
			walkDistance := geometry.Distance(startStation.CenterPoint, endStation.CenterPoint)
			if walkDistance > MAX_TRANSFER_WALKING_DISTANCE {
				continue
			}
			{
				costs := make([]float64, algo.TIME_SLICE_LENGTH)
				// 车站入口到车站出口的边
				for index := range costs {
					costs[index] = walkDistance/PERSON_SPEED + WALK_LANE_TRANSFER_PENALTY
				}
				busGraph.InitEdge(
					startStation.StationOutNodeId,
					endStation.StationInNodeId,
					costs,
					&algo.BusEdgeAttr{FromID: startStationID, ToID: endStationID, SublineID: TRANSFER_SUBLINE_ID, SublineType: int(mapv2.SublineType_SUBLINE_TYPE_UNSPECIFIED)},
				)
				busGraph.InitEdge(
					endStation.StationOutNodeId,
					startStation.StationInNodeId,
					costs,
					&algo.BusEdgeAttr{FromID: endStationID, ToID: startStationID, SublineID: TRANSFER_SUBLINE_ID, SublineType: int(mapv2.SublineType_SUBLINE_TYPE_UNSPECIFIED)},
				)

			}
		}

	}

	r.busGraph = busGraph

}

// 起点是否是在运营的车站
func (r *Router) isStartServiceStation(start *geov2.Position, time float64) (bool, bool, *Aoi) {
	isStation := false
	inService := false
	var aoi *Aoi
	if aoiPosition := start.GetAoiPosition(); aoiPosition != nil {
		aoiID := aoiPosition.GetAoiId()
		aoi = r.aois[aoiID]
		for _, tazCosts := range aoi.StationTazCosts {
			isStation = true
			for _, tazCost := range tazCosts {
				if algo.InServiceTime(tazCost, time) {
					inService = true
					break
				}
			}
			if inService {
				break
			}

		}
	}
	return isStation, inService, aoi
}

// 得到站点的ID
func (r *Router) aoiStationID(pb *geov2.Position) (int32, bool) {
	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
		return aoiPosition.GetAoiId(), true
	} else {
		// panic("wrong type")
		return -1, false
	}
}

func (r *Router) positionToPoint(pb *geov2.Position) geometry.Point {
	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
		aoiID := aoiPosition.GetAoiId()
		aoi := r.aois[aoiID]
		return aoi.CenterPoint
	} else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
		lane := r.lanes[lanePosition.GetLaneId()]
		return lane.line[0]
	} else {
		panic("wrong type")
	}
}
func (r *Router) isStartEndInSameTAZ(start, end *geov2.Position) bool {
	startP := geometry.Point{}
	endP := geometry.Point{}
	if aoiPosition := start.GetAoiPosition(); aoiPosition != nil {
		startP = r.aois[aoiPosition.GetAoiId()].CenterPoint
	} else if lanePosition := start.GetLanePosition(); lanePosition != nil {
		startLane := r.lanes[lanePosition.GetLaneId()]
		startP = startLane.line[0]
	}
	if aoiPosition := end.GetAoiPosition(); aoiPosition != nil {
		endP = r.aois[aoiPosition.GetAoiId()].CenterPoint
	} else if lanePosition := end.GetLanePosition(); lanePosition != nil {
		endLane := r.lanes[lanePosition.GetLaneId()]
		endP = endLane.line[0]
	}
	startTaz := algo.PointToTaz(startP, xStep, yStep, xMin, yMin)
	endTaz := algo.PointToTaz(endP, xStep, yStep, xMin, yMin)
	return startTaz.X == endTaz.X && startTaz.Y == endTaz.Y

}

// 寻找最佳起点车站
func (r *Router) findBestStartStation(start *geov2.Position, pEnd geometry.Point, time float64) (startStationIds []int32) {
	if lanePosition := start.GetLanePosition(); lanePosition != nil {
		aoiIDs := r.lanes[lanePosition.GetLaneId()].AoiIds
		stationIDs := make([]int32, 0)
		for _, aoiID := range aoiIDs {
			aoi := r.aois[aoiID]
			if aoi.IsStation {
				stationIDs = append(stationIDs, aoiID)
			}
		}
		if len(stationIDs) > 0 {
			if len(stationIDs) > SAME_TAZ_STATION_NODES {
				stationIDs = stationIDs[:SAME_TAZ_STATION_NODES]
			}
			return stationIDs
		}
	}
	pStart := r.positionToPoint(start)
	// startTazPair := algo.PointToTaz(pStart, xStep, yStep, xMin, yMin)
	// startTaz := &TransportationAnalysisZone{}
	// if taz, ok := r.tazs[startTazPair]; ok {
	// 	startTaz = taz
	// } else {
	// 	log.Debugf("No stations at %v", startTazPair)
	// 	return nil
	// }
	var startTaz *TransportationAnalysisZone
	candidateStationIDs := make([]int32, 0)
	candidateCosts := make([]float64, 0)
	for _, startTazPair := range algo.PointToNearTAZs(pStart, xStep, yStep, xMin, yMin) {
		if taz, ok := r.tazs[startTazPair]; ok {
			startTaz = taz
		} else {
			continue
		}
		for _, stationID := range startTaz.StationIds {
			station := r.aois[stationID]
			if len(station.WalkingPositions) == 0 {
				// 不可步行抵达的车站不做选择
				continue
			}
			costs := station.StationTazCosts
			sublineCosts := station.SublineTazCosts
			minCost := math.Inf(0)
			if tazCosts, ok := costs[startTazPair]; ok {
				for _, tazCost := range tazCosts {
					if !algo.InServiceTime(tazCost, time) {
						continue
					}
					if tazCost.Cost < minCost {
						minCost = tazCost.Cost
					}
				}
			}
			if minCost == math.Inf(0) {
				// 不可乘车抵达指定TAZ 则计算在离目标点最近的TAZ下车 + 步行到目标TAZ的cost
				for _, tazCosts := range sublineCosts {
					{
						minDistance := math.Inf(0)
						closestTaz := algo.TazCost{}
						for _, tazCost := range tazCosts {
							if !algo.InServiceTime(tazCost, time) {
								continue
							}
							distance := algo.TazDistance(tazCost.TazPair, startTazPair, xStep, yStep)
							if distance < minDistance {
								closestTaz = tazCost
								minDistance = distance
							}
						}
						if minDistance != math.Inf(0) {
							minCost = closestTaz.Cost + minDistance/PERSON_SPEED
						}

					}
				}
			}
			if minCost == math.Inf(0) {
				minCost = geometry.Distance(algo.TazCenterPoint(startTazPair, xStep, yStep, xMin, yMin), pEnd) / PERSON_SPEED
			}
			minCost += geometry.Distance(station.CenterPoint, pStart) / PERSON_SPEED
			candidateStationIDs = append(candidateStationIDs, stationID)
			candidateCosts = append(candidateCosts, minCost)
		}
		if len(candidateStationIDs) > 0 {
			break
		}
	}

	// 起点TAZ内没有车站
	if len(candidateStationIDs) == 0 {
		log.Debugf("No stations at %v ", algo.PointToTaz(pStart, xStep, yStep, xMin, yMin))
		return nil
	}
	sort.Slice(candidateStationIDs, func(i, j int) bool {
		return candidateCosts[i] < candidateCosts[j]
	})
	// 去掉重复的TAZ aoi id
	encountered := map[int32]bool{}
	deduplicatedIDs := make([]int32, 0)
	for _, stationID := range candidateStationIDs {
		if !encountered[stationID] {
			encountered[stationID] = true
			deduplicatedIDs = append(deduplicatedIDs, stationID)
		}
	}
	candidateStationIDs = deduplicatedIDs
	if len(candidateStationIDs) > SAME_TAZ_STATION_NODES {
		candidateStationIDs = candidateStationIDs[:SAME_TAZ_STATION_NODES]
	}
	return candidateStationIDs

}

// 寻找车站到车站之间的路程
func (r *Router) searchBusTransfer(startStation *Aoi, end *geov2.Position, endP geometry.Point, sameTazDistance float64, time float64) (transferSegments []*routingv2.TransferSegment, timeCost float64, err error) {
	// 如果两站在同一条subline上 不使用图搜索算法
	if isEndStation, inEndService, _ := r.isStartServiceStation(end, time); isEndStation && inEndService {
		sameLineTransferSegments := make([]*routingv2.TransferSegment, 0)
		endStationID, _ := r.aoiStationID(end)
		for _, sublineID := range startStation.SublineIds {
			subline := r.sublines[sublineID]
			startIndex := 0
			for index, aoiID := range subline.AoiIds {
				if aoiID == startStation.Id {
					startIndex = index
					break
				}
			}
			followAoiIDs := subline.AoiIds[startIndex+1:]
			transferCost := 0.0
			for index, followAoiID := range followAoiIDs {
				transferCost += (subline.Schedules.OffsetTimes[index+startIndex] + STATION_PASSBY_TIME)
				if followAoiID == endStationID {
					sameLineTransferSegments = append(sameLineTransferSegments, &routingv2.TransferSegment{SublineId: sublineID, StartStationId: startStation.Id, EndStationId: endStationID})
					return sameLineTransferSegments, transferCost, nil
				}
			}
		}
	}
	startStationNodeID := startStation.StationInNodeId
	path, cost := r.busGraph.ShortestTAZPath(startStationNodeID, algo.PointToTaz(endP, xStep, yStep, xMin, yMin), endP, sameTazDistance, time)
	// panic recover
	defer func() {
		if e := recover(); e != nil {
			err = fmt.Errorf("panic: SearchBusTransfer %v with input start=%v, end=%v, time=%v", e, startStationNodeID, endP, time)
			log.Errorln(err)
		}
	}()
	if cost == math.Inf(0) || len(path) == 0 {
		return nil, cost, fmt.Errorf("routing failed: no path")
	} else {
		stationPath := make([]algo.PathItem[algo.BusNodeAttr, *algo.BusEdgeAttr], 0)
		for _, pathItem := range path {
			if pathItem.EdgeAttr != nil && pathItem.EdgeAttr.SublineID != TRANSFER_SUBLINE_ID {
				stationPath = append(stationPath, pathItem)
			}
		}
		if len(stationPath) == 0 {
			return
		}
		currentLineID := stationPath[0].EdgeAttr.SublineID
		startNodeID := stationPath[0].EdgeAttr.FromID
		endNodeID := stationPath[0].EdgeAttr.ToID
		if len(stationPath) == 1 {
			curSegment := &routingv2.TransferSegment{SublineId: currentLineID, StartStationId: startNodeID, EndStationId: endNodeID}
			transferSegments = append(transferSegments, protoutil.Clone(curSegment))
		}
		for i := 1; i < len(stationPath); i++ {
			edgeAttr := stationPath[i].EdgeAttr
			stationPairs := r.sublines[currentLineID].StationPairs
			inStationIDs := r.sublines[currentLineID].InStationIds
			inSublineFlag := false
			// if toIndex, ok := inStationIDs[edgeAttr.ToID]; ok {
			if _, ok := inStationIDs[edgeAttr.ToID]; ok {
				// endIndex := inStationIDs[endNodeID]
				// startIndex := inStationIDs[startNodeID]
				// if startIndex <= toIndex && endIndex <= toIndex {
				// 	inSublineFlag = true
				// }
				inSublineFlag = true
			}
			if edgeAttr.SublineID == currentLineID {
				endNodeID = edgeAttr.ToID
			} else if stationPairs[StationIdPairs{endNodeID, edgeAttr.ToID}] || inSublineFlag {
				endNodeID = edgeAttr.ToID
			} else {
				curSegment := &routingv2.TransferSegment{SublineId: currentLineID, StartStationId: startNodeID, EndStationId: endNodeID}
				transferSegments = append(transferSegments, protoutil.Clone(curSegment))
				currentLineID = edgeAttr.SublineID
				startNodeID = edgeAttr.FromID
				endNodeID = edgeAttr.ToID
				lastSegment := &routingv2.TransferSegment{SublineId: currentLineID, StartStationId: startNodeID, EndStationId: endNodeID}
				if i == len(stationPath)-1 {
					// 添加最后一部分
					if len(transferSegments) == 0 || transferSegments[len(transferSegments)-1].SublineId != currentLineID {
						transferSegments = append(transferSegments, protoutil.Clone(lastSegment))
					}
				}
			}

		}
		if len(transferSegments) == 0 {
			return nil, cost, fmt.Errorf("routing failed: no path")
		}
		lastSegment := transferSegments[len(transferSegments)-1]
		endStationID := lastSegment.EndStationId
		endStation := r.aois[endStationID]
		endDistance := geometry.Distance(endStation.CenterPoint, endP)
		followAoiIDs := make([]int32, 0)
		endSubline := r.sublines[lastSegment.SublineId]

		for index, aoiID := range endSubline.AoiIds {
			if aoiID == endStationID {
				followAoiIDs = endSubline.AoiIds[index:]
				break
			}
		}
		// 寻找后续有无更近的下车站点（必须包含步行道）
		for _, aoiID := range followAoiIDs {
			followStation := r.aois[aoiID]
			if followDistance := geometry.Distance(followStation.CenterPoint, endP); followDistance < endDistance && len(followStation.WalkingPositions) > 0 {
				endDistance = followDistance
				endStationID = aoiID
			}
		}
		if endStationID != endStation.Id {
			*lastSegment = routingv2.TransferSegment{SublineId: endSubline.Id, StartStationId: lastSegment.StartStationId, EndStationId: endStationID}
			cost += (geometry.Distance(endStation.CenterPoint, endP) - endDistance) / BUS_SPEED
		}
		resultTransferSegments := make([]*routingv2.TransferSegment, 0)
		for _, segment := range transferSegments {
			subline := r.sublines[segment.SublineId]
			startID := segment.StartStationId
			endID := segment.EndStationId
			inStationIDs := subline.InStationIds
			if inStationIDs[startID] < inStationIDs[endID] {
				resultTransferSegments = append(resultTransferSegments, protoutil.Clone(segment))
			} else {
				if otherSubline, ok := r.sublines[subline.SameLineID]; ok {
					otherInStationIDs := otherSubline.InStationIds
					startIndex, startExist := otherInStationIDs[startID]
					endIndex, endExist := otherInStationIDs[endID]
					if startExist && endExist && startIndex < endIndex {
						resultTransferSegments = append(resultTransferSegments, &routingv2.TransferSegment{SublineId: otherSubline.Id, StartStationId: startID, EndStationId: endID})
						continue
					}
				}
				return nil, math.Inf(0), fmt.Errorf("routing failed: no path")
			}

		}
		return resultTransferSegments, cost, nil
	}
}
func (r *Router) searchStationWalkRoute(routeStation *Aoi, position *geov2.Position, aoiId int32, time float64, routeToAoi bool) (walkSegments []*routingv2.WalkingRouteSegment, walkCost float64, ok bool) {
	if routeStation.Aoi.Id == aoiId {
		return nil, 0.0, true
	} else {
		pb := &geov2.Position{
			AoiPosition: &geov2.AoiPosition{
				AoiId: aoiId,
			},
		}
		if routeToAoi {
			walkSegments, walkCost, _ = r.SearchWalking(position, pb, time)
		} else {
			walkSegments, walkCost, _ = r.SearchWalking(pb, position, time)
		}
		if walkCost == math.Inf(0) || walkCost > MAX_WALKING_TOLERATE_TIME {
			return nil, 0.0, false
		} else {
			return walkSegments, walkCost, true
		}
	}
}
func (r *Router) SearchBus(
	start, end *geov2.Position, time float64,
) ([]*routingv2.WalkingRouteSegment, float64, []*routingv2.TransferSegment, float64, []*routingv2.WalkingRouteSegment, float64, error) {
	pEnd := r.positionToPoint(end)
	// 在同一TAZ内只进行步行导航
	if r.isStartEndInSameTAZ(start, end) {
		startWalkSegments, startWalkCost, err := r.SearchWalking(start, end, time)
		return startWalkSegments, startWalkCost, nil, 0.0, nil, 0.0, err
	} else {
		// 起点终点在两个TAZ内 分起点是否是车站 是否在运营分类讨论
		isStation, inService, routeStartStation := r.isStartServiceStation(start, time)
		routeResults := make([]BusRouteResult, 0)
		switch {
		case (!isStation || inService) && (routeStartStation != nil && routeStartStation.Aoi != nil):
			// 寻找当前TAZ内的综合步行距离和终点距离的最优车站
			stationIDs := r.findBestStartStation(start, pEnd, time)
			// 当前TAZ没有车站 无法坐车
			if stationIDs == nil {
				err := fmt.Errorf("routing failed: no stations at start position TAZ")
				return nil, 0.0, nil, 0.0, nil, 0.0, err
			}
			for _, stationID := range stationIDs {
				originalWalkSegments, originalWalkCost, ok := r.searchStationWalkRoute(routeStartStation, start, stationID, time, true)
				if !ok {
					continue
				}
				startStation := r.aois[stationID]
				for _, sameTazDistance := range SAME_TAZ_DISTANCES {
					// 从起点车站乘公交到终点车站
					startWalkSegments, startWalkCost := originalWalkSegments, originalWalkCost
					transferSegment, transferCost, _ := r.searchBusTransfer(startStation, end, pEnd, sameTazDistance, time+startWalkCost)
					if transferSegment == nil || transferCost > MAX_BUS_TOLERATE_TIME {
						continue
					}
					if startStation.Aoi.Id != transferSegment[0].StartStationId {
						// 说明公交导航更换了通路的起点站
						startWalkSegments, startWalkCost, ok = r.searchStationWalkRoute(routeStartStation, start, transferSegment[0].StartStationId, time, true)
						if !ok {
							continue
						}
					}
					// 从终点车站步行到终点
					_, _, routeEndStation := r.isStartServiceStation(end, time+transferCost)
					endWalkSegments, endWalkCost, ok := r.searchStationWalkRoute(routeEndStation, end, transferSegment[len(transferSegment)-1].EndStationId, time+startWalkCost+transferCost, false)
					if !ok {
						continue
					}
					routeResults = append(routeResults, BusRouteResult{startWalkSegments, startWalkCost, transferSegment, transferCost, endWalkSegments, endWalkCost, nil})
				}
			}
			if len(routeResults) > 0 {
				sort.Slice(routeResults, func(i, j int) bool {
					return routeResults[i].BusRouteResultCost() < routeResults[j].BusRouteResultCost()
				})
				startWalkSegments, startWalkCost, transferSegment, transferCost, endWalkSegments, endWalkCost := routeResults[0].StartWalkSegments, routeResults[0].StartWalkCost, routeResults[0].TransferSegment, routeResults[0].TransferCost, routeResults[0].EndWalkSegments, routeResults[0].EndWalkCost
				if transferSegment != nil {
					firstTransfer, lastTransfer := transferSegment[0], transferSegment[len(transferSegment)-1]
					// 去除无必要的步行导航
					if startAoiId, ok := r.aoiStationID(start); ok && firstTransfer.StartStationId == startAoiId {
						startWalkSegments = nil
					}
					if endAoiId, ok := r.aoiStationID(end); ok && lastTransfer.EndStationId == endAoiId {
						endWalkSegments = nil
					}
				}
				return startWalkSegments, startWalkCost, transferSegment, transferCost, endWalkSegments, endWalkCost, nil
			} else {
				return nil, math.Inf(0), nil, math.Inf(0), nil, math.Inf(0), fmt.Errorf("routing failed: no path")
			}
		default:
			// 车站不再运营 返回步行导航
			startWalkSegments, startWalkCost, err := r.SearchWalking(start, end, time)
			return startWalkSegments, startWalkCost, nil, 0.0, nil, 0.0, err
		}
	}
}
