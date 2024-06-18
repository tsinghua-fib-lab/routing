package router

import (
	mapv2 "git.fiblab.net/sim/protos/go/city/map/v2"
	routingv2 "git.fiblab.net/sim/protos/go/city/routing/v2"
	"git.fiblab.net/sim/routing/router/algo"
)

// type StopDistance struct {
// 	Stop     algo.Pair
// 	Distance float64
// }

// func StopDistanceIter(stops []int32, distances []float64) []StopDistance {
// 	if len(stops) != len(distances) {
// 		log.Fatalf("wrong inputs")
// 	}
// 	n := len(stops)
// 	stops = append(stops, stops[:len(stops)-1]...)
// 	ds := []float64{0}
// 	for _, v := range append(distances, distances[:len(distances)-2]...) {
// 		ds = append(ds, ds[len(ds)-1]+v)
// 	}
// 	iter := make([]StopDistance, 0)
// 	for i := 0; i < n; i++ {
// 		for j := i + 1; j < i+n; j++ {
// 			iter = append(iter, StopDistance{
// 				algo.Pair{U: int(stops[i]), V: int(stops[j])},
// 				ds[j] - ds[i],
// 			})
// 		}
// 	}
// 	return iter
// }

type Router struct {
	// driveGraph Topo
	// 	                             · [JL1 algo.TAIL]
	//                  [JL algo.HEAD]    |
	//                  [ROAD algo.TAIL]  /
	// [ROAD algo.HEAD]·----------·--------------· [JL2 algo.TAIL]
	//             \       /         \
	//              \    /           |
	//       [AOI algo.HEAD]·             · [JL3 algo.TAIL]
	// 1. 拓扑中的点为road的起点终点、junction lane的起点终点、AOI出入口
	// 2. 拓扑中的边为有四类：
	//    - road(road head->road tail) attr[id=roadId,type=forward]
	//    - junction lane(jl head->jl tail) attr[id=juncId,type=lane turn]
	//    - AOI出口到road tail的连边(aoi gate->road tail) attr[id=aoi gate对应的车道ID,type=forward]
	//    - road head到AOI入口的连边(road head->aoi gate) attr[id=aoi gate对应的车道ID,type=forward]
	// 3. cost为长度/预计算的平均速度，在路口内会对左转/右转/掉头在长度上施加额外的代价
	lanes    map[int32]*Lane
	aois     map[int32]*Aoi
	roads    map[int32]*Road
	sublines map[int32]*PublicSubline
	tazs     map[algo.TazPair]*TransportationAnalysisZone
	tazInfo  TransportationAnalysisZoneInfo

	driveGraph *algo.SearchGraph[algo.DriveNodeAttr, algo.DriveEdgeAttr]
	walkGraph  *algo.SearchGraph[algo.WalkNodeAttr, *routingv2.WalkingRouteSegment]
	busGraph   *algo.SearchGraph[algo.BusNodeAttr, *algo.BusEdgeAttr]
}

func New(
	mapData *mapv2.Map,
	//busLines *routingv2.BusLines,
	roadStatus *routingv2.RoadStatuses,
) *Router {

	lanes, aois, roads, sublines, tazs, tazInfo := initMap(mapData, roadStatus)
	r := &Router{lanes: lanes, roads: roads, aois: aois, sublines: sublines, tazs: tazs, tazInfo: tazInfo}
	r.buildDriveGraph()
	r.buildWalkGraph()
	r.buildBusGraph()
	return r
}

// setter

// func (r *Router) TDSetDriveRoadWeight(road *Road, weight float64, curTime *float64) {

// 	r.driveGraph.TDSetEdgeWeight(road.Extra[algo.HEAD], road.Extra[algo.TAIL], weight, curTime)
// }

// getter

func (r *Router) Roads() map[int32]*Road {
	return r.roads
}

func (r *Router) HasAoiID(id int32) bool {
	_, ok := r.aois[id]
	return ok
}

func (r *Router) HasLaneID(id int32) bool {
	_, ok := r.lanes[id]
	return ok
}

func (r *Router) HasRoadLaneID(id int32) bool {
	parentID := r.lanes[id].ParentId
	_, ok := r.roads[parentID]
	return ok
}

// close
func (r *Router) Close() {}
