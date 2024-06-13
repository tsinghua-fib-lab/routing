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
	lanes map[int32]*Lane
	aois  map[int32]*Aoi
	roads map[int32]*Road

	driveGraph *algo.SearchGraph[DriveNodeAttr, DriveEdgeAttr]
	walkGraph  *algo.SearchGraph[WalkNodeAttr, *routingv2.WalkingRouteSegment]
	// busGraph   *algo.SearchGraph[WalkNodeAttr, WalkEdgeAttr]
}

func New(
	mapData *mapv2.Map,
	busLines *routingv2.BusLines,
	roadStatus *routingv2.RoadStatuses,
) *Router {

	lanes, aois, roads := initMap(mapData, roadStatus)
	r := &Router{lanes: lanes, roads: roads, aois: aois}
	r.buildDriveGraph()
	r.buildWalkGraph()
	return r

	// for _, lane := range lanes {
	// 	if lane.Type == mapv2.LaneType_LANE_TYPE_WALKING {
	// 		walkGraph.AddEdge(
	// 			lane.Extra[algo.HEAD], lane.Extra[algo.TAIL], []float64{lane.Length},
	// 			algo.Attr{ID: lane.Id, Type: algo.FORWARD},
	// 			false,
	// 		)
	// 		walkGraph.AddEdge(
	// 			lane.Extra[algo.TAIL], lane.Extra[algo.HEAD], []float64{lane.Length},
	// 			algo.Attr{ID: lane.Id, Type: algo.BACKWARD},
	// 			false,
	// 		)
	// 		sort.Slice(lane.AoiNodes, func(i, j int) bool {
	// 			return lane.AoiNodes[i].S < lane.AoiNodes[j].S
	// 		})
	// 		lane.AoiNodes = append(lane.AoiNodes, AoiNode{S: lane.Length, I: lane.Extra[algo.TAIL]})
	// 		nodes := append([]AoiNode{{S: 0, I: lane.Extra[algo.HEAD]}}, lane.AoiNodes...)
	// 		lane.AoiNodes = nodes
	// 		// // lane.AoiEdgeSize = len(busGraph.Edges())
	// 		for i, n := range nodes[:len(nodes)-1] {
	// 			u := n.I
	// 			u_s := n.S
	// 			v := nodes[i+1].I
	// 			v_s := nodes[i+1].S
	// 			busGraph.AddEdge(
	// 				u, v, []float64{(v_s - u_s)},
	// 				algo.Attr{ID: lane.Id, Type: algo.FORWARD},
	// 				false,
	// 			)
	// 			busGraph.AddEdge(
	// 				v, u, []float64{(v_s - u_s)},
	// 				algo.Attr{ID: lane.Id, Type: algo.BACKWARD},
	// 				false,
	// 			)
	// 		}
	// 	}
	// }

	// if len(busLines.Lines) > 0 {
	// 	var wg sync.WaitGroup
	// 	var busGraphMtx sync.Mutex
	// 	for _, line := range busLines.Lines {
	// 		waitTime := line.Interval / 2
	// 		stops := line.Stops
	// 		distances := line.Distances
	// 		iter := StopDistanceIter(stops, distances)
	// 		line := line
	// 		wg.Add(1)
	// 		go func() {
	// 			for _, sd := range iter {
	// 				up := aois[int32(sd.Stop.U)]
	// 				vp := aois[int32(sd.Stop.V)]
	// 				// 车站aoi仅有一个walking position
	// 				u := up.NodeId[1]
	// 				v := vp.NodeId[1]
	// 				dBus := (sd.Distance/algo.BUS_SPEED + float64(waitTime)) * algo.PERSON_SPEED
	// 				add := false
	// 				if e, ok := walkGraph.EdgesLookup()[algo.Pair{U: u, V: v}]; ok {
	// 					add = dBus < walkGraph.EdgesLength()[e][0]
	// 				} else {
	// 					dLine := Distance(
	// 						up.WalkingGates[0],
	// 						vp.WalkingGates[0],
	// 					)
	// 					add = dLine > dBus || dLine > algo.BUS_DISTANCE
	// 				}
	// 				if add {
	// 					busGraphMtx.Lock()
	// 					busGraph.AddEdge(
	// 						u, v, []float64{dBus},
	// 						algo.Attr{ID: line.LineId, Type: algo.BUS},
	// 						true,
	// 					)
	// 					busGraphMtx.Unlock()
	// 				}
	// 			}
	// 			wg.Done()
	// 		}()
	// 	}
	// 	wg.Wait()
}

// type SearchPos struct {
// 	Node        int
// 	Edge        int
// 	EdgeReverse int
// 	S           float64
// }

// type BusRet struct {
// 	LaneId int
// 	Start  int
// 	End    int
// 	Walk   []algo.Pair
// }

// func (r *Router) parseGeoPositionToBusStartSearchNode(pb *geov2.Position) (
// 	Aoi int, Nodes algo.Pair, S float64,
// ) {
// 	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
// 		return r.aois[aoiPosition.GetAoiId()].NodeId[0], algo.Pair{U: -1, V: -1}, 0
// 	} else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
// 		lane := r.lanes[lanePosition.GetLaneId()]
// 		s := lanePosition.GetS()
// 		var k int
// 		for i, an := range lane.AoiNodes {
// 			if s < an.S {
// 				k = i
// 				break
// 			}
// 		}
// 		return -1,
// 			algo.Pair{U: lane.AoiNodes[k-1].I, V: lane.AoiNodes[k].I},
// 			s - lane.AoiNodes[k-1].S
// 	} else {
// 		panic("wrong type")
// 	}
// }

// func (r *Router) parseGeoPositionToBusEndSearchNode(pb *geov2.Position) (
// 	Aoi int, Nodes algo.Pair, S float64,
// ) {
// 	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
// 		return r.aois[aoiPosition.GetAoiId()].NodeId[len(r.aois[aoiPosition.GetAoiId()].NodeId)-1],
// 			algo.Pair{U: -1, V: -1}, 0
// 	} else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
// 		lane := r.lanes[lanePosition.GetLaneId()]
// 		s := lanePosition.GetS()
// 		var k int
// 		for i, an := range lane.AoiNodes {
// 			if s < an.S {
// 				k = i
// 				break
// 			}
// 		}
// 		return -1,
// 			algo.Pair{U: lane.AoiNodes[k-1].I, V: lane.AoiNodes[k].I},
// 			s - lane.AoiNodes[k-1].S
// 	} else {
// 		panic("wrong type")
// 	}
// }

// func (r *Router) SearchBus(start, end *geov2.Position, time float64) ([]BusRet, error) {
// 	if len(r.busGraph.Edges()) == len(r.walkGraph.Edges()) {
// 		log.Fatalf("no bus graph for routing")
// 	}
// 	var startS, endS float64
// 	startPos := make([]SearchPos, 0)
// 	if aoiPosition := start.GetAoiPosition(); aoiPosition != nil {
// 		aoi := r.aois[aoiPosition.GetAoiId()]
// 		startPos = append(startPos, SearchPos{
// 			Node:        aoi.NodeId[0],
// 			Edge:        -1,
// 			EdgeReverse: -1,
// 			S:           0,
// 		})

// 	} else {
// 		_, nodes, s := r.parseGeoPositionToBusStartSearchNode(start)
// 		if s == 0 {
// 			startPos = append(startPos, SearchPos{
// 				Node:        nodes.U,
// 				Edge:        -1,
// 				EdgeReverse: -1,
// 				S:           0,
// 			})
// 		} else {
// 			lane := r.lanes[r.busGraph.EdgesAttr()[r.busGraph.EdgesLookup()[nodes]].ID]
// 			ns := lane.AoiNodes
// 			var k int
// 			for i, bn := range ns {
// 				if s <= bn.S {
// 					k = i
// 					break
// 				}
// 			}
// 			startS = s
// 			edge := lane.AoiEdgeSize + (k-1)*2
// 			startPos = append(startPos, SearchPos{
// 				Node:        ns[k].I,
// 				Edge:        edge,
// 				EdgeReverse: edge + 1,
// 				S:           ns[k].S - s - ns[k-1].S,
// 			})
// 			startPos = append(startPos, SearchPos{
// 				Node:        ns[k-1].I,
// 				Edge:        edge + 1,
// 				EdgeReverse: edge,
// 				S:           s,
// 			})
// 		}
// 	}
// 	endPos := make([]SearchPos, 0)
// 	if aoiPosition := end.GetAoiPosition(); aoiPosition != nil {
// 		aoi := r.aois[aoiPosition.GetAoiId()]
// 		endPos = append(endPos, SearchPos{
// 			Node:        aoi.NodeId[len(aoi.NodeId)-1],
// 			Edge:        -1,
// 			EdgeReverse: -1,
// 			S:           0,
// 		})
// 	} else {
// 		_, nodes, s := r.parseGeoPositionToBusEndSearchNode(end)
// 		if s == 0 {
// 			endPos = append(endPos, SearchPos{
// 				Node:        nodes.U,
// 				Edge:        -1,
// 				EdgeReverse: -1,
// 				S:           0,
// 			})
// 		} else {
// 			lane := r.lanes[r.busGraph.EdgesAttr()[r.busGraph.EdgesLookup()[nodes]].ID]
// 			ns := lane.AoiNodes
// 			var k int
// 			for i, bn := range ns {
// 				if s <= bn.S {
// 					k = i
// 					break
// 				}
// 			}
// 			endS = s
// 			edge := lane.AoiEdgeSize + (k-1)*2
// 			endPos = append(endPos, SearchPos{
// 				Node:        ns[k-1].I,
// 				Edge:        edge,
// 				EdgeReverse: edge + 1,
// 				S:           s,
// 			})
// 			endPos = append(endPos, SearchPos{
// 				Node:        ns[k].I,
// 				Edge:        edge + 1,
// 				EdgeReverse: edge,
// 				S:           ns[k].S - s - ns[k-1].S,
// 			})
// 		}
// 	}
// 	if len(startPos) == 2 && len(endPos) == 2 && startPos[0].Edge == endPos[0].Edge {
// 		if startS <= endS {
// 			return []BusRet{{-1, 0, 0, []algo.Pair{{U: int(r.busGraph.EdgesAttr()[startPos[0].Edge].ID), V: algo.FORWARD}}}}, nil
// 		} else {
// 			return []BusRet{{-1, 0, 0, []algo.Pair{{U: int(r.busGraph.EdgesAttr()[startPos[0].Edge].ID), V: algo.BACKWARD}}}}, nil
// 		}
// 	}
// 	var pt []int
// 	cost := math.Inf(0)
// 	for _, startSp := range startPos {
// 		for _, endSp := range endPos {
// 			p, c := r.busGraph.ShortestPath(startSp.Node, endSp.Node, algo.BUS, time)
// 			if len(p) > 1 {
// 				first := r.busGraph.EdgesLookup()[algo.Pair{U: p[0], V: p[1]}]
// 				last := r.busGraph.EdgesLookup()[algo.Pair{U: p[len(p)-2], V: p[len(p)-1]}]
// 				if first == startSp.EdgeReverse || last == endSp.EdgeReverse {
// 					continue
// 				}
// 				if c = c + startSp.S + endSp.S; c < cost {
// 					cost = c
// 					if startSp.Edge != -1 && endSp.Edge != -1 {
// 						pt = append(
// 							append([]int{r.busGraph.Edges()[startSp.Edge].U}, p...),
// 							r.busGraph.Edges()[endSp.Edge].V,
// 						)
// 					} else if startSp.Edge != -1 {
// 						pt = append([]int{r.busGraph.Edges()[startSp.Edge].U}, p...)
// 					} else if endSp.Edge != -1 {
// 						pt = append(p, r.busGraph.Edges()[endSp.Edge].V)
// 					} else {
// 						pt = p
// 					}
// 				}
// 			}
// 		}
// 	}

// 	var ret []BusRet
// 	walk := make([]algo.Pair, 0)
// 	if cost == math.Inf(0) ||
// 		r.node2Aoi[pt[0]] == r.node2Aoi[pt[len(pt)-1]] {
// 		log.Debug("routing failed")
// 		return []BusRet{}, fmt.Errorf("routing failed: no path")
// 	} else {
// 		for i, n := range pt[:len(pt)-1] {
// 			e := r.busGraph.EdgesLookup()[algo.Pair{U: n, V: pt[i+1]}]
// 			attr := r.busGraph.EdgesAttr()[e]
// 			if attr.Type == algo.BUS {
// 				if len(walk) > 0 {
// 					ret = append(ret, BusRet{-1, 0, 0, walk})
// 					walk = make([]algo.Pair, 0)
// 				}
// 				if len(ret) > 0 && ret[len(ret)-1].LaneId == int(attr.ID) {
// 					ret[len(ret)-1].End = int(r.node2Aoi[pt[i+1]])
// 				} else {
// 					ret = append(ret, BusRet{
// 						LaneId: int(attr.ID),
// 						Start:  int(r.node2Aoi[n]),
// 						End:    int(r.node2Aoi[pt[i+1]]),
// 					})
// 				}
// 			} else {
// 				if attr.ID == -1 {
// 					continue
// 				}
// 				if len(walk) > 0 && walk[len(walk)-1].U == int(attr.ID) {
// 					if walk[len(walk)-1].V != int(attr.Type) {
// 						if len(walk) == 1 {
// 							lane := r.lanes[attr.ID]
// 							start := r.aois[r.node2Aoi[pt[i-1]]]
// 							end := r.aois[r.node2Aoi[pt[i+1]]]
// 							pt[i] = pt[i-1]
// 							var startS, endS float64
// 							for _, p := range start.WalkingPositions {
// 								if p.LaneId == lane.Id {
// 									startS = p.S
// 								}
// 							}
// 							for _, p := range end.WalkingPositions {
// 								if p.LaneId == lane.Id {
// 									endS = p.S
// 								}
// 							}
// 							if startS < endS {
// 								walk = []algo.Pair{{U: int(lane.Id), V: algo.FORWARD}}
// 							} else {
// 								walk = []algo.Pair{{U: int(lane.Id), V: algo.BACKWARD}}
// 							}
// 						} else if len(pt) == 3 {
// 							lane := r.lanes[attr.ID]
// 							start := r.aois[r.node2Aoi[pt[0]]]
// 							end := r.aois[r.node2Aoi[pt[2]]]
// 							var startS, endS float64
// 							for _, p := range start.WalkingPositions {
// 								if p.LaneId == lane.Id {
// 									startS = p.S
// 								}
// 							}
// 							for _, p := range end.WalkingPositions {
// 								if p.LaneId == lane.Id {
// 									endS = p.S
// 								}
// 							}
// 							if startS < endS {
// 								return []BusRet{{-1, 0, 0, []algo.Pair{{U: int(lane.Id), V: algo.FORWARD}}}}, nil
// 							} else {
// 								return []BusRet{{-1, 0, 0, []algo.Pair{{U: int(lane.Id), V: algo.BACKWARD}}}}, nil
// 							}
// 						} else {
// 							log.Fatalf("error in bus routing")
// 						}
// 					}
// 				} else {
// 					walk = append(walk, algo.Pair{U: int(attr.ID), V: int(attr.Type)})
// 				}
// 			}
// 		}
// 		if len(walk) > 0 {
// 			ret = append(ret, BusRet{-1, 0, 0, walk})
// 		}
// 	}
// 	return ret, nil
// }

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
