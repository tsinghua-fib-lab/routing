package main

import (
	"context"
	"fmt"
	"log"
	"math"
	"sort"
	"time"

	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/bson/primitive"
	"go.mongodb.org/mongo-driver/mongo"
	"gonum.org/v1/gonum/graph"
	"gonum.org/v1/gonum/graph/simple"
)

type DisjointSet struct {
	Map map[int]int
}

func NewDisjointSet() *DisjointSet {
	set := make(map[int]int)
	return &DisjointSet{Map: set}
}

func (d *DisjointSet) Add(x int) error {
	if _, ok := d.Map[x]; ok {
		return fmt.Errorf("existed x in set")
	}
	d.Map[x] = x
	return nil
}

func (d *DisjointSet) GetRoot(x int) int {
	r := d.Map[x]
	if r == x {
		return r
	}
	d.Map[x] = d.GetRoot(r)
	return d.Map[x]
}

func (d *DisjointSet) SetRoot(x int, y int) {
	d.Map[d.GetRoot(x)] = y
}

func (d *DisjointSet) Simplify() {
	for k := range d.Map {
		d.GetRoot(k)
	}
}

type StopDistance struct {
	Stop     Pair
	Distance float64
}

func getFloat64(x interface{}) float64 {
	switch x := x.(type) {
	case float64:
		return x
	case float32:
		return float64(x)
	case int32:
		return float64(x)
	default:
		panic(fmt.Sprint("Get wrong number:", x))
	}
}

func StopDistanceIter(stops, distances primitive.A) []StopDistance {
	if len(stops) != len(distances) {
		log.Fatalf("wrong inputs")
	}
	n := len(stops)
	stops = append(stops, stops[:len(stops)-1]...)
	ds := []float64{0}
	for _, v := range append(distances, distances[:len(distances)-2]...) {
		ds = append(ds, ds[len(ds)-1]+getFloat64(v))
	}
	iter := make([]StopDistance, 0)
	for i := 0; i < n; i++ {
		for j := i + 1; j < i+n; j++ {
			iter = append(iter, StopDistance{
				Pair{U: int(stops[i].(int32)), V: int(stops[j].(int32))},
				ds[j] - ds[i],
			})
		}
	}
	return iter
}

func Distance(u, v Position) float64 {
	a, b := u.X-v.X, u.Y-v.Y
	return math.Sqrt(a*a + b*b)
}

type Position struct {
	X float64
	Y float64
}
type BusNode struct {
	S float64
	I int
}
type Router struct {
	Graph    *Graph
	BusGraph *Graph
	Lanes    map[int32]primitive.M
	Pois     map[int32]primitive.M
	Node2Poi map[int]int32
}

func NewRouter(mapData *mongo.Collection, busData *mongo.Collection) *Router {
	log.Print("Get map from database")
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	laneCur, err := mapData.Find(ctx, bson.M{"class": "lane"})
	if err != nil {
		log.Fatal(err)
	}
	defer laneCur.Close(ctx)
	lanes := make(map[int32]primitive.M)
	for laneCur.Next(ctx) {
		var result bson.M
		err := laneCur.Decode(&result)
		if err != nil {
			log.Fatal(err)
		}
		lanes[result["data"].(primitive.M)["id"].(int32)] = result["data"].(primitive.M)
	}
	if err := laneCur.Err(); err != nil {
		log.Fatal(err)
	}

	poiCur, err := mapData.Find(ctx, bson.M{"class": "poi"})
	if err != nil {
		log.Fatal(err)
	}
	defer poiCur.Close(ctx)
	pois := make(map[int32]primitive.M)
	for poiCur.Next(ctx) {
		var result bson.M
		err := poiCur.Decode(&result)
		if err != nil {
			log.Fatal(err)
		}
		pois[result["data"].(primitive.M)["id"].(int32)] = result["data"].(primitive.M)
	}
	if err := poiCur.Err(); err != nil {
		log.Fatal(err)
	}

	busLines := make([]primitive.M, 0)
	if busData.Name() != "" {
		log.Printf("Get bus lines from database")
		busCur, err := busData.Find(ctx, bson.D{})
		if err != nil {
			log.Fatal(err)
		}
		defer busCur.Close(ctx)
		for busCur.Next(ctx) {
			var result bson.M
			err := busCur.Decode(&result)
			if err != nil {
				log.Fatal(err)
			}
			busLines = append(busLines, result)
		}
		if err := busCur.Err(); err != nil {
			log.Fatal(err)
		}
	}

	nodeId := -1
	nodeSet := NewDisjointSet()
	for _, lane := range lanes {
		for head, predecessors := range map[string]string{
			"HEAD": "predecessors",
			"TAIL": "successors",
		} {
			if _, ok := lane[head]; !ok {
				nodeId++
				nodeSet.Add(nodeId)
				lane[head] = nodeId
				for _, pre := range lane[predecessors].(primitive.A) {
					if node_in_set, ok := lanes[pre.(primitive.M)["id"].(int32)][pre.(primitive.M)["type"].(string)[21:25]]; ok {
						nodeSet.SetRoot(node_in_set.(int), nodeId)
					} else {
						lanes[pre.(primitive.M)["id"].(int32)][pre.(primitive.M)["type"].(string)[21:25]] = nodeId
					}
				}
			}
		}
	}
	nodeSet.Simplify()
	nodeId = 0
	reMap := make(map[int]int)
	for _, v := range nodeSet.Map {
		if _, ok := reMap[v]; !ok {
			reMap[v] = nodeId
			nodeId++
		}
	}
	nodeMap := make(map[int]int)
	for k, v := range nodeSet.Map {
		nodeMap[k] = reMap[v]
	}
	for _, lane := range lanes {
		lane["HEAD"] = nodeMap[lane["HEAD"].(int)]
		lane["TAIL"] = nodeMap[lane["TAIL"].(int)]
	}
	log.Printf("build vanilla graph")
	vanillaGraph := NewGraph()
	for _, lane := range lanes {
		centerLine := lane["center_line"].(primitive.M)["nodes"].(primitive.A)
		maxSpeed := getFloat64(lane["max_speed"])
		vanillaGraph.NodePosition[int64(lane["HEAD"].(int))] = Position{
			X: getFloat64(centerLine[0].(primitive.M)["x"]),
			Y: getFloat64(centerLine[0].(primitive.M)["y"]),
		}
		vanillaGraph.NodePosition[int64(lane["TAIL"].(int))] = Position{
			X: getFloat64(centerLine[len(centerLine)-1].(primitive.M)["x"]),
			Y: getFloat64(centerLine[len(centerLine)-1].(primitive.M)["y"]),
		}
		if lane["type"] == "LANE_TYPE_DRIVING" {
			vanillaGraph.AddEdge(
				lane["HEAD"].(int),
				lane["TAIL"].(int),
				getFloat64(lane["length"])/maxSpeed,
				map[string]int32{"id": lane["id"].(int32), "type": FORWARD},
				false,
			)
		}
		if lane["type"] == "LANE_TYPE_WALKING" {
			vanillaGraph.AddEdge(
				lane["HEAD"].(int),
				lane["TAIL"].(int),
				getFloat64(lane["length"]),
				map[string]int32{"id": lane["id"].(int32), "type": FORWARD},
				false,
			)
			vanillaGraph.AddEdge(
				lane["TAIL"].(int),
				lane["HEAD"].(int),
				getFloat64(lane["length"]),
				map[string]int32{"id": lane["id"].(int32), "type": BACKWARD},
				false,
			)
		}
		if len(lane["left_lane_ids"].(primitive.A)) != 0 {
			vanillaGraph.AddEdge(
				lane["TAIL"].(int),
				lanes[lane["left_lane_ids"].(primitive.A)[0].(int32)]["TAIL"].(int),
				LANE_CHANGE_TIME_COST,
				map[string]int32{
					"id":   lane["left_lane_ids"].(primitive.A)[0].(int32),
					"type": LEFT,
				},
				false,
			)
		}
		if len(lane["right_lane_ids"].(primitive.A)) != 0 {
			vanillaGraph.AddEdge(
				lane["TAIL"].(int),
				lanes[lane["right_lane_ids"].(primitive.A)[0].(int32)]["TAIL"].(int),
				LANE_CHANGE_TIME_COST,
				map[string]int32{
					"id":   lane["right_lane_ids"].(primitive.A)[0].(int32),
					"type": RIGHT,
				},
				false,
			)
		}
	}
	log.Printf("%v nodes and %v edges",
		vanillaGraph.SearchGraph.Nodes().Len(),
		vanillaGraph.SearchGraph.Edges().Len(),
	)

	busGraph := NewGraph()
	nodeToPoi := make(map[int]int32, 0)
	if busData.Name() != "" {
		log.Printf("Build bus graph")
		walkGraph := NewGraph()
		for _, lane := range lanes {
			if lane["type"] == "LANE_TYPE_WALKING" {
				lane["NODES"] = make([]BusNode, 0)
			}
			centerLine := lane["center_line"].(primitive.M)["nodes"].(primitive.A)
			walkGraph.NodePosition[int64(lane["HEAD"].(int))] = Position{
				X: getFloat64(centerLine[0].(primitive.M)["x"]),
				Y: getFloat64(centerLine[0].(primitive.M)["y"]),
			}
			walkGraph.NodePosition[int64(lane["TAIL"].(int))] = Position{
				X: getFloat64(centerLine[len(centerLine)-1].(primitive.M)["x"]),
				Y: getFloat64(centerLine[len(centerLine)-1].(primitive.M)["y"]),
			}
			busGraph.NodePosition[int64(lane["HEAD"].(int))] = Position{
				X: getFloat64(centerLine[0].(primitive.M)["x"]),
				Y: getFloat64(centerLine[0].(primitive.M)["y"]),
			}
			busGraph.NodePosition[int64(lane["TAIL"].(int))] = Position{
				X: getFloat64(centerLine[len(centerLine)-1].(primitive.M)["x"]),
				Y: getFloat64(centerLine[len(centerLine)-1].(primitive.M)["y"]),
			}
		}
		for _, poi := range pois {
			if poi["type"] == "POI_TYPE_BUS_STATION" {
				nodeId++
				p := poi["walking_position"].(primitive.M)["lane_id"].(int32)
				s := getFloat64(poi["walking_position"].(primitive.M)["s"])
				lanes[p]["NODES"] = append(
					lanes[p]["NODES"].([]BusNode),
					BusNode{S: s, I: nodeId},
				)
				poi["NODE"] = nodeId
				nodeToPoi[nodeId] = poi["id"].(int32)
				poi["POS"] = Position{
					X: getFloat64(poi["walking_position"].(primitive.M)["x"]),
					Y: getFloat64(poi["walking_position"].(primitive.M)["y"]),
				}
				walkGraph.NodePosition[int64(poi["NODE"].(int))] = poi["POS"].(Position)
				busGraph.NodePosition[int64(poi["NODE"].(int))] = poi["POS"].(Position)
			}
		}
		for _, lane := range lanes {
			if lane["type"] == "LANE_TYPE_WALKING" {
				sort.Slice(lane["NODES"].([]BusNode), func(i, j int) bool {
					return lane["NODES"].([]BusNode)[i].S < lane["NODES"].([]BusNode)[j].S
				})
				lane["NODES"] = append(
					lane["NODES"].([]BusNode),
					BusNode{S: getFloat64(lane["length"]), I: lane["TAIL"].(int)},
				)
				nodes := append(
					[]BusNode{{S: 0, I: lane["HEAD"].(int)}},
					lane["NODES"].([]BusNode)...,
				)
				lane["NODES"] = nodes
				lane["EDGE"] = len(busGraph.Edges)
				for i, n := range nodes[:len(nodes)-1] {
					u := n.I
					u_s := n.S
					v := nodes[i+1].I
					v_s := nodes[i+1].S
					busGraph.AddEdge(
						u, v, (v_s - u_s),
						map[string]int32{"id": lane["id"].(int32), "type": FORWARD},
						false,
					)
					busGraph.AddEdge(
						v, u, (v_s - u_s),
						map[string]int32{"id": lane["id"].(int32), "type": BACKWARD},
						false,
					)
					walkGraph.AddEdge(
						u, v, (v_s - u_s),
						map[string]int32{"id": lane["id"].(int32), "type": FORWARD},
						false,
					)
					walkGraph.AddEdge(
						v, u, (v_s - u_s),
						map[string]int32{"id": lane["id"].(int32), "type": BACKWARD},
						false,
					)
				}
			}
		}
		for _, line := range busLines {
			waitTime := line["interval"].(int32) / 2
			stops := line["stops"].(primitive.A)
			distances := line["distances"].(primitive.A)
			iter := StopDistanceIter(stops, distances)
			for _, sd := range iter {
				up := pois[int32(sd.Stop.U)]
				vp := pois[int32(sd.Stop.V)]
				u := up["NODE"].(int)
				v := vp["NODE"].(int)
				dBus := (sd.Distance/BUS_SPEED + float64(waitTime)) * PERSON_SPEED
				add := false
				if e, ok := busGraph.EdgesLookup[Pair{u, v}]; ok {
					add = dBus < busGraph.EdgesLength[e]
				} else {
					dLine := Distance(up["POS"].(Position), vp["POS"].(Position))
					_, dWalk := walkGraph.ShortestPath(u, v, WALK)
					add = dLine > dBus || dWalk > dBus || dLine > BUS_DISTANCE
				}
				if add {
					busGraph.AddEdge(
						u, v, dBus,
						map[string]int32{"id": line["line_id"].(int32), "type": BUS},
						true,
					)
				}
			}
		}
		log.Printf("%v nodes and %v edges",
			busGraph.SearchGraph.Nodes().Len(),
			len(busGraph.Edges),
		)
	}
	return &Router{Graph: vanillaGraph, BusGraph: busGraph, Lanes: lanes, Pois: pois, Node2Poi: nodeToPoi}
}

type SearchPos struct {
	Node        int
	Edge        int
	EdgeReverse int
	S           float64
}
type BusRet struct {
	LaneId int
	Start  int
	End    int
	Walk   []Pair
}

func (r *Router) SearchBus(start, end map[string]interface{}) ([]BusRet, error) {
	if len(r.BusGraph.Edges) == 0 {
		log.Fatalf("no bus graph for routing")
	}
	var startS, endS float64
	startPos := make([]SearchPos, 0)
	if poiId, ok := start["poi_id"]; ok && r.Pois[int32(poiId.(uint32))]["type"] == "POI_TYPE_BUS_STATION" {
		startPos = append(startPos, SearchPos{
			Node:        r.Pois[int32(poiId.(uint32))]["NODE"].(int),
			Edge:        -1,
			EdgeReverse: -1,
			S:           0,
		})
	} else {
		var l int32
		var s float64
		if poiId, ok := start["poi_id"]; ok {
			p := r.Pois[int32(poiId.(uint32))]["walking_position"].(primitive.M)
			l = p["lane_id"].(int32)
			s = getFloat64(p["s"])
		} else {
			l = start["lane_id"].(int32)
			s = getFloat64(start["s"])
		}
		if s == 0 {
			startPos = append(startPos, SearchPos{
				Node:        r.Lanes[l]["HEAD"].(int),
				Edge:        -1,
				EdgeReverse: -1,
				S:           0,
			})
		} else {
			lane := r.Lanes[l]
			ns := lane["NODES"].([]BusNode)
			var k int
			for i, bn := range ns {
				if s <= bn.S {
					k = i
					break
				}
			}
			startS = s
			edge := lane["EDGE"].(int) + (k-1)*2
			startPos = append(startPos, SearchPos{
				Node:        ns[k].I,
				Edge:        edge,
				EdgeReverse: edge + 1,
				S:           ns[k].S - s,
			})
			startPos = append(startPos, SearchPos{
				Node:        ns[k-1].I,
				Edge:        edge + 1,
				EdgeReverse: edge,
				S:           s - ns[k-1].S,
			})
		}
	}
	endPos := make([]SearchPos, 0)
	if poiId, ok := end["poi_id"]; ok && r.Pois[int32(poiId.(uint32))]["type"] == "POI_TYPE_BUS_STATION" {
		endPos = append(endPos, SearchPos{
			Node:        r.Pois[int32(poiId.(uint32))]["NODE"].(int),
			Edge:        -1,
			EdgeReverse: -1,
			S:           0,
		})
	} else {
		var l int32
		var s float64
		if poiId, ok := end["poi_id"]; ok {
			p := r.Pois[int32(poiId.(uint32))]["walking_position"].(primitive.M)
			l = p["lane_id"].(int32)
			s = getFloat64(p["s"])
		} else {
			l = end["lane_id"].(int32)
			s = getFloat64(end["s"])
		}
		if s == 0 {
			endPos = append(endPos, SearchPos{
				Node:        r.Lanes[l]["HEAD"].(int),
				Edge:        -1,
				EdgeReverse: -1,
				S:           0,
			})
		} else {
			lane := r.Lanes[l]
			ns := lane["NODES"].([]BusNode)
			var k int
			for i, bn := range ns {
				if s <= bn.S {
					k = i
					break
				}
			}
			endS = s
			edge := lane["EDGE"].(int) + (k-1)*2
			endPos = append(endPos, SearchPos{
				Node:        ns[k-1].I,
				Edge:        edge,
				EdgeReverse: edge + 1,
				S:           s - ns[k-1].S,
			})
			endPos = append(endPos, SearchPos{
				Node:        ns[k].I,
				Edge:        edge + 1,
				EdgeReverse: edge,
				S:           ns[k].S - s,
			})
		}
	}
	if len(startPos) == 2 && len(endPos) == 2 && startPos[0].Edge == endPos[0].Edge {
		if startS <= endS {
			return []BusRet{{-1, 0, 0, []Pair{{int(r.BusGraph.EdgesAttr[startPos[0].Edge]["id"]), FORWARD}}}}, nil
		} else {
			return []BusRet{{-1, 0, 0, []Pair{{int(r.BusGraph.EdgesAttr[startPos[0].Edge]["id"]), BACKWARD}}}}, nil
		}
	}
	var pt []graph.Node
	cost := math.Inf(0)
	for _, startSp := range startPos {
		for _, endSp := range endPos {
			p, c := r.BusGraph.ShortestPath(startSp.Node, endSp.Node, BUS)
			if len(p) > 1 {
				first := r.BusGraph.EdgesLookup[Pair{U: int(p[0].ID()), V: int(p[1].ID())}]
				last := r.BusGraph.EdgesLookup[Pair{U: int(p[len(p)-2].ID()), V: int(p[len(p)-1].ID())}]
				if first == startSp.EdgeReverse || last == endSp.EdgeReverse {
					continue
				}
				if c = c + startSp.S + endSp.S; c < cost {
					cost = c
					if startSp.Edge != -1 && endSp.Edge != -1 {
						pt = append(
							append([]graph.Node{simple.Node(r.BusGraph.Edges[startSp.Edge].U)}, p...),
							simple.Node(r.BusGraph.Edges[endSp.Edge].V),
						)
					} else if startSp.Edge != -1 {
						pt = append([]graph.Node{simple.Node(r.BusGraph.Edges[startSp.Edge].U)}, p...)
					} else if endSp.Edge != -1 {
						pt = append(p, simple.Node(r.BusGraph.Edges[endSp.Edge].V))
					} else {
						pt = p
					}
				}
			}
		}
	}

	var ret []BusRet
	walk := make([]Pair, 0)
	if cost == math.Inf(0) {
		log.Printf("routing failed")
		return []BusRet{}, fmt.Errorf("routing failed: no path")
	} else {
		for i, n := range pt[:len(pt)-1] {
			e := r.BusGraph.EdgesLookup[Pair{U: int(n.ID()), V: int(pt[i+1].ID())}]
			attr := r.BusGraph.EdgesAttr[e]
			if attr["type"] == BUS {
				if len(walk) > 0 {
					ret = append(ret, BusRet{-1, 0, 0, walk})
					walk = make([]Pair, 0)
				}
				if len(ret) > 0 && ret[len(ret)-1].LaneId == int(attr["id"]) {
					ret[len(ret)-1].End = int(r.Node2Poi[int(pt[i+1].ID())])
				} else {
					ret = append(ret, BusRet{
						LaneId: int(attr["id"]),
						Start:  int(r.Node2Poi[int(n.ID())]),
						End:    int(r.Node2Poi[int(pt[i+1].ID())]),
					})
				}
			} else {
				if len(walk) > 0 && walk[len(walk)-1].U == int(attr["id"]) {
					if walk[len(walk)-1].V != int(attr["type"]) {
						log.Fatalf("error in bus routing")
					}
				} else {
					walk = append(walk, Pair{U: int(attr["id"]), V: int(attr["type"])})
				}
			}
		}
		if len(walk) > 0 {
			ret = append(ret, BusRet{-1, 0, 0, walk})
		}
	}
	return ret, nil
}

func (r *Router) SearchDriving(start, end map[string]interface{}) ([]Pair, error) {
	var startLane, endLane int32
	var startS, endS float64
	var ret []Pair
	if poiId, ok := start["poi_id"]; ok {
		p := r.Pois[int32(poiId.(uint32))]["driving_position"].(primitive.M)
		startLane = p["lane_id"].(int32)
		startS = getFloat64(p["s"])
	} else {
		startLane = start["lane_id"].(int32)
		startS = getFloat64(start["s"])
	}
	if poiId, ok := end["poi_id"]; ok {
		p := r.Pois[int32(poiId.(uint32))]["driving_position"].(primitive.M)
		endLane = p["lane_id"].(int32)
		endS = getFloat64(p["s"])
	} else {
		endLane = end["lane_id"].(int32)
		endS = getFloat64(end["s"])
	}
	var pt []graph.Node
	cost := math.Inf(0)
	ret = append(ret, Pair{int(startLane), FORWARD})
	if startLane == endLane {
		if startS <= endS {
			return ret, nil
		} else {
			lane := r.Lanes[startLane]
			v := lane["TAIL"].(int)
			successors := r.Graph.SearchGraph.From(int64(v))
			for successors.Next() {
				e := r.Graph.EdgesLookup[Pair{U: v, V: int(successors.Node().ID())}]
				if r.Graph.EdgesAttr[e]["type"] != FORWARD {
					continue
				}
				p, c := r.Graph.ShortestPath(int(successors.Node().ID()), v, DRIVE)
				if c+r.Graph.EdgesLength[e] < cost {
					cost = c + r.Graph.EdgesLength[e]
					pt = append([]graph.Node{simple.Node(v)}, p...)
				}
			}
		}
	} else {
		pt, cost = r.Graph.ShortestPath(
			r.Lanes[startLane]["TAIL"].(int),
			r.Lanes[endLane]["TAIL"].(int),
			DRIVE,
		)
	}
	if cost == math.Inf(0) {
		log.Printf(
			"routing failed, no path between lane: %v and lane: %v",
			startLane,
			endLane,
		)
		return []Pair{}, fmt.Errorf("routing failed: no path")
	} else {
		for i, n := range pt[:len(pt)-1] {
			e := r.Graph.EdgesLookup[Pair{U: int(n.ID()), V: int(pt[i+1].ID())}]
			if t := r.Graph.EdgesAttr[e]["type"]; t >= 2 {
				ret[len(ret)-1].V = int(t)
			}
			ret = append(ret, Pair{U: int(r.Graph.EdgesAttr[e]["id"]), V: FORWARD})
		}
	}
	return ret, nil
}

type SearchLane struct {
	Head int
	Tail int
	S    float64
}

func (r *Router) SearchWalking(start, end map[string]interface{}) ([]Pair, error) {
	var startLane, endLane int32
	var startS, endS float64
	var ret []Pair
	if poiId, ok := start["poi_id"]; ok {
		p := r.Pois[int32(poiId.(uint32))]["walking_position"].(primitive.M)
		startLane = p["lane_id"].(int32)
		startS = getFloat64(p["s"])
	} else {
		startLane = start["lane_id"].(int32)
		startS = getFloat64(start["s"])
	}
	if poiId, ok := end["poi_id"]; ok {
		p := r.Pois[int32(poiId.(uint32))]["walking_position"].(primitive.M)
		endLane = p["lane_id"].(int32)
		endS = getFloat64(p["s"])
	} else {
		endLane = end["lane_id"].(int32)
		endS = getFloat64(end["s"])
	}
	var pt []graph.Node
	cost := math.Inf(0)
	if startLane == endLane {
		if startS <= endS {
			return []Pair{{U: int(startLane), V: FORWARD}}, nil
		} else {
			return []Pair{{U: int(startLane), V: BACKWARD}}, nil
		}
	}
	startHead, startTail := r.Lanes[startLane]["HEAD"].(int), r.Lanes[startLane]["TAIL"].(int)
	startLength := getFloat64(r.Lanes[startLane]["length"])
	endHead, endTail := r.Lanes[endLane]["HEAD"].(int), r.Lanes[endLane]["TAIL"].(int)
	endLength := getFloat64(r.Lanes[endLane]["length"])
	for _, start := range []SearchLane{
		{Head: startHead, Tail: startTail, S: startS},
		{Head: startTail, Tail: startHead, S: startLength - startS},
	} {
		for _, end := range []SearchLane{
			{Head: endHead, Tail: endTail, S: endS},
			{Head: endTail, Tail: endHead, S: endLength - endS},
		} {
			p, c := r.Graph.ShortestPath(start.Head, end.Head, WALK)
			if len(p) > 0 {
				if p[1].ID() == int64(start.Tail) {
					continue
				}
				if p[len(p)-2].ID() == int64(end.Tail) {
					if start.S+c-end.S < cost {
						cost = start.S + c - end.S
						pt = append([]graph.Node{simple.Node(start.Tail)}, p...)
					}
				} else {
					if start.S+c+end.S < cost {
						cost = start.S + c + end.S
						pt = append(
							append([]graph.Node{simple.Node(start.Tail)}, p...),
							simple.Node(end.Tail),
						)
					}
				}
			}
		}
	}
	if cost == math.Inf(0) {
		log.Printf(
			"routing failed, no path between lane: %v and lane: %v",
			startLane,
			endLane,
		)
		return []Pair{}, fmt.Errorf("routing failed: no path")
	} else {
		for i, n := range pt[:len(pt)-1] {
			e := r.Graph.EdgesLookup[Pair{U: int(n.ID()), V: int(pt[i+1].ID())}]
			ret = append(ret, Pair{
				U: int(r.Graph.EdgesAttr[e]["id"]),
				V: int(r.Graph.EdgesAttr[e]["type"]),
			})
		}
	}
	return ret, nil
}
