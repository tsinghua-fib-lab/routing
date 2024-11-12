package algo

import (
	"container/heap"
	"log"
	"math"

	mapv2 "git.fiblab.net/sim/protos/v2/go/city/map/v2"

	"git.fiblab.net/general/common/v2/geometry"
	"github.com/puzpuzpuz/xsync/v3"
	"github.com/samber/lo"
)

type node[T any] struct {
	p     geometry.Point
	attr  T
	noOut bool // 是否没有出边
}

type edge[T any] struct {
	v    []float64
	attr T
}

type SearchGraph[NT any, ET any] struct {
	// 邻接表，in node -> out node -> edge length with time
	// Runtime期间出边入边不变，因此不需要考虑并发问题
	// 但edge length会被改变，因此需要考虑并发问题
	edges []map[int]edge[ET]
	// 点的位置
	nodes []node[NT]
	// 是否是time dependent的图
	isTD bool
	// A Star距离预估函数
	h IHeuristics[NT, ET]
	// edge权值提取函数
	w IEdgeWeight[ET]

	mu *xsync.RBMutex
}
type IHeuristics[NT any, ET any] interface {
	HeuristicEuclidean(geometry.Point, geometry.Point) float64
	HeuristicBus(NT, []ET, geometry.Point, float64) float64
}

type IEdgeWeight[ET any] interface {
	GetRuntimeEdgeWeight(ET, []float64, int, []mapv2.SublineType) float64
}

// type Heuristic func(geometry.Point, geometry.Point) float64

func NewSearchGraph[NT any, ET any](isTD bool, h IHeuristics[NT, ET], w IEdgeWeight[ET]) *SearchGraph[NT, ET] {
	return &SearchGraph[NT, ET]{
		edges: make([]map[int]edge[ET], 0),
		nodes: make([]node[NT], 0),
		isTD:  isTD,
		h:     h,
		w:     w,
		mu:    xsync.NewRBMutex(),
	}
}

func (g *SearchGraph[NT, ET]) InitNode(p geometry.Point, attr NT, noOut bool) int {
	g.nodes = append(g.nodes, node[NT]{p: p, attr: attr, noOut: noOut})
	g.edges = append(g.edges, make(map[int]edge[ET]))
	return len(g.nodes) - 1
}

func (g *SearchGraph[NT, ET]) InitEdge(from, to int, lengths []float64, attr ET) {
	if !g.isTD {
		// 非time dependent的图，length长度为1
		if len(lengths) != 1 {
			panic("lengths length is not 1")
		}
	}
	// if from >= len(g.edges) || to >= len(g.edges) {
	// 	panic("node not exists")
	// }
	if from >= len(g.edges) {
		log.Panicf("from node %d >= len(g.edges) %d", from, len(g.edges))
	}
	g.edges[from][to] = edge[ET]{
		v:    lengths,
		attr: attr,
	}
}

func (g *SearchGraph[NT, ET]) GetEdgeLengthAndAttr(from, to int, tIndex int) (float64, ET) {
	if !g.isTD {
		tIndex = 0
	}
	edge := g.edges[from][to]
	return edge.v[tIndex], edge.attr
}

func (g *SearchGraph[NT, ET]) GetEdgeLength(from, to int, tIndex int) float64 {
	if !g.isTD {
		tIndex = 0
	}
	edge := g.edges[from][to]
	return edge.v[tIndex]
}

func (g *SearchGraph[NT, ET]) SetEdgeLength(from, to int, tIndex int, length float64) error {
	if !g.isTD {
		tIndex = 0
	}
	if tIndex >= TIME_SLICE_LENGTH {
		return ErrOutOfTimeSlice
	}
	edge := g.edges[from][to]
	edge.v[tIndex] = length
	return nil
}

func (g *SearchGraph[NT, ET]) SetEdgeLengths(from, to int, lengths []float64) error {
	if !g.isTD {
		if len(lengths) != 1 {
			return ErrNoTDGraph
		}
	}
	edge := g.edges[from][to]
	edge.v = lengths
	return nil
}

type PathItem[NT any, ET any] struct {
	NodeAttr NT
	EdgeAttr ET
}

func (g *SearchGraph[NT, ET]) reconstructPath(cameFrom map[int]int, curNode int, curTime float64) ([]PathItem[NT, ET], float64) {
	pathBeforeReversed := []PathItem[NT, ET]{{NodeAttr: g.nodes[curNode].attr}}
	cost := curTime
	for {
		if from, ok := cameFrom[curNode]; ok {
			thisCost, attr := g.GetEdgeLengthAndAttr(from, curNode, TimeToIndex(cost))
			cost += thisCost
			curNode = from
			pathBeforeReversed = append(pathBeforeReversed, PathItem[NT, ET]{
				NodeAttr: g.nodes[curNode].attr,
				EdgeAttr: attr,
			})
		} else {
			break
		}
	}
	cost -= curTime
	return lo.Reverse(pathBeforeReversed), cost
}

func (g *SearchGraph[NT, ET]) ShortestPath(start, end int, curTime float64) ([]PathItem[NT, ET], float64) {
	return g.ShortestPathAStar(start, end, curTime)
}
func (g *SearchGraph[NT, ET]) ShortestTAZPath(start int, endTaz TazPair, endP geometry.Point, sameTazDistance, curTime float64, availableSublineTypes []mapv2.SublineType) ([]PathItem[NT, ET], float64) {
	return g.ShortestPathAStarToTaz(start, endTaz, endP, sameTazDistance, curTime, availableSublineTypes)
}

// A Star算法求最短路
func (g *SearchGraph[NT, ET]) ShortestPathAStar(start, end int, curTime float64) ([]PathItem[NT, ET], float64) {
	token := g.mu.RLock()
	defer g.mu.RUnlock(token)
	if start == end {
		return []PathItem[NT, ET]{{NodeAttr: g.nodes[start].attr}}, 0
	}
	openSet := make(PriorityQueue, 1)
	openSetMap := make(map[int]*Item, 1) // openSet value -> openSet item
	cameFrom := make(map[int]int, 0)
	gScore := make(map[int]float64, 0)
	gScore[start] = .0
	fScore := g.h.HeuristicEuclidean(g.nodes[start].p, g.nodes[end].p)
	openSet[0] = &Item{Value: start, Priority: fScore, Index: 0}
	openSetMap[start] = openSet[0]
	heap.Init(&openSet)
	for openSet.Len() > 0 {
		cur := heap.Pop(&openSet).(*Item).Value
		if cur == end {
			return g.reconstructPath(cameFrom, cur, curTime)
		}
		for neighbor, edge := range g.edges[cur] {
			// 如果没有出边，跳过
			if g.nodes[neighbor].noOut && neighbor != end {
				continue
			}
			// Time Dependent图
			tIndex := 0
			if g.isTD {
				tIndex = TimeToIndex(curTime + gScore[cur])
			}
			gScoreTentative := gScore[cur] + g.w.GetRuntimeEdgeWeight(edge.attr, edge.v, tIndex, nil)
			var gScoreNeighbor float64
			s, ok := gScore[neighbor]
			if ok {
				gScoreNeighbor = s
			} else {
				gScoreNeighbor = math.Inf(0)
			}
			if gScoreTentative < gScoreNeighbor {
				cameFrom[neighbor] = cur
				gScore[neighbor] = gScoreTentative
				fScore := gScoreTentative + g.h.HeuristicEuclidean(g.nodes[neighbor].p, g.nodes[end].p)
				if ok {
					// 已经访问过的节点，修改其在heap中的优先级
					openSetMap[neighbor].Priority = fScore
					heap.Fix(&openSet, openSetMap[neighbor].Index)
				} else {
					// 新访问的节点
					item := &Item{Value: neighbor, Priority: fScore}
					heap.Push(&openSet, item)
					openSetMap[neighbor] = item
				}
			}
		}
	}
	return nil, math.Inf(0)
}

// 非固定终点最短路 遍历到终点所在taz即停止
func (g *SearchGraph[NT, ET]) ShortestPathAStarToTaz(start int, endTaz TazPair, endP geometry.Point, sameTazDistance float64, curTime float64, availableSublineTypes []mapv2.SublineType) ([]PathItem[NT, ET], float64) {
	token := g.mu.RLock()
	defer g.mu.RUnlock(token)
	openSet := make(PriorityQueue, 1)
	openSetMap := make(map[int]*Item, 1) // openSet value -> openSet item
	cameFrom := make(map[int]int, 0)
	gScore := make(map[int]float64, 0)
	gScore[start] = .0
	startNode := g.nodes[start]
	startAttr := startNode.attr
	if geometry.Distance(startNode.p, endP) < sameTazDistance {
		return []PathItem[NT, ET]{{NodeAttr: g.nodes[start].attr}}, 0
	}
	fScore := g.h.HeuristicBus(startAttr, make([]ET, 0), endP, curTime)
	openSet[0] = &Item{Value: start, Priority: fScore, Index: 0}
	openSetMap[start] = openSet[0]
	heap.Init(&openSet)
	for openSet.Len() > 0 {
		cur := heap.Pop(&openSet).(*Item).Value
		curNode := g.nodes[cur]
		if geometry.Distance(curNode.p, endP) < sameTazDistance {
			return g.reconstructPath(cameFrom, cur, curTime)
		}
		fromEdgeAttrs := make([]ET, 0)
		if from, ok := cameFrom[cur]; ok {
			fromEdgeAttrs = append(fromEdgeAttrs, g.edges[from][cur].attr)
			if from2, ok := cameFrom[from]; ok {
				fromEdgeAttrs = append(fromEdgeAttrs, g.edges[from2][from].attr)
			}
		}
		for neighbor, edge := range g.edges[cur] {
			// 如果没有出边，跳过
			neighborNode := g.nodes[neighbor]
			if neighborNode.noOut && geometry.Distance(neighborNode.p, endP) > sameTazDistance {
				continue
			}
			// Time Dependent图
			tIndex := 0
			if g.isTD {
				tIndex = TimeToIndex(curTime + gScore[cur])
			}
			gScoreTentative := gScore[cur] + g.w.GetRuntimeEdgeWeight(edge.attr, edge.v, tIndex, availableSublineTypes)
			var gScoreNeighbor float64
			s, ok := gScore[neighbor]
			if ok {
				gScoreNeighbor = s
			} else {
				gScoreNeighbor = math.Inf(0)
			}
			if gScoreTentative < gScoreNeighbor {
				cameFrom[neighbor] = cur
				gScore[neighbor] = gScoreTentative
				fScore := gScoreTentative + g.h.HeuristicBus(neighborNode.attr, append(fromEdgeAttrs, edge.attr), endP, curTime+gScore[cur])
				if ok {
					// 已经访问过的节点，修改其在heap中的优先级
					openSetMap[neighbor].Priority = fScore
					heap.Fix(&openSet, openSetMap[neighbor].Index)
				} else {
					// 新访问的节点
					item := &Item{Value: neighbor, Priority: fScore}
					heap.Push(&openSet, item)
					openSetMap[neighbor] = item
				}
			}
		}
	}
	return nil, math.Inf(0)
}

// // 扩展一个图，保留nodes和edges不变，moreNodes和moreEdges清空并不复用原有内存空间
// func (g *SearchGraph[NT, ET]) Extend() *SearchGraph[NT, ET] {
// 	return &SearchGraph[NT, ET]{
// 		edges:     g.edges,
// 		moreEdges: make(map[int]map[int]*edge[ET]),
// 		nodes:     g.nodes,
// 		moreNodes: make(map[int]node[NT]),
// 		isTD:      g.isTD,
// 		h:         g.h,
// 	}
// }
