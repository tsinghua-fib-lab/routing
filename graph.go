package main

import (
	"fmt"
	"log"
	"math"

	"gonum.org/v1/gonum/graph"
	"gonum.org/v1/gonum/graph/path"
	"gonum.org/v1/gonum/graph/simple"
)

type Pair struct {
	U, V int
}

type Graph struct {
	NodeId       int
	Edges        []Pair
	EdgesAttr    []map[string]int32
	EdgesLength  []float64
	EdgesLookup  map[Pair]int
	SearchGraph  simple.WeightedDirectedGraph
	NodePosition map[int64]Position
}

func NewGraph() *Graph {
	nodeId := 0
	edges := make([]Pair, 0)
	edgesAttr := make([]map[string]int32, 0)
	edgesLength := make([]float64, 0)
	edgesLookup := make(map[Pair]int)
	searchGraph := simple.NewWeightedDirectedGraph(1, math.Inf(1))
	nodePosition := make(map[int64]Position, 0)
	return &Graph{
		NodeId: nodeId, Edges: edges,
		EdgesAttr: edgesAttr, EdgesLength: edgesLength, EdgesLookup: edgesLookup,
		SearchGraph:  *searchGraph,
		NodePosition: nodePosition,
	}
}

func (g *Graph) AddEdge(u, v int, length float64, attr map[string]int32, overwrite bool) error {
	if e, ok := g.EdgesLookup[Pair{u, v}]; ok {
		if !overwrite {
			return fmt.Errorf("existed (%v,%v) in graph without overwrite", u, v)
		}
		g.EdgesLength[e] = length
		g.EdgesAttr[e] = attr
		g.SearchGraph.SetWeightedEdge(simple.WeightedEdge{
			F: simple.Node(u),
			T: simple.Node(v),
			W: float64(length),
		})
		return nil
	} else {
		g.EdgesLookup[Pair{u, v}] = len(g.Edges)
		g.Edges = append(g.Edges, Pair{u, v})
		g.EdgesAttr = append(g.EdgesAttr, attr)
		g.EdgesLength = append(g.EdgesLength, length)
		g.SearchGraph.SetWeightedEdge(simple.WeightedEdge{
			F: simple.Node(u),
			T: simple.Node(v),
			W: float64(length),
		})
		return nil
	}
}

func (g *Graph) EuclideanDrivingTime(x, y graph.Node) float64 {
	uPosition, vPosition := g.NodePosition[x.ID()], g.NodePosition[y.ID()]
	a, b := uPosition.X-vPosition.X, uPosition.Y-vPosition.Y
	return math.Sqrt(a*a+b*b) / VEHICLE_SPEED
}
func (g *Graph) EuclideanWalkingDistance(x, y graph.Node) float64 {
	uPosition, vPosition := g.NodePosition[x.ID()], g.NodePosition[y.ID()]
	a, b := uPosition.X-vPosition.X, uPosition.Y-vPosition.Y
	return math.Sqrt(a*a + b*b)
}

func (g *Graph) EuclideanBusDistance(x, y graph.Node) float64 {
	uPosition, vPosition := g.NodePosition[x.ID()], g.NodePosition[y.ID()]
	a, b := uPosition.X-vPosition.X, uPosition.Y-vPosition.Y
	return (math.Sqrt(a*a+b*b)/BUS_SPEED + INTERVAL/2) * PERSON_SPEED
}

func (g *Graph) ShortestPath(u, v, searchType int) ([]graph.Node, float64) {
	var pt path.Shortest
	switch searchType {
	case WALK:
		pt, _ = path.AStar(
			simple.Node(u),
			simple.Node(v),
			&g.SearchGraph,
			g.EuclideanWalkingDistance,
		)
	case DRIVE:
		pt, _ = path.AStar(
			simple.Node(u),
			simple.Node(v),
			&g.SearchGraph,
			g.EuclideanDrivingTime,
		)
	case BUS:
		pt, _ = path.AStar(
			simple.Node(u),
			simple.Node(v),
			&g.SearchGraph,
			g.EuclideanBusDistance,
		)
	default:
		log.Fatalf("wrong search type")
	}
	return pt.To(int64(v))
}
