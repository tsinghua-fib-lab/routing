package algo_test

import (
	"testing"

	"git.fiblab.net/general/common/v2/geometry"
	"git.fiblab.net/general/common/v2/mathutil"
	"git.fiblab.net/sim/routing/v2/router/algo"
	"github.com/stretchr/testify/assert"
)

type TestHeuristics1 struct {
}

func (h TestHeuristics1) HeuristicEuclidean(p1 geometry.Point, p2 geometry.Point) float64 {
	return geometry.Distance(p1, p2)
}
func (h TestHeuristics1) HeuristicBus(attrNode int, fromEdgeAttrs []int, pEnd geometry.Point, time float64) float64 {
	return 0
}
func TestSearchGraph(t *testing.T) {
	g := algo.NewSearchGraph[int, int](false, TestHeuristics1{})

	// 初始化点
	n1 := g.InitNode(geometry.Point{X: 0, Y: 0}, 1, false)
	n2 := g.InitNode(geometry.Point{X: 0, Y: 1}, 2, false)
	n3 := g.InitNode(geometry.Point{X: 1, Y: 0}, 3, false)
	n4 := g.InitNode(geometry.Point{X: 1, Y: 1}, 4, true)

	// 初始化边
	g.InitEdge(n1, n2, []float64{1}, 12)
	g.InitEdge(n2, n3, []float64{1}, 23)
	g.InitEdge(n3, n4, []float64{1}, 34)

	length := g.GetEdgeLength(n1, n2, 0)
	assert.Equal(t, 1.0, length)
	g.SetEdgeLength(n1, n2, 0, 2.0)
	length = g.GetEdgeLength(n1, n2, 0)
	assert.Equal(t, 2.0, length)
	g.SetEdgeLength(n1, n2, 0, 1.0)

	// 计算最短路
	path, cost := g.ShortestPath(n1, n4, 0)
	assert.Len(t, path, 4)
	assert.Equal(t, 1, path[0].NodeAttr)
	assert.Equal(t, 12, path[0].EdgeAttr)
	assert.Equal(t, 2, path[1].NodeAttr)
	assert.Equal(t, 23, path[1].EdgeAttr)
	assert.Equal(t, 3, path[2].NodeAttr)
	assert.Equal(t, 34, path[2].EdgeAttr)
	assert.Equal(t, 4, path[3].NodeAttr)
	assert.Equal(t, 3.0, cost)

	path, cost = g.ShortestPath(n3, n3, 0)
	assert.Len(t, path, 1)
	assert.Equal(t, 3, path[0].NodeAttr)
	assert.Equal(t, 0.0, cost)

	// 加入不可达的点
	n5 := g.InitNode(geometry.Point{X: 2, Y: 2}, 5, true)
	path, cost = g.ShortestPath(n1, n5, 0)
	assert.Nil(t, path)
	assert.Equal(t, mathutil.INF, cost)
}

type TestHeuristics2 struct {
}

func (h TestHeuristics2) HeuristicEuclidean(p1 geometry.Point, p2 geometry.Point) float64 {
	return geometry.Distance(p1, p2)
}
func (h TestHeuristics2) HeuristicBus(attrNode int, fromEdgeAttrs []int, pEnd geometry.Point, time float64) float64 {
	return 0
}
func TestSearchGraph2(t *testing.T) {
	g := algo.NewSearchGraph[int, int](false, TestHeuristics2{})

	// 初始化点
	n1 := g.InitNode(geometry.Point{X: 0, Y: 0}, 1, false)
	n2 := g.InitNode(geometry.Point{X: 0, Y: 1}, 2, false)
	n3 := g.InitNode(geometry.Point{X: 1, Y: 0}, 3, false)

	// 初始化边
	g.InitEdge(n1, n2, []float64{10}, 12)
	g.InitEdge(n1, n3, []float64{2}, 13)
	g.InitEdge(n3, n2, []float64{1}, 32)

	// 计算最短路
	path, cost := g.ShortestPath(n1, n2, 0)
	assert.Len(t, path, 3)
	assert.Equal(t, 1, path[0].NodeAttr)
	assert.Equal(t, 13, path[0].EdgeAttr)
	assert.Equal(t, 3, path[1].NodeAttr)
	assert.Equal(t, 32, path[1].EdgeAttr)
	assert.Equal(t, 2, path[2].NodeAttr)
	assert.Equal(t, 3.0, cost)
}
