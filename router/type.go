package router

import (
	"sort"

	"git.fiblab.net/general/common/geometry"
	mapv2 "git.fiblab.net/sim/protos/go/city/map/v2"
	"github.com/samber/lo"
)

type DriveNodeAttr struct {
	ID    int32
	IsAoi bool
}

type DriveEdgeAttr struct {
	ID int32
}

type WalkNodeAttr struct {
	ID    int32
	IsAoi bool
}

type WalkLaneNode struct {
	S      float64 // 距离起点的距离
	NodeId int     // 在graph中的nodeId
}

type Lane struct {
	*mapv2.Lane
	line        []geometry.Point
	lineLengths []float64

	Speeds []float64

	// o----o----o------o :Nodes
	//      |    |
	//     AOI  AOI
	WalkNodeIds map[int]int    // Head, Tail
	Nodes       []WalkLaneNode // 所有，包含Head和Tail和中间的AOI
}

// 将当前车道s坐标转换为xy坐标
func (l *Lane) GetPositionByS(s float64) (pos geometry.Point) {
	if s < l.lineLengths[0] || s > l.lineLengths[len(l.lineLengths)-1] {
		log.Warnf("get position with s %v out of range{%v,%v}",
			s, l.lineLengths[0], l.lineLengths[len(l.lineLengths)-1])
		s = lo.Clamp(s, l.lineLengths[0], l.lineLengths[len(l.lineLengths)-1])
	}
	if i := sort.SearchFloat64s(l.lineLengths, s); i == 0 {
		pos = l.line[0]
	} else {
		sHigh, sSlow := l.lineLengths[i], l.lineLengths[i-1]
		pos = geometry.Blend(l.line[i-1], l.line[i], (s-sSlow)/(sHigh-sSlow))
	}
	return
}

type Road struct {
	*mapv2.Road

	DriveHeadNodeId, DriveTailNodeId int
	// road内的driving lanes
	DrivingLanes []*Lane
}

type Aoi struct {
	*mapv2.Aoi

	CenterPoint                   geometry.Point
	DriveOutNodeId, DriveInNodeId int
	WalkOutNodeId, WalkInNodeId   int
	StationNodeId                 int
}

type MongoPath struct {
	DB  string
	Col string
}
