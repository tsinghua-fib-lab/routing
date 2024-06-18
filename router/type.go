package router

import (
	"sort"

	"git.fiblab.net/general/common/geometry"
	mapv2 "git.fiblab.net/sim/protos/go/city/map/v2"
	routingv2 "git.fiblab.net/sim/protos/go/city/routing/v2"
	"git.fiblab.net/sim/routing/router/algo"
	"github.com/samber/lo"
)

type Lane struct {
	*mapv2.Lane
	line        []geometry.Point
	lineLengths []float64

	Speeds []float64

	// o----o----o------o :Nodes
	//      |    |
	//     AOI  AOI
	WalkNodeIds map[int]int         // Head, Tail
	Nodes       []algo.WalkLaneNode // 所有，包含Head和Tail和中间的AOI
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

	CenterPoint                       geometry.Point
	DriveOutNodeId, DriveInNodeId     int
	WalkOutNodeId, WalkInNodeId       int
	StationOutNodeId, StationInNodeId int
	StationTazCosts                   map[algo.TazPair][]algo.TazCost // 乘指定线路到达指定TAZ的cost
	SublineTazCosts                   map[int32][]algo.TazCost        // 经过此车站的所有线路到达指定TAZ的cost
	StationTaz                        algo.TazPair                    // 车站所在TAZ
	IsStation                         bool                            // 是否是车站
}
type PublicSubline struct {
	*mapv2.PublicTransportSubline
	StationPairs map[StationIdPairs]bool
	InStationIds map[int32]int
	SameLineID   int32
}
type TransportationAnalysisZone struct {
	StationIds []int32
}
type TransportationAnalysisZoneInfo struct {
	xStep float64
	yStep float64
	xMin  float64
	yMin  float64
}
type BusRouteResult struct {
	StartWalkSegments []*routingv2.WalkingRouteSegment
	StartWalkCost     float64
	TransferSegment   []*routingv2.TransferSegment
	TransferCost      float64
	EndWalkSegments   []*routingv2.WalkingRouteSegment
	EndWalkCost       float64
	Err               error
}
type StationIdPairs struct {
	FromID int32
	ToID   int32
}

func (r *BusRouteResult) BusRouteResultCost() float64 {
	//return r.EndWalkCost + r.TransferCost*float64(1+len(r.TransferSegment)/2)
	return float64(len(r.TransferSegment)) * r.TransferCost
}

type MongoPath struct {
	DB  string
	Col string
}
