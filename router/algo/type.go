package algo

import "git.fiblab.net/general/common/geometry"

type TazCost struct {
	Cost             float64
	SublineID        int32
	SublineStartTime float64
	SublineEndTime   float64
	TazPair          TazPair
}
type TazPair struct {
	X int32
	Y int32
}
type DriveNodeAttr struct {
	ID    int32
	IsAoi bool
}

type DriveEdgeAttr struct {
	ID int32
}
type BusNodeAttr struct {
	ID              int32                 // 车站aoi id
	StationTAZCosts map[TazPair][]TazCost // 车站成指定线路到达指定TAZ的cost
	SublineTazCosts map[int32][]TazCost   // 经过此车站的所有线路到达指定TAZ的cost
	//StationTAZ      TazPair               // 车站所在TAZ
	CenterPoint geometry.Point
}

type BusEdgeAttr struct {
	FromID      int32 // 出发车站aoi id
	ToID        int32 // 到达车站aoi id
	SublineID   int32 // 线路 id
	SublineType int
}

type WalkNodeAttr struct {
	ID    int32
	IsAoi bool
}

type WalkLaneNode struct {
	S      float64 // 距离起点的距离
	NodeId int     // 在graph中的nodeId
}
