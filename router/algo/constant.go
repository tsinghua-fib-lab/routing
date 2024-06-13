package algo

import (
	"errors"

	mapv2 "git.fiblab.net/sim/protos/go/city/map/v2"
)

const (
	// 行人、车辆、公交速度
	BUS_SPEED = 40 / 3.6

	// 行人必坐公交车的距离阈值
	BUS_DISTANCE = 1000

	// 数值常量
	FORWARD  = 1
	BACKWARD = 2

	BUS = 4

	// 道路头尾，与连接关系对应
	HEAD = int(mapv2.LaneConnectionType_LANE_CONNECTION_TYPE_HEAD)
	TAIL = int(mapv2.LaneConnectionType_LANE_CONNECTION_TYPE_TAIL)

	// 道路边权的分辨率/s
	TIME_SLICE_INTERVAl = 300
	// 道路边权时间片数
	TIME_SLICE_LENGTH = 288
)

var (
	// 错误：超出道路边权时间片数
	ErrOutOfTimeSlice = errors.New("out of time slice, should be less than 288")
	// 错误：对非时序图设置长度超过1的边权
	ErrNoTDGraph = errors.New("no time dependent graph, should set edge length with length 1")
)
