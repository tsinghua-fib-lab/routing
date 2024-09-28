package main

import (
	"context"
	"os"
	"testing"

	"connectrpc.com/connect"
	geov2 "git.fiblab.net/sim/protos/v2/go/city/geo/v2"
	routingv2 "git.fiblab.net/sim/protos/v2/go/city/routing/v2"
	"github.com/stretchr/testify/assert"
)

func FuzzRouter(f *testing.F) {
	mapPath, err := NewPath(os.Getenv("MAP_PATH"))
	assert.NoError(f, err)
	server := NewRoutingServer(
		os.Getenv("MONGO_URI"),
		mapPath, nil, "./data",
	)

	// 构造随机请求
	f.Fuzz(func(t *testing.T, driving bool, startAoi bool, startID uint16, startS float64, endAoi bool, endID uint16, endS float64) {
		req := &routingv2.GetRouteRequest{}
		if driving {
			req.Type = routingv2.RouteType_ROUTE_TYPE_DRIVING
		} else {
			req.Type = routingv2.RouteType_ROUTE_TYPE_WALKING
		}
		if startAoi {
			req.Start = &geov2.Position{
				AoiPosition: &geov2.AoiPosition{
					AoiId: 5_0000_0000 + int32(startID),
				},
			}
		} else {
			req.Start = &geov2.Position{
				LanePosition: &geov2.LanePosition{
					LaneId: int32(startID),
					S:      startS,
				},
			}
		}
		if endAoi {
			req.End = &geov2.Position{
				AoiPosition: &geov2.AoiPosition{
					AoiId: 5_0000_0000 + int32(endID),
				},
			}
		} else {
			req.End = &geov2.Position{
				LanePosition: &geov2.LanePosition{
					LaneId: int32(endID),
					S:      endS,
				},
			}
		}
		res, err := server.GetRoute(context.Background(), connect.NewRequest(req))
		// 有且只有一个是nil
		assert.True(f, (res == nil) != (err == nil))
	})
}
