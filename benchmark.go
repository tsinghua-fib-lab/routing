package main

import (
	"context"
	"flag"
	"runtime"
	"sync"
	"sync/atomic"
	"time"

	"math/rand"

	"connectrpc.com/connect"
	geov2 "git.fiblab.net/sim/protos/v2/go/city/geo/v2"
	routingv2 "git.fiblab.net/sim/protos/v2/go/city/routing/v2"
	"github.com/sirupsen/logrus"
)

var (
	benchmarkCount      = flag.Int("benchmark.count", 1000, "the random routing count for benchmark")
	benchmarkAoiIDStart = flag.Int("benchmark.aoi_id_start", 5_0000_0000, "the start aoi id for benchmark")
	benchmarkAoiIDEnd   = flag.Int("benchmark.aoi_id_end", 5_0000_0000+10000, "the end aoi id for benchmark")
	benchmarkSeed       = flag.Int64("benchmark.seed", 0, "the seed for benchmark")
	benchmarkCPU        = flag.Int("benchmark.cpu", 1, "the cpu count for benchmark")
)

func runBenchmark(server *RoutingServer) {
	log.Logger.SetLevel(logrus.WarnLevel)
	// 设置随机种子
	e := rand.New(rand.NewSource(*benchmarkSeed))
	// 随机生成benchmarkCount个路径规划请求，每个请求的起点和终点都是随机的
	idRange := int32(*benchmarkAoiIDEnd - *benchmarkAoiIDStart)
	reqs := make([]*connect.Request[routingv2.GetRouteRequest], *benchmarkCount)
	for i := 0; i < *benchmarkCount; i++ {
		startAoiID := e.Int31n(idRange) + int32(*benchmarkAoiIDStart)
		endAoiID := e.Int31n(idRange) + int32(*benchmarkAoiIDStart)
		req := connect.NewRequest(&routingv2.GetRouteRequest{
			Type: routingv2.RouteType_ROUTE_TYPE_DRIVING,
			Start: &geov2.Position{
				AoiPosition: &geov2.AoiPosition{
					AoiId: startAoiID,
				},
			},
			End: &geov2.Position{
				AoiPosition: &geov2.AoiPosition{
					AoiId: endAoiID,
				},
			},
		})
		reqs[i] = req
	}

	// 开始benchmark
	start := time.Now()
	var wg sync.WaitGroup
	var success atomic.Int32
	if *benchmarkCPU == 1 {
		for _, req := range reqs {
			res, err := server.GetRoute(context.Background(), req)
			if err != nil {
				log.Error("benchmark failed, err:", err)
			}
			if len(res.Msg.Journeys) > 0 {
				success.Add(1)
			}
		}
	} else {
		// 设置cpu数量
		runtime.GOMAXPROCS(*benchmarkCPU)
		wg.Add(*benchmarkCount)
		for _, req := range reqs {
			go func(req *connect.Request[routingv2.GetRouteRequest]) {
				defer wg.Done()
				res, err := server.GetRoute(context.Background(), req)
				if err != nil {
					log.Error("benchmark failed, err:", err)
				}
				if len(res.Msg.Journeys) > 0 {
					success.Add(1)
				}
				log.Info("benchmark finished one")
			}(req)
		}
		wg.Wait()
	}
	timeCost := time.Since(start) * time.Duration(*benchmarkCPU)
	log.Error(
		"benchmark finished", "\n",
		"count:", *benchmarkCount, "\n",
		"time:", timeCost, "\n",
		"avg:", timeCost/time.Duration(*benchmarkCount), "\n",
		"success:", success.Load(), "\n",
	)
}
