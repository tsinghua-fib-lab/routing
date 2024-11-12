package main

import (
	"context"
	"errors"
	"fmt"
	"sync"

	"connectrpc.com/connect"
	"git.fiblab.net/general/common/v2/cache"
	"git.fiblab.net/general/common/v2/mongoutil"
	geov2 "git.fiblab.net/sim/protos/v2/go/city/geo/v2"
	mapv2 "git.fiblab.net/sim/protos/v2/go/city/map/v2"
	routingv2 "git.fiblab.net/sim/protos/v2/go/city/routing/v2"
	"git.fiblab.net/sim/protos/v2/go/city/routing/v2/routingv2connect"
	"git.fiblab.net/sim/routing/v2/router"
	"go.mongodb.org/mongo-driver/mongo"
)

func CheckPosition(pb *geov2.Position) error {
	if aoiPosition := pb.GetAoiPosition(); aoiPosition != nil {
		return nil
	} else if lanePosition := pb.GetLanePosition(); lanePosition != nil {
		return nil
	} else {
		return fmt.Errorf("no position data in request")
	}
}

type RoutingServer struct {
	routingv2connect.UnimplementedRoutingServiceHandler
	router *router.Router

	// 接口开启true或关闭false
	ok bool
	// 条件变量
	cond *sync.Cond
}

func NewRoutingServer(
	mongoURI string,
	mapPath, roadStatusPath *Path,
	cacheDir string,
) *RoutingServer {
	var client *mongo.Client
	lazyClient := func() *mongo.Client {
		if client == nil {
			client = mongoutil.NewClient(mongoURI)
		}
		return client
	}
	defer func() {
		if client != nil {
			client.Disconnect(context.Background())
		}
	}()

	mapData, err := cache.LoadWithCache(cacheDir, mapPath, func() *mapv2.Map {
		pb, errs := mongoutil.DownloadPbFromMongo[mapv2.Map](
			context.Background(), mongoutil.GetMongoColl(lazyClient(), mapPath), nil, nil,
		)
		if len(errs) > 0 {
			log.Panicf("failed to download map from %s: %v", mapPath, errs)
		}
		return pb
	})
	if err != nil {
		log.Panicf("failed to load map from %s: %v", mapPath, err)
	}

	var roadStatuses *routingv2.RoadStatuses
	if roadStatusPath != nil {
		roadStatuses, err = cache.LoadWithCache(cacheDir, roadStatusPath, func() *routingv2.RoadStatuses {
			pb, errs := mongoutil.DownloadPbFromMongo[routingv2.RoadStatuses](
				context.Background(), mongoutil.GetMongoColl(lazyClient(), roadStatusPath), nil, nil,
			)
			if len(errs) > 0 {
				log.Panicf("failed to download road statuses from %s: %v", roadStatusPath, errs)
			}
			return pb
		})
		if err != nil {
			log.Panicf("failed to load road statuses from %s: %v", roadStatusPath, err)
		}
	}

	// router
	// r := router.New(mapData, busLines, roadStatuses)
	r := router.New(mapData, roadStatuses)

	return &RoutingServer{
		router: r,
		ok:     true, cond: sync.NewCond(&sync.Mutex{})}
}

func (s *RoutingServer) GetRoute(
	ctx context.Context,
	req *connect.Request[routingv2.GetRouteRequest],
) (*connect.Response[routingv2.GetRouteResponse], error) {
	in := req.Msg
	// 暂停-恢复机制
	s.cond.L.Lock()
	for !s.ok {
		// 暂停中
		s.cond.Wait()
	}
	s.cond.L.Unlock()
	// 请求处理
	start := in.Start
	end := in.End
	// 检查数据格式
	if err := CheckPosition(start); err != nil {
		return nil, err
	}
	if err := CheckPosition(end); err != nil {
		return nil, err
	}
	// 检查数据是否超出范围
	if aoiPosition := start.GetAoiPosition(); aoiPosition != nil {
		if !s.router.HasAoiID(aoiPosition.AoiId) {
			return nil, connect.NewError(
				connect.CodeInvalidArgument,
				fmt.Errorf("no start Aoi ID: %v", aoiPosition.AoiId),
			)
		}
	} else if lanePosition := start.GetLanePosition(); lanePosition != nil {
		if !s.router.HasLaneID(lanePosition.LaneId) {
			return nil, connect.NewError(
				connect.CodeInvalidArgument,
				fmt.Errorf("no start Lane ID: %v", lanePosition.LaneId),
			)
		}
	}
	if aoiPosition := end.GetAoiPosition(); aoiPosition != nil {
		if !s.router.HasAoiID(aoiPosition.AoiId) {
			return nil, connect.NewError(
				connect.CodeInvalidArgument,
				fmt.Errorf("no end Aoi ID: %v", aoiPosition.AoiId),
			)
		}
	} else if lanePosition := end.GetLanePosition(); lanePosition != nil {
		if !s.router.HasLaneID(lanePosition.LaneId) {
			return nil, connect.NewError(
				connect.CodeInvalidArgument,
				fmt.Errorf("no end Lane ID: %v", lanePosition.LaneId),
			)
		}
	}
	ret := new(routingv2.GetRouteResponse)
	switch in.GetType() {
	case routingv2.RouteType_ROUTE_TYPE_DRIVING:
		// 检查start lane是否在road中
		if lanePosition := start.GetLanePosition(); lanePosition != nil {
			if !s.router.HasRoadLaneID(lanePosition.LaneId) {
				return nil, connect.NewError(
					connect.CodeInvalidArgument,
					errors.New("start lane is not valid road lane"),
				)
			}
		}
		log.Debugf("Search driving route from %v to %v", start, end)
		if roadIDs, cost, err := s.router.SearchDriving(start, end, in.Time); err != nil {
			// 无法找到通路，返回空响应
			return connect.NewResponse(&routingv2.GetRouteResponse{}), nil
		} else {
			ret.Journeys = append(ret.Journeys, &routingv2.Journey{
				Type: routingv2.JourneyType_JOURNEY_TYPE_DRIVING,
				Driving: &routingv2.DrivingJourneyBody{
					RoadIds: roadIDs,
					Eta:     cost,
				},
			})
		}
	case routingv2.RouteType_ROUTE_TYPE_WALKING:
		log.Debugf("Search walking route from %v to %v", start, end)
		if route, cost, err := s.router.SearchWalking(start, end, in.Time); err != nil {
			// 无法找到通路，返回空响应
			return connect.NewResponse(&routingv2.GetRouteResponse{}), nil
		} else {
			ret.Journeys = append(ret.Journeys, &routingv2.Journey{
				Type: routingv2.JourneyType_JOURNEY_TYPE_WALKING,
				Walking: &routingv2.WalkingJourneyBody{
					Route: route,
					Eta:   cost,
				},
			})
		}
	case routingv2.RouteType_ROUTE_TYPE_BUS, routingv2.RouteType_ROUTE_TYPE_SUBWAY, routingv2.RouteType_ROUTE_TYPE_BUS_SUBWAY:
		var availableSublineTypes []mapv2.SublineType
		var ptType string
		if in.GetType() == routingv2.RouteType_ROUTE_TYPE_BUS {
			availableSublineTypes = []mapv2.SublineType{mapv2.SublineType_SUBLINE_TYPE_BUS}
			ptType = "bus"
		} else if in.GetType() == routingv2.RouteType_ROUTE_TYPE_SUBWAY {
			availableSublineTypes = []mapv2.SublineType{mapv2.SublineType_SUBLINE_TYPE_SUBWAY}
			ptType = "subway"
		} else if in.GetType() == routingv2.RouteType_ROUTE_TYPE_BUS_SUBWAY {
			availableSublineTypes = []mapv2.SublineType{mapv2.SublineType_SUBLINE_TYPE_BUS, mapv2.SublineType_SUBLINE_TYPE_SUBWAY}
			ptType = "bus, subway"
		}
		log.Debugf("Search %v route from %v to %v", ptType, start, end)
		if startWalkSegments, startWalkCost, transferSegment, transferCost, endWalkSegments, endWalkCost, err := s.router.SearchBus(start, end, in.Time, availableSublineTypes); err != nil {
			// 无法找到通路，返回空响应
			return connect.NewResponse(&routingv2.GetRouteResponse{}), nil
		} else {
			if startWalkSegments != nil {
				ret.Journeys = append(ret.Journeys, &routingv2.Journey{
					Type: routingv2.JourneyType_JOURNEY_TYPE_WALKING,
					Walking: &routingv2.WalkingJourneyBody{
						Route: startWalkSegments,
						Eta:   startWalkCost,
					},
				})
			}
			if transferSegment != nil {
				ret.Journeys = append(ret.Journeys, &routingv2.Journey{
					Type: routingv2.JourneyType_JOURNEY_TYPE_BY_BUS,
					ByBus: &routingv2.BusJourneyBody{
						Transfers: transferSegment,
						Eta:       transferCost,
					},
				})
			}
			if endWalkSegments != nil {
				ret.Journeys = append(ret.Journeys, &routingv2.Journey{
					Type: routingv2.JourneyType_JOURNEY_TYPE_WALKING,
					Walking: &routingv2.WalkingJourneyBody{
						Route: endWalkSegments,
						Eta:   endWalkCost,
					},
				})
			}
		}
	default:
		return nil, connect.NewError(
			connect.CodeInvalidArgument,
			fmt.Errorf("unknown route type: %v", in.GetType()),
		)
	}
	return connect.NewResponse(ret), nil
}

func (s *RoutingServer) SetDrivingCosts(
	ctx context.Context,
	req *connect.Request[routingv2.SetDrivingCostsRequest],
) (*connect.Response[routingv2.SetDrivingCostsResponse], error) {
	in := req.Msg
	for _, c := range in.Costs {
		if err := s.router.SetRoadCost(c.Id, c.Cost, c.Time); err != nil {
			return nil, connect.NewError(connect.CodeNotFound, err)
		}
	}
	return connect.NewResponse(&routingv2.SetDrivingCostsResponse{}), nil
}

func (s *RoutingServer) GetDrivingCosts(
	ctx context.Context,
	req *connect.Request[routingv2.GetDrivingCostsRequest],
) (*connect.Response[routingv2.GetDrivingCostsResponse], error) {
	in := req.Msg
	out := &routingv2.GetDrivingCostsResponse{
		Costs: in.Costs,
	}
	for _, c := range out.Costs {
		if cost, err := s.router.GetRoadCost(c.Id, c.Time); err != nil {
			return nil, connect.NewError(connect.CodeNotFound, err)
		} else {
			c.Cost = cost
		}
	}
	return connect.NewResponse(out), nil
}

// 暂停导航服务
func (s *RoutingServer) Suspend() {
	s.cond.L.Lock()
	defer s.cond.L.Unlock()
	s.ok = false
}

// 恢复导航服务
func (s *RoutingServer) Resume() {
	s.cond.L.Lock()
	defer s.cond.L.Unlock()
	s.ok = true
	s.cond.Broadcast()
}

// 关闭导航服务
func (s *RoutingServer) Close() {
	s.router.Close()
}
