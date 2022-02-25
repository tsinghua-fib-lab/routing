package main

import (
	"context"
	"flag"
	"fmt"
	"log"
	"net"
	"strings"
	"time"

	geov1 "protos/wolong/geo/v1"
	pbRouting "protos/wolong/routing/v1"

	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
	"google.golang.org/grpc"
)

const (
	/// 行人、车辆、公交速度
	PERSON_SPEED      = 1.1
	VEHICLE_SPEED     = 60 / 3.6
	BUS_SPEED         = 40 / 3.6
	MAX_VEHICLE_SPEED = 80.0

	/// 行人必坐公交车的距离阈值
	BUS_DISTANCE = 1000

	/// 变道的距离和时间代价
	LANE_CHANGE_DISTANCE_COST = 1
	LANE_CHANGE_TIME_COST     = 5

	/// 公交车间隔
	INTERVAL = 300

	/// 数值常量
	FORWARD  = 1
	BACKWARD = 2
	LEFT     = 2
	RIGHT    = 3
	BUS      = 4

	DRIVE = 1
	WALK  = 2
)

var (
	/// 配置信息
	mongoUri = flag.String(
		"mongo_uri",
		"mongodb://localhost:27017/",
		"mongo db uri [example mongodb://localhost:27017/]",
	)
	mapPath = flag.String(
		"map",
		"db.col",
		"map database and collection [format: {db}.{col}]",
	)
	busPath = flag.String(
		"bus",
		"",
		"bus line database and collection, can be empty [format: {db}.{col}]",
	)
	grpcEndpoint = flag.String(
		"listen",
		"localhost:20218",
		"gRPC listening address",
	)
	numWorker = flag.Int(
		"j",
		16,
		"max grpc workers",
	)
)

func ParsePosition(pb *geov1.Position) (map[string]interface{}, error) {
	switch p := pb.MapPosition.(type) {
	case *geov1.Position_LanePosition:
		return map[string]interface{}{
			"lane_id": p.LanePosition.GetLaneId(),
			"s":       p.LanePosition.GetS(),
		}, nil
	case *geov1.Position_PoiPosition:
		return map[string]interface{}{
			"poi_id": p.PoiPosition.GetPoiId(),
		}, nil
	default:
		return make(map[string]interface{}, 0), fmt.Errorf("no position data in request")
	}
}

type Server struct {
	pbRouting.UnimplementedRoutingServiceServer
	Router *Router
}

func NewServer(
	mapDb string, mapCol string,
	busDb string, busCol string,
) *Server {
	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()
	client, err := mongo.Connect(ctx, options.Client().ApplyURI(*mongoUri))
	if err != nil {
		log.Fatal("mongo connect failed, err:", err)
	}
	mapData := client.Database(mapDb).Collection(mapCol)
	busData := new(mongo.Collection)
	if busDb != "" {
		busData = client.Database(busDb).Collection(busCol)
	}
	router := NewRouter(mapData, busData)
	log.Printf("Begin serving")
	return &Server{Router: router}
}

func (s *Server) GetRoute(ctx context.Context, in *pbRouting.GetRouteRequest) (*pbRouting.GetRouteResponse, error) {
	start, err := ParsePosition(in.GetStart())
	if err != nil {
		log.Fatal(err)
	}
	end, err := ParsePosition(in.GetEnd())
	if err != nil {
		log.Fatal(err)
	}
	if id, ok := start["poi_id"]; ok {
		if _, ok := s.Router.Pois[int32(id.(uint32))]; !ok {
			log.Fatalf("No such POI ID: %v", id)
		}
	} else if _, ok := s.Router.Lanes[int32(start["lane_id"].(uint32))]; !ok {
		log.Fatalf("No such Lane ID: %v", start["lane_id"])
	}
	if id, ok := end["poi_id"]; ok {
		if _, ok := s.Router.Pois[int32(id.(uint32))]; !ok {
			log.Fatalf("No such POI ID: %v", id)
		}
	} else if _, ok := s.Router.Lanes[int32(end["lane_id"].(uint32))]; !ok {
		log.Fatalf("No such Lane ID: %v", end["lane_id"])
	}
	ret := new(pbRouting.GetRouteResponse)
	switch in.GetType() {
	case pbRouting.RouteType_ROUTE_TYPE_DRIVING:
		log.Printf("Search driving route from %v to %v", start, end)
		var route []*pbRouting.DrivingRouteSegment
		if path, err := s.Router.SearchDriving(start, end); err != nil {
			return &pbRouting.GetRouteResponse{}, nil
		} else {
			for _, p := range path {
				route = append(route, &pbRouting.DrivingRouteSegment{
					LaneId:       uint32(p.U),
					NextLaneType: pbRouting.NextLaneType(p.V),
				})
			}
			route[len(route)-1].NextLaneType = pbRouting.NextLaneType_NEXT_LANE_TYPE_LAST
			ret.Journeys = append(ret.Journeys, &pbRouting.Journey{
				Type: pbRouting.JourneyType_JOURNEY_TYPE_DRIVING,
				JourneyBody: &pbRouting.Journey_Driving{
					Driving: &pbRouting.DrivingJourneyBody{
						Route: route,
					},
				},
			})
		}
	case pbRouting.RouteType_ROUTE_TYPE_WALKING:
		log.Printf("Search walking route from %v to %v", start, end)
		var route []*pbRouting.WalkingRouteSegment
		if path, err := s.Router.SearchWalking(start, end); err != nil {
			return &pbRouting.GetRouteResponse{}, nil
		} else {
			for _, p := range path {
				route = append(route, &pbRouting.WalkingRouteSegment{
					LaneId:          uint32(p.U),
					MovingDirection: pbRouting.MovingDirection(p.V),
				})
			}
			ret.Journeys = append(ret.Journeys, &pbRouting.Journey{
				Type: pbRouting.JourneyType_JOURNEY_TYPE_WALKING,
				JourneyBody: &pbRouting.Journey_Walking{
					Walking: &pbRouting.WalkingJourneyBody{
						Route: route,
					},
				},
			})
		}
	case pbRouting.RouteType_ROUTE_TYPE_BY_BUS:
		log.Printf("Search bus route from %v to %v", start, end)
		if path, err := s.Router.SearchBus(start, end); err != nil {
			return &pbRouting.GetRouteResponse{}, nil
		} else {
			for _, br := range path {
				if br.LaneId == -1 {
					var route []*pbRouting.WalkingRouteSegment
					for _, p := range br.Walk {
						route = append(route, &pbRouting.WalkingRouteSegment{
							LaneId:          uint32(p.U),
							MovingDirection: pbRouting.MovingDirection(p.V),
						})
					}
					ret.Journeys = append(ret.Journeys, &pbRouting.Journey{
						Type: pbRouting.JourneyType_JOURNEY_TYPE_WALKING,
						JourneyBody: &pbRouting.Journey_Walking{
							Walking: &pbRouting.WalkingJourneyBody{
								Route: route,
							},
						},
					})
				} else {
					ret.Journeys = append(ret.Journeys, &pbRouting.Journey{
						Type: pbRouting.JourneyType_JOURNEY_TYPE_BY_BUS,
						JourneyBody: &pbRouting.Journey_ByBus{
							ByBus: &pbRouting.BusJourneyBody{
								LineId:         uint32(br.LaneId),
								StartStationId: uint32(br.Start),
								EndStationId:   uint32(br.End),
							},
						},
					})
				}
			}
		}
	default:
		log.Fatalf("wrong routing type")
	}
	return ret, nil
}

func main() {
	log.SetPrefix("[routing] ")
	flag.Parse()
	mongoMap := strings.Split(*mapPath, ".")
	mongoBus := []string{"", ""}
	if *busPath != "" {
		mongoBus = strings.Split(*busPath, ".")
	}
	server := NewServer(
		mongoMap[0], mongoMap[1],
		mongoBus[0], mongoBus[1],
	)
	// 开启grpc服务
	lis, err := net.Listen("tcp", *grpcEndpoint)
	if err != nil {
		log.Fatalf("failed to listen: %v", err)
	}
	opts := []grpc.ServerOption{grpc.MaxConcurrentStreams(uint32(*numWorker))}
	s := grpc.NewServer(opts...)
	pbRouting.RegisterRoutingServiceServer(s, server)
	log.Printf("server listening at %v", lis.Addr())
	if err := s.Serve(lis); err != nil {
		log.Fatalf("failed to serve: %v", err)
	}
}
