import grpc
import pymongo
import random
import sys
import os
from tqdm import tqdm
import time

from routing_server import MovingDirection

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "protos", "py")
)
from protos.py.wolong.routing.v1 import routing_pb2
from protos.py.wolong.routing.v1 import routing_service_pb2
from protos.py.wolong.routing.v1 import routing_service_pb2_grpc

NextLaneType = "UNSPECIFIED FORWARD LEFT RIGHT LAST".split()
MovingDirection = "UNSPECIFIED FORWARD BACKWARD".split()


def to_position(p_in, p_out):
    if "poi_id" in p_in:
        p_out.poi_position.poi_id = p_in["poi_id"]
    else:
        p_out.lane_position.lane_id = p_in["lane_id"]
        p_out.lane_position.s = p_in["s"]


def request_driving(stub, start, end):
    req = routing_service_pb2.GetRouteRequest()
    req.type = routing_pb2.RouteType.ROUTE_TYPE_DRIVING
    to_position(start, req.start)
    to_position(end, req.end)
    res = stub.GetRoute(req)
    if res.journeys:
        assert len(res.journeys) == 1
        journey = res.journeys[0]
        assert journey.type == routing_pb2.JourneyType.JOURNEY_TYPE_DRIVING
        ret = []
        for seg in journey.driving.route:
            ret.append([seg.lane_id, NextLaneType[seg.next_lane_type]])
        return ret
    else:
        return []


def request_walking(stub, start, end):
    req = routing_service_pb2.GetRouteRequest()
    req.type = routing_pb2.RouteType.ROUTE_TYPE_WALKING
    to_position(start, req.start)
    to_position(end, req.end)
    res = stub.GetRoute(req)
    if res.journeys:
        assert len(res.journeys) == 1
        journey = res.journeys[0]
        assert journey.type == routing_pb2.JourneyType.JOURNEY_TYPE_WALKING
        ret = []
        for seg in journey.walking.route:
            ret.append([seg.lane_id, MovingDirection[seg.moving_direction]])
        return ret
    else:
        return []


def request_bus(stub, start, end):
    req = routing_service_pb2.GetRouteRequest()
    req.type = routing_pb2.RouteType.ROUTE_TYPE_BY_BUS
    to_position(start, req.start)
    to_position(end, req.end)
    res = stub.GetRoute(req)
    ret = []
    if res.journeys:
        for journey in res.journeys:
            if journey.type == routing_pb2.JourneyType.JOURNEY_TYPE_WALKING:
                walk = []
                for seg in journey.walking.route:
                    walk.append([seg.lane_id, MovingDirection[seg.moving_direction]])
                ret.append(walk)
            else:
                assert journey.type == routing_pb2.JourneyType.JOURNEY_TYPE_BY_BUS
                ret.append(
                    [
                        journey.by_bus.line_id,
                        journey.by_bus.start_station_id,
                        journey.by_bus.end_station_id,
                    ]
                )
    return ret


with grpc.insecure_channel("localhost:20218") as channel:
    stub = routing_service_pb2_grpc.RoutingServiceStub(channel)

    # print(request_driving(stub, {"poi_id": 400001010}, {"poi_id": 400001011}))
    # print(request_driving(stub, {"poi_id": 400001011}, {"poi_id": 400001010}))
    # print(request_driving(stub, {"poi_id": 400003813}, {"poi_id": 400001249}))

    # print(request_walking(stub, {"poi_id": 400000982}, {"poi_id": 400000954}))
    # print(request_walking(stub, {"poi_id": 400000954}, {"poi_id": 400000982}))
    # print(request_walking(stub, {"poi_id": 400000949}, {"poi_id": 400000943}))
    # print(request_walking(stub, {"poi_id": 400000938}, {"poi_id": 400003858}))

    # print(request_bus(stub, {"poi_id": 400002132}, {"poi_id": 400002128}))
    # for i in request_bus(stub, {"poi_id": 400003788}, {"poi_id": 400003736}):
    #     print(i)

    client = pymongo.MongoClient("mongodb://root:root%400226@rl1.cityzoom.cn:27017/")
    pois = {
        i["data"]["id"]: i["data"]
        for i in client.dev_t.map_beijing_3_1223.find({"class": "poi"})
    }

    N = 10000

    print("Test driving")
    d_p = [i for i, j in pois.items() if "driving_position" in j]
    total = 0
    routed = 0
    length = 0
    for _ in tqdm(range(N)):
        a, b = random.sample(d_p, 2)
        t = time.time()
        p = request_driving(stub, {"poi_id": a}, {"poi_id": b})
        total += time.time() - t
        if p:
            routed += 1
            length += len(p)
    print(f"Routed: {routed/N*100:.2f}%")
    print(f"Average time: {total/N}")
    print(f"Average length (in lanes): {length/N}")

    print("\nTest walking")
    w_p = [i for i, j in pois.items() if "walking_position" in j]
    total = 0
    routed = 0
    length = 0
    for _ in tqdm(range(N)):
        a, b = random.sample(w_p, 2)
        t = time.time()
        p = request_walking(stub, {"poi_id": a}, {"poi_id": b})
        total += time.time() - t
        if p:
            routed += 1
            length += len(p)
    print(f"Routed: {routed/N*100:.2f}%")
    print(f"Average time: {total/N}")
    print(f"Average length (in lanes): {length/N}")

    print("\nTest bus only")
    b_p = [i for i, j in pois.items() if j["type"] == "POI_TYPE_BUS_STATION"]
    total = 0
    routed = 0
    length = 0
    for _ in tqdm(range(N)):
        a, b = random.sample(b_p, 2)
        t = time.time()
        p = request_bus(stub, {"poi_id": a}, {"poi_id": b})
        total += time.time() - t
        if p:
            routed += 1
            length += len(p)
    print(f"Routed: {routed/N*100:.2f}%")
    print(f"Average time: {total/N}")
    print(f"Average length (in lanes): {length/N}")

    print("\nTest bus+walk")
    b_p = [i for i, j in pois.items() if "walking_position" in j]
    total = 0
    routed = 0
    length = 0
    for _ in tqdm(range(N)):
        a, b = random.sample(b_p, 2)
        t = time.time()
        p = request_bus(stub, {"poi_id": a}, {"poi_id": b})
        total += time.time() - t
        if p:
            routed += 1
            length += len(p)
    print(f"Routed: {routed/N*100:.2f}%")
    print(f"Average time: {total/N}")
    print(f"Average length (in lanes): {length/N}")
