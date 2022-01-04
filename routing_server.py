#!/usr/bin/env python3

from concurrent import futures
from math import inf, sqrt
from tqdm import tqdm
import argparse
import grpc
import igraph
import logging, coloredlogs
import os
import pickle
import pymongo
from protos.py import routing_pb2, routing_service_pb2, routing_service_pb2_grpc

# 行人、车辆、公交速度
PERSON_SPEED = 1.1
VEHICLE_SPEED = 60 / 3.6
BUS_SPEED = 40 / 3.6

# 行人必坐公交的距离阈值
BUS_DISTANCE = 1000

# 变道的距离和时间代价
LANE_CHANGE_DISTANCE_COST = 1
LANE_CHANGE_TIME_COST = 5

# 数值常量
FORWARD = 0
BACKWARD = 1
LEFT = 2
RIGHT = 3
BUS = 4

NextLaneType = [
    routing_pb2.NextLaneType.NEXT_LANE_TYPE_FORWARD,
    None,
    routing_pb2.NextLaneType.NEXT_LANE_TYPE_LEFT,
    routing_pb2.NextLaneType.NEXT_LANE_TYPE_RIGHT,
]

MovingDirection = [
    routing_pb2.MovingDirection.MOVING_DIRECTION_FORWARD,
    routing_pb2.MovingDirection.MOVING_DIRECTION_BACKWARD,
]


class Graph:
    def __init__(self, directed=True):
        self.directed = directed
        self.node_id = 0
        self.edges = []
        self.edges_attr = []
        self.edges_length = []
        self.edges_lookup = {}
        self.graph = None
        self.stash_mode = False
        self.stashed_node_id = 0
        self.stashed_edge_ids = []
        self.stashed_edge_lengths = []

    def add_node(self):
        self.node_id += 1
        if self.node_id >= len(self.graph.vs):
            self.graph.add_vertices(1)
        return self.node_id

    def add_edge(self, u, v, length, overwrite=False, **attr):
        assert (
            (not self.stash_mode and self.graph is None)
            if overwrite
            else ((u, v) not in self.edges_lookup)
        )
        if overwrite and (u, v) in self.edges_lookup:
            e = self.edges_lookup[u, v]
            self.edges_length[e] = length
            self.edges_attr[e] = attr
        else:
            self.edges_lookup[u, v] = len(self.edges)
            self.edges.append((u, v))
            if self.graph is not None:
                e = self.graph.add_edge(u, v, length=length)
                if self.stash_mode:
                    self.stashed_edge_ids.append(e.index)
            self.edges_length.append(length)
            self.edges_attr.append(attr)

    def set_edge_length(self, u, v, length):
        assert self.graph is not None
        assert (u, v) in self.edges_lookup
        e = self.edges_lookup[u, v]
        if self.stash_mode:
            self.stashed_edge_lengths.append([e, self.edges_length[e]])
        self.edges_length[e] = length
        self.graph.es[e]["length"] = length

    def build(self):
        assert self.graph is None
        assert len(self.edges_lookup) == len(self.edges)
        self.graph = igraph.Graph(self.edges, directed=self.directed)
        self.graph.es["length"] = self.edges_length
        self.node_id = len(self.graph.vs) - 1

    def shortest_path(self, u, v, check_equal=True):
        """
        Find shortest path between `u` and `v`

        Return a list of edge ids of the path
        - `[]` if `u`==`v`
        - `None` if no path found between `u` and `v`
        """
        if u == v:
            assert not check_equal
            return []
        else:
            return (
                self.graph.get_shortest_paths(u, v, weights="length", output="epath")[0]
                or None
            )

    def shortest_path_length(self, u, v):
        """
        Return inf if no such path exists
        """
        if u == v:
            return 0
        else:
            p = self.graph.get_shortest_paths(u, v, weights="length", output="epath")[0]
            if p:
                return self.path_length(p)
            else:
                return inf

    def path_length(self, path):
        return sum(self.edges_length[i] for i in path)

    def stash(self):
        """
        Put the graph in stash mode.

        Used together with `pop`
        """
        assert self.graph is not None and not self.stash_mode
        self.stash_mode = True
        self.stashed_node_id = self.node_id
        self.stashed_edge_ids = []
        self.stashed_edge_lengths = []

    def pop(self):
        """
        Put the graph in normal mode and remove any modifications made in stash mode.

        Used together with `stash`
        """
        assert self.stash_mode
        self.stash_mode = False
        if self.stashed_edge_ids:
            self.node_id = self.stashed_node_id
            n = len(self.stashed_edge_ids)
            self.edges = self.edges[:-n]
            self.edges_attr = self.edges_attr[:-n]
            self.edges_length = self.edges_length[:-n]
            for _ in range(n):
                self.edges_lookup.popitem()
            self.graph.delete_edges(self.stashed_edge_ids[::-1])
        for e, l in self.stashed_edge_lengths:
            self.edges_length[e] = l
            self.graph.es[e]["length"] = l

    def stashed(self):
        """
        Helper function to use stash mode with `with`

        Example:
        ```python
        with G.stashed():
            pass
        ```
        """
        return self

    def __enter__(self):
        self.stash()

    def __exit__(self, *_):
        self.pop()


def stop_distance_iter(stops, distances):
    """
    Used for enumerating all possible stop-stop pairs and their distances
    """
    assert len(stops) == len(distances)
    n = len(stops)
    stops = stops + stops[:-1]
    ds = [0]
    for i in distances + distances[:-2]:
        ds.append(ds[-1] + i)
    for i in range(n):
        for j in range(i + 1, i + n):
            yield [stops[i], stops[j], ds[j] - ds[i]]


def distance(x1, y1, x2, y2):
    x1 -= x2
    y1 -= y2
    return sqrt(x1 * x1 + y1 * y1)


class Router:
    def __init__(self, map_, bus_):
        logging.info("Get map from database")
        self.lanes = lanes = {
            i["data"]["id"]: i["data"] for i in map_.find({"class": "lane"})
        }
        self.pois = pois = {
            i["data"]["id"]: i["data"] for i in map_.find({"class": "poi"})
        }
        self.bus_graph = None
        if bus_ is not None:
            logging.info("Get bus lines from database")
            bus_lines = list(bus_.find())
        logging.info("Build vanilla graph")
        node_id = -1
        self.vanilla_graph = vanilla_graph = Graph()
        for lane in lanes.values():
            if "HEAD" not in lane:
                node_id += 1
                lane["HEAD"] = node_id
                for i in lane["predecessors"]:
                    assert i["type"][-4:] not in lanes[i["id"]]
                    lanes[i["id"]][i["type"][-4:]] = node_id
            if "TAIL" not in lane:
                node_id += 1
                lane["TAIL"] = node_id
                for i in lane["successors"]:
                    assert i["type"][-4:] not in lanes[i["id"]]
                    lanes[i["id"]][i["type"][-4:]] = node_id
            vanilla_graph.add_edge(
                lane["HEAD"],
                lane["TAIL"],
                id=lane["id"],
                length=lane["length"],
                type=FORWARD,
            )
            if lane["type"] == "LANE_TYPE_WALKING":
                vanilla_graph.add_edge(
                    lane["TAIL"],
                    lane["HEAD"],
                    id=lane["id"],
                    length=lane["length"],
                    type=BACKWARD,
                )
        for i in lanes.values():
            if i["left_lane_ids"]:
                vanilla_graph.add_edge(
                    i["TAIL"],
                    lanes[i["left_lane_ids"][0]]["TAIL"],
                    id=i["left_lane_ids"][0],
                    length=LANE_CHANGE_DISTANCE_COST,
                    type=LEFT,
                )
            if i["right_lane_ids"]:
                vanilla_graph.add_edge(
                    i["TAIL"],
                    lanes[i["right_lane_ids"][0]]["TAIL"],
                    id=i["right_lane_ids"][0],
                    length=LANE_CHANGE_DISTANCE_COST,
                    type=RIGHT,
                )
        vanilla_graph.build()
        logging.info(f"{node_id} nodes and {len(vanilla_graph.edges)} edges")
        logging.info(
            f"degree: max {max(vanilla_graph.graph.degree())} mean {sum(vanilla_graph.graph.degree())/node_id:.2f}"
        )
        if bus_ is not None:
            logging.info("Build bus graph")
            self.bus_graph = bus_graph = Graph()
            self.node2poi = node2poi = {}
            walk_graph = Graph()
            for i in lanes.values():
                if i["type"] == "LANE_TYPE_WALKING":
                    i["NODES"] = []
            for i in pois.values():
                if i["type"] == "POI_TYPE_BUS_STATION":
                    node_id += 1
                    lanes[i["walking_position"]["lane_id"]]["NODES"].append(
                        [i["walking_position"]["s"], node_id]
                    )
                    i["NODE"] = node_id
                    node2poi[node_id] = i["id"]
                    i["POS"] = [i["walking_position"]["x"], i["walking_position"]["y"]]
            for i in lanes.values():
                if i["type"] == "LANE_TYPE_WALKING":
                    i["NODES"].sort(key=lambda x: x[0])
                    ns = i["NODES"] = [
                        [0, i["HEAD"]],
                        *i["NODES"],
                        [i["length"], i["TAIL"]],
                    ]
                    i["EDGE"] = len(bus_graph.edges)
                    for (u_s, u), (v_s, v) in zip(ns, ns[1:]):
                        bus_graph.add_edge(u, v, v_s - u_s, id=i["id"], type=FORWARD)
                        bus_graph.add_edge(v, u, v_s - u_s, id=i["id"], type=BACKWARD)
                        walk_graph.add_edge(u, v, v_s - u_s, id=i["id"], type=FORWARD)
                        walk_graph.add_edge(v, u, v_s - u_s, id=i["id"], type=BACKWARD)
            walk_graph.build()
            for i in tqdm(bus_lines):
                wait_time = i["interval"] / 2
                for u, v, d in stop_distance_iter(i["stops"], i["distances"]):
                    u_p = pois[u]
                    v_p = pois[v]
                    u = u_p["NODE"]
                    v = v_p["NODE"]
                    d = (d / BUS_SPEED + wait_time) * PERSON_SPEED
                    add = False
                    if (u, v) in bus_graph.edges_lookup:
                        add = d < bus_graph.edges_length[bus_graph.edges_lookup[u, v]]
                    else:
                        d_uv = distance(*u_p["POS"], *v_p["POS"])
                        add = (
                            d_uv > d
                            or d_uv > BUS_DISTANCE
                            or walk_graph.shortest_path_length(u, v) > d
                        )
                    if add:
                        bus_graph.add_edge(
                            u,
                            v,
                            d,
                            overwrite=True,
                            id=i["line_id"],
                            type=BUS,
                        )
            bus_graph.build()
            logging.info(
                f"{len(bus_graph.graph.vs)} nodes and {len(bus_graph.edges)} edges"
            )
            logging.info(
                f"degree: max {max(bus_graph.graph.degree())} mean {sum(bus_graph.graph.degree())/node_id:.2f}"
            )

    def search_driving(self, start, end):
        if "poi_id" in start:
            p = self.pois[start["poi_id"]]["driving_position"]
            start_lane = p["lane_id"]
            start_s = p["s"]
        else:
            start_lane = start["lane_id"]
            start_s = start["s"]
        if "poi_id" in end:
            p = self.pois[end["poi_id"]]["driving_position"]
            end_lane = p["lane_id"]
            end_s = p["s"]
        else:
            end_lane = end["lane_id"]
            end_s = end["s"]
        G = self.vanilla_graph
        ret = [[start_lane, FORWARD]]
        if start_lane == end_lane:
            if start_s <= end_s:
                return ret
            else:
                lane = self.lanes[start_lane]
                paths = []
                v = lane["TAIL"]
                for u in G.graph.successors(v):
                    e = G.edges_lookup[v, u]
                    if G.edges_attr[e]["type"] != FORWARD:
                        continue
                    p = G.shortest_path(u, v)
                    if p:
                        paths.append([e, *p])
                if not paths:
                    return None
                path = min(
                    paths,
                    key=lambda x: G.path_length(x),
                )
        else:
            assert self.lanes[start_lane]["TAIL"] != self.lanes[end_lane]["TAIL"]
            path = G.shortest_path(
                self.lanes[start_lane]["TAIL"], self.lanes[end_lane]["TAIL"]
            )
            if not path:
                return None
        for e in path:
            edge = G.edges_attr[e]
            if edge["type"] >= 2:
                ret[-1][1] = edge["type"]
            ret.append([edge["id"], FORWARD])
        return ret

    def search_walking(self, start, end):
        if "poi_id" in start:
            p = self.pois[start["poi_id"]]["walking_position"]
            start_lane = p["lane_id"]
            start_s = p["s"]
        else:
            start_lane = start["lane_id"]
            start_s = start["s"]
        if "poi_id" in end:
            p = self.pois[end["poi_id"]]["walking_position"]
            end_lane = p["lane_id"]
            end_s = p["s"]
        else:
            end_lane = end["lane_id"]
            end_s = end["s"]
        G = self.vanilla_graph
        if start_lane == end_lane:
            if start_s <= end_s:
                return [[start_lane, FORWARD]]
            else:
                return [[start_lane, BACKWARD]]
        s = self.lanes[start_lane]
        e = self.lanes[end_lane]
        paths = []
        _debug_flag = False
        for s_head, s_tail, s_s in [
            (s["HEAD"], s["TAIL"], start_s),
            (s["TAIL"], s["HEAD"], s["length"] - start_s),
        ]:
            for e_head, e_tail, e_s in [
                (e["HEAD"], e["TAIL"], end_s),
                (e["TAIL"], e["HEAD"], e["length"] - end_s),
            ]:
                p = G.shortest_path(s_head, e_head, False)
                if p is not None:
                    _debug_flag = True
                    if G.edges[p[0]][1] == s_tail:
                        continue
                    if p and G.edges[p[-1]][0] == e_tail:
                        paths.append(
                            [
                                s_s + G.path_length(p) - e_s,
                                [G.edges_lookup[s_tail, s_head], *p],
                            ]
                        )
                    else:
                        paths.append(
                            [
                                s_s + G.path_length(p) + e_s,
                                [
                                    G.edges_lookup[s_tail, s_head],
                                    *p,
                                    G.edges_lookup[e_head, e_tail],
                                ],
                            ]
                        )
        if not paths:
            assert not _debug_flag
            return None
        path = min(paths, key=lambda x: x[0])[1]
        path = [[i["id"], i["type"]] for i in (G.edges_attr[e] for e in path)]
        if __debug__:
            for (a, _), (b, _) in zip(path, path[1:]):
                assert a != b
        return path
        # with G.stashed():
        #     u = G.add_node()
        #     G.add_edge(u, s["HEAD"], start_s, id=start_lane, type=BACKWARD)
        #     G.add_edge(u, s["TAIL"], s["length"] - start_s, id=start_lane, type=FORWARD)
        #     G.set_edge_length(s["HEAD"], s["TAIL"], inf)
        #     G.set_edge_length(s["TAIL"], s["HEAD"], inf)
        #     v = G.add_node()
        #     G.add_edge(e["HEAD"], v, end_s, id=end_lane, type=FORWARD)
        #     G.add_edge(e["TAIL"], v, e["length"] - end_s, id=end_lane, type=BACKWARD)
        #     G.set_edge_length(e["HEAD"], e["TAIL"], inf)
        #     G.set_edge_length(e["TAIL"], e["HEAD"], inf)
        #     path = G.shortest_path(u, v)
        #     if not path:
        #         return None
        #     return [[i["id"], i["type"]] for i in (G.edges_attr[e] for e in path)]

    def search_bus(self, start, end):
        G = self.bus_graph
        assert G is not None
        start_pos = []  # node, edge_id, s
        if (
            "poi_id" in start
            and self.pois[start["poi_id"]]["type"] == "POI_TYPE_BUS_STATION"
        ):
            start_pos.append([self.pois[start["poi_id"]]["NODE"], [], -1, 0])
        else:
            if "poi_id" in start:
                p = self.pois[start["poi_id"]]["walking_position"]
                l = p["lane_id"]
                s = p["s"]
            else:
                l = start["lane_id"]
                s = start["s"]
            if s == 0:
                start_pos.append([self.lanes[l]["HEAD"], [], -1, 0])
            else:
                l = self.lanes[l]
                ns = l["NODES"]
                for i in range(1, len(ns)):
                    if s <= ns[i][0]:
                        break
                else:
                    assert False
                s1, p1 = ns[i - 1]
                s2, p2 = ns[i]
                assert s > s1
                start_s = s
                edge = l["EDGE"] + (i - 1) * 2
                # assert G.edges[edge] == (p1, p2)
                # assert G.edges[edge + 1] == (p2, p1)
                start_pos.append([p2, [edge], edge + 1, s2 - s])
                start_pos.append([p1, [edge + 1], edge, s - s1])

        end_pos = []  # node, edge_id, s
        if (
            "poi_id" in end
            and self.pois[end["poi_id"]]["type"] == "POI_TYPE_BUS_STATION"
        ):
            end_pos.append([self.pois[end["poi_id"]]["NODE"], [], -1, 0])
        else:
            if "poi_id" in end:
                p = self.pois[end["poi_id"]]["walking_position"]
                l = p["lane_id"]
                s = p["s"]
            else:
                l = end["lane_id"]
                s = end["s"]
            if s == 0:
                end_pos.append([self.lanes[l]["HEAD"], [], -1, 0])
            else:
                l = self.lanes[l]
                ns = l["NODES"]
                for i in range(1, len(ns)):
                    if s <= ns[i][0]:
                        break
                else:
                    assert False
                s1, p1 = ns[i - 1]
                s2, p2 = ns[i]
                assert s > s1
                end_s = s
                edge = l["EDGE"] + (i - 1) * 2
                # assert G.edges[edge] == (p1, p2)
                # assert G.edges[edge + 1] == (p2, p1)
                end_pos.append([p1, [edge], edge + 1, s - s1])
                end_pos.append([p2, [edge + 1], edge, s2 - s])
        if len(start_pos) == 2 == len(end_pos) and start_pos[0][1] == end_pos[0][1]:
            if start_s <= end_s:
                return [[[G.edges_attr[start_pos[0][1][0]]["id"], FORWARD]]]
            else:
                return [[[G.edges_attr[start_pos[0][1][0]]["id"], BACKWARD]]]
        paths = []
        _debug_flag = False
        for start_node, start_edge, start_edge_reverse, start_s in start_pos:
            for end_node, end_edge, end_edge_reverse, end_s in end_pos:
                p = G.shortest_path(start_node, end_node, False)
                if p is not None:
                    _debug_flag = True
                    if p and (p[0] == start_edge_reverse or p[-1] == end_edge_reverse):
                        continue
                    paths.append(
                        [start_s + G.path_length(p) + end_s, start_edge + p + end_edge]
                    )
        if not paths:
            assert not _debug_flag
            return None
        path = min(paths, key=lambda x: x[0])[1]
        ret = []
        walk = []
        for e in path:
            attr = G.edges_attr[e]
            if attr["type"] == BUS:
                if walk:
                    ret.append(walk)
                    walk = []
                u, v = G.edges[e]
                if ret and ret[-1][0] is BUS and ret[-1][1] == attr["id"]:
                    ret[-1][3] = self.node2poi[v]
                else:
                    ret.append([BUS, attr["id"], self.node2poi[u], self.node2poi[v]])
            else:
                if walk and walk[-1][0] == attr["id"]:
                    assert walk[-1][1] == attr["type"]
                else:
                    walk.append([attr["id"], attr["type"]])
        if walk:
            ret.append(walk)
        return ret


def parse_position(pb):
    if pb.HasField("lane_position"):
        return {"lane_id": pb.lane_position.lane_id, "s": pb.lane_position.s}
    elif pb.HasField("poi_position"):
        return {"poi_id": pb.poi_position.poi_id}
    else:
        return None


class RoutingServiceServicer(routing_service_pb2_grpc.RoutingServiceServicer):
    def __init__(self, mongo_uri, map_db, map_col, bus_db, bus_col, cache_path):
        if cache_path and os.path.exists(cache_path):
            logging.info("Load from " + cache_path)
            self.router = pickle.load(open(cache_path, "rb"))
        else:
            client = pymongo.MongoClient(mongo_uri)
            map_ = client[map_db][map_col]
            if bus_db:
                bus_ = client[bus_db][bus_col]
            else:
                bus_ = None
            self.router = Router(map_, bus_)
            if cache_path:
                logging.info("Save to " + cache_path)
                pickle.dump(self.router, open(cache_path, "wb"))
        logging.info("Begin serving")

    def GetRoute(self, request, context):
        start = parse_position(request.start)
        if start is None:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details("No start position")
            logging.error("No start position")

        end = parse_position(request.end)
        if end is None:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details("No end position")
            logging.error("No end position")

        for i in [start, end]:
            if "poi_id" in i:
                if i["poi_id"] not in self.router.pois:
                    context.set_code(grpc.StatusCode.NOT_FOUND)
                    context.set_details(f"No such POI ID: {i['poi_id']}")
                    logging.error(f"No such POI ID: {i['poi_id']}")
            elif i["lane_id"] not in self.router.lanes:
                context.set_code(grpc.StatusCode.NOT_FOUND)
                context.set_details(f"No such Lane ID: {i['lane_id']}")
                logging.error(f"No such Lane ID: {i['lane_id']}")
        ret = routing_service_pb2.GetRouteResponse()
        if request.type == routing_pb2.RouteType.ROUTE_TYPE_DRIVING:
            logging.info(f"Search driving route from {start} to {end}")
            path = self.router.search_driving(start, end)
            if path is not None:
                journey = ret.journeys.add()
                journey.type = routing_pb2.JourneyType.JOURNEY_TYPE_DRIVING
                route = journey.driving.route
                for i, j in path:
                    seg = route.add()
                    seg.lane_id = i
                    seg.next_lane_type = NextLaneType[j]
                seg.next_lane_type = routing_pb2.NextLaneType.NEXT_LANE_TYPE_LAST
        elif request.type == routing_pb2.RouteType.ROUTE_TYPE_WALKING:
            logging.info(f"Search walking route from {start} to {end}")
            path = self.router.search_walking(start, end)
            if path is not None:
                journey = ret.journeys.add()
                journey.type = routing_pb2.JourneyType.JOURNEY_TYPE_WALKING
                route = journey.walking.route
                for i, j in path:
                    seg = route.add()
                    seg.lane_id = i
                    seg.moving_direction = MovingDirection[j]
        elif request.type == routing_pb2.RouteType.ROUTE_TYPE_BY_BUS:
            logging.info(f"Search bus route from {start} to {end}")
            path = self.router.search_bus(start, end)
            if path is not None:
                for seg in path:
                    journey = ret.journeys.add()
                    if seg[0] is BUS:
                        journey.type = routing_pb2.JourneyType.JOURNEY_TYPE_BY_BUS
                        journey.by_bus.line_id = seg[1]
                        journey.by_bus.start_station_id = seg[2]
                        journey.by_bus.end_station_id = seg[3]
                    else:
                        journey.type = routing_pb2.JourneyType.JOURNEY_TYPE_WALKING
                        route = journey.walking.route
                        for i, j in seg:
                            seg = route.add()
                            seg.lane_id = i
                            seg.moving_direction = MovingDirection[j]
        else:
            context.set_code(grpc.StatusCode.INVALID_ARGUMENT)
            context.set_details(f"Invalid RouteType: {request.type}")
            logging.error(f"Invalid RouteType: {request.type}")
            return
        return ret


if __name__ == "__main__":
    coloredlogs.install(fmt="%(asctime)s.%(msecs)03d %(levelname)s %(message)s")
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mongo_uri",
        help="mongo db uri [example mongodb://localhost:27017/]",
        required=True,
    )
    parser.add_argument(
        "--map",
        required=True,
        help="map database and collection [format: {db}.{col}]",
    )
    parser.add_argument(
        "--bus",
        help="bus line database and collection, can be empty [format: {db}.{col}]",
        default="",
    )
    parser.add_argument(
        "--listen", help="gRPC listening address", default="0.0.0.0:20218"
    )
    parser.add_argument(
        "--cache",
        help="graph cache path, use empty string to disable caching",
        default="",
    )
    parser.add_argument("-j", type=int, help="max gRPC workers", default=10)

    args = parser.parse_args()
    assert args.map.count(".") == 1
    map_db, map_col = args.map.split(".")
    if args.bus:
        assert args.bus.count(".") == 1
        bus_db, bus_col = args.bus.split(".")
    else:
        bus_db = bus_col = ""

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=args.j))
    routing_service_pb2_grpc.add_RoutingServiceServicer_to_server(
        RoutingServiceServicer(
            args.mongo_uri, map_db, map_col, bus_db, bus_col, args.cache
        ),
        server,
    )
    server.add_insecure_port(args.listen)
    server.start()
    logging.info(f"Listening to {args.listen}")
    server.wait_for_termination()
