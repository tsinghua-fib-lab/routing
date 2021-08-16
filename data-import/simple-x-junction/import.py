#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import pymongo
import json
from pathlib import Path

SIMPLE_X_JUNCTION = Path(__file__).parent / "simple-x-junction.json"


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--uri", default="mongodb://localhost:27017/", type=str, help="Mongodb URI")
    parser.add_argument("--db", default="dev_t", type=str, help="Map Database")
    parser.add_argument("--col", default="map",
                        type=str, help="Map Collection")
    parser.add_argument("--setid", default="simple-x-junction", type=str, help="Map Setid")
    return parser.parse_args()

def fix_float_in_lane(v: dict):
    v["max_speed"] = float(v["max_speed"])
    v["length"] = float(v["length"])
    polylines = ["center_line",
                 "left_border_line", "right_border_line"]
    for k in polylines:
        for node in v[k]["nodes"]:
            node["x"] = float(node["x"])
            node["y"] = float(node["y"])
            # TODO(zhangjun): overlap

def fix_float_in_poi(v: dict):
    v["driving_position"]["s"] = float(v["driving_position"]["s"])
    v["walking_position"]["s"] = float(v["walking_position"]["s"])

def load_data(setid: str):
    with open(SIMPLE_X_JUNCTION, 'r') as f:
        all_map = json.load(f)
    data = []
    keys = [("lane", "lanes"),
            ("road", "roads"),
            ("junction", "junctions"),
            ("poi", "pois"),
            ("aoi", "aois")]
    for name, names in keys:
        for v in all_map[names].values():
            # type fix
            if name == "lane":
                fix_float_in_lane(v)
            elif name == "poi":
                fix_float_in_poi(v)
            data.append({
                "class": name,
                "setid": setid,
                "data": v
            })
    if "header" in all_map:
        raise NotImplementedError
    return data


def main():
    args = parse_args()
    client = pymongo.MongoClient(args.uri)
    col = client[args.db][args.col]
    data = load_data(args.setid)
    if col.find_one({"setid": args.setid}):
        raise RuntimeError(f"{args.setid} exists in {args.db}.{args.col}")
    col.insert_many(data)


if __name__ == "__main__":
    main()
