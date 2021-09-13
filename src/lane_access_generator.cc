/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-23 9:55:43 pm
 */

#include <absl/flags/flag.h>
#include <absl/flags/parse.h>
#include <cstdint>
#include <iostream>
#include "simulet/map_runtime/v1/map_runtime.pb.h"

using PbLaneAccessSetting = simulet::proto::map_runtime::v1::LaneAccessSetting;
using PbBatchAccessSetting =
    simulet::proto::map_runtime::v1::BatchAccessSetting;
using PbLaneAccessType = simulet::proto::map_runtime::v1::LaneAccessType;

ABSL_FLAG(uint32_t, lane_id, 0, "access setting lane id");
ABSL_FLAG(int, type, 0, "0-invalid 1-normal 2-red 3-noentry");

int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  PbBatchAccessSetting settings;
  auto& setting = *settings.add_lanes();
  setting.set_lane_id(absl::GetFlag(FLAGS_lane_id));
  setting.set_type(PbLaneAccessType(absl::GetFlag(FLAGS_type)));
  setting.SerializeToOstream(&std::cout);

  return 0;
}
