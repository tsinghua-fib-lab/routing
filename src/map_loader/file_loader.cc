/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-17 1:57:57 pm
 */

#include "map_loader/file_loader.h"
#include <fmt/format.h>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <string>
#include "simulet/map/v1/map.pb.h"

namespace routing {

namespace map_loader {

simulet::proto::map::v1::Map LoadMapFromFile(const std::string& path) {
  std::ifstream fin(path, std::ios_base::binary | std::ios_base::in);
  if (!fin) {
    throw std::runtime_error(fmt::format("file_loader: Fail to open {}", path));
  }
  simulet::proto::map::v1::Map map;
  if (!map.ParseFromIstream(&fin)) {
    throw std::runtime_error(
        fmt::format("file_loader: Fail to parse map from {}", path));
  }
  return map;
}

}  // namespace map_loader

}  // namespace routing
