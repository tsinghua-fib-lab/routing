/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-17 1:58:07 pm
 */

#ifndef SRC_MAP_LOADER_FILE_LOADER_H_
#define SRC_MAP_LOADER_FILE_LOADER_H_

#include <string>
#include "simulet/map/v1/map.pb.h"

namespace routing {

namespace map_loader {

simulet::proto::map::v1::Map LoadMapFromFile(const std::string& path);

}  // namespace map_loader

}  // namespace routing

#endif  // SRC_MAP_LOADER_FILE_LOADER_H_