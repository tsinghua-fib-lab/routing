/*
 * Copyright 2021-2021 Simulet Authors
 *
 * Author: root
 * Created: 2021-08-14 12:29:51 am
 */

#include <absl/flags/parse.h>
#include <iostream>
#include "map_loader/mongo_loader.h"
#include "simulet/map/v1/map.pb.h"


int main(int argc, char** argv) {
  absl::ParseCommandLine(argc, argv);

  auto map = routing::map_loader::LoadMapFromMongo("simple-x-junction");
  std::cout << map.ShortDebugString() << std::endl;

  return 0;
}
