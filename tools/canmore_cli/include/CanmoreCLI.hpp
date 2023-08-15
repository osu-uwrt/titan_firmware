#pragma once

#include "DeviceMap.hpp"
#include "canmore_cpp/Discovery.hpp"

// Discovery
std::shared_ptr<Canmore::Device> getTargetDevice(const DeviceMap &devMap,
                                                 std::vector<std::shared_ptr<Canmore::Discovery>> discoverySources);

// CLI Handler
void runCli(std::shared_ptr<Canmore::Device> dev);
