#pragma once

#include "DeviceMap.hpp"
#include "canmore_cpp/Discovery.hpp"

// Discovery
std::shared_ptr<Canmore::Device>
getTargetDevice(const DeviceMap &devMap, const std::vector<std::shared_ptr<Canmore::Discovery>> &discoverySources);

std::shared_ptr<Canmore::Device>
waitForTargetDevice(const std::vector<std::shared_ptr<Canmore::Discovery>> &discoverySources, uint64_t serialNum);

// CLI Handler
void runCli(std::shared_ptr<Canmore::Device> dev);
void runCliCommand(std::shared_ptr<Canmore::Device> dev, const std::string &cmd, const std::vector<std::string> &args);
