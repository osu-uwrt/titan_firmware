#pragma once

#include "canmore_cpp/DebugClient.hpp"

void runGdbServer(uint16_t port, std::shared_ptr<Canmore::DebugClient> client);
