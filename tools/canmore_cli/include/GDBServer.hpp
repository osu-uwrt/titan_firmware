#pragma once

#include "canmore_cpp/GDBClient.hpp"

#include <memory>

void runGdbServer(uint16_t port, std::shared_ptr<Canmore::GDBClient> client);
