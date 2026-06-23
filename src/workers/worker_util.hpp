#pragma once

#include <atomic>
#include <cstdint>

// Utilities defined in main.cpp with external linkage — accessible to all worker
// translation units via shared executable linkage. Declared here so the extern
// contract is written once rather than repeated in every worker .cpp.

extern std::atomic<bool> g_running;

void pin_to_core(int core);
uint64_t now_us();
