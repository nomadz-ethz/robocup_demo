#pragma once

#include "chrono"
#include <string>
#include <random>
#include <sstream>

// 计算从 start_time 开始到现在, 经过了多少毫秒
inline int msecsSince(std::chrono::high_resolution_clock::time_point start_time)
{
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}