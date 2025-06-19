#pragma once

#include "chrono"
#include <string>
#include <random>
#include <sstream>

// Calculate how many milliseconds have passed since start_time
inline int msecsSince(std::chrono::high_resolution_clock::time_point start_time)
{
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
}

inline std::string gen_uuid() {
    std::random_device rd;
    std::mt19937 rng(rd());
    std::uniform_int_distribution<int> uni(0, 15);
    std::uniform_int_distribution<int> uni8(8, 11);

    std::stringstream ss;
    ss << std::hex;
    for (int i = 0; i < 8; i++) ss << uni(rng);
    ss << "-";
    for (int i = 0; i < 4; i++) ss << uni(rng);
    ss << "-4"; // version 4 UUID
    for (int i = 0; i < 3; i++) ss << uni(rng);
    ss << "-";
    ss << uni8(rng); // 17th character is 8, 9, A, or B
    for (int i = 0; i < 3; i++) ss << uni(rng);
    ss << "-";
    for (int i = 0; i < 12; i++) ss << uni(rng);

    return ss.str();
}