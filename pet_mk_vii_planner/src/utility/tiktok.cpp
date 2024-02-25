#include "utility/tiktok.hpp"

#include <iostream>

namespace pet
{
namespace util
{

std::unordered_map<std::string, TimingData> TikTok::s_aggregatedData{};

void TikTok::printData()
{
    for (const auto &[name, data] : s_aggregatedData)
    {
        const auto totalTime =
            std::chrono::duration_cast<std::chrono::microseconds>(data.totalCallTime);
        const auto meanTime = std::chrono::duration_cast<std::chrono::microseconds>(
            data.totalCallTime / data.callCount);

        std::cout << "Name: \"" << name << '\"' << std::endl;
        std::cout << "  Count          : " << data.callCount << std::endl;
        std::cout << "  Mean Time [us] : " << meanTime.count() << std::endl;
        std::cout << "  Total Time [us]: " << totalTime.count() << std::endl;
    }
}

} // namespace util
} // namespace pet
