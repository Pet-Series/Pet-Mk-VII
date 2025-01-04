#pragma once

#include <chrono>
#include <string>
#include <unordered_map>

namespace pet
{
namespace util
{

struct TimingData
{
    int                      callCount = 0;
    std::chrono::nanoseconds totalCallTime{0};
};

class TikTok
{
  public:
    explicit TikTok(std::string_view name);

    ~TikTok();

    static void printData();

  private:
    static std::chrono::nanoseconds getCurrentTime();

  private:
    std::string              m_name;
    std::chrono::nanoseconds m_startTime;

    static std::unordered_map<std::string, TimingData> s_aggregatedData;
};

inline TikTok::TikTok(std::string_view name) : m_name(name), m_startTime(getCurrentTime()) {};

inline TikTok::~TikTok()
{
    const std::chrono::nanoseconds endTime  = getCurrentTime();
    const std::chrono::nanoseconds callTime = endTime - m_startTime;

    TimingData &data = s_aggregatedData[m_name];
    data.callCount += 1;
    data.totalCallTime += callTime;
}

inline std::chrono::nanoseconds TikTok::getCurrentTime()
{
    return std::chrono::steady_clock::now().time_since_epoch();
}

} // namespace util
} // namespace pet
