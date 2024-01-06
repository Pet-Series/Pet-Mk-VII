#pragma once

namespace pet
{
namespace util
{

template <typename ForwardIt, typename BinaryFunction>
constexpr void adjacent_for_each(ForwardIt first, ForwardIt last, BinaryFunction function)
{
    if (first == last)
    {
        return;
    }

    ForwardIt next = first;
    ++next;

    for (; next != last; ++next, ++first)
    {
        function(*first, *next);
    }
}

} // namespace util
} // namespace pet
