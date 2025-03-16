#pragma once

#include <type_traits>

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

template <typename ForwardIt, typename UnaryFunction>
[[nodiscard]] constexpr ForwardIt min_element_transform(ForwardIt     first,
                                                        ForwardIt     last,
                                                        UnaryFunction transform)
{
    using ResultType = std::invoke_result_t<UnaryFunction>;

    ForwardIt  minIterator = last;
    ResultType minValue;
    for (; first != last; ++first)
    {
        const auto value = transform(*first);
        if (value < minValue)
        {
            minIterator = first;
            minValue    = value;
        }
    }
    return minIterator;
}

} // namespace util
} // namespace pet
