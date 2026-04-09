#ifndef BYTE_SWAP_HPP
#define BYTE_SWAP_HPP

#if __cplusplus >= 202002L
// C++20 and later: use standard library
#include <bit>
#include <array>
#include <algorithm>

#if !__cpp_lib_byteswap
namespace std {
// This will be added in C++23 but we backport it here
template<typename T>
constexpr T byteswap(T n) noexcept {
    static_assert(std::has_unique_object_representations_v<T>,
        "T may not have padding bits");
    std::array<std::byte, sizeof(T)>& as_arr =
        reinterpret_cast<std::array<std::byte, sizeof(T)>&>(n);
    std::reverse(as_arr.begin(), as_arr.end());
    return n;
}
}
#endif // !__cpp_lib_byteswap

template<std::endian endianness, typename T>
T maybe_byteswap(T val) {
    if constexpr (std::endian::native == endianness) {
        return val;
    } else {
        return std::byteswap(val);
    }
}

#else
// C++17 fallback: use GCC built-in macros for endianness
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace compat {
    enum class endian {
        little = __ORDER_LITTLE_ENDIAN__,
        big    = __ORDER_BIG_ENDIAN__,
        native = __BYTE_ORDER__
    };

    template<typename T>
    T byteswap(T val) noexcept {
        static_assert(std::is_arithmetic<T>::value, "byteswap requires arithmetic type");
        T result;
        const auto* src = reinterpret_cast<const uint8_t*>(&val);
        auto* dst = reinterpret_cast<uint8_t*>(&result);
        for (std::size_t i = 0; i < sizeof(T); ++i) {
            dst[i] = src[sizeof(T) - 1 - i];
        }
        return result;
    }
}

template<compat::endian endianness, typename T>
T maybe_byteswap(T val) {
    if (compat::endian::native == endianness) {
        return val;
    } else {
        return compat::byteswap(val);
    }
}

// Convenience aliases so call sites can use endian::little / endian::big
namespace endian_compat {
    constexpr auto little = compat::endian::little;
    constexpr auto big    = compat::endian::big;
    constexpr auto native = compat::endian::native;
}

#endif // __cplusplus >= 202002L

template<typename T>
T read_le(const unsigned char* buf) {
#if __cplusplus >= 202002L
    return maybe_byteswap<std::endian::little>(*(T*)buf);
#else
    return maybe_byteswap<compat::endian::little>(*(T*)buf);
#endif
}

template<typename T>
void write_le(const T& val, const unsigned char* buf) {
#if __cplusplus >= 202002L
    *(T*)buf = maybe_byteswap<std::endian::little>(val);
#else
    *(T*)buf = maybe_byteswap<compat::endian::little>(val);
#endif
}

#endif // BYTE_SWAP_HPP
