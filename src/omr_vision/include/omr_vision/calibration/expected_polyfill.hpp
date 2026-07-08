#pragma once
#include <string>
#include <variant>

// Minimal polyfill of std::expected<T, E> for GCC 11 compatibility.
// Only covers the subset used in this project.  Avoids naming conflicts
// with std::unexpected(void) present in GCC 11 <exception>.

template <typename E>
struct Unexpected {
    E error;
    explicit Unexpected(E e) : error(std::move(e)) {}
};

template <typename T, typename E = std::string>
class Expected {
    std::variant<T, E> data_;

public:
    // NOLINTNEXTLINE(google-explicit-constructor,hicpp-explicit-conversions)
    Expected(T val) : data_(std::move(val)) {}
    // NOLINTNEXTLINE(google-explicit-constructor,hicpp-explicit-conversions)
    Expected(Unexpected<E> err) : data_(std::move(err.error)) {}

    [[nodiscard]] bool has_value() const { return data_.index() == 0; }

    [[nodiscard]] T& value() & { return std::get<0>(data_); }
    [[nodiscard]] const T& value() const& { return std::get<0>(data_); }
    [[nodiscard]] T&& value() && { return std::get<0>(std::move(data_)); }

    T* operator->() { return &value(); }
    const T* operator->() const { return &value(); }
    T& operator*() & { return value(); }
    const T& operator*() const& { return value(); }

    [[nodiscard]] E& error() & { return std::get<1>(data_); }
    [[nodiscard]] const E& error() const& { return std::get<1>(data_); }

    explicit operator bool() const { return has_value(); }
};

// Alias for the common case used in this codebase
template <typename T>
using Result = Expected<T, std::string>;
