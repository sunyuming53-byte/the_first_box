#pragma once
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace std {

// Minimal polyfill for std::format (GCC 11 compat).
// Handles {} and {:.<N>f} specifiers used in this codebase.

namespace detail {
inline void format_impl(std::ostringstream& ss, std::string_view fmt, size_t pos) {
    ss << fmt.substr(pos);
}
template <typename T, typename... Args>
void format_impl(std::ostringstream& ss, std::string_view fmt, size_t pos, T&& arg, Args&&... rest) {
    auto brace = fmt.find('{', pos);
    if (brace == std::string_view::npos) {
        ss << fmt.substr(pos);
        return;
    }
    ss << fmt.substr(pos, brace - pos);
    auto end = fmt.find('}', brace);
    auto spec = fmt.substr(brace + 1, end - brace - 1);
    if (spec.empty()) {
        ss << std::forward<T>(arg);
    } else if (spec.size() >= 1 && spec[0] == ':' && spec.back() == 'f') {
        if (spec.size() > 2 && spec[1] == '.') {
            try {
                int prec = std::stoi(std::string(spec.substr(2, spec.size() - 3)));
                ss << std::fixed << std::setprecision(prec) << std::forward<T>(arg);
            } catch (...) { ss << std::forward<T>(arg); }
        } else {
            ss << std::fixed << std::forward<T>(arg);
        }
    } else {
        ss << std::forward<T>(arg);
    }
    format_impl(ss, fmt, end + 1, std::forward<Args>(rest)...);
}
} // namespace detail

template <typename... Args>
inline std::string format(std::string_view fmt, Args&&... args) {
    std::ostringstream ss;
    detail::format_impl(ss, fmt, 0, std::forward<Args>(args)...);
    return ss.str();
}

} // namespace std
