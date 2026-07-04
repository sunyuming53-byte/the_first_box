#pragma once
#include <stdexcept>
#include <string>

namespace rm {

class ArmError : public std::runtime_error {
public:
    explicit ArmError(int code, const std::string& message)
        : std::runtime_error(message), code_(code) {}

    [[nodiscard]] int code() const noexcept { return code_; }

private:
    int code_;
};

}  // namespace rm
