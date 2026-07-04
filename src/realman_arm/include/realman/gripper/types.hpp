#pragma once

namespace rm {

struct GripperState {
    int enable_state{0};  // 0=disabled, 1=enabled
    int status{0};        // 0=offline, 1=online
    int error{0};         // error bitfield (bit0: stall, bit1: over-temp, bit2: over-current, bit3:
                          // driver, bit4: internal)
    int mode{0};  // 1=open-idle, 2=closed-idle, 3=stopped-idle, 4=closing, 5=opening, 6=force-stop
    int current_force{0};  // current gripping force, grams
    int temperature{0};    // degrees Celsius
    int actpos{0};         // current opening position (0–1000, dimensionless)
};

}  // namespace rm
