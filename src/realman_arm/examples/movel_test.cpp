#include "realman/core/arm.hpp"
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

int main() {
    rm::ArmConfig cfg;
    cfg.ip = "192.168.1.18";
    cfg.model = rm::ArmModel::RM_65;
    cfg.dof = 6;

    try {
        rm::Arm arm(cfg);
        std::cout << std::fixed << std::setprecision(4);

        // Read current tool pose
        auto start = arm.toolPose();
        std::cout << "Current TCP: x=" << start.x << " y=" << start.y << " z=" << start.z
                  << " roll=" << start.roll << " pitch=" << start.pitch << " yaw=" << start.yaw << "\n";

        // Target: Z+50mm, same orientation
        rm::CartesianPose target = start;
        target.z += 0.050;  // +50 mm

        std::cout << "Target TCP:  x=" << target.x << " y=" << target.y << " z=" << target.z
                  << " roll=" << target.roll << " pitch=" << target.pitch << " yaw=" << target.yaw << "\n";
        std::cout << "Executing moveL (speed=20, blocking)...\n";

        arm.moveL(target, 20, true);

        // Read final pose
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto end = arm.toolPose();
        std::cout << "Final TCP:   x=" << end.x << " y=" << end.y << " z=" << end.z
                  << " roll=" << end.roll << " pitch=" << end.pitch << " yaw=" << end.yaw << "\n";

        double dz = end.z - start.z;
        std::cout << "Delta Z = " << dz << " m  (expected ~0.050)\n";

        if (std::abs(dz - 0.050) < 0.005) {
            std::cout << "moveL test PASSED\n";
        } else {
            std::cout << "moveL test: Z deviation " << dz * 1000.0 << " mm, check visually\n";
        }

    } catch (const rm::ArmError& e) {
        std::cerr << "FAIL: " << e.what() << " (code " << e.code() << ")" << std::endl;
        return 1;
    }

    return 0;
}
