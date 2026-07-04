#include "realman/core/arm.hpp"

#include <cmath>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

int main() {
    rm::ArmConfig cfg;
    cfg.ip = "192.168.1.18";
    cfg.model = rm::ArmModel::RM_65;
    cfg.dof = 6;

    try {
        rm::Arm arm(cfg);
        std::cout << "Connected to arm. Reading joint positions at 1 Hz...\n";
        std::cout << "Press Ctrl+C to stop.\n\n";

        std::cout << std::fixed << std::setprecision(4);
        for (int i = 0; i < 20; ++i) {
            auto st = arm.state();

            std::cout << "[" << i + 1 << "] Joints (rad): ";
            for (size_t j = 0; j < st.joint_position.radians.size(); ++j) {
                std::cout << "J" << (j + 1) << "=" << st.joint_position.radians[j] << "  ";
            }
            std::cout << "\n";

            std::cout << "               (deg): ";
            for (size_t j = 0; j < st.joint_position.radians.size(); ++j) {
                std::cout << "J" << (j + 1) << "=" << st.joint_position.radians[j] * 180.0 / M_PI
                          << "  ";
            }
            std::cout << "\n\n";

            if (i < 19) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }

        std::cout << "Done.\n";

    } catch (const rm::ArmError& e) {
        std::cerr << "FAIL: " << e.what() << " (code " << e.code() << ")" << std::endl;
        return 1;
    }

    return 0;
}
