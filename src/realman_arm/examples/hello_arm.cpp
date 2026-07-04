#include "realman/core/arm.hpp"
#include <iostream>
#include <vector>

int main() {
    rm::ArmConfig cfg;
    cfg.ip = "192.168.1.18";
    cfg.model = rm::ArmModel::RM_65;
    cfg.dof = 6;

    try {
        rm::Arm arm(cfg);
        std::cout << "Connected to arm" << std::endl;

        rm::JointPosition target(std::vector<double>{0.5, 0.0, 0.0, 0.0, 0.0, 0.0});
        arm.moveJ(target, 30);
        std::cout << "MoveJ complete" << std::endl;

        auto state = arm.state();
        std::cout << "Joint 1 position: " << state.joint_position.radians[0] << " rad" << std::endl;

    } catch (const rm::ArmError& e) {
        std::cerr << "Error: " << e.what() << " (code " << e.code() << ")" << std::endl;
        return 1;
    }

    return 0;
}
