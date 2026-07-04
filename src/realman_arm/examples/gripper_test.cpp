#include "realman/core/arm.hpp"

#include <unistd.h>

#include <iomanip>
#include <iostream>

static void printGripperState(const rm::GripperState& gs) {
    std::cout << "  enable: " << gs.enable_state << "  online: " << gs.status
              << "  mode: " << gs.mode << "  pos: " << gs.actpos << "  force: " << gs.current_force
              << "g"
              << "  temp: " << gs.temperature << "C"
              << "  err: 0x" << std::hex << gs.error << std::dec << std::endl;
}

int main() {
    rm::ArmConfig cfg;
    cfg.ip = "192.168.1.18";
    cfg.model = rm::ArmModel::RM_65;
    cfg.dof = 6;

    try {
        rm::Arm arm(cfg);
        std::cout << "[1] Connected to arm\n" << std::endl;

        // Pre-check: read gripper state
        std::cout << "[2] Gripper pre-check:" << std::endl;
        rm::GripperState gs = arm.gripperState();
        printGripperState(gs);
        if (gs.status != 1) {
            std::cerr << "WARNING: gripper offline (status=" << gs.status
                      << "). Check RS-485 connection & power." << std::endl;
        }
        std::cout << std::endl;

        // Set route
        std::cout << "[3] Set gripper route [0, 1000]" << std::endl;
        arm.setGripperRoute(0, 1000);
        std::cout << "  ok" << std::endl << std::endl;

        // Release (open fully)
        std::cout << "[4] gripperRelease(speed=800)" << std::endl;
        arm.gripperRelease(800);
        usleep(500000);
        gs = arm.gripperState();
        printGripperState(gs);
        std::cout << std::endl;

        // Position control: move to 500 (mid)
        std::cout << "[5] gripper(position=500)" << std::endl;
        arm.gripper(500);
        usleep(500000);
        gs = arm.gripperState();
        printGripperState(gs);
        std::cout << std::endl;

        // Force-controlled pick (close with force threshold)
        std::cout << "[6] gripperPick(speed=600, force=200)" << std::endl;
        arm.gripperPick(600, 200);
        usleep(500000);
        gs = arm.gripperState();
        printGripperState(gs);
        std::cout << std::endl;

        // Position control: close fully
        std::cout << "[7] gripper(position=1000)" << std::endl;
        arm.gripper(1000);
        usleep(500000);
        gs = arm.gripperState();
        printGripperState(gs);
        std::cout << std::endl;

        // Release again
        std::cout << "[8] gripperRelease(speed=800) — final open" << std::endl;
        arm.gripperRelease(800);
        usleep(500000);
        gs = arm.gripperState();
        printGripperState(gs);
        std::cout << std::endl;

        std::cout << "All gripper tests passed." << std::endl;

    } catch (const rm::ArmError& e) {
        std::cerr << "FAIL: " << e.what() << " (code " << e.code() << ")" << std::endl;
        return 1;
    }

    return 0;
}
