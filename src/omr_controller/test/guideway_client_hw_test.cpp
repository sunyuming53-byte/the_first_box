// SPDX-License-Identifier: Apache-2.0
// guideway_client_hw_test — manual hardware smoke for omr_controller::GuidewayClientImpl.
//
// This is NOT a gtest and is NOT run by colcon test. It exercises the real
// GuidewayClient API against photogate + D-AIS hardware (same path future BT
// / Orchestrator will call).
//
// Do NOT run alongside DaisHardware / PhotogateHardware (serial port conflict).
//
// Examples (inside develop container, after sourcing install/setup.bash):
//   guideway_client_hw_test --mode status
//   guideway_client_hw_test --mode jog --dir up --rpm 20 --duration 8
//   guideway_client_hw_test --mode calibrate
//   guideway_client_hw_test --mode move --rail-m 0.05 --timeout 60
//   guideway_client_hw_test --mode home
//   guideway_client_hw_test --mode velocity --mps 0.01 --duration 5

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "omr_controller/clients/guideway_client.hpp"
#include "omr_hardware/rail_controller.hpp"
#include <rclcpp/rclcpp.hpp>

namespace {

using omr_controller::GuidewayCalibration;
using omr_controller::GuidewayClientImpl;
using omr_controller::GuidewayConfig;
using omr_controller::GuidewayDirection;
using omr_controller::GuidewaySafetyState;
using omr_controller::GuidewayStatus;

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

enum class Mode { Status, Jog, Calibrate, Move, Home, Velocity };

struct CliOptions {
    Mode mode = Mode::Status;
    GuidewayConfig cfg;
    std::string config_path;
    GuidewayDirection jog_dir = GuidewayDirection::UP;
    double jog_rpm = 10.0;
    double jog_duration_s = 8.0;
    double rail_m = 0.0;
    bool rail_m_set = false;
    double timeout_s = 60.0;
    double mps = 0.01;
};

const char* safety_name(GuidewaySafetyState s) {
    switch (s) {
        case GuidewaySafetyState::NORMAL:
            return "NORMAL";
        case GuidewaySafetyState::UP_LIMITED:
            return "UP_LIMITED";
        case GuidewaySafetyState::DOWN_LIMITED:
            return "DOWN_LIMITED";
        case GuidewaySafetyState::SAFETY_FAULT:
            return "SAFETY_FAULT";
    }
    return "?";
}

void print_status(const GuidewayStatus& st) {
    printf(
        "[%s] connected=%d L=%d H=%d U=%d valid=%d calibrated=%d homed=%d "
        "motion_done=%d rail_m=%.4f rail_mps=%.4f motor_pos=%.3f motor_rpm=%.0f\n",
        safety_name(st.safety), static_cast<int>(st.connected),
        static_cast<int>(st.gates.lower), static_cast<int>(st.gates.home),
        static_cast<int>(st.gates.upper), static_cast<int>(st.gates.valid),
        static_cast<int>(st.calibrated), static_cast<int>(st.homed),
        static_cast<int>(st.motion_done), st.rail_position_m, st.rail_velocity_mps,
        st.motor_position_rad, st.motor_velocity_rpm);
}

void print_calibration(const GuidewayCalibration& c) {
    printf("calibrated=%d homed=%d\n", static_cast<int>(c.calibrated),
           static_cast<int>(c.homed));
    printf("theta_lower=%.6f theta_home=%.6f theta_upper=%.6f\n", c.theta_lower, c.theta_home,
           c.theta_upper);
    printf("lower_travel=%.6f rad (%.4f m)\n", c.lower_travel_rad, c.lower_travel_m);
    printf("upper_travel=%.6f rad (%.4f m)\n", c.upper_travel_rad, c.upper_travel_m);
    printf("total_travel=%.6f rad (%.4f m)\n", c.total_travel_rad, c.total_travel_m);
}

int run_status(GuidewayClientImpl& client, double duration_s) {
    if (!client.connect()) {
        fprintf(stderr, "connect failed\n");
        return 1;
    }
    printf("status monitor (Ctrl+C to stop), duration=%.1fs\n", duration_s);
    const auto start = std::chrono::steady_clock::now();
    while (!g_stop) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (duration_s > 0.0 && elapsed >= duration_s) {
            break;
        }
        print_status(client.status());
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return 0;
}

int run_jog(GuidewayClientImpl& client, GuidewayDirection dir, double rpm, double duration_s) {
    if (!client.connect()) {
        fprintf(stderr, "connect failed\n");
        return 1;
    }
    printf("jog via GuidewayClient::jog dir=%s rpm=%.1f duration=%.1fs\n",
           dir == GuidewayDirection::UP ? "UP" : "DOWN", rpm, duration_s);
    if (!client.jog(dir, rpm)) {
        printf("initial jog rejected (maybe already limited). continue monitoring…\n");
    }

    const auto start = std::chrono::steady_clock::now();
    while (!g_stop) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_s) {
            break;
        }
        const auto st = client.status();
        if (st.safety == GuidewaySafetyState::SAFETY_FAULT) {
            client.stop();
            print_status(st);
            return 2;
        }
        if (st.safety == GuidewaySafetyState::NORMAL ||
            (st.safety == GuidewaySafetyState::UP_LIMITED && dir == GuidewayDirection::DOWN) ||
            (st.safety == GuidewaySafetyState::DOWN_LIMITED && dir == GuidewayDirection::UP)) {
            client.jog(dir, rpm);
        }
        printf("\r");
        print_status(st);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    client.stop();
    printf("\njog done\n");
    print_status(client.status());
    return client.safetyState() == GuidewaySafetyState::SAFETY_FAULT ? 2 : 0;
}

int run_velocity(GuidewayClientImpl& client, double mps, double duration_s) {
    if (!client.connect()) {
        fprintf(stderr, "connect failed\n");
        return 1;
    }
    printf("velocity via GuidewayClient::setVelocity mps=%.4f duration=%.1fs\n", mps, duration_s);
    if (!client.setVelocity(mps)) {
        printf("initial setVelocity rejected. continue monitoring…\n");
    }

    const auto start = std::chrono::steady_clock::now();
    while (!g_stop) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_s) {
            break;
        }
        const auto st = client.status();
        if (st.safety == GuidewaySafetyState::SAFETY_FAULT) {
            client.stop();
            print_status(st);
            return 2;
        }
        const bool toward_up = mps > 0.0;
        if (st.safety == GuidewaySafetyState::NORMAL ||
            (st.safety == GuidewaySafetyState::UP_LIMITED && !toward_up) ||
            (st.safety == GuidewaySafetyState::DOWN_LIMITED && toward_up)) {
            client.setVelocity(mps);
        }
        printf("\r");
        print_status(st);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    client.stop();
    printf("\nvelocity done\n");
    print_status(client.status());
    return client.safetyState() == GuidewaySafetyState::SAFETY_FAULT ? 2 : 0;
}

int run_move(GuidewayClientImpl& client, double rail_m, double timeout_s);

int run_calibrate(GuidewayClientImpl& client, bool do_move_after, double rail_m,
                  double timeout_s) {
    if (!client.connect()) {
        fprintf(stderr, "connect failed\n");
        return 1;
    }
    printf("calibrate via GuidewayClient::calibrate() — may take minutes\n");
    const bool ok = client.calibrate();
    print_calibration(client.calibration());
    print_status(client.status());
    if (!ok) {
        fprintf(stderr, "calibrate failed\n");
        return 3;
    }
    printf("calibrate OK\n");
    // Optional: keep going in the same process so moveToRail can use live calib.
    if (do_move_after) {
        printf("\n--- continue: moveToRail %.4f m ---\n", rail_m);
        return run_move(client, rail_m, timeout_s);
    }
    return 0;
}

int run_move(GuidewayClientImpl& client, double rail_m, double timeout_s) {
    if (!client.connect()) {
        fprintf(stderr, "connect failed\n");
        return 1;
    }
    if (!client.isCalibrated()) {
        fprintf(stderr,
                "not calibrated — run: --mode calibrate --rail-m <m>  "
                "(same process; calibration is not persisted)\n");
        return 4;
    }
    printf("moveToRail via GuidewayClient target=%.4f m timeout=%.1fs\n", rail_m, timeout_s);
    const bool ok = client.moveToRail(rail_m, timeout_s);
    print_status(client.status());
    if (!ok) {
        fprintf(stderr, "moveToRail failed\n");
        return 5;
    }
    printf("moveToRail OK\n");
    return 0;
}

int run_home(GuidewayClientImpl& client, double timeout_s) {
    if (!client.connect()) {
        fprintf(stderr, "connect failed\n");
        return 1;
    }
    if (!client.isCalibrated()) {
        fprintf(stderr, "not calibrated — run --mode calibrate first\n");
        return 4;
    }
    printf("moveToHome via GuidewayClient timeout=%.1fs\n", timeout_s);
    const bool ok = client.moveToHome(timeout_s);
    print_status(client.status());
    if (!ok) {
        fprintf(stderr, "moveToHome failed\n");
        return 5;
    }
    printf("moveToHome OK\n");
    return 0;
}

void print_usage(const char* argv0) {
    fprintf(stderr,
            "用法: %s [选项]\n"
            "  --mode status|jog|velocity|calibrate|move|home\n"
            "  --config <yaml>          (默认可用 omr_hardware rail_photogate.yaml)\n"
            "  --photogate <dev>  --motor <dev>\n"
            "  --dir up|down  --rpm <n>  --duration <s>\n"
            "  --mps <m/s>              (velocity 模式)\n"
            "  --rail-m <m>  --timeout <s>  (move/home；calibrate 时带 --rail-m 会标定后继续定点)\n"
            "  --up-sign +1|-1\n"
            "  --help\n"
            "\n"
            "说明: 本程序调用 omr_controller::GuidewayClientImpl（不是硬件联调直接库）。\n"
            "标定结果不跨进程保存；要测定点请用: --mode calibrate --rail-m 0.05\n",
            argv0);
}

bool parse_cli(int argc, char** argv, CliOptions& opt) {
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        auto need = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                fprintf(stderr, "%s 需要参数\n", name);
                return nullptr;
            }
            return argv[++i];
        };
        if (std::strcmp(a, "--help") == 0) {
            print_usage(argv[0]);
            std::exit(0);
        } else if (std::strcmp(a, "--mode") == 0) {
            const char* v = need("--mode");
            if (!v) {
                return false;
            }
            if (std::strcmp(v, "status") == 0) {
                opt.mode = Mode::Status;
            } else if (std::strcmp(v, "jog") == 0) {
                opt.mode = Mode::Jog;
            } else if (std::strcmp(v, "velocity") == 0) {
                opt.mode = Mode::Velocity;
            } else if (std::strcmp(v, "calibrate") == 0) {
                opt.mode = Mode::Calibrate;
            } else if (std::strcmp(v, "move") == 0) {
                opt.mode = Mode::Move;
            } else if (std::strcmp(v, "home") == 0) {
                opt.mode = Mode::Home;
            } else {
                fprintf(stderr, "未知 mode: %s\n", v);
                return false;
            }
        } else if (std::strcmp(a, "--config") == 0) {
            const char* v = need("--config");
            if (!v) {
                return false;
            }
            opt.config_path = v;
        } else if (std::strcmp(a, "--photogate") == 0) {
            const char* v = need("--photogate");
            if (!v) {
                return false;
            }
            opt.cfg.photogate_port = v;
        } else if (std::strcmp(a, "--motor") == 0) {
            const char* v = need("--motor");
            if (!v) {
                return false;
            }
            opt.cfg.motor_port = v;
        } else if (std::strcmp(a, "--dir") == 0) {
            const char* v = need("--dir");
            if (!v) {
                return false;
            }
            if (std::strcmp(v, "up") == 0) {
                opt.jog_dir = GuidewayDirection::UP;
            } else if (std::strcmp(v, "down") == 0) {
                opt.jog_dir = GuidewayDirection::DOWN;
            } else {
                fprintf(stderr, "未知 dir: %s\n", v);
                return false;
            }
        } else if (std::strcmp(a, "--rpm") == 0) {
            const char* v = need("--rpm");
            if (!v) {
                return false;
            }
            opt.jog_rpm = std::stod(v);
        } else if (std::strcmp(a, "--duration") == 0) {
            const char* v = need("--duration");
            if (!v) {
                return false;
            }
            opt.jog_duration_s = std::stod(v);
        } else if (std::strcmp(a, "--mps") == 0) {
            const char* v = need("--mps");
            if (!v) {
                return false;
            }
            opt.mps = std::stod(v);
        } else if (std::strcmp(a, "--rail-m") == 0) {
            const char* v = need("--rail-m");
            if (!v) {
                return false;
            }
            opt.rail_m = std::stod(v);
            opt.rail_m_set = true;
        } else if (std::strcmp(a, "--timeout") == 0) {
            const char* v = need("--timeout");
            if (!v) {
                return false;
            }
            opt.timeout_s = std::stod(v);
        } else if (std::strcmp(a, "--up-sign") == 0) {
            const char* v = need("--up-sign");
            if (!v) {
                return false;
            }
            opt.cfg.up_sign = std::stoi(v);
        } else {
            fprintf(stderr, "未知参数: %s\n", a);
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // rclcpp needed for GuidewayClientImpl logger; no spinning required.
    rclcpp::init(argc, argv);

    CliOptions opt;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            std::string error;
            if (!omr_hardware::loadRailConfig(argv[i + 1], opt.cfg, &error)) {
                fprintf(stderr, "%s\n", error.c_str());
                rclcpp::shutdown();
                return 1;
            }
            opt.config_path = argv[i + 1];
            break;
        }
    }
    if (!parse_cli(argc, argv, opt)) {
        print_usage(argv[0]);
        rclcpp::shutdown();
        return 1;
    }

    printf("GuidewayClientImpl smoke  photogate=%s motor=%s L%d/H%d/U%d up_sign=%d\n",
           opt.cfg.photogate_port.c_str(), opt.cfg.motor_port.c_str(), opt.cfg.lower_gate,
           opt.cfg.home_gate, opt.cfg.upper_gate, opt.cfg.up_sign);

    GuidewayClientImpl client(opt.cfg, rclcpp::get_logger("guideway_client_hw_test"));

    std::atomic<bool> watcher_done{false};
    std::thread watcher([&client, &watcher_done] {
        while (!watcher_done.load()) {
            if (g_stop) {
                client.stop();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    int rc = 0;
    switch (opt.mode) {
        case Mode::Status:
            rc = run_status(client, opt.jog_duration_s);
            break;
        case Mode::Jog:
            rc = run_jog(client, opt.jog_dir, opt.jog_rpm, opt.jog_duration_s);
            break;
        case Mode::Velocity:
            rc = run_velocity(client, opt.mps, opt.jog_duration_s);
            break;
        case Mode::Calibrate:
            rc = run_calibrate(client, opt.rail_m_set, opt.rail_m, opt.timeout_s);
            break;
        case Mode::Move:
            rc = run_move(client, opt.rail_m, opt.timeout_s);
            break;
        case Mode::Home:
            rc = run_home(client, opt.timeout_s);
            break;
    }

    watcher_done.store(true);
    watcher.join();
    client.disconnect();
    rclcpp::shutdown();
    return rc;
}
