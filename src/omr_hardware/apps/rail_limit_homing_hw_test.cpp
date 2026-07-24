// SPDX-License-Identifier: Apache-2.0
// rail_limit_homing_hw_test — thin CLI over omr_hardware::RailController.
//
// All interlock / calibration / motion logic lives in the rail_control
// library (src/rail_controller.cpp); this tool only parses CLI/YAML, drives
// the three modes and prints status.
//
// Examples:
//   rail_limit_homing_hw_test --mode sensors
//   rail_limit_homing_hw_test --mode interlock --dir up --rpm 10 --duration 20
//   rail_limit_homing_hw_test --mode calibrate --config .../rail_photogate.yaml

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "omr_hardware/rail_controller.hpp"
#include "photogate/photogate.hpp"

namespace {

using omr_hardware::RailController;
using omr_hardware::RailControllerConfig;
using omr_hardware::RailDirection;
using omr_hardware::RailLogLevel;
using omr_hardware::RailSafetyState;

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

enum class Mode { Sensors, Interlock, Calibrate };

struct CliOptions {
    Mode mode = Mode::Sensors;
    RailControllerConfig cfg;
    std::string config_path;
    RailDirection jog_dir = RailDirection::UP;
    double jog_rpm = 10.0;
    double jog_duration_s = 15.0;
};

const char* safety_name(RailSafetyState s) {
    switch (s) {
        case RailSafetyState::NORMAL:
            return "NORMAL";
        case RailSafetyState::UP_LIMITED:
            return "UP_LIMITED";
        case RailSafetyState::DOWN_LIMITED:
            return "DOWN_LIMITED";
        case RailSafetyState::SAFETY_FAULT:
            return "SAFETY_FAULT";
    }
    return "?";
}

void log_to_console(RailLogLevel lvl, const std::string& msg) {
    std::FILE* out = (lvl == RailLogLevel::INFO) ? stdout : stderr;
    fprintf(out, "%s\n", msg.c_str());
    fflush(out);
}

int run_sensors(const RailControllerConfig& cfg) {
    photogate::PhotogateConfig pcfg;
    pcfg.serial_port = cfg.photogate_port;
    pcfg.baud_rate = cfg.photogate_baud;
    pcfg.gate_count = cfg.gate_count;
    photogate::Photogate pg(pcfg);
    if (!pg.connect()) {
        fprintf(stderr, "连接光电门失败: %s\n", cfg.photogate_port.c_str());
        return 1;
    }

    printf("传感器监控（Ctrl+C 退出） map L=gate%d H=gate%d U=gate%d\n", cfg.lower_gate,
           cfg.home_gate, cfg.upper_gate);
    fflush(stdout);
    // USB-CDC may reset on open; allow settle + first frames.
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    bool got = false;
    while (std::chrono::steady_clock::now() < deadline && !g_stop) {
        pg.read_frames();
        const auto lo = pg.gate_state(cfg.lower_gate);
        const auto hm = pg.gate_state(cfg.home_gate);
        const auto up = pg.gate_state(cfg.upper_gate);
        if (lo.host_rx_us != 0 && hm.host_rx_us != 0 && up.host_rx_us != 0) {
            got = true;
            printf("已锁定三路帧: lower=%d home=%d upper=%d\n", static_cast<int>(lo.blocked),
                   static_cast<int>(hm.blocked), static_cast<int>(up.blocked));
            break;
        }
        printf("\r等待有效帧… host_rx L/H/U=%llu/%llu/%llu   ",
               static_cast<unsigned long long>(lo.host_rx_us),
               static_cast<unsigned long long>(hm.host_rx_us),
               static_cast<unsigned long long>(up.host_rx_us));
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    printf("\n");
    if (!got) {
        fprintf(stderr, "8s 内未收到三路有效帧（检查 ESP32 是否在发 0xA5 帧）\n");
        pg.disconnect();
        return 1;
    }
    while (!g_stop) {
        pg.read_frames();
        printf("lower=%d home=%d upper=%d\n",
               static_cast<int>(pg.gate_state(cfg.lower_gate).blocked),
               static_cast<int>(pg.gate_state(cfg.home_gate).blocked),
               static_cast<int>(pg.gate_state(cfg.upper_gate).blocked));
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    pg.disconnect();
    return 0;
}

int run_interlock(RailController& rail, RailDirection dir, double rpm, double duration_s) {
    printf("联锁 jog：dir=%s rpm=%.1f duration=%.1fs\n",
           dir == RailDirection::UP ? "UP" : "DOWN", rpm, duration_s);
    if (!rail.jog(dir, rpm)) {
        printf("初始命令被拒绝（可能已在限位）。继续监控…\n");
    }

    const auto start = std::chrono::steady_clock::now();
    while (!g_stop) {
        const double elapsed =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_s) {
            break;
        }
        const auto st = rail.status();
        if (st.safety == RailSafetyState::SAFETY_FAULT) {
            rail.stop();
            return 2;
        }
        if (st.safety == RailSafetyState::NORMAL ||
            (st.safety == RailSafetyState::UP_LIMITED && dir == RailDirection::DOWN) ||
            (st.safety == RailSafetyState::DOWN_LIMITED && dir == RailDirection::UP)) {
            rail.jog(dir, rpm);
        }
        printf("\r[%s] L=%d H=%d U=%d pos=%.3f vel=%.0f   ", safety_name(st.safety),
               static_cast<int>(st.gates.lower), static_cast<int>(st.gates.home),
               static_cast<int>(st.gates.upper), st.motor_position_rad, st.motor_velocity_rpm);
        fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    rail.stop();
    printf("\n联锁 jog 结束\n");
    return rail.status().safety == RailSafetyState::SAFETY_FAULT ? 2 : 0;
}

int run_calibrate(RailController& rail) {
    const bool ok = rail.calibrate();
    if (!ok) {
        fprintf(stderr, "标定失败\n");
        return 3;
    }
    const auto c = rail.calibration();
    printf("theta_lower = %.6f rad\n", c.theta_lower);
    printf("theta_home  = %.6f rad\n", c.theta_home);
    printf("theta_upper = %.6f rad\n", c.theta_upper);
    printf("lower_travel = %.6f rad (%.4f m)\n", c.lower_travel_rad, c.lower_travel_m);
    printf("upper_travel = %.6f rad (%.4f m)\n", c.upper_travel_rad, c.upper_travel_m);
    printf("total_travel = %.6f rad (%.4f m)\n", c.total_travel_rad, c.total_travel_m);
    printf("=== 标定成功 calibrated=%d homed=%d 导轨位置=0 ===\n",
           static_cast<int>(c.calibrated), static_cast<int>(c.homed));
    return 0;
}

void print_usage(const char* argv0) {
    fprintf(stderr,
            "用法: %s [选项]\n"
            "  --mode sensors|interlock|calibrate\n"
            "  --config <yaml>\n"
            "  --photogate <dev>   --motor <dev>\n"
            "  --dir up|down       --rpm <n>   --duration <s>\n"
            "  --up-sign +1|-1\n"
            "  --help\n",
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
            if (std::strcmp(v, "sensors") == 0) {
                opt.mode = Mode::Sensors;
            } else if (std::strcmp(v, "interlock") == 0) {
                opt.mode = Mode::Interlock;
            } else if (std::strcmp(v, "calibrate") == 0) {
                opt.mode = Mode::Calibrate;
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
                opt.jog_dir = RailDirection::UP;
            } else if (std::strcmp(v, "down") == 0) {
                opt.jog_dir = RailDirection::DOWN;
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

    // Load --config first, then apply CLI overrides.
    CliOptions opt;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            std::string error;
            if (!omr_hardware::loadRailConfig(argv[i + 1], opt.cfg, &error)) {
                fprintf(stderr, "%s\n", error.c_str());
                return 1;
            }
            opt.config_path = argv[i + 1];
            break;
        }
    }
    if (!parse_cli(argc, argv, opt)) {
        print_usage(argv[0]);
        return 1;
    }

    printf("photogate=%s motor=%s map L%d/H%d/U%d up_sign=%d\n", opt.cfg.photogate_port.c_str(),
           opt.cfg.motor_port.c_str(), opt.cfg.lower_gate, opt.cfg.home_gate, opt.cfg.upper_gate,
           opt.cfg.up_sign);

    if (opt.mode == Mode::Sensors) {
        return run_sensors(opt.cfg);
    }

    RailController rail(opt.cfg, log_to_console);
    if (!rail.connect()) {
        fprintf(stderr, "连接失败（光电门或电机串口）\n");
        return 1;
    }

    // SIGINT watcher: abort a blocking operation (calibrate) promptly.
    std::atomic<bool> watcher_done{false};
    std::thread watcher([&rail, &watcher_done] {
        while (!watcher_done.load()) {
            if (g_stop) {
                rail.stop();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    int rc = 0;
    if (opt.mode == Mode::Interlock) {
        rc = run_interlock(rail, opt.jog_dir, opt.jog_rpm, opt.jog_duration_s);
    } else {
        rc = run_calibrate(rail);
    }

    watcher_done.store(true);
    watcher.join();
    rail.disconnect();
    return rc;
}
