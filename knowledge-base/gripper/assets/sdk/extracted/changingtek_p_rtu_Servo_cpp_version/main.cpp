#include "Changingtek_p_rtu_Servo.h"
#include <iostream>
#include <thread>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
#endif

int main() {
#ifdef _WIN32
    // 设置控制台代码页为 UTF-8，解决中文乱码问题
    SetConsoleOutputCP(65001);
#endif

    std::string port;
#ifdef _WIN32
    port = "COM3"; // Windows 默认端口
#else
    port = "/dev/ttyUSB0"; // Linux 默认端口
#endif
    
    int slave_id = 1;

    std::cout << "正在初始化伺服步进控制器 SDK (端口: " << port << ", ID: " << slave_id << ")..." << std::endl;

    try {
        Changingtek_p_rtu_Servo sdk(port, slave_id);

        if (!sdk.connect()) {
            std::cerr << "连接失败！" << std::endl;
            return 1;
        }
        std::cout << "连接成功！" << std::endl;

        // 启动实时监控
        sdk.start_monitoring(0.5);

        // 循环控制：5次往复运动
        // 假设 9000 为闭合位置，0 为张开位置
        // 最后一次循环结束后，确保处于张开位置 (0)
        for (int i = 0; i < 5; ++i) {
            std::cout << "\n--- 第 " << (i + 1) << "/5 次循环 ---" << std::endl;

            // 阶段1: 运动到位置 9000 (闭合?)
            std::cout << "执行阶段 1: 移动到位置 9000..." << std::endl;
            sdk.temp_move(9000, 50, 25, 60, 60, true);
            std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待运动完成

            // 阶段2: 运动到位置 0 (张开?)
            std::cout << "执行阶段 2: 移动到位置 0..." << std::endl;
            sdk.temp_move(0, 50, 25, 60, 60, true);
            std::this_thread::sleep_for(std::chrono::seconds(3)); // 等待运动完成
        }

        std::cout << "\n循环结束，确认最终状态为张开 (位置 0)..." << std::endl;
        // 再次发送指令确保最终状态为 0 (虽然循环最后一步已经是 0，但这保证了"最后要张开"的要求)
        sdk.temp_move(0, 50, 25, 60, 60, true);
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // 停止监控
        sdk.stop_monitoring();
        std::cout << "程序已完成。" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "发生异常: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
