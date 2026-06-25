#ifndef CHANGINGTEK_P_RTU_SERVO_H
#define CHANGINGTEK_P_RTU_SERVO_H

#include <string>
#include <vector>
#include <cstdint>
#include <thread>
#include <atomic>
#include <mutex>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#ifdef _WIN32
#include <windows.h>
typedef HANDLE SerialHandle;
#define INVALID_SERIAL_HANDLE INVALID_HANDLE_VALUE
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <cstring>
typedef int SerialHandle;
#define INVALID_SERIAL_HANDLE -1
#endif
#include <iostream>

// -----------------------------
// 寄存器地址映射 (保持寄存器, 功能码 0x03/0x06/0x10)
// -----------------------------

// 写入寄存器 (目标值/控制)
const uint16_t REG_TARGET_POS_HIGH       = 0x0102;  // 目标位置（高16位）
const uint16_t REG_TARGET_POS_LOW        = 0x0103;  // 目标位置（低16位）
const uint16_t REG_TARGET_SPEED          = 0x0104;  // 目标速度
const uint16_t REG_TARGET_FORCE          = 0x0105;  // 目标力/力矩
const uint16_t REG_TARGET_ACCELERATION   = 0x0106;  // 目标加速度
const uint16_t REG_TARGET_DECELERATION   = 0x0107;  // 目标减速度
const uint16_t REG_MOTION_TRIGGER        = 0x0108;  // 运动触发（0: 空闲, 1: 触发）

// 读取寄存器 (反馈/状态)
const uint16_t REG_REAL_POS_HIGH         = 0x0418;  // 实时位置（高16位）
const uint16_t REG_REAL_POS_LOW          = 0x0419;  // 实时位置（低16位）
const uint16_t REG_REAL_SPEED            = 0x041A;  // 实时速度反馈
const uint16_t REG_REAL_CURRENT          = 0x041B;  // 实时电流反馈

/**
 * Changingtek_p_rtu_Servo 类
 * 
 * 封装了基于 Modbus RTU (RS-485) 的伺服步进控制器协议。
 * 提供了连接、运动控制以及状态读取功能。
 */
class Changingtek_p_rtu_Servo {
public:
    /**
     * 构造函数
     * @param port 串口名称，例如 "COM3" 或 "/dev/ttyUSB0"
     * @param slave_id Modbus 从站地址 (默认 1)
     * @param baudrate 波特率 (默认 115200)
     * @param timeout 读写超时时间 (秒, 默认 1.0)
     */
    Changingtek_p_rtu_Servo(const std::string& port, int slave_id = 1, int baudrate = 115200, double timeout = 1.0);
    ~Changingtek_p_rtu_Servo();

    /**
     * 连接串口
     * @return 成功返回 true，失败返回 false
     */
    bool connect();

    /**
     * 断开串口连接
     */
    void disconnect();

    // ------------- 目标值/控制写入操作 -------------

    /**
     * 设置目标位置
     * @param position 目标位置值
     */
    void set_target_position(int position);

    /**
     * 设置目标速度
     * @param speed 目标速度值
     */
    void set_target_speed(int speed);

    /**
     * 设置目标力/力矩
     * @param force 目标力/力矩值
     */
    void set_target_force(int force);

    /**
     * 设置目标加速度
     * @param acceleration 目标加速度值
     */
    void set_target_acceleration(int acceleration);

    /**
     * 设置目标减速度
     * @param deceleration 目标减速度值
     */
    void set_target_deceleration(int deceleration);

    /**
     * 触发运动
     */
    void trigger_motion();

    /**
     * 临时运动控制方法，一次性设置所有运动参数并可选触发运动
     * 
     * @param position_mm 目标位置(毫米/脉冲)
     * @param speed_pct 速度百分比
     * @param force_pct 力/力矩百分比
     * @param accel 加速度值
     * @param decel 减速度值
     * @param trigger 是否立即触发运动，默认为 true
     */
    void temp_move(int position_mm, int speed_pct, int force_pct, int accel, int decel, bool trigger = true);

    // ------------- 反馈/状态读取操作 -------------

    /**
     * 读取实时位置
     * @return 组合后的32位实时位置
     */
    int read_real_position();

    /**
     * 读取实时速度反馈
     * @return 实时速度值
     */
    int read_real_speed();

    /**
     * 读取实时电流反馈
     * @return 实时电流值
     */
    int read_real_current();

    // ------------- 实时监控 -------------

    /**
     * 启动实时数据监控线程
     * @param interval 监控间隔（秒，默认: 0.5）
     */
    void start_monitoring(double interval = 0.5);

    /**
     * 停止实时数据监控线程
     */
    void stop_monitoring();

private:
    std::string port_name;
    int slave_id;
    int baudrate;
    double timeout;
    SerialHandle hSerial;

    // 线程安全锁
    std::mutex _mutex;

    // 监控线程相关
    std::atomic<bool> _monitor_running;
    std::thread _monitor_thread;
    void _monitor_loop(double interval);

    // 内部 Modbus 辅助函数
    void _write_register(uint16_t addr, uint16_t value);
    void _write_registers(uint16_t addr, const std::vector<uint16_t>& values);
    uint16_t _read_register(uint16_t addr);
    std::vector<uint16_t> _read_registers(uint16_t addr, int count);
    
    // 底层通信函数
    void send_frame(const std::vector<uint8_t>& frame);
    std::vector<uint8_t> receive_response(int expected_min_bytes = 0);
    uint16_t calculate_crc(const std::vector<uint8_t>& data);
    void throw_error(const std::string& msg);
};

#endif // CHANGINGTEK_P_RTU_SERVO_H
