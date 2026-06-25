#include "Changingtek_p_rtu_Servo.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>
#include <iomanip>
#include <cstring>

#ifndef _WIN32
#include <sys/select.h>
#endif

// 辅助函数: 字节交换 (Modbus 是大端序)
static uint16_t swap_bytes(uint16_t val) {
    return (val << 8) | (val >> 8);
}

Changingtek_p_rtu_Servo::Changingtek_p_rtu_Servo(const std::string& port, int slave_id, int baudrate, double timeout)
    : port_name(port), slave_id(slave_id), baudrate(baudrate), timeout(timeout), hSerial(INVALID_SERIAL_HANDLE), _monitor_running(false) {
}

Changingtek_p_rtu_Servo::~Changingtek_p_rtu_Servo() {
    stop_monitoring();
    disconnect();
}

bool Changingtek_p_rtu_Servo::connect() {
    std::lock_guard<std::mutex> lock(_mutex);
#ifdef _WIN32
    // 打开串口
    // 使用 \\.\COMxx 格式以支持 COM10 以上的端口
    std::string full_port_name = port_name;
    if (full_port_name.find("\\\\.\\") == std::string::npos) {
        full_port_name = "\\\\.\\" + port_name;
    }
    
    hSerial = CreateFileA(full_port_name.c_str(),
                          GENERIC_READ | GENERIC_WRITE,
                          0,
                          NULL,
                          OPEN_EXISTING,
                          0,
                          NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cerr << "Error opening serial port: " << port_name << " (Error: " << GetLastError() << ")" << std::endl;
        return false;
    }

    // 配置串口参数
    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    if (!GetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error getting serial state" << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        std::cerr << "Error setting serial state" << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    // 设置超时
    COMMTIMEOUTS timeouts = {0};
    DWORD timeout_ms = static_cast<DWORD>(timeout * 1000);
    
    // ReadIntervalTimeout: 字节间最大间隔时间
    timeouts.ReadIntervalTimeout = 50; 
    // 总读取超时 = Multiplier * 字节数 + Constant
    timeouts.ReadTotalTimeoutConstant = timeout_ms;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    timeouts.WriteTotalTimeoutConstant = timeout_ms;
    timeouts.WriteTotalTimeoutMultiplier = 10;

    if (!SetCommTimeouts(hSerial, &timeouts)) {
        std::cerr << "Error setting serial timeouts" << std::endl;
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        return false;
    }

    return true;
#else
    // Linux implementation
    hSerial = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (hSerial < 0) {
        std::cerr << "Error opening serial port: " << port_name << " (Error: " << strerror(errno) << ")" << std::endl;
        return false;
    }

    struct termios tty;
    if (tcgetattr(hSerial, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
        close(hSerial);
        hSerial = INVALID_SERIAL_HANDLE;
        return false;
    }

    cfsetospeed(&tty, B115200); // 默认先设为 115200, 后面根据参数调整
    cfsetispeed(&tty, B115200);

    // 设置自定义波特率
    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600; break;
        case 19200:  speed = B19200; break;
        case 38400:  speed = B38400; break;
        case 57600:  speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // disable break processing
    tty.c_lflag = 0;                                // no signaling chars, no echo,
                                                    // no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls,
                                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit

    if (tcsetattr(hSerial, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(hSerial);
        hSerial = INVALID_SERIAL_HANDLE;
        return false;
    }
    return true;
#endif
}

void Changingtek_p_rtu_Servo::disconnect() {
    std::lock_guard<std::mutex> lock(_mutex);
    if (hSerial != INVALID_SERIAL_HANDLE) {
#ifdef _WIN32
        CloseHandle(hSerial);
#else
        close(hSerial);
#endif
        hSerial = INVALID_SERIAL_HANDLE;
    }
}

// -----------------------------
// Modbus 核心实现
// -----------------------------

uint16_t Changingtek_p_rtu_Servo::calculate_crc(const std::vector<uint8_t>& data) {
    uint16_t crc = 0xFFFF;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Changingtek_p_rtu_Servo::send_frame(const std::vector<uint8_t>& frame) {
    if (hSerial == INVALID_SERIAL_HANDLE) {
        // 尝试重新连接，但不在此处加锁，因为调用方已经加锁了
        // 这里需要小心死锁，假设 send_frame 总是由公有方法调用，而公有方法已经加锁
        // 如果 connect 内部也加锁，需要使用 std::recursive_mutex 或者把锁逻辑移到外层
        // 为了简单起见，我们假设 connect 是线程安全的或者被外层锁保护
        // 但 connect 目前使用了 lock_guard，这会导致死锁如果在这里调用。
        // 修改策略：底层函数不负责连接，只负责发送。连接由上层保证。
        // 或者：connect 使用 try_lock 或者 recursive_mutex。
        // 鉴于目前架构，我们在 connect 中使用了 lock_guard。
        // 我们应该在 send_frame 之前确保连接。
        throw_error("Port not open");
    }

#ifdef _WIN32
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    DWORD bytes_written;
    if (!WriteFile(hSerial, frame.data(), static_cast<DWORD>(frame.size()), &bytes_written, NULL)) {
        throw_error("Write failed");
    }
    if (bytes_written != frame.size()) {
        throw_error("Write incomplete");
    }
#else
    tcflush(hSerial, TCIOFLUSH);
    ssize_t bytes_written = write(hSerial, frame.data(), frame.size());
    if (bytes_written < 0) {
        throw_error("Write failed: " + std::string(strerror(errno)));
    }
    if (static_cast<size_t>(bytes_written) != frame.size()) {
        throw_error("Write incomplete");
    }
#endif
}

std::vector<uint8_t> Changingtek_p_rtu_Servo::receive_response(int expected_min_bytes) {
    std::vector<uint8_t> buffer;
    buffer.reserve(256);
    uint8_t tmp_buf[256];
    
#ifdef _WIN32
    DWORD bytes_read;
    if (!ReadFile(hSerial, tmp_buf, 256, &bytes_read, NULL)) {
        throw_error("Read failed");
    }
    if (bytes_read == 0) {
        throw_error("Read timeout (no data)");
    }
    for (DWORD i = 0; i < bytes_read; i++) {
        buffer.push_back(tmp_buf[i]);
    }
#else
    fd_set set;
    struct timeval timeout_tv;
    
    timeout_tv.tv_sec = static_cast<long>(timeout);
    timeout_tv.tv_usec = static_cast<long>((timeout - static_cast<long>(timeout)) * 1000000);

    FD_ZERO(&set);
    FD_SET(hSerial, &set);

    int rv = select(hSerial + 1, &set, NULL, NULL, &timeout_tv);
    if (rv == -1) {
        throw_error("Select failed: " + std::string(strerror(errno)));
    } else if (rv == 0) {
        throw_error("Read timeout (no data)");
    } else {
        ssize_t n = read(hSerial, tmp_buf, sizeof(tmp_buf));
        if (n < 0) {
             throw_error("Read failed: " + std::string(strerror(errno)));
        }
        if (n == 0) {
             throw_error("Read returned 0 (EOF?)");
        }
        for (ssize_t i = 0; i < n; i++) {
            buffer.push_back(tmp_buf[i]);
        }
    }
#endif

    if (buffer.size() < 2) throw_error("Response too short");
    
    uint16_t received_crc = buffer[buffer.size() - 2] | (buffer[buffer.size() - 1] << 8);
    std::vector<uint8_t> data_for_crc(buffer.begin(), buffer.end() - 2);
    uint16_t calc_crc = calculate_crc(data_for_crc);

    if (received_crc != calc_crc) {
        throw_error("CRC Error");
    }

    return buffer;
}

void Changingtek_p_rtu_Servo::throw_error(const std::string& msg) {
    throw std::runtime_error("Changingtek_p_rtu_Servo Error: " + msg);
}

// -----------------------------
// Modbus 辅助方法
// -----------------------------

void Changingtek_p_rtu_Servo::_write_register(uint16_t addr, uint16_t value) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (hSerial == INVALID_SERIAL_HANDLE) throw_error("Not connected");

    std::vector<uint8_t> frame;
    frame.push_back(static_cast<uint8_t>(slave_id));
    frame.push_back(0x06); // Write Single Register
    frame.push_back(addr >> 8);
    frame.push_back(addr & 0xFF);
    frame.push_back(value >> 8);
    frame.push_back(value & 0xFF);

    uint16_t crc = calculate_crc(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    send_frame(frame);
    
    // 接收响应 (回显)
    receive_response();
}

void Changingtek_p_rtu_Servo::_write_registers(uint16_t addr, const std::vector<uint16_t>& values) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (hSerial == INVALID_SERIAL_HANDLE) throw_error("Not connected");

    std::vector<uint8_t> frame;
    frame.push_back(static_cast<uint8_t>(slave_id));
    frame.push_back(0x10); // Write Multiple Registers
    frame.push_back(addr >> 8);
    frame.push_back(addr & 0xFF);
    uint16_t count = static_cast<uint16_t>(values.size());
    frame.push_back(count >> 8);
    frame.push_back(count & 0xFF);
    frame.push_back(count * 2); // Byte count

    for (uint16_t val : values) {
        frame.push_back(val >> 8);
        frame.push_back(val & 0xFF);
    }

    uint16_t crc = calculate_crc(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    send_frame(frame);
    
    // 接收响应
    receive_response();
}

uint16_t Changingtek_p_rtu_Servo::_read_register(uint16_t addr) {
    auto res = _read_registers(addr, 1);
    if (res.empty()) throw_error("Empty response");
    return res[0];
}

std::vector<uint16_t> Changingtek_p_rtu_Servo::_read_registers(uint16_t addr, int count) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (hSerial == INVALID_SERIAL_HANDLE) throw_error("Not connected");

    std::vector<uint8_t> frame;
    frame.push_back(static_cast<uint8_t>(slave_id));
    frame.push_back(0x03); // Read Holding Registers
    frame.push_back(addr >> 8);
    frame.push_back(addr & 0xFF);
    frame.push_back(count >> 8);
    frame.push_back(count & 0xFF);

    uint16_t crc = calculate_crc(frame);
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    send_frame(frame);
    
    std::vector<uint8_t> response = receive_response();
    
    // 解析响应
    // [SlaveID, FuncCode, ByteCount, Data..., CRC_L, CRC_H]
    if (response.size() < static_cast<size_t>(3 + count * 2 + 2)) {
        throw_error("Response data length mismatch");
    }

    if (response[1] != 0x03) {
        throw_error("Response function code mismatch");
    }

    std::vector<uint16_t> values;
    for (int i = 0; i < count; ++i) {
        uint16_t val = (response[3 + i * 2] << 8) | response[3 + i * 2 + 1];
        values.push_back(val);
    }
    return values;
}

// -----------------------------
// 目标值/控制写入操作
// -----------------------------

void Changingtek_p_rtu_Servo::set_target_position(int position) {
    uint16_t hi = (position >> 16) & 0xFFFF;
    uint16_t lo = position & 0xFFFF;
    std::vector<uint16_t> values = {hi, lo};
    _write_registers(REG_TARGET_POS_HIGH, values);
}

void Changingtek_p_rtu_Servo::set_target_speed(int speed) {
    _write_register(REG_TARGET_SPEED, static_cast<uint16_t>(speed));
}

void Changingtek_p_rtu_Servo::set_target_force(int force) {
    _write_register(REG_TARGET_FORCE, static_cast<uint16_t>(force));
}

void Changingtek_p_rtu_Servo::set_target_acceleration(int acceleration) {
    _write_register(REG_TARGET_ACCELERATION, static_cast<uint16_t>(acceleration));
}

void Changingtek_p_rtu_Servo::set_target_deceleration(int deceleration) {
    _write_register(REG_TARGET_DECELERATION, static_cast<uint16_t>(deceleration));
}

void Changingtek_p_rtu_Servo::trigger_motion() {
    _write_register(REG_MOTION_TRIGGER, 1);
}

void Changingtek_p_rtu_Servo::temp_move(int position_mm, int speed_pct, int force_pct, int accel, int decel, bool trigger) {
    // 优化：一次性写入所有参数 (REG_TARGET_POS_HIGH ~ REG_MOTION_TRIGGER)
    // 地址连续: 
    // 0x0102: POS_H
    // 0x0103: POS_L
    // 0x0104: SPEED
    // 0x0105: FORCE
    // 0x0106: ACCEL
    // 0x0107: DECEL
    // 0x0108: TRIGGER
    
    std::vector<uint16_t> values;
    values.push_back((position_mm >> 16) & 0xFFFF);
    values.push_back(position_mm & 0xFFFF);
    values.push_back(static_cast<uint16_t>(speed_pct));
    values.push_back(static_cast<uint16_t>(force_pct));
    values.push_back(static_cast<uint16_t>(accel));
    values.push_back(static_cast<uint16_t>(decel));
    
    if (trigger) {
        values.push_back(1);
    } else {
        // 如果不触发，是否需要写入0或者不写该寄存器？
        // 为了保持连续写入，写入0 (空闲)
        values.push_back(0);
    }
    
    _write_registers(REG_TARGET_POS_HIGH, values);

    if (trigger) {
        std::cout << "已触发运动到位置 " << position_mm << "，速度 " << speed_pct << "%，力 " << force_pct << "%" << std::endl;
    }
}

// -----------------------------
// 反馈/状态读取操作
// -----------------------------

int Changingtek_p_rtu_Servo::read_real_position() {
    // 连续读取高低位
    std::vector<uint16_t> regs = _read_registers(REG_REAL_POS_HIGH, 2);
    uint32_t combined = (regs[0] << 16) | regs[1];
    
    // 处理32位有符号整数
    int32_t final_val;
    if (combined & 0x80000000) {
        final_val = static_cast<int32_t>(combined); // C++ 中 uint32 转 int32，如果是补码表示则直接转换即可
        // 或者更安全的做法:
        // final_val = -static_cast<int32_t>((~combined + 1)); 
        // 但通常 combined 就是补码形式，直接 cast 即可。
    } else {
        final_val = static_cast<int32_t>(combined);
    }
    return final_val;
}

int Changingtek_p_rtu_Servo::read_real_speed() {
    return static_cast<int16_t>(_read_register(REG_REAL_SPEED));
}

int Changingtek_p_rtu_Servo::read_real_current() {
    return static_cast<int16_t>(_read_register(REG_REAL_CURRENT));
}

// -----------------------------
// 实时监控
// -----------------------------

void Changingtek_p_rtu_Servo::_monitor_loop(double interval) {
    while (_monitor_running) {
        try {
            int pos = read_real_position();
            int speed = read_real_speed();
            int current = read_real_current();
            
            // 使用 printf 格式化输出，或者 iomanip
            // 为了防止输出混乱，可以加个简单的锁或者直接输出
            // std::cout 是线程安全的(字符级)，但多个线程输出可能会交错
            std::cout << "[实时数据] 位置: " << std::setw(6) << pos 
                      << ", 速度: " << std::setw(3) << speed 
                      << ", 电流: " << std::setw(3) << current << std::endl;
        } catch (const std::exception& e) {
            std::cout << "监控错误: " << e.what() << std::endl;
        }
        
        // sleep
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interval * 1000)));
    }
}

void Changingtek_p_rtu_Servo::start_monitoring(double interval) {
    if (!_monitor_running) {
        _monitor_running = true;
        _monitor_thread = std::thread(&Changingtek_p_rtu_Servo::_monitor_loop, this, interval);
        _monitor_thread.detach(); // 分离线程，类似 Python daemon=True
        std::cout << "实时监控已启动。" << std::endl;
    }
}

void Changingtek_p_rtu_Servo::stop_monitoring() {
    if (_monitor_running) {
        _monitor_running = false;
        // 由于 detach 了，不能 join。
        // 但我们可以等待一小段时间让循环退出
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::cout << "实时监控已停止。" << std::endl;
    }
}
