#pragma once

#include <array>
#include <memory>
#include <algorithm>

#include "bsp_can.h"
#include "control_modes.h"
#include "motor.h"
#include "sbus.h"
#include "system_state_task.h"
#include "task_config.h"

//==================================================================================================
// 高摩擦环境优化的车辆控制类
// 核心优化：
// 1. 激进PID控制，充分利用Motor2006的12A电流能力
// 2. 动态功率分配，转弯时智能调节左右轮功率比例
// 3. 转弯增强模式，检测转弯意图时提供额外扭矩
// 4. 保持原有高速度性能，通过控制策略提升灵活性
//
// 硬件配置：
// - 左轮电机：CAN ID 0x201 (Motor2006) - 正向
// - 右轮电机：CAN ID 0x202 (Motor2006) - 反向（背靠背安装）
// - 减速比：10:1，最大电流：10A每个电机
//==================================================================================================

class VehicleController {
public:
    static constexpr uint32_t TASK_DELAY_MS = VEHICLE_TASK_DELAY_MS;

    VehicleController() = default;

    /**
     * @brief 初始化车辆控制器
     */
    void Init(bsp::CAN* can, SystemStateManager* state_manager, remote::SBUS* sbus);

    /**
     * @brief 主执行函数
     */
    void Execute();

    /**
     * @brief 紧急停止功能
     */
    void EmergencyStop();

    /**
     * @brief 设置计算机控制指令
     */
    void SetComputerControl(float linear_vel, float angular_vel);

    /**
     * @brief 获取电机指针
     */
    control::Motor2006* GetLeftMotor() { return left_motor_.get(); }
    control::Motor2006* GetRightMotor() { return right_motor_.get(); }

private:
    // ============ 优化的控制参数 ============
    // 保持高速度，通过控制策略提升转向能力

    // 速度参数（保持原有性能）
    static constexpr float MAX_VEHICLE_SPEED = 10.0f;      // 保持原速度
    static constexpr float MAX_ANGULAR_SPEED = 20.0f;      // 保持原角速度

    // 物理参数
    static constexpr float WHEEL_RADIUS = 0.143f;
    static constexpr float WHEEL_BASE = 0.3f;
    static constexpr float GEAR_RATIO =9.0f;    // 用户确认的减速比

    // 激进PID参数（充分利用Motor2006的10A能力）
    static constexpr std::array<float, 3> PID_PARAMS = {35.0f, 2.5f, 0.8f}; // 激进PID
    static constexpr std::array<float, 3> PID_PARAMS_AGGRESSIVE = {35.0f, 2.5f, 0.8f}; // 转弯增强PID

    // 电机输出参数（接近Motor2006最大能力）
    static constexpr float MAX_MOTOR_OUTPUT = 16384.0f;
    static constexpr float AGGRESSIVE_OUTPUT_LIMIT = 12000.0f;  // 接近12A极限（12000）
    static constexpr float NORMAL_OUTPUT_LIMIT = 10000.0f;      // 正常输出限制

    // 转弯增强参数
    // 动态功率分配，转弯时给转向轮更多功率
    static constexpr float TURN_THRESHOLD = 15.0f;         // 转弯检测阈值 (rad/s)
    static constexpr float POWER_REDISTRIBUTION_FACTOR = 1.5f; // 功率重分配系数
    static constexpr float TURNING_BOOST_FACTOR = 1.2f;    // 转弯时的输出增强系数

    // SBUS参数
    static constexpr int SBUS_CENTER = 1024;
    static constexpr int SBUS_MIN_VALUE = -824;
    static constexpr int SBUS_MAX_VALUE = 776;
    static constexpr int SBUS_DEADZONE = 50;

    // 速度限制
    static constexpr float MAX_WHEEL_SPEED = 40.0f;        // 保持原有轮速上限

    // ============ 硬件指针 ============
    bsp::CAN* can_ = nullptr;
    SystemStateManager* state_manager_ = nullptr;
    remote::SBUS* sbus_ = nullptr;

    // ============ 电机和控制器 ============
    std::unique_ptr<control::Motor2006> left_motor_;
    std::unique_ptr<control::Motor2006> right_motor_;
    std::unique_ptr<control::ConstrainedPID> left_pid_;
    std::unique_ptr<control::ConstrainedPID> right_pid_;

    // ============ 新增：动态PID控制器 ============
    // 转弯时使用更激进的PID参数
    std::unique_ptr<control::ConstrainedPID> left_pid_aggressive_;
    std::unique_ptr<control::ConstrainedPID> right_pid_aggressive_;

    // ============ 控制状态变量 ============
    float target_linear_speed_ = 0.0f;
    float target_angular_speed_ = 0.0f;
    float left_wheel_speed_ = 0.0f;
    float right_wheel_speed_ = 0.0f;

    // ============ 转弯增强状态 ============
    bool is_turning_mode_ = false;              // 当前是否处于转弯模式
    float previous_angular_speed_ = 0.0f;       // 上次角速度，用于检测转弯
    uint32_t turning_start_time_ = 0;           // 转弯开始时间
    static constexpr uint32_t TURNING_MODE_DURATION = 500; // 转弯模式持续时间(ms)

    // ============ 工具函数 ============
    static int16_t ApplyDeadzone(int16_t input, int16_t deadzone);
    static float NormalizeSBUS(int16_t input);

    template<typename T>
    static T clip(T value, T min_val, T max_val) {
        return std::max(min_val, std::min(value, max_val));
    }

    // ============ 核心控制函数 ============
    void ProcessSBUSInputs();
    void CalculateDifferentialSpeeds();
    void ControlMotors();
    void StopMotors();
    void SendMotorCommands();
    void ResetPIDs();

    // ============ 转弯优化函数 ============
    /**
     * @brief 检测并更新转弯模式
     * @note 根据角速度变化检测转弯意图，动态调整控制策略
     */
    void UpdateTurningMode();

    /**
     * @brief 应用动态功率分配
     * @param left_speed 左轮基础速度
     * @param right_speed 右轮基础速度
     * @param angular_speed 当前角速度
     * @note 转弯时重新分配功率，给转向轮更多扭矩
     */
    void ApplyPowerRedistribution(float& left_speed, float& right_speed, float angular_speed);

    /**
     * @brief 获取当前时间戳
     */
    uint32_t GetCurrentTimeMs();
};