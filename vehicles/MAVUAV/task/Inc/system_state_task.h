#pragma once

#include <functional>
#include "cmsis_os.h"
#include "sbus.h"
#include "control_modes.h"

//==================================================================================================
// 系统状态管理类
// 功能描述：
// 1. 管理机器人的控制模式（紧急停止、遥控、计算机控制）
// 2. 管理机器人的变形模式（车辆模式、飞行模式）
// 3. 基于SBUS遥控器输入进行模式切换
// 4. 提供线程安全的电机控制互斥锁
// 5. 监控SBUS连接状态和超时检测
//==================================================================================================

class SystemStateManager {
public:
    SystemStateManager() = default;

    // 禁用拷贝构造函数和赋值运算符，确保单例模式
    SystemStateManager(const SystemStateManager&) = delete;
    SystemStateManager& operator=(const SystemStateManager&) = delete;

    /**
     * @brief 初始化系统状态管理器
     * @param sbus_ptr SBUS遥控器接收器指针
     */
    void Init(remote::SBUS* sbus_ptr);

    /**
     * @brief 获取当前控制模式
     * @return 当前的控制模式枚举值
     */
    ControlMode GetCurrentControlMode() const { return current_control_mode_; }

    /**
     * @brief 获取当前变形模式
     * @return 当前的变形模式枚举值
     */
    TransformMode GetCurrentTransformMode() const { return current_transform_mode_; }

    /**
     * @brief 获取电机控制互斥锁
     * @return 电机控制互斥锁句柄，用于多任务间的线程安全控制
     */
    osMutexId_t GetMotorControlMutex() const { return motor_control_mutex_; }

    /**
     * @brief 基于SBUS输入更新控制模式和变形模式
     * @note 这是主要的状态更新函数，应该在主循环中周期性调用
     */
    void UpdateControlModes();

    /**
     * @brief 检查系统是否处于紧急停止状态
     * @return true：紧急停止状态，false：正常状态
     */
    bool IsInEmergencyStop() const;

    /**
     * @brief 设置PID控制器重置回调函数
     * @param callback 当模式切换时需要重置PID控制器的回调函数
     * @note 用于在模式切换时重置车辆控制的PID参数，防止积分饱和
     */
    void SetPIDResetCallback(std::function<void()> callback);

private:
    // 模式切换阈值常量
    static constexpr int16_t MODE_UPPER_THRESHOLD = 400;    // 模式通道上限阈值
    static constexpr int16_t MODE_LOWER_THRESHOLD = -400;   // 模式通道下限阈值
    static constexpr uint32_t SBUS_TIMEOUT_MS = 500;       // SBUS超时时间（毫秒）

    // 硬件和外设指针
    remote::SBUS* sbus_ = nullptr;                          // SBUS遥控器接收器指针
    osMutexId_t motor_control_mutex_ = nullptr;             // 电机控制互斥锁

    // 当前状态变量
    ControlMode current_control_mode_ = ControlMode::EMERGENCY_STOP;     // 当前控制模式，默认紧急停止
    ControlMode previous_control_mode_ = ControlMode::EMERGENCY_STOP;    // 上一次控制模式
    TransformMode current_transform_mode_ = TransformMode::VEHICLE;      // 当前变形模式，默认车辆模式
    TransformMode previous_transform_mode_ = TransformMode::VEHICLE;     // 上一次变形模式

    // 回调函数
    std::function<void()> pid_reset_callback_;              // PID重置回调函数

    /**
     * @brief 根据SBUS通道6的值确定控制模式
     * @return 解析出的控制模式
     * @note 通道5值映射：
     *       < -400: 紧急停止
     *       -400 ~ 400: 遥控模式
     *       > 400: 计算机控制模式
     */
    ControlMode DetermineControlMode();

    /**
     * @brief 根据SBUS通道5的值确定变形模式
     * @return 解析出的变形模式
     * @note 通道4值映射：
     *       <= 0: 车辆模式
     *       > 0: 飞行模式
     */
    TransformMode DetermineTransformMode();

    /**
     * @brief 处理模式切换时的相关操作
     * @note 主要处理切换到紧急停止模式时的PID重置等操作
     */
    void HandleModeSwitch();
};