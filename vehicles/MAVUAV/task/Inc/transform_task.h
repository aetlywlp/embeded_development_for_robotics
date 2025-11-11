// transform_task.h
#pragma once

#include <memory>

#include "bsp_can.h"
#include "bsp_gpio.h"
#include "control_modes.h"
#include "motor.h"
#include "system_state_task.h"
#include "task_config.h"
#include "sbus.h"
#include "controller.h"

//==================================================================================================
// 变形控制类
// 功能描述：
// 1. 通过遥控器直接控制变形电机和电推杆
// 2. ch[6]控制电推杆
// 3. ch[7]控制3508电机
//
//
// 硬件配置：
// - 变形电机：CAN ID 0x203 (Motor3508) - 速度控制模式
// - 电推杆：GPIO控制（电平驱动）
//==================================================================================================

class TransformController {
public:
    static constexpr uint32_t TASK_DELAY_MS = TRANSFORM_TASK_DELAY_MS;  // 任务循环延时

    TransformController() = default;

    /**
     * @brief 初始化变形控制器
     * @param can CAN总线接口指针，用于电机通信
     * @param state_manager 系统状态管理器指针，用于系统状态管理
     * @param sbus SBUS遥控器指针，用于获取遥控数据
     */
    void Init(bsp::CAN* can, SystemStateManager* state_manager, remote::SBUS* sbus);

    /**
     * @brief 主执行函数，应在任务循环中周期性调用
     * @note 执行流程：
     *       1. 获取遥控器数据
     *       2. 控制电推杆
     *       3. 控制3508电机
     *       4. 发送CAN指令
     */
    void Execute();

    /**
     * @brief 获取变形电机指针（用于状态查询和诊断）
     */
    control::Motor3508* GetTransformMotor() { return transform_motor_.get(); }

private:
    // 控制参数常量
    static constexpr float TRANSFORM_MOTOR_SPEED = 40.0f;          // 变形电机速度 (rad/s) - 安全速度
    static constexpr int16_t RC_HIGH_THRESHOLD = 400;              // 遥控高阈值
    static constexpr int16_t RC_LOW_THRESHOLD = -400;              // 遥控低阈值


  static constexpr float PID_KP = 30.0f;                         // 比例系数（增强控制力）
  static constexpr float PID_KI = 8.0f;                          // 积分系数（增强抗负载能力）
  static constexpr float PID_KD = 10.0f;                         // 微分系数（增强稳定性）
  static constexpr float PID_MAX_IOUT = 5000.0f;                 // 积分输出限制（增加）
  static constexpr float PID_MAX_OUT = 11000.0f;                 // 总输出限制（增加，接近3508最大电流）

  // 添加最小输出参数：
  static constexpr float MIN_OUTPUT_THRESHOLD = 1500.0f;         // 最小输出阈值（克服静摩擦）

    // 硬件和外设指针
    bsp::CAN* can_ = nullptr;                                      // CAN总线接口
    SystemStateManager* state_manager_ = nullptr;                  // 系统状态管理器
    remote::SBUS* sbus_ = nullptr;                                 // SBUS遥控器接口

    // 控制对象
    std::unique_ptr<control::Motor3508> transform_motor_;          // 变形电机对象
    std::unique_ptr<bsp::GPIO> actuator_gpio_;                     // 电推杆GPIO控制
    std::unique_ptr<control::ConstrainedPID> motor_pid_;           // 电机约束PID控制器

    // 状态变量
    float target_speed_ = 0.0f;                                    // 目标速度 (rad/s)

    /**
     * @brief 控制电推杆
     * @param rc_value 遥控器ch[6]值
     */
    void ControlActuator(int16_t rc_value);

    /**
     * @brief 控制3508电机
     * @param rc_value 遥控器ch[7]值
     */
    void ControlMotor(int16_t rc_value);
};