#pragma once

#include <memory>

#include "bsp_pwm.h"
#include "control_modes.h"
#include "main.h"
#include "sbus.h"
#include "system_state_task.h"
#include "task_config.h"

//==================================================================================================
// 吸附控制器类
// 功能描述：
// 1. 控制机器人的吸盘系统
// 2. 通过PWM信号控制吸盘风扇的转速，调节吸力大小
// 3. 支持SBUS遥控器控制，实现实时吸力调节
// 4. 仅在遥控模式下工作，确保操作安全性
// 5. 提供死区处理，避免微小摇杆抖动影响
//
// 硬件配置：
// - 吸盘风扇：PWM控制的风扇
// - PWM频率：1000Hz
// - 脉宽范围：0-1000μs（对应0-100%占空比）
// - 控制通道：SBUS通道2（索引1）
//
// 工作原理：
// - 安全机制：仅在遥控模式下工作，避免意外启动
// - 信号映射：SBUS值 → PWM脉宽 → 风扇转速 → 吸力大小
//==================================================================================================

class SuctionController {
public:
    static constexpr uint32_t TASK_DELAY_MS = SUCTION_TASK_DELAY_MS;     // 任务循环延时

    /**
     * @brief 默认构造函数
     * @note 初始化时不分配资源，需要调用Init()完成初始化
     */
    SuctionController() = default;

    /**
     * @brief 初始化吸盘控制器
     * @param htim PWM定时器句柄，用于生成风扇控制信号
     * @param state_manager 系统状态管理器指针，用于检查控制模式
     * @param sbus SBUS遥控器指针，用于获取吸力控制指令
     * @note 配置PWM输出，设置初始状态为0μs（风扇停止）
     */
    void Init(TIM_HandleTypeDef* htim, SystemStateManager* state_manager, remote::SBUS* sbus);

    /**
     * @brief 主执行函数，处理控制逻辑
     * @note 执行流程：
     *       1. 检查系统是否处于遥控模式
     *       2. 检查SBUS连接状态
     *       3. 读取SBUS通道2的吸力控制值
     *       4. 应用死区处理和信号映射
     *       5. 设置PWM脉宽控制风扇转速
     */
    void Execute();

private:
    // PWM控制参数常量
    static constexpr uint32_t FAN_PWM_FREQUENCY = 1000;          // PWM频率1000Hz
    static constexpr uint32_t FAN_TIMER_CLOCK = 1000000;         // 定时器时钟频率1MHz
    static constexpr uint32_t FAN_PERIOD_US = 1000;              // PWM周期 = 1/1000Hz = 1000μs
  // todo:cubemx修改tim1ch1为Counter Period (AutoReload Register) = 999
    static constexpr uint32_t FAN_MAX_PULSE_WIDTH = 1000;        // 最大脉宽1000μs（100%占空比）
    static constexpr uint32_t FAN_START_PULSE_WIDTH = 9;         // 启动脉宽约9μs（对应6%占空比）
    static constexpr int SBUS_SUCTION_DEADZONE = 30;             // SBUS死区大小，避免摇杆抖动
  static constexpr int SBUS_MAX_VALUE = 824;  // 对应SBUS下阈值的绝对值

    // 外设和控制器指针
    SystemStateManager* state_manager_ = nullptr;                // 系统状态管理器指针
    remote::SBUS* sbus_ = nullptr;                              // SBUS遥控器指针
    std::unique_ptr<bsp::PWM> suction_fan_;                     // 吸附风扇PWM控制对象

    /**
     * @brief 对输入值应用死区处理
     * @param input 原始输入值
     * @param deadzone 死区大小
     * @return 处理后的输出值
     * @note 当输入值的绝对值小于死区时，输出为0
     *       这可以避免遥控器摇杆的微小抖动造成不必要的动作
     */
    static int16_t ApplyDeadzone(int16_t input, int16_t deadzone);

    /**
     * @brief 将SBUS吸力值映射为PWM脉宽
     * @param sbus_input SBUS通道值（经过死区处理）
     * @return PWM脉宽值（微秒，0-1000μs）
     * @note 映射关系：
     *       - 0或负值 → 0μs（风扇停止）
     *       - 正值线性映射到0-1000μs范围
     *       - 超过最大值时限制为1000μs
     */
    static uint32_t MapSuctionToPulseWidth(int16_t sbus_input);
};