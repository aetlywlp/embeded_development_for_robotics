#include "cmsis_os.h"
#include "suction_task.h"

/**
 * @brief 初始化吸盘控制器
 * @param htim PWM定时器句柄，用于生成吸盘风扇控制信号
 * @param state_manager 系统状态管理器指针，用于检查当前控制模式
 * @param sbus SBUS遥控器指针，用于获取吸力控制指令
 * @note 初始化流程：
 *       1. 保存外设指针供后续使用
 *       2. 创建PWM对象并配置参数
 *       3. 启动PWM输出
 *       4. 设置初始脉宽为0μs（风扇停止）
 */
void SuctionController::Init(TIM_HandleTypeDef* htim, SystemStateManager* state_manager, remote::SBUS* sbus) {
    // 保存控制器和外设指针
    state_manager_ = state_manager;
    sbus_ = sbus;

    // 创建PWM控制对象
    // 参数说明：
    // - htim: PWM定时器（如TIM1）
    // - 1: PWM通道号（通道1）
    // - FAN_TIMER_CLOCK: 定时器时钟频率1MHz
    // - FAN_PWM_FREQUENCY: PWM频率1000Hz（与bsp_fan.h一致）
    // - 0: 初始脉宽0μs（风扇停止状态）
    suction_fan_ = std::make_unique<bsp::PWM>(
            htim, 1, FAN_TIMER_CLOCK, FAN_PWM_FREQUENCY, 0);

    // 启动PWM输出
    suction_fan_->Start();

    // 设置初始脉宽为0μs，确保风扇处于停止状态
    // 这是安全措施，防止系统启动时风扇意外启动
    suction_fan_->SetPulseWidth(0);
}

/**
 * @brief 吸盘控制器主执行函数
 * @note 执行流程：
 *       1. 检查PWM对象是否已初始化
 *       2. 检查系统是否处于遥控模式（安全检查）
 *       3. 检查SBUS连接状态
 *       4. 读取并处理SBUS吸力控制指令
 *       5. 设置相应的PWM脉宽控制风扇转速
 */
void SuctionController::Execute() {
    // 检查PWM对象是否已成功初始化
    if (!suction_fan_) {
        osDelay(TASK_DELAY_MS);
        return;
    }

    // 安全检查：只有在遥控模式下才允许吸盘工作
    // 这防止了在自主模式或紧急停止状态下意外启动吸盘
    if (state_manager_->GetCurrentControlMode() != ControlMode::REMOTE_CONTROL) {
        // 非遥控模式，强制停止风扇
        suction_fan_->SetPulseWidth(0);  // 设置脉宽为0μs
        osDelay(TASK_DELAY_MS);
        return;
    }

    // 检查SBUS遥控器连接状态
    if (!sbus_ || !sbus_->connection_flag_) {
        // SBUS未连接，出于安全考虑停止风扇
        suction_fan_->SetPulseWidth(0);  // 设置脉宽为0μs
        osDelay(TASK_DELAY_MS);
        return;
    }

    // 读取SBUS通道3（索引2）的吸力控制值并应用死区处理
    // 死区处理可以避免遥控器摇杆的微小抖动造成不必要的风扇启动
    int16_t suction_raw = ApplyDeadzone(sbus_->ch[2], SBUS_SUCTION_DEADZONE);

    // 将SBUS值映射为PWM脉宽
    uint32_t target_pulse_width = MapSuctionToPulseWidth(suction_raw);

    // 设置PWM脉宽，控制风扇转速和吸力大小
    suction_fan_->SetPulseWidth(target_pulse_width);

    // 任务延时，控制执行频率
    osDelay(TASK_DELAY_MS);
}

/**
 * @brief 对输入值应用死区处理
 * @param input 原始输入值
 * @param deadzone 死区大小
 * @return 处理后的输出值
 * @note 死区处理原理：
 *       - 当输入值的绝对值小于死区时，输出为0
 *       - 这可以消除遥控器摇杆的微小抖动和中位偏差
 *       - 提高控制精度和系统稳定性
 */
int16_t SuctionController::ApplyDeadzone(int16_t input, int16_t deadzone) {
    return (abs(input) < deadzone) ? 0 : input;
}

/**
 * @brief 将SBUS吸力值映射为PWM脉宽
 * @param sbus_input SBUS通道值（已经过死区处理）
 * @return PWM脉宽值（微秒，0-1000μs）
 * @note 映射算法：
 *       1. 负值或零值映射为0μs（风扇停止）
 *       2. 正值线性映射到0-1000μs范围
 *       3. 超过最大值时自动限制为1000μs
 *       4. 这种映射方式确保了安全性和线性控制特性
 */
/**
 * @brief 将SBUS吸力值映射为PWM脉宽
 * @param sbus_input SBUS通道值（已经过死区处理）
 * @return PWM脉宽值（微秒，0-1000μs）
 * @note 映射算法：
 *       1. 零值映射为0μs（风扇停止）
 *       2. 负值（向下拉摇杆）线性映射到0-1000μs范围
 *       3. 正值暂不使用，映射为0μs
 *       4. 超过最大值时自动限制为1000μs
 */
uint32_t SuctionController::MapSuctionToPulseWidth(int16_t sbus_input) {
  // 如果输入值为0，返回0μs（风扇停止）
  if (sbus_input == 0) {
    return 0;
  }

  // 只有负值（向下拉摇杆）才启动风机
  if (sbus_input > 0) {
    return 0;  // 正值不启动风机
  }

  // 处理负值：将-824到0映射到1000μs到0μs
  // 取绝对值进行计算
  uint16_t abs_input = static_cast<uint16_t>(-sbus_input);

  // 将绝对值标准化到[0, 1]范围，使用824*2作为最大值
  float normalized_input = static_cast<float>(abs_input) / 1648.0f;

  // 限制标准化值不超过1.0，防止溢出
  if (normalized_input > 1.0f) normalized_input = 1.0f;

  // 线性映射到PWM脉宽范围[0, 1000μs]
  uint32_t pulse_width = static_cast<uint32_t>(normalized_input * FAN_MAX_PULSE_WIDTH);

  // 确保不超过最大值
  if (pulse_width > FAN_MAX_PULSE_WIDTH) pulse_width = FAN_MAX_PULSE_WIDTH;

  return pulse_width;
}