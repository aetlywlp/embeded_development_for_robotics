#include "main.h"
#include "system_state_task.h"

/**
 * @brief 初始化系统状态管理器
 * @param sbus_ptr SBUS遥控器接收器指针
 * @note 创建电机控制互斥锁，如果创建失败则进入死循环
 */
void SystemStateManager::Init(remote::SBUS* sbus_ptr) {
    // 保存SBUS指针用于后续模式检测
    sbus_ = sbus_ptr;

    // 创建电机控制互斥锁，确保多任务间电机控制的线程安全
    motor_control_mutex_ = osMutexNew(nullptr);
    if (motor_control_mutex_ == nullptr) {
        // 互斥锁创建失败，系统无法正常工作，进入死循环
        // 这是一个严重的系统错误，需要硬件复位
        while(1) {
            HAL_Delay(100);  // 延时后继续循环，避免CPU过载
        }
    }
}

/**
 * @brief 主要的状态更新函数，解析SBUS输入并更新系统模式
 * @note 这个函数应该在主循环中周期性调用，通常每10-20ms调用一次
 */
void SystemStateManager::UpdateControlModes() {
    // 解析SBUS输入确定新地控制模式和变形模式
    current_control_mode_ = DetermineControlMode();
    current_transform_mode_ = DetermineTransformMode();

    // 检测控制模式是否发生变化
    if (current_control_mode_ != previous_control_mode_) {
        // 模式发生切换，执行相应的切换处理
        HandleModeSwitch();
        // 更新上一次模式记录
        previous_control_mode_ = current_control_mode_;
    }

    // 检测变形模式是否发生变化
    if (current_transform_mode_ != previous_transform_mode_) {
        // 变形模式发生切换，更新上一次模式记录
        // 注意：变形模式的具体切换逻辑在TransformController中处理
        previous_transform_mode_ = current_transform_mode_;
    }
}

/**
 * @brief 检查系统是否处于紧急停止状态
 * @return true：紧急停止状态，false：正常运行状态
 * @note 紧急停止状态下所有电机都应该停止运行
 */
bool SystemStateManager::IsInEmergencyStop() const {
    return current_control_mode_ == ControlMode::EMERGENCY_STOP;
}

/**
 * @brief 设置PID控制器重置回调函数
 * @param callback 回调函数，在模式切换时调用以重置PID控制器
 * @note 主要用于VehicleController注册其PID重置函数
 */
void SystemStateManager::SetPIDResetCallback(std::function<void()> callback) {
    pid_reset_callback_ = callback;
}

/**
 * @brief 根据SBUS通道6的值确定控制模式
 * @return 解析出的控制模式
 * @note 控制模式优先级：
 *       1. SBUS连接检查（无连接 -> 紧急停止）
 *       2. SBUS超时检查（超时 -> 紧急停止）
 *       3. 通道值映射（-400以下 -> 紧急停止，400以上 -> 计算机控制，中间 -> 遥控）
 */
ControlMode SystemStateManager::DetermineControlMode() {
    // 首先检查SBUS连接状态
    if (!sbus_ || !sbus_->connection_flag_) {
        return ControlMode::EMERGENCY_STOP;
    }

    // 检查SBUS数据是否超时
    uint32_t current_time = HAL_GetTick();
    if (current_time - sbus_->timestamp > SBUS_TIMEOUT_MS) {
        return ControlMode::EMERGENCY_STOP;
    }

    // 读取模式切换通道（通道5，索引4）
    int16_t mode_channel = sbus_->ch[4];

    // 根据通道值确定控制模式
    if (mode_channel < MODE_LOWER_THRESHOLD) {
        // 通道值小于-400，进入紧急停止模式
        return ControlMode::EMERGENCY_STOP;
    } else if (mode_channel > MODE_UPPER_THRESHOLD) {
        // 通道值大于400，进入计算机控制模式
        return ControlMode::COMPUTER_CONTROL;
    } else {
        // 通道值在-400到400之间，进入遥控模式
        return ControlMode::REMOTE_CONTROL;
    }
}

/**
 * @brief 根据SBUS通道4的值确定变形模式
 * @return 解析出的变形模式
 * @note 变形模式映射：
 *       <= 0: 车辆模式（地面行驶）
 *       > 0: 飞行模式（空中飞行）
 */
TransformMode SystemStateManager::DetermineTransformMode() {
    // 检查SBUS连接状态，无连接时默认为车辆模式
    if (!sbus_ || !sbus_->connection_flag_) {
        return TransformMode::VEHICLE;
    }

    // 读取变形切换通道（通道6，索引5）
    int16_t transform_channel = sbus_->ch[5];

    // 根据通道值确定变形模式
    return (transform_channel > 0) ? TransformMode::FLIGHT : TransformMode::VEHICLE;
}

/**
 * @brief 处理模式切换时的相关操作
 * @note 主要处理以下切换场景：
 *       1. 切换到紧急停止模式：立即停止所有电机并重置PID控制器
 *       2. 其他模式切换：由各自的控制器处理
 */
void SystemStateManager::HandleModeSwitch() {
    if (current_control_mode_ == ControlMode::EMERGENCY_STOP) {
        // 进入紧急停止模式：立即停止所有电机并重置PID
        if (pid_reset_callback_) {
            // 调用注册的PID重置回调函数
            // 这会重置车辆控制器的PID积分项，防止积分饱和
            pid_reset_callback_();
        }
    }
    // 注意：其他模式的具体切换逻辑在各自的控制器中处理
    // 例如：VehicleController处理遥控/计算机控制切换
    //      TransformController处理变形模式切换
}