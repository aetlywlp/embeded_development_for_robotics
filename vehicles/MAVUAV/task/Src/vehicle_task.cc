#include "cmsis_os.h"
#include "vehicle_task.h"
#include <cmath>

/**
 * @brief 初始化车辆控制器
 * 创建双PID系统（正常+激进），充分利用Motor2006的10A能力
 */
void VehicleController::Init(bsp::CAN* can, SystemStateManager* state_manager, remote::SBUS* sbus) {
    can_ = can;
    state_manager_ = state_manager;
    sbus_ = sbus;

    left_motor_ = std::make_unique<control::Motor2006>(can_, 0x201);
    right_motor_ = std::make_unique<control::Motor2006>(can_, 0x202);

    // 正常PID控制器（激进版本）
    // 基础PID就使用激进参数，充分发挥Motor2006性能
    left_pid_ = std::make_unique<control::ConstrainedPID>(
        PID_PARAMS[0], PID_PARAMS[1], PID_PARAMS[2],
        NORMAL_OUTPUT_LIMIT, NORMAL_OUTPUT_LIMIT);
    right_pid_ = std::make_unique<control::ConstrainedPID>(
        PID_PARAMS[0], PID_PARAMS[1], PID_PARAMS[2],
        NORMAL_OUTPUT_LIMIT, NORMAL_OUTPUT_LIMIT);

    // 转弯增强PID控制器（超激进版本）
    // 转弯时使用接近电机极限的控制参数，突破摩擦阻力
    left_pid_aggressive_ = std::make_unique<control::ConstrainedPID>(
        PID_PARAMS_AGGRESSIVE[0], PID_PARAMS_AGGRESSIVE[1], PID_PARAMS_AGGRESSIVE[2],
        AGGRESSIVE_OUTPUT_LIMIT, AGGRESSIVE_OUTPUT_LIMIT);
    right_pid_aggressive_ = std::make_unique<control::ConstrainedPID>(
        PID_PARAMS_AGGRESSIVE[0], PID_PARAMS_AGGRESSIVE[1], PID_PARAMS_AGGRESSIVE[2],
        AGGRESSIVE_OUTPUT_LIMIT, AGGRESSIVE_OUTPUT_LIMIT);

    state_manager_->SetPIDResetCallback([this]() { ResetPIDs(); });
}

/**
 * @brief 主执行函数
 * 保持原有逻辑稳定性，只在内部优化控制策略
 */
void VehicleController::Execute() {
    // 检查系统状态
    if (state_manager_->GetCurrentTransformMode() != TransformMode::VEHICLE ||
        state_manager_->GetCurrentControlMode() != ControlMode::REMOTE_CONTROL) {
        osDelay(TASK_DELAY_MS);
        return;
    }

    // 检查SBUS连接
    if (!sbus_ || !sbus_->connection_flag_) {
        StopMotors();
        osDelay(TASK_DELAY_MS);
        return;
    }

    // 处理输入
    ProcessSBUSInputs();

    // 更新转弯模式
    UpdateTurningMode();

    // 计算轮速
    CalculateDifferentialSpeeds();

    // 执行控制
    ControlMotors();

    osDelay(TASK_DELAY_MS);
}

/**
 * @brief 紧急停止
 * 重置所有PID控制器，包括激进模式的
 */
void VehicleController::EmergencyStop() {
    if (osMutexAcquire(state_manager_->GetMotorControlMutex(), 100) == osOK) {
        if (left_motor_) left_motor_->SetOutput(0);
        if (right_motor_) right_motor_->SetOutput(0);
        SendMotorCommands();
        osMutexRelease(state_manager_->GetMotorControlMutex());

        // 重置转弯状态
        is_turning_mode_ = false;
        previous_angular_speed_ = 0.0f;
    }
}

/**
 * @brief 设置计算机控制指令
 * 原理：保持原有接口，内部使用优化的控制策略
 */
void VehicleController::SetComputerControl(float linear_vel, float angular_vel) {
    // 临时更新角速度用于转弯检测
    target_angular_speed_ = angular_vel;
    UpdateTurningMode();

    // 差分驱动运动学
    float left_wheel_vel = (linear_vel - angular_vel * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;
    float right_wheel_vel = (linear_vel + angular_vel * WHEEL_BASE / 2.0f) / WHEEL_RADIUS;

    // 转换为电机角速度
    float target_motor_speed_left = left_wheel_vel * GEAR_RATIO;
    float target_motor_speed_right = -right_wheel_vel * GEAR_RATIO;

    // 应用动态功率分配（关键优化）
    ApplyPowerRedistribution(target_motor_speed_left, target_motor_speed_right, angular_vel);

    // 速度限制
    float max_motor_speed = MAX_WHEEL_SPEED * GEAR_RATIO;
    target_motor_speed_left = clip<float>(target_motor_speed_left, -max_motor_speed, max_motor_speed);
    target_motor_speed_right = clip<float>(target_motor_speed_right, -max_motor_speed, max_motor_speed);

    if (left_motor_ && right_motor_ && left_pid_ && right_pid_) {
        float left_error = target_motor_speed_left - left_motor_->GetOmega();
        float right_error = target_motor_speed_right - right_motor_->GetOmega();

        // 根据转弯模式选择PID控制器（关键优化）
        int16_t left_output, right_output;
        if (is_turning_mode_) {
            left_output = left_pid_aggressive_->ComputeConstrainedOutput(left_error);
            right_output = right_pid_aggressive_->ComputeConstrainedOutput(right_error);
        } else {
            left_output = left_pid_->ComputeConstrainedOutput(left_error);
            right_output = right_pid_->ComputeConstrainedOutput(right_error);
        }

        if (osMutexAcquire(state_manager_->GetMotorControlMutex(), 10) == osOK) {
            left_motor_->SetOutput(left_output);
            right_motor_->SetOutput(right_output);
            SendMotorCommands();
            osMutexRelease(state_manager_->GetMotorControlMutex());
        }
    }
}

/**
 * @brief 处理SBUS输入
 * 保持原有输入处理逻辑的稳定性
 */
int16_t VehicleController::ApplyDeadzone(int16_t input, int16_t deadzone) {
    return (abs(input) < deadzone) ? 0 : input;
}

float VehicleController::NormalizeSBUS(int16_t input) {
    const float range = static_cast<float>(SBUS_MAX_VALUE - SBUS_MIN_VALUE);
    const float center = static_cast<float>(SBUS_MAX_VALUE + SBUS_MIN_VALUE) / 2.0f;
    return (static_cast<float>(input) - center) / (range / 2.0f);
}

void VehicleController::ProcessSBUSInputs() {
    int16_t throttle_raw = ApplyDeadzone(sbus_->ch[1], SBUS_DEADZONE);
    int16_t steering_raw = ApplyDeadzone(sbus_->ch[0], SBUS_DEADZONE);

    float throttle = NormalizeSBUS(throttle_raw);
    float steering = NormalizeSBUS(steering_raw);

    // 保持原有的速度范围
    target_linear_speed_ = throttle * MAX_VEHICLE_SPEED;
    target_angular_speed_ = steering * MAX_ANGULAR_SPEED;
}

/**
 * @brief 计算差分驱动轮速
 * 在基础计算后应用动态功率分配优化
 */
void VehicleController::CalculateDifferentialSpeeds() {
    // 标准差分驱动公式
    float left_wheel_linear_vel = target_linear_speed_ - target_angular_speed_ * WHEEL_BASE / 2.0f;
    float right_wheel_linear_vel = target_linear_speed_ + target_angular_speed_ * WHEEL_BASE / 2.0f;

    float left_wheel_angular_vel = left_wheel_linear_vel / WHEEL_RADIUS;
    float right_wheel_angular_vel = right_wheel_linear_vel / WHEEL_RADIUS;

    left_wheel_speed_ = left_wheel_angular_vel * GEAR_RATIO;
    right_wheel_speed_ = -right_wheel_angular_vel * GEAR_RATIO;

    // 关键优化：动态功率分配
    ApplyPowerRedistribution(left_wheel_speed_, right_wheel_speed_, target_angular_speed_);

    // 速度限制
    float max_motor_speed = MAX_WHEEL_SPEED * GEAR_RATIO;
    left_wheel_speed_ = clip<float>(left_wheel_speed_, -max_motor_speed, max_motor_speed);
    right_wheel_speed_ = clip<float>(right_wheel_speed_, -max_motor_speed, max_motor_speed);
}

/**
 * @brief 电机PID控制
 * 根据转弯模式动态选择PID控制器
 */
void VehicleController::ControlMotors() {
    if (!left_motor_ || !right_motor_ || !left_pid_ || !right_pid_) return;

    float left_error = left_wheel_speed_ - left_motor_->GetOmega();
    float right_error = right_wheel_speed_ - right_motor_->GetOmega();

    // 动态PID选择（关键优化）
    int16_t left_output, right_output;
    if (is_turning_mode_) {
        // 转弯时使用激进PID，突破摩擦阻力
        left_output = left_pid_aggressive_->ComputeConstrainedOutput(left_error);
        right_output = right_pid_aggressive_->ComputeConstrainedOutput(right_error);
    } else {
        // 直行时使用正常PID
        left_output = left_pid_->ComputeConstrainedOutput(left_error);
        right_output = right_pid_->ComputeConstrainedOutput(right_error);
    }

    if (osMutexAcquire(state_manager_->GetMotorControlMutex(), 10) == osOK) {
        left_motor_->SetOutput(left_output);
        right_motor_->SetOutput(right_output);
        SendMotorCommands();
        osMutexRelease(state_manager_->GetMotorControlMutex());
    }
}

/**
 * @brief 停止电机
 * 重置转弯状态
 */
void VehicleController::StopMotors() {
    if (left_motor_) left_motor_->SetOutput(0);
    if (right_motor_) right_motor_->SetOutput(0);
    SendMotorCommands();

    // 重置转弯状态
    is_turning_mode_ = false;
    previous_angular_speed_ = 0.0f;
}

/**
 * @brief 发送电机指令
 * 保持原有稳定的发送逻辑
 */
void VehicleController::SendMotorCommands() {
    if (left_motor_ && right_motor_ &&
        (left_motor_->connection_flag_ || right_motor_->connection_flag_)) {

        control::MotorCANBase* motors[2] = {left_motor_.get(), right_motor_.get()};
        control::MotorCANBase::TransmitOutput(motors, 2);
    }
}

/**
 * @brief 重置PID控制器
 * 重置所有PID控制器，包括激进版本
 */
void VehicleController::ResetPIDs() {
    StopMotors();

    if (left_pid_) left_pid_->Reset();
    if (right_pid_) right_pid_->Reset();
    if (left_pid_aggressive_) left_pid_aggressive_->Reset();
    if (right_pid_aggressive_) right_pid_aggressive_->Reset();

    // 重置转弯状态
    is_turning_mode_ = false;
    previous_angular_speed_ = 0.0f;
    turning_start_time_ = 0;
}

// ============ 新增优化函数实现 ============

/**
 * @brief 更新转弯模式
 * 智能检测转弯意图，动态切换控制策略
 */
void VehicleController::UpdateTurningMode() {
    uint32_t current_time = GetCurrentTimeMs();
    float angular_speed_abs = std::abs(target_angular_speed_);

    // 检测转弯开始
    if (!is_turning_mode_ && angular_speed_abs > TURN_THRESHOLD) {
        is_turning_mode_ = true;
        turning_start_time_ = current_time;
    }

    // 检测转弯结束（角速度降低或超时）
    if (is_turning_mode_) {
        bool angular_speed_low = angular_speed_abs < (TURN_THRESHOLD * 0.3f);
        bool timeout = (current_time - turning_start_time_) > TURNING_MODE_DURATION;

        if (angular_speed_low || timeout) {
            is_turning_mode_ = false;
        }
    }

    previous_angular_speed_ = target_angular_speed_;
}

// 功率重分配逻辑
void VehicleController::ApplyPowerRedistribution(float& left_speed, float& right_speed, float angular_speed) {
  if (std::abs(angular_speed) < 1.0f) return;

  float turn_intensity = std::abs(angular_speed) / MAX_ANGULAR_SPEED;
  turn_intensity = clip<float>(turn_intensity, 0.0f, 1.0f);

  if (angular_speed > 0) {
    // 左转：增强外侧轮（右轮）功率
    right_speed *= (1.0f + turn_intensity * (POWER_REDISTRIBUTION_FACTOR - 1.0f));
    left_speed *= (1.0f + turn_intensity * (POWER_REDISTRIBUTION_FACTOR - 1.0f));
  } else {
    // 右转：增强外侧轮（左轮）功率
    left_speed *= (1.0f + turn_intensity * (POWER_REDISTRIBUTION_FACTOR - 1.0f));
    right_speed *= (1.0f + turn_intensity * (POWER_REDISTRIBUTION_FACTOR - 1.0f));
  }
}

/**
 * @brief 获取系统时间
 * 为转弯模式计时提供时间基准
 */
uint32_t VehicleController::GetCurrentTimeMs() {
    return osKernelGetTickCount();
}