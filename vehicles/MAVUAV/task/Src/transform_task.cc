// transform_task.cpp
#include "cmsis_os.h"
#include "main.h"
#include "transform_task.h"

/**
 * @brief 初始化变形控制器
 * @param can CAN总线接口指针
 * @param state_manager 系统状态管理器指针
 * @note 初始化变形电机、电推杆GPIO等硬件组件
 */
void TransformController::Init(bsp::CAN* can, SystemStateManager* state_manager, remote::SBUS* sbus) {
    can_ = can;
    state_manager_ = state_manager;
    sbus_ = sbus;

    // 初始化变形电机（CAN ID 0x203）
    transform_motor_ = std::make_unique<control::Motor3508>(can_, 0x203);

    // 初始化电机约束PID控制器
    motor_pid_ = std::make_unique<control::ConstrainedPID>();
    float pid_params[3] = {PID_KP, PID_KI, PID_KD};
    motor_pid_->Reinit(pid_params, PID_MAX_IOUT, PID_MAX_OUT);

    // 初始化电推杆GPIO控制（GPIOI Pin 7）
    actuator_gpio_ = std::make_unique<bsp::GPIO>(GPIOI, GPIO_PIN_7);
    actuator_gpio_->High();  // 默认设置为高电平
}

/**
 * @brief 主执行函数，变形控制的核心循环
 * @note 执行顺序：
 *       1. 获取遥控器数据
 *       2. 控制电推杆
 *       3. 控制3508电机
 *       4. 发送电机控制指令
 */
void TransformController::Execute() {
    if (!sbus_) {
        osDelay(TASK_DELAY_MS);
        return;
    }

    // 1. 控制电推杆 (ch[6])
    ControlActuator(sbus_->ch[6]);

    // 2. 控制3508电机 (ch[7])
    ControlMotor(sbus_->ch[7]);

    // 3. 发送电机控制指令
    if (transform_motor_ && transform_motor_->connection_flag_) {
        control::MotorCANBase* motors[1] = {transform_motor_.get()};
        control::MotorCANBase::TransmitOutput(motors, 1);
    }

    // 任务延时
    osDelay(TASK_DELAY_MS);
}

/**
 * @brief 控制电推杆
 * @param rc_value 遥控器ch[6]值
 * @note 高电平(>400)伸出，低电平(<-400)收回，中间位置保持
 */
void TransformController::ControlActuator(int16_t rc_value) {
    if (!actuator_gpio_) return;

    if (rc_value > RC_HIGH_THRESHOLD) {
        // 遥控器大于400，发送高电平，电推杆伸出
        actuator_gpio_->High();
    } else if (rc_value < RC_LOW_THRESHOLD) {
        // 遥控器小于-400，发送低电平，电推杆收回
        actuator_gpio_->Low();
    }
    // 中间位置保持当前状态
}

/**
 * @brief 控制3508电机
 * @param rc_value 遥控器ch[7]值
 * @note 高电平(>400)正转，低电平(<-400)反转，中位停止
 */
void TransformController::ControlMotor(int16_t rc_value) {
  if (!transform_motor_ || !motor_pid_) return;

  // 根据遥控器输入设置目标速度
  if (rc_value > RC_HIGH_THRESHOLD) {
    // 遥控器大于400，电机正转
    target_speed_ = TRANSFORM_MOTOR_SPEED;
  } else if (rc_value < RC_LOW_THRESHOLD) {
    // 遥控器小于-400，电机反转
    target_speed_ = -TRANSFORM_MOTOR_SPEED;
  } else {
    // 中位，电机停止
    target_speed_ = 0.0f;
    // 停止时重置PID，避免积分累积
    motor_pid_->Reset();
    transform_motor_->SetOutput(0);
    return;
  }

  // 使用约束PID控制器计算输出
  float speed_diff = transform_motor_->GetOmegaDelta(target_speed_);
  int16_t motor_output = (int16_t)motor_pid_->ComputeConstrainedOutput(speed_diff);

  // 添加最小输出补偿（克服静摩擦）
  if (motor_output != 0) {
    if (motor_output > 0 && motor_output < MIN_OUTPUT_THRESHOLD) {
      motor_output = MIN_OUTPUT_THRESHOLD;
    } else if (motor_output < 0 && motor_output > -MIN_OUTPUT_THRESHOLD) {
      motor_output = -MIN_OUTPUT_THRESHOLD;
    }
  }

  // 最终安全限制
  motor_output = control::ClipMotorRange(motor_output);
  transform_motor_->SetOutput(motor_output);
}