#include "cmsis_os.h"
#include "selftest_task.h"

/**
 * @brief 初始化自检控制器
 * @param hi2c I2C总线句柄，用于OLED通信
 * @param htim PWM定时器句柄，用于蜂鸣器控制
 * @param vehicle_ctrl 车辆控制器指针，用于检测车轮电机状态
 * @param transform_ctrl 变形控制器指针，用于检测变形电机状态
 * @param sbus SBUS遥控器指针，用于检测遥控器连接状态
 * @param state_manager 系统状态管理器指针，用于获取系统模式信息
 * @param imu_ctrl IMU控制器指针，用于获取姿态数据
 */
void SelfTestController::Init(I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef* htim,
                              VehicleController* vehicle_ctrl,
                              TransformController* transform_ctrl,
                              remote::SBUS* sbus,
                              SystemStateManager* state_manager,
                              IMUController* imu_ctrl) {
    // 保存各个控制器的指针，用于后续状态查询
    vehicle_ctrl_ = vehicle_ctrl;
    transform_ctrl_ = transform_ctrl;
    sbus_ = sbus;
    state_manager_ = state_manager;
    imu_ctrl_ = imu_ctrl;

    // 初始化OLED显示屏
    // 参数：I2C句柄，设备地址0x3C（标准OLED地址）
    OLED_ = std::make_unique<display::OLED>(hi2c, 0x3C);

    // 初始化蜂鸣器
    // 参数：PWM定时器，通道3，时钟频率1MHz
    buzzer_ = std::make_unique<bsp::Buzzer>(htim, 3, 1000000);
}

/**
 * @brief 自检控制器主执行函数
 * @note 执行流程：
 *       1. 首次运行时播放启动序列（只执行一次）
 *       2. 周期性更新设备连接状态
 *       3. 刷新OLED显示内容
 *       4. 任务延时控制更新频率
 */
void SelfTestController::Execute() {
    // 检查是否已播放启动序列，确保只播放一次
    if (!startup_sequence_played_) {
        ShowStartupSequence();           // 播放启动序列
        startup_sequence_played_ = true; // 设置标志，防止重复播放
    }

    // 更新各设备的连接状态
    UpdateDeviceStatus();

    // 更新OLED显示内容
    UpdateOLEDDisplay();

    // 任务延时，控制更新频率
    osDelay(TASK_DELAY_MS);
}

/**
 * @brief 设置上位机连接状态
 * @param connected true=上位机已连接，false=上位机未连接
 * @note 此函数由通信控制器调用，用于实时更新上位机连接状态
 */
void SelfTestController::SetComputerStatus(bool connected) {
    device_status_.computer = connected;
}

/**
 * @brief 显示启动序列
 * @note 启动序列包括：
 *       1. 显示LOGO
 *       2. 播放音乐
 *       3. 清屏并设置显示标签
 *       4. 记录系统启动时间
 */
void SelfTestController::ShowStartupSequence() {
    // 检查硬件是否已初始化
    if (!OLED_ || !buzzer_) return;

    // 显示CAT
    OLED_->DrawCat();

    // 定义音乐音符序列
    using Note = bsp::BuzzerNote;
  static bsp::BuzzerNoteDelayed PowerOn[] = {
    {Note::Do1M, 200}, {Note::Silent, 50},
   {Note::Mi3M, 200}, {Note::Silent, 50},
   {Note::So5M, 200}, {Note::Silent, 50},
   {Note::Do1H, 400}, {Note::Silent, 100},
   {Note::Finish, 0}
  };


    // 播放启动音乐，使用lambda函数提供延时回调
    buzzer_->SingSong(PowerOn, [](uint32_t milli) { osDelay(milli); });

    // 清除显示内容，准备显示状态信息
    OLED_->OperateGram(display::PEN_CLEAR);
    OLED_->RefreshGram();

    // 显示固定的设备标签（第0行）
    OLED_->ShowString(0, 1, (uint8_t*)"LW");   // 左轮电机标签
    OLED_->ShowString(0, 5, (uint8_t*)"RW");   // 右轮电机标签
    OLED_->ShowString(0, 9, (uint8_t*)"TF");   // 变形电机标签
    OLED_->ShowString(0, 13, (uint8_t*)"RC");  // 遥控器标签
    OLED_->ShowString(0, 17, (uint8_t*)"PC");  // 上位机标签

    // 记录系统启动时间，用于计算运行时间
    start_time_ = osKernelGetTickCount();
}

/**
 * @brief 更新设备连接状态
 * @note 检测方法：
 *       1. 首先重置所有连接标志为false
 *       2. 短暂延时等待设备响应
 *       3. 读取各设备的实际连接状态
 *       4. 更新device_status_结构体
 */
void SelfTestController::UpdateDeviceStatus() {
    // 重置所有设备的连接标志，为新一轮检测做准备
    if (auto left = vehicle_ctrl_->GetLeftMotor()) left->connection_flag_ = false;
    if (auto right = vehicle_ctrl_->GetRightMotor()) right->connection_flag_ = false;
    if (auto transform = transform_ctrl_->GetTransformMotor()) transform->connection_flag_ = false;
    if (sbus_) sbus_->connection_flag_ = false;

    // 延时等待设备响应，给设备足够时间更新连接状态
    osDelay(TASK_DELAY_MS);

    // 读取各设备的实际连接状态
    auto* left_motor = vehicle_ctrl_->GetLeftMotor();
    auto* right_motor = vehicle_ctrl_->GetRightMotor();
    auto* transform_motor = transform_ctrl_->GetTransformMotor();

    // 更新设备状态记录
    device_status_.left_motor = left_motor && left_motor->connection_flag_;
    device_status_.right_motor = right_motor && right_motor->connection_flag_;
    device_status_.transform_motor = transform_motor && transform_motor->connection_flag_;
    device_status_.sbus = sbus_ && sbus_->connection_flag_;
    // 上位机状态由SetComputerStatus()函数设置
}

/**
 * @brief 更新OLED显示内容
 * @note 显示布局：
 *       行0：设备连接状态图形指示器
 *       行1：控制模式 + 变形模式 + 运行时间
 *       行2：变形电机角度信息
 *       行3：IMU姿态角信息
 */
void SelfTestController::UpdateOLEDDisplay() {
    if (!OLED_) return;

    // 第0行：显示设备连接状态指示器（图形块）
    // ShowBlock()在连接时显示实心块，断开时显示空心块
    OLED_->ShowBlock(0, 3, device_status_.left_motor);      // 左轮电机状态
    OLED_->ShowBlock(0, 7, device_status_.right_motor);     // 右轮电机状态
    OLED_->ShowBlock(0, 11, device_status_.transform_motor); // 变形电机状态
    OLED_->ShowBlock(0, 15, device_status_.sbus);           // 遥控器状态
    OLED_->ShowBlock(0, 19, device_status_.computer);       // 上位机状态

    // 第1行：显示控制模式、变形模式和运行时间
    // 控制模式字符串映射
    const char* mode_str = state_manager_->GetCurrentControlMode() == ControlMode::EMERGENCY_STOP ? "STOP" :
                           state_manager_->GetCurrentControlMode() == ControlMode::REMOTE_CONTROL ? "RC" : "PC";
    // 变形模式字符串映射
    const char* transform_str = state_manager_->GetCurrentTransformMode() == TransformMode::VEHICLE ? "CAR" : "FLY";

    // 计算系统运行时间（小时:分钟:秒格式）
    uint32_t runtime_sec = (osKernelGetTickCount() - start_time_) / 1000;
    // uint32_t hours = runtime_sec / 3600;
    uint32_t minutes = (runtime_sec % 3600) / 60;
    uint32_t seconds = runtime_sec % 60;

    // 显示模式和运行时间信息
    OLED_->Printf(1, 1, "M:%s T:%s %02d:%02d",
                  mode_str, transform_str, minutes, seconds);

    // 第2行：显示变形电机角度信息
    if (auto motor = transform_ctrl_->GetTransformMotor(); motor && motor->connection_flag_) {
        // 电机已连接，显示角度信息（弧度转换为度，保留1位小数）
        // 573 ≈ 180/π * 10，用于弧度到度的转换并保留1位小数
        int angle_int = static_cast<int>(motor->GetTheta() * 573);
        OLED_->Printf(2, 1, "TF: %4d.%d deg", angle_int/10, abs(angle_int%10));
    } else {
        // 电机未连接，显示占位符
        OLED_->Printf(2, 1, "TF: --- deg");
    }

    // 第3行：显示IMU姿态角信息（俯仰、横滚、偏航）
    if (auto imu = imu_ctrl_->GetIMU()) {
        // IMU已连接，显示欧拉角（弧度转换为度，保留1位小数）
        int pitch_int = static_cast<int>(imu->INS_angle[1] * 573);  // 俯仰角
        int roll_int = static_cast<int>(imu->INS_angle[2] * 573);   // 横滚角
        int yaw_int = static_cast<int>(imu->INS_angle[0] * 573);    // 偏航角

        OLED_->Printf(3, 1, "P:%d.%d R:%d.%d Y:%d.%d",
                      pitch_int/10, abs(pitch_int%10),
                      roll_int/10, abs(roll_int%10),
                      yaw_int/10, abs(yaw_int%10));
    } else {
        // IMU未连接，显示占位符
        OLED_->Printf(3, 1, "P:--- R:--- Y:---");
    }

    // 刷新OLED显示缓存到屏幕
    OLED_->RefreshGram();
}