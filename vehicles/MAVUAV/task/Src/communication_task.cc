#include "cmsis_os.h"
#include "communication_task.h"

/**
 * @brief 初始化通信控制器
 * @param state_manager 系统状态管理器指针
 * @param vehicle_ctrl 车辆控制器指针
 * @param imu_ctrl IMU控制器指针
 * @param self_test 自检控制器指针
 * @note 设置控制器指针，初始化USB通信和协议会话
 */
void CommunicationController::Init(SystemStateManager* state_manager,
                                   VehicleController* vehicle_ctrl,
                                   IMUController* imu_ctrl,
                                   SelfTestController* self_test) {
    // 保存控制器指针
    state_manager_ = state_manager;
    vehicle_ctrl_ = vehicle_ctrl;
    imu_ctrl_ = imu_ctrl;
    self_test_ = self_test;

    // 初始化USB通信对象
    comm_usb_ = std::make_unique<CustomCommUSB>();
    comm_usb_->SetupRx(512);    // 设置接收缓冲区大小为512字节
    comm_usb_->SetupTx(512);    // 设置发送缓冲区大小为512字节

    // 初始化MiniPC通信协议会话
    minipc_session_ = std::make_unique<communication::MinipcPort>();

    // 初始化时间戳
    last_pc_data_time_ = 0;
}

/**
 * @brief 设置任务句柄用于USB接收中断通知
 * @param handle 通信任务的线程句柄
 */
void CommunicationController::SetTaskHandle(osThreadId_t handle) {
    if (comm_usb_) {
        comm_usb_->SetTaskHandle(handle);
    }
}

/**
 * @brief 主执行函数，处理所有通信任务
 * @note 执行流程：
 *       1. 等待USB接收信号并处理接收数据
 *       2. 解析通信协议并执行相应指令
 *       3. 更新计算机连接状态
 *       4. 周期性发送各类数据
 */
void CommunicationController::Execute() {
    uint32_t current_time = osKernelGetTickCount();

    // 处理USB接收数据
    uint8_t* data;
    uint32_t length;

    // 检查是否有USB接收信号，非阻塞等待
    uint32_t flags = osThreadFlagsWait(USB_RX_SIGNAL, osFlagsWaitAny, 0);
    if (flags & USB_RX_SIGNAL && comm_usb_) {
        // 读取USB接收到的数据
        length = comm_usb_->Read(&data);
        if (length > 0 && minipc_session_) {
            // 更新最后接收数据时间
            last_pc_data_time_ = current_time;

            // 解析接收到的数据包
            minipc_session_->ParseUartBuffer(data, length);

            // 检查是否解析到有效数据包
            if (minipc_session_->GetValidFlag()) {
                uint8_t cmd_id = minipc_session_->GetCmdId();
                const communication::status_data_t* status = minipc_session_->GetStatus();

                // 根据指令ID执行相应处理
                switch (cmd_id) {
                    case communication::MOTION_CMD_ID:
                        // 处理运动控制指令
                        HandleMotionCommand(status);
                        break;
                    case communication::SELFCHECK_CMD_ID:
                        // 处理自检指令
                        HandleSelfcheckCommand(status);
                        break;
                    default:
                        // 未知指令，忽略
                        break;
                }
            }
        }
    }

    // 更新计算机连接状态
    // 如果超过PC_TIMEOUT_MS没有收到数据，认为连接断开
    bool computer_connected = (current_time - last_pc_data_time_) < PC_TIMEOUT_MS;
    if (self_test_) {
        self_test_->SetComputerStatus(computer_connected);
    }

    // 周期性发送数据
    // IMU数据：10ms周期
    if (current_time - last_imu_send_ >= IMU_SEND_PERIOD) {
        SendIMUData();
        last_imu_send_ = current_time;
    }

    // 里程计数据：20ms周期
    if (current_time - last_odom_send_ >= ODOM_SEND_PERIOD) {
        SendOdometryData();
        last_odom_send_ = current_time;
    }

    // 系统状态数据：200ms周期
    if (current_time - last_status_send_ >= STATUS_SEND_PERIOD) {
        SendSystemStatus();
        last_status_send_ = current_time;
    }

    // 任务延时
    osDelay(TASK_DELAY_MS);
}

/**
 * @brief 处理运动控制指令
 * @param status 接收到的状态数据指针
 * @note 仅在计算机控制模式下处理运动指令
 */
void CommunicationController::HandleMotionCommand(const communication::status_data_t* status) {
    // 检查数据有效性和控制模式
    if (!status || state_manager_->GetCurrentControlMode() != ControlMode::COMPUTER_CONTROL) return;

    // 检查紧急停止标志
    if (status->emergency_stop) {
        vehicle_ctrl_->EmergencyStop();
        return;
    }

    // 设置车辆控制指令（线速度和角速度）
    vehicle_ctrl_->SetComputerControl(status->target_linear_vel, status->target_angular_vel);
}

/**
 * @brief 处理自检指令
 * @param status 接收到的状态数据指针
 * @note 支持ping、echo、状态查询、统计复位等功能
 */
void CommunicationController::HandleSelfcheckCommand(const communication::status_data_t* status) {
    if (!status) return;

    // 增加接收计数
    selfcheck_stats_.rx_count++;

    communication::selfcheck_data_t response;
    bool send_response = true;

    // 根据自检模式执行相应操作
    switch (status->mode) {
        case 0: // PING模式
            response.mode = 0;
            response.debug_int = status->debug_int;     // 回显ping序列号
            selfcheck_stats_.last_ping_time = osKernelGetTickCount();
            selfcheck_stats_.last_ping_seq = status->debug_int;
            break;

        case 1: // ECHO模式
            response.mode = 1;
            response.debug_int = status->debug_int;     // 回显接收到的数据
            break;

        case 2: // STATUS查询模式
            response.mode = 2;
            response.debug_int = GetSelfcheckStatusValue(status->debug_int);
            break;

        case 3: // RESET模式
            response.mode = 3;
            response.debug_int = ExecuteSelfcheckReset(status->debug_int);
            break;

        default:
            // 未知模式，增加错误计数，不发送响应
            selfcheck_stats_.error_count++;
            send_response = false;
            break;
    }

    // 发送响应数据
    if (send_response) {
        SendSelfcheckResponse(&response);
    }
}

/**
 * @brief 获取自检状态值
 * @param query_type 查询类型
 * @return 对应的状态值
 */
uint8_t CommunicationController::GetSelfcheckStatusValue(uint8_t query_type) {
    switch (query_type) {
        case 0: return static_cast<uint8_t>(selfcheck_stats_.rx_count & 0xFF);          // 接收计数
        case 1: return static_cast<uint8_t>((osKernelGetTickCount() / 1000) & 0xFF);    // 运行时间（秒）
        case 2: return static_cast<uint8_t>(selfcheck_stats_.error_count & 0xFF);       // 错误计数
        case 3: return static_cast<uint8_t>(selfcheck_stats_.tx_count & 0xFF);          // 发送计数
        case 4:     // ping延时
            if (selfcheck_stats_.last_ping_time > 0) {
                uint32_t delay = osKernelGetTickCount() - selfcheck_stats_.last_ping_time;
                return static_cast<uint8_t>(delay > 255 ? 255 : delay);
            }
            return 255;     // 无ping记录
        default: return 0xFF;   // 无效查询类型
    }
}

/**
 * @brief 执行自检统计复位
 * @param reset_type 复位类型
 * @return 复位结果：0=成功, 1=失败
 */
uint8_t CommunicationController::ExecuteSelfcheckReset(uint8_t reset_type) {
    switch (reset_type) {
        case 0:     // 复位发送和接收计数
            selfcheck_stats_.tx_count = 0;
            selfcheck_stats_.rx_count = 0;
            return 0;
        case 1:     // 复位错误计数
            selfcheck_stats_.error_count = 0;
            return 0;
        case 2:     // 复位所有统计
            selfcheck_stats_ = {};
            return 0;
        default:    // 无效复位类型
            return 1;
    }
}

/**
 * @brief 发送自检响应数据
 * @param response_data 响应数据指针
 */
void CommunicationController::SendSelfcheckResponse(const communication::selfcheck_data_t* response_data) {
    if (!response_data || !minipc_session_ || !comm_usb_) return;

    // 封装数据包
    uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
    minipc_session_->Pack(packet, (void*)response_data, communication::SELFCHECK_CMD_ID);

    // 发送数据包
    comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::SELFCHECK_CMD_ID));

    // 增加发送计数
    selfcheck_stats_.tx_count++;
}

/**
 * @brief 发送IMU数据
 * @note 包含三轴加速度、角速度、欧拉角、温度和时间戳
 */
void CommunicationController::SendIMUData() {
    if (!imu_ctrl_ || !imu_ctrl_->GetIMU() || !minipc_session_ || !comm_usb_) return;

    communication::imu_data_t imu_data;
    auto imu = imu_ctrl_->GetIMU();

    // 获取加速度和角速度数据
    const float* accel_data = imu->GetAccel();
    const float* gyro_data = imu->GetGyro();

    // 填充IMU数据结构
    imu_data.accel_x = accel_data[0];       // X轴加速度
    imu_data.accel_y = accel_data[1];       // Y轴加速度
    imu_data.accel_z = accel_data[2];       // Z轴加速度
    imu_data.gyro_x = gyro_data[0];         // X轴角速度
    imu_data.gyro_y = gyro_data[1];         // Y轴角速度
    imu_data.gyro_z = gyro_data[2];         // Z轴角速度
    imu_data.pitch = imu->INS_angle[1];     // 俯仰角
    imu_data.roll = imu->INS_angle[2];      // 横滚角
    imu_data.yaw = imu->INS_angle[0];       // 偏航角
    imu_data.temperature = imu->Temp;       // 温度
    imu_data.timestamp = osKernelGetTickCount();   // 时间戳

    // 封装并发送数据包
    uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
    minipc_session_->Pack(packet, &imu_data, communication::IMU_CMD_ID);
    comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::IMU_CMD_ID));
}

/**
 * @brief 发送里程计数据
 * @note 包含编码器计数、轮速、线速度、角速度、机器人参数等
 */
void CommunicationController::SendOdometryData() {
    auto left_motor = vehicle_ctrl_->GetLeftMotor();
    auto right_motor = vehicle_ctrl_->GetRightMotor();

    if (!left_motor || !right_motor || !minipc_session_ || !comm_usb_) return;

    communication::odometry_data_t odom_data;

    // 机器人物理参数
  // todo
    static constexpr float WHEEL_RADIUS = 0.076f;   // 轮子半径 (m)
    static constexpr float WHEEL_BASE = 0.335f;     // 轮距 (m)

    // 获取当前轮速
    float left_wheel_speed = left_motor->GetOmega();    // 左轮角速度 (rad/s)
    float right_wheel_speed = right_motor->GetOmega();  // 右轮角速度 (rad/s)

    // 差分驱动运动学正解：计算机器人线速度和角速度
    float linear_vel = (left_wheel_speed + right_wheel_speed) * WHEEL_RADIUS / 2.0f;
    float angular_vel = (right_wheel_speed - left_wheel_speed) * WHEEL_RADIUS / WHEEL_BASE;

    // 计算累积编码器计数（用于位置估计）
    static int32_t left_encoder_total = 0;
    static int32_t right_encoder_total = 0;
    // 假设控制周期为50ms (0.05s)，编码器分辨率为8192 CPR
    // todo
    left_encoder_total += static_cast<int32_t>(left_wheel_speed * 0.05f * 8192 / (2 * 3.14159f));
    right_encoder_total += static_cast<int32_t>(right_wheel_speed * 0.05f * 8192 / (2 * 3.14159f));

    // 填充里程计数据结构
    odom_data.left_encoder = left_encoder_total;        // 左轮编码器累积计数
    odom_data.right_encoder = right_encoder_total;      // 右轮编码器累积计数
    odom_data.left_wheel_speed = left_wheel_speed;      // 左轮速度
    odom_data.right_wheel_speed = right_wheel_speed;    // 右轮速度
    odom_data.linear_velocity = linear_vel;             // 机器人线速度
    odom_data.angular_velocity = angular_vel;           // 机器人角速度
    odom_data.wheel_base = WHEEL_BASE;                  // 轮距
    odom_data.wheel_radius = WHEEL_RADIUS;              // 轮半径
    odom_data.timestamp = osKernelGetTickCount();       // 时间戳

    // 封装并发送数据包
    uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
    minipc_session_->Pack(packet, &odom_data, communication::ODOMETRY_CMD_ID);
    comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::ODOMETRY_CMD_ID));
}

/**
 * @brief 发送系统状态数据
 * @note 包含控制模式、变形模式、设备连接状态等系统信息
 */
void CommunicationController::SendSystemStatus() {
    if (!minipc_session_ || !comm_usb_) return;

    communication::system_status_t status;

    // 填充系统状态信息
    status.robot_mode = static_cast<uint8_t>(state_manager_->GetCurrentControlMode());      // 机器人控制模式
    status.transform_mode = static_cast<uint8_t>(state_manager_->GetCurrentTransformMode());// 变形模式

    // 设备连接状态检查
    status.sbus_connected = (vehicle_ctrl_->GetLeftMotor() && vehicle_ctrl_->GetLeftMotor()->connection_flag_) ? 1 : 0;
    status.imu_connected = imu_ctrl_->GetIMU() ? 1 : 0;
    status.vl_motor_online = (vehicle_ctrl_->GetLeftMotor() && vehicle_ctrl_->GetLeftMotor()->connection_flag_) ? 1 : 0;
    status.vr_motor_online = (vehicle_ctrl_->GetRightMotor() && vehicle_ctrl_->GetRightMotor()->connection_flag_) ? 1 : 0;
    status.tf_motor_online = 0; // 变形电机状态检查暂未实现
    status.reserved = 0;        // 保留字段
    status.timestamp = osKernelGetTickCount();  // 时间戳

    // 封装并发送数据包
    uint8_t packet[minipc_session_->MAX_PACKET_LENGTH];
    minipc_session_->Pack(packet, &status, communication::SYSTEM_STATUS_CMD_ID);
    comm_usb_->Write(packet, minipc_session_->GetPacketLen(communication::SYSTEM_STATUS_CMD_ID));
}