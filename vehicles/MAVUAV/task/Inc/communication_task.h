#pragma once

#include <memory>

#include "bsp_usb.h"
#include "cmsis_os.h"
#include "imu_task.h"
#include "minipc_protocol.h"
#include "selftest_task.h"
#include "system_state_task.h"
#include "task_config.h"
#include "vehicle_task.h"

//==================================================================================================
// 通信控制类
// 功能描述：
// 1. 管理与上位机（MiniPC）的USB通信
// 2. 处理运动控制指令和自检指令
// 3. 周期性发送IMU数据、里程计数据和系统状态
// 4. 实现通信协议的解析和封装
// 5. 监控通信连接状态和超时检测
//
// 通信协议：
// - 运动控制指令：linear_vel, angular_vel, emergency_stop
// - 自检指令：ping, echo, status查询, 统计复位
// - 数据上报：IMU数据(10ms), 里程计数据(20ms), 系统状态(200ms)
//==================================================================================================

class CommunicationController {
public:
    static constexpr uint32_t TASK_DELAY_MS = COMMUNICATION_TASK_DELAY_MS;  // 任务循环延时
    static constexpr uint32_t USB_RX_SIGNAL = (1 << 0);                    // USB接收信号标志

    /**
     * @brief 自定义USB通信类
     * @note 继承自bsp::VirtualUSB，添加了任务通知机制
     */
    class CustomCommUSB : public bsp::VirtualUSB {
    protected:
        /**
         * @brief USB接收完成回调函数
         * @note 当USB接收到数据时，向任务发送信号进行异步处理
         */
        void RxCompleteCallback() override final {
            if (task_handle_) {
                osThreadFlagsSet(task_handle_, USB_RX_SIGNAL);
            }
        }

    public:
        /**
         * @brief 设置任务句柄用于信号通知
         * @param handle 通信任务的线程句柄
         */
        void SetTaskHandle(osThreadId_t handle) {
            task_handle_ = handle;
        }

    private:
        osThreadId_t task_handle_ = nullptr;  // 任务句柄
    };

    CommunicationController() = default;

    /**
     * @brief 初始化通信控制器
     * @param state_manager 系统状态管理器指针
     * @param vehicle_ctrl 车辆控制器指针
     * @param imu_ctrl IMU控制器指针
     * @param self_test 自检控制器指针
     */
    void Init(SystemStateManager* state_manager,
              VehicleController* vehicle_ctrl,
              IMUController* imu_ctrl,
              SelfTestController* self_test);

    /**
     * @brief 设置任务句柄
     * @param handle 通信任务的线程句柄
     */
    void SetTaskHandle(osThreadId_t handle);

    /**
     * @brief 主执行函数，处理通信和数据传输
     * @note 执行流程：
     *       1. 检查USB接收数据并解析协议
     *       2. 处理运动控制和自检指令
     *       3. 周期性发送IMU、里程计和状态数据
     *       4. 更新计算机连接状态
     */
    void Execute();

private:
    // 通信超时和发送周期常量
    static constexpr uint32_t PC_TIMEOUT_MS = 2000;         // 上位机超时时间 (ms)
    static constexpr uint32_t IMU_SEND_PERIOD = 10;         // IMU数据发送周期 (ms)
    static constexpr uint32_t ODOM_SEND_PERIOD = 20;        // 里程计数据发送周期 (ms)
    static constexpr uint32_t STATUS_SEND_PERIOD = 200;     // 状态数据发送周期 (ms)

    // 控制器指针
    SystemStateManager* state_manager_ = nullptr;           // 系统状态管理器
    VehicleController* vehicle_ctrl_ = nullptr;             // 车辆控制器
    IMUController* imu_ctrl_ = nullptr;                     // IMU控制器
    SelfTestController* self_test_ = nullptr;               // 自检控制器

    // 通信对象
    std::unique_ptr<CustomCommUSB> comm_usb_;               // USB通信对象
    std::unique_ptr<communication::MinipcPort> minipc_session_;  // 通信协议会话

    // 时间戳记录
    uint32_t last_pc_data_time_ = 0;                        // 上次接收上位机数据时间
    uint32_t last_imu_send_ = 0;                            // 上次发送IMU数据时间
    uint32_t last_odom_send_ = 0;                           // 上次发送里程计数据时间
    uint32_t last_status_send_ = 0;                         // 上次发送状态数据时间

    /**
     * @brief 自检统计结构体
     * @note 用于监控通信质量和连接状态
     */
    struct {
        uint32_t tx_count = 0;          // 发送数据包计数
        uint32_t rx_count = 0;          // 接收数据包计数
        uint32_t error_count = 0;       // 错误计数
        uint32_t last_ping_time = 0;    // 最后一次ping时间
        uint8_t last_ping_seq = 0;      // 最后一次ping序列号
    } selfcheck_stats_;

    /**
     * @brief 处理运动控制指令
     * @param status 接收到的状态数据指针
     * @note 解析线速度、角速度指令，处理紧急停止
     */
    void HandleMotionCommand(const communication::status_data_t* status);

    /**
     * @brief 处理自检指令
     * @param status 接收到的状态数据指针
     * @note 处理ping、echo、状态查询、统计复位等指令
     */
    void HandleSelfcheckCommand(const communication::status_data_t* status);

    /**
     * @brief 获取自检状态值
     * @param query_type 查询类型
     * @return 对应的状态值
     * @note 查询类型：0=rx计数, 1=运行时间, 2=错误计数, 3=tx计数, 4=ping延时
     */
    uint8_t GetSelfcheckStatusValue(uint8_t query_type);

    /**
     * @brief 执行自检统计复位
     * @param reset_type 复位类型
     * @return 复位结果：0=成功, 1=失败
     * @note 复位类型：0=tx/rx计数, 1=错误计数, 2=全部统计
     */
    uint8_t ExecuteSelfcheckReset(uint8_t reset_type);

    /**
     * @brief 发送自检响应数据
     * @param response_data 响应数据指针
     */
    void SendSelfcheckResponse(const communication::selfcheck_data_t* response_data);

    /**
     * @brief 发送IMU数据
     * @note 包含加速度、角速度、欧拉角、温度等信息
     */
    void SendIMUData();

    /**
     * @brief 发送里程计数据
     * @note 包含编码器计数、轮速、线速度、角速度等信息
     */
    void SendOdometryData();

    /**
     * @brief 发送系统状态数据
     * @note 包含控制模式、变形模式、设备连接状态等信息
     */
    void SendSystemStatus();
};