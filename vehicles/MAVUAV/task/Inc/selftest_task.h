#pragma once

#include <memory>

#include "bsp_buzzer.h"
#include "i2c.h"
#include "imu_task.h"
#include "main.h"
#include "oled.h"
#include "sbus.h"
#include "system_state_task.h"
#include "task_config.h"
#include "transform_task.h"
#include "vehicle_task.h"

//==================================================================================================
// 自检控制器类
// 功能描述：
// 1. 负责机器人启动时的系统自检和硬件检测
// 2. 实时监控各个子系统的连接状态和工作状态
// 3. 通过OLED显示屏提供直观的状态信息显示
// 4. 播放启动音乐和提示音，增强用户体验
// 5. 检测电机、传感器、通信模块等关键硬件的在线状态
// 6. 提供系统运行时间、模式状态、设备参数等信息显示
//
// 硬件配置：
// - OLED显示屏：128x64像素，I2C接口（地址0x3C）
// - 蜂鸣器：PWM控制，支持音调和节拍播放
// - 检测对象：左轮电机、右轮电机、变形电机、SBUS接收器、上位机通信
//
// 显示内容：
// - 第0行：设备连接状态指示（图形块显示）
// - 第1行：控制模式、变形模式、系统运行时间
// - 第2行：变形电机角度信息
// - 第3行：IMU姿态角信息（俯仰、横滚、偏航）
//==================================================================================================

class SelfTestController {
public:
    static constexpr uint32_t TASK_DELAY_MS = SELFTEST_TASK_DELAY_MS;    // 任务循环延时

    /**
     * @brief 设备连接状态结构体
     * @note 记录各个关键硬件设备的在线状态
     */
    struct DeviceStatus {
        bool left_motor = false;        // 左轮电机连接状态
        bool right_motor = false;       // 右轮电机连接状态
        bool transform_motor = false;   // 变形电机连接状态
        bool sbus = false;              // SBUS遥控接收器连接状态
        bool computer = false;          // 上位机通信连接状态
    };

    /**
     * @brief 默认构造函数
     * @note 初始化时不分配资源，需要调用Init()完成初始化
     */
    SelfTestController() = default;

    /**
     * @brief 初始化自检控制器
     * @param hi2c I2C总线句柄，用于OLED显示屏通信
     * @param htim PWM定时器句柄，用于蜂鸣器控制
     * @param vehicle_ctrl 车辆控制器指针，用于检测电机状态
     * @param transform_ctrl 变形控制器指针，用于检测变形电机状态
     * @param sbus SBUS接收器指针，用于检测遥控器连接状态
     * @param state_manager 系统状态管理器指针，用于获取系统模式信息
     * @param imu_ctrl IMU控制器指针，用于获取姿态信息
     * @note 保存各个控制器的指针，初始化OLED和蜂鸣器硬件
     */
    void Init(I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef* htim,
              VehicleController* vehicle_ctrl,
              TransformController* transform_ctrl,
              remote::SBUS* sbus,
              SystemStateManager* state_manager,
              IMUController* imu_ctrl);

    /**
     * @brief 主执行函数，周期性执行自检和显示更新
     * @note 执行流程：
     *       1. 首次运行时播放启动序列（LOGO显示+音乐）
     *       2. 更新各设备的连接状态
     *       3. 刷新OLED显示内容
     *       4. 周期性监控系统状态
     */
    void Execute();

    /**
     * @brief 设置上位机连接状态
     * @param connected true=已连接，false=未连接
     * @note 由通信控制器调用，用于更新上位机连接状态
     */
    void SetComputerStatus(bool connected);

private:
    bool startup_sequence_played_ = false;               // 启动序列播放标志，确保只播放一次

    // 控制器指针，用于获取各子系统状态
    VehicleController* vehicle_ctrl_ = nullptr;         // 车辆控制器指针
    TransformController* transform_ctrl_ = nullptr;     // 变形控制器指针
    remote::SBUS* sbus_ = nullptr;                      // SBUS遥控器指针
    SystemStateManager* state_manager_ = nullptr;       // 系统状态管理器指针
    IMUController* imu_ctrl_ = nullptr;                 // IMU控制器指针

    // 硬件设备对象
    std::unique_ptr<display::OLED> OLED_;               // OLED显示屏对象
    std::unique_ptr<bsp::Buzzer> buzzer_;               // 蜂鸣器对象

    // 状态记录
    DeviceStatus device_status_;                        // 设备连接状态记录
    uint32_t start_time_ = 0;                          // 系统启动时间戳，用于计算运行时间

    /**
     * @brief 显示启动序列
     * @note 执行流程：
     *       1. 显示团队LOGO
     *       2. 播放音乐
     *       3. 清屏并显示固定标签
     *       4. 记录启动时间
     */
    void ShowStartupSequence();

    /**
     * @brief 更新设备连接状态
     * @note 检测过程：
     *       1. 重置所有设备的连接标志
     *       2. 延时等待设备响应
     *       3. 读取各设备的连接状态
     *       4. 更新device_status_结构体
     */
    void UpdateDeviceStatus();

    /**
     * @brief 更新OLED显示内容
     * @note 显示内容：
     *       - 行0：设备状态图形指示器（LW/RW/TF/RC/PC）
     *       - 行1：控制模式+变形模式+运行时间（HH:MM:SS）
     *       - 行2：变形电机角度（度数，保留1位小数）
     *       - 行3：IMU姿态角（俯仰/横滚/偏航，保留1位小数）
     */
    void UpdateOLEDDisplay();
};