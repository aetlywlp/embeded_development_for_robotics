//==================================================================================================
// MAVUAV:main.cc
//
// 系统架构：
// - 基于FreeRTOS的多任务嵌入式系统
// - 支持变形机器人的车辆和飞行两种模式
// - 实现多种控制方式：遥控、上位机控制、急停
// - 集成IMU、电机、传感器等多种硬件设备
//
// 主要功能模块：
// 1. 车辆控制：差分驱动的轮式移动
// 2. 变形控制：车辆与飞行模式间的形态切换
// 3. 通信控制：与上位机的数据交互
// 4. 自检系统：硬件状态监控和故障诊断
// 5. 吸附腔控制：机器人吸附
// 6. LED指示：状态显示和视觉反馈
// 7. IMU数据：姿态感知和导航支持
//
// 硬件平台：STM32F407IGH7(DJIboardC) + FreeRTOS
//==================================================================================================

#include "bsp_imu.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "main.h"
#include "spi.h"
#include "sbus.h"
#include "oled.h"
#include "bsp_buzzer.h"
#include "motor.h"
#include "rgb.h"
#include "bsp_usb.h"
#include "minipc_protocol.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"

#include <memory>
#include <functional>
#include <array>

// Include controller headers
#include "system_state_task.h"
#include "vehicle_task.h"
#include "transform_task.h"
#include "suction_task.h"
#include "imu_task.h"
#include "led_task.h"
#include "selftest_task.h"
#include "communication_task.h"

#include "control_modes.h"
#include "task_config.h"

//==================================================================================================
// 全局实例声明
// 注意：使用静态变量和智能指针确保资源管理和线程安全
//==================================================================================================

// 硬件接口全局实例
static remote::SBUS* g_sbus = nullptr;    // SBUS遥控器接收器，串口3接收
static bsp::CAN* g_can1 = nullptr;        // CAN1总线接口，用于电机通信

// 控制器全局实例（使用智能指针自动管理资源）
static std::unique_ptr<SystemStateManager> g_state_manager;      // 系统状态管理器
static std::unique_ptr<VehicleController> g_vehicle_controller;  // 车辆运动控制器
static std::unique_ptr<TransformController> g_transform_controller; // 变形控制器
static std::unique_ptr<SuctionController> g_suction_controller;  // 吸盘控制器
static std::unique_ptr<IMUController> g_imu_controller;          // IMU控制器
static std::unique_ptr<LEDController> g_led_controller;          // LED控制器
static std::unique_ptr<SelfTestController> g_self_test_controller; // 自检控制器
static std::unique_ptr<CommunicationController> g_comm_controller; // 通信控制器

//==================================================================================================
// 任务函数定义
// 每个任务运行在独立的线程中，实现系统的并发处理能力
//==================================================================================================

/**
 * @brief 车辆控制任务
 * @param arg 任务参数（未使用）
 * @note 负责：
 *       - 差分驱动轮式移动控制
 *       - SBUS遥控器输入处理
 *       - 上位机运动指令执行
 *       - 电机PID速度控制
 */
void vehicleTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_vehicle_controller) {
            g_vehicle_controller->Execute();
        }
    }
}

/**
 * @brief 变形控制任务
 * @param arg 任务参数（未使用）
 * @note 负责：
 *       - 车辆与飞行模式间的形态切换
 *       - 变形电机的旋转控制
 *       - 气动执行器的伸缩控制
 *       - 变形序列状态机管理
 */
void transformTask(void* arg) {
    UNUSED(arg);
    osDelay(1000); // 等待系统初始化完成，避免启动冲突
    while (true) {
        if (g_transform_controller) {
            g_transform_controller->Execute();
        }
    }
}

/**
 * @brief 吸附控制任务
 * @param arg 任务参数（未使用）
 * @note 负责：
 *       - 吸附风机的PWM速度控制
 *       - SBUS通道的吸力指令处理
 *       - 安全模式检查和保护
 */
void suctionTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_suction_controller) {
            g_suction_controller->Execute();
        }
    }
}

/**
 * @brief IMU数据处理任务
 * @param arg 任务参数（未使用）
 * @note 负责：
 *       - IMU传感器数据采集
 *       - 传感器融合和姿态解算
 *       - 中断驱动的高频数据更新
 */
void imuTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_imu_controller) {
            g_imu_controller->Execute();
        }
    }
}

/**
 * @brief 自检和状态监控任务
 * @param arg 任务参数（未使用）
 * @note 负责：
 *       - 硬件设备连接状态检测
 *       - OLED显示屏信息更新
 *       - 启动序列和蜂鸣器控制
 *       - 系统运行状态监控
 */
void selfTestTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_self_test_controller) {
            g_self_test_controller->Execute();
        }
    }
}

/**
 * @brief 通信控制任务
 * @param arg 任务参数（未使用）
 * @note 负责：
 *       - USB虚拟串口通信管理
 *       - 上位机协议解析和封装
 *       - IMU、里程计、状态数据上报
 *       - 运动控制和自检指令处理
 */
void communicationTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_comm_controller) {
            g_comm_controller->Execute();
        }
    }
}

/**
 * @brief LED指示灯控制任务
 * @param arg 任务参数（未使用）
 * @note 负责：
 *       - RGB LED流水灯效果
 *       - 系统状态的视觉指示
 *       - 彩色渐变和动画效果
 */
void ledTask(void* arg) {
    UNUSED(arg);
    while (true) {
        if (g_led_controller) {
            g_led_controller->Execute();
        }
    }
}

//==================================================================================================
// 任务属性配置
// 定义每个任务的栈大小、优先级等参数
//==================================================================================================

// 车辆控制任务属性
const osThreadAttr_t vehicleTaskAttribute = {
        .name = "vehicleTask",                           // 任务名称
        .attr_bits = osThreadDetached,                   // 分离属性，任务结束时自动清理
        .cb_mem = nullptr,                               // 控制块内存（系统分配）
        .cb_size = 0,                                    // 控制块大小
        .stack_mem = nullptr,                            // 栈内存（系统分配）
        .stack_size = TASK_STACK_SIZE,                   // 栈大小
        .priority = (osPriority_t)osPriorityNormal,      // 普通优先级
        .tz_module = 0,                                  // TrustZone模块（未使用）
        .reserved = 0                                    // 保留字段
};

// 变形控制任务属性
const osThreadAttr_t transformTaskAttribute = {
        .name = "transformTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

// 吸附腔控制任务属性
const osThreadAttr_t suctionTaskAttribute = {
        .name = "suctionTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

// IMU数据处理任务属性
const osThreadAttr_t imuTaskAttribute = {
        .name = "imuTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

// 自检任务属性
const osThreadAttr_t selfTestTaskAttribute = {
        .name = "selfTestTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

// 通信控制任务属性
const osThreadAttr_t communicationTaskAttribute = {
        .name = "communicationTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityNormal,
        .tz_module = 0,
        .reserved = 0
};

// LED控制任务属性（低优先级，非关键任务）
const osThreadAttr_t ledTaskAttribute = {
        .name = "ledTask",
        .attr_bits = osThreadDetached,
        .cb_mem = nullptr,
        .cb_size = 0,
        .stack_mem = nullptr,
        .stack_size = TASK_STACK_SIZE,
        .priority = (osPriority_t)osPriorityLow,         // 低优先级
        .tz_module = 0,
        .reserved = 0
};

// 任务句柄声明（用于任务间通信和控制）
osThreadId_t vehicleTaskHandle;
osThreadId_t transformTaskHandle;
osThreadId_t suctionTaskHandle;
osThreadId_t imuTaskHandle;
osThreadId_t selfTestTaskHandle;
osThreadId_t communicationTaskHandle;
osThreadId_t ledTaskHandle;

//==================================================================================================
// RTOS系统初始化
//==================================================================================================

/**
 * @brief RTOS系统和硬件初始化
 * @note 初始化顺序很重要：
 *       1. 基础硬件接口（串口、CAN、I2C、SPI等）
 *       2. 底层控制器（状态管理器）
 *       3. 功能控制器（车辆、变形、吸附腔等）
 *       4. 传感器控制器（IMU）
 *       5. 显示和通信控制器
 */
void RM_RTOS_Init() {
    // 初始化串口打印功能，用于调试输出
    print_use_uart(&huart1);

    // 初始化全局硬件接口
    g_sbus = new remote::SBUS(&huart3);    // SBUS遥控器，使用串口3
    g_can1 = new bsp::CAN(&hcan1, true);   // CAN1总线，启用接收

    // 初始化系统状态管理器（最高层控制器）
    g_state_manager = std::make_unique<SystemStateManager>();
    g_state_manager->Init(g_sbus);

    // 初始化车辆控制器
    g_vehicle_controller = std::make_unique<VehicleController>();
    g_vehicle_controller->Init(g_can1, g_state_manager.get(), g_sbus);

    // 初始化变形控制器
    g_transform_controller = std::make_unique<TransformController>();
    g_transform_controller->Init(g_can1, g_state_manager.get(),g_sbus);

    // 初始化吸盘控制器
    g_suction_controller = std::make_unique<SuctionController>();
    g_suction_controller->Init(&htim1, g_state_manager.get(), g_sbus);

    // 配置并初始化IMU控制器
    // IST8310磁力计配置
    bsp::IST8310_init_t IST8310_init;
    IST8310_init.hi2c = &hi2c3;                    // I2C3接口
    IST8310_init.int_pin = DRDY_IST8310_Pin;       // 数据就绪中断引脚
    IST8310_init.rst_group = GPIOG;                // 复位引脚组
    IST8310_init.rst_pin = GPIO_PIN_6;             // 复位引脚号

    // BMI088 6轴IMU配置
    bsp::BMI088_init_t BMI088_init;
    BMI088_init.hspi = &hspi1;                     // SPI1接口
    BMI088_init.CS_ACCEL_Port = CS1_ACCEL_GPIO_Port; // 加速度计片选端口
    BMI088_init.CS_ACCEL_Pin = CS1_ACCEL_Pin;      // 加速度计片选引脚
    BMI088_init.CS_GYRO_Port = CS1_GYRO_GPIO_Port; // 陀螺仪片选端口
    BMI088_init.CS_GYRO_Pin = CS1_GYRO_Pin;        // 陀螺仪片选引脚

    // IMU加热器配置（温度稳定控制）
    bsp::heater_init_t heater_init;
    heater_init.htim = &htim10;                    // 加热器PWM定时器
    heater_init.channel = 1;                       // PWM通道
    heater_init.clock_freq = 1000000;              // 时钟频率1MHz
    heater_init.temp = 45;                         // 目标温度45°C

    // IMU综合配置
    bsp::IMU_typeC_init_t imu_init;
    imu_init.IST8310 = IST8310_init;
    imu_init.BMI088 = BMI088_init;
    imu_init.heater = heater_init;
    imu_init.hspi = &hspi1;                        // SPI接口
    imu_init.hdma_spi_rx = &hdma_spi1_rx;          // SPI接收DMA
    imu_init.hdma_spi_tx = &hdma_spi1_tx;          // SPI发送DMA
    imu_init.Accel_INT_pin_ = INT1_ACCEL_Pin;      // 加速度计中断引脚
    imu_init.Gyro_INT_pin_ = INT1_GYRO_Pin;        // 陀螺仪中断引脚

    g_imu_controller = std::make_unique<IMUController>();
    g_imu_controller->Init(imu_init);

    // 初始化LED控制器
    g_led_controller = std::make_unique<LEDController>();
    g_led_controller->Init(&htim5);                // 使用定时器5

    // 初始化自检控制器
    g_self_test_controller = std::make_unique<SelfTestController>();
    g_self_test_controller->Init(&hi2c2, &htim4,   // I2C2用于OLED，定时器4用于蜂鸣器
                                 g_vehicle_controller.get(),
                                 g_transform_controller.get(),
                                 g_sbus,
                                 g_state_manager.get(),
                                 g_imu_controller.get());

    // 初始化通信控制器
    g_comm_controller = std::make_unique<CommunicationController>();
    g_comm_controller->Init(g_state_manager.get(),
                            g_vehicle_controller.get(),
                            g_imu_controller.get(),
                            g_self_test_controller.get());
}

//==================================================================================================
// RTOS任务创建和启动
//==================================================================================================

/**
 * @brief 创建并启动所有RTOS任务
 * @note 创建顺序：
 *       1. 关键任务：IMU、车辆、变形
 *       2. 功能任务：吸盘、通信
 *       3. 非关键任务：自检、LED
 *       4. 设置任务句柄用于中断回调
 */
void RM_RTOS_Threads_Init(void) {
    // 创建所有任务
    imuTaskHandle = osThreadNew(imuTask, nullptr, &imuTaskAttribute);
    vehicleTaskHandle = osThreadNew(vehicleTask, nullptr, &vehicleTaskAttribute);
    transformTaskHandle = osThreadNew(transformTask, nullptr, &transformTaskAttribute);
    suctionTaskHandle = osThreadNew(suctionTask, nullptr, &suctionTaskAttribute);
    communicationTaskHandle = osThreadNew(communicationTask, nullptr, &communicationTaskAttribute);
    selfTestTaskHandle = osThreadNew(selfTestTask, nullptr, &selfTestTaskAttribute);
    ledTaskHandle = osThreadNew(ledTask, nullptr, &ledTaskAttribute);

    // 设置任务句柄，用于中断回调通信
    if (g_imu_controller) {
        g_imu_controller->SetTaskHandle(imuTaskHandle);
    }

    if (g_comm_controller) {
        g_comm_controller->SetTaskHandle(communicationTaskHandle);
    }
}

//==================================================================================================
// RTOS默认任务（系统监控任务）
//==================================================================================================

/**
 * @brief RTOS默认任务，系统级监控和控制
 * @param args 任务参数（未使用）
 * @note 主要功能：
 *       1. 周期性更新系统控制模式
 *       2. 监控紧急停止状态
 *       3. 执行全局安全检查
 *       4. 提供系统级的故障恢复机制
 */
void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);

    while (true) {
        // 更新系统状态管理器（模式检测和切换）
        if (g_state_manager) {
            g_state_manager->UpdateControlModes();

            // 检查紧急停止状态，如果激活则立即停止车辆
            if (g_state_manager->IsInEmergencyStop() && g_vehicle_controller) {
                g_vehicle_controller->EmergencyStop();
            }
        }

        // 系统级延时，默认任务的执行频率
        osDelay(DEFAULT_TASK_DELAY_MS);
    }
}