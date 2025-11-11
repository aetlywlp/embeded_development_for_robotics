#pragma once

#include <memory>
#include "cmsis_os.h"
#include "bsp_imu.h"
#include "task_config.h"

//==================================================================================================
// IMU控制器类
// 功能描述：
// 1. 管理IMU（惯性测量单元）的数据采集和处理
// 2. 实现IMU数据的中断驱动接收和更新
// 3. 提供三轴加速度、角速度、姿态角等传感器数据
// 4. 支持IMU校准和温度控制
// 5. 与通信控制器配合，定期上报IMU数据给上位机
//
// 硬件配置：
// - IMU芯片：BMI088（加速度计+陀螺仪）+ IST8310（磁力计）
// - 通信接口：SPI（BMI088）+ I2C（IST8310）
// - 中断引脚：INT1_ACCEL_Pin, INT1_GYRO_Pin
// - 温度控制：PWM加热器，目标温度45°C
//
// 数据更新：
// - 更新频率：由IMU硬件中断触发，通常1kHz
// - 数据内容：加速度(m/s²)、角速度(rad/s)、欧拉角(rad)、温度(°C)
//==================================================================================================

class IMUController {
public:
    static constexpr uint32_t TASK_DELAY_MS = IMU_TASK_DELAY_MS;    // 任务循环延时，通常1ms
    static constexpr uint32_t RX_SIGNAL = (1 << 1);               // IMU数据接收完成信号标志

    /**
     * @brief 自定义IMU类
     * @note 继承自bsp::IMU_typeC，添加了任务通知机制
     *       当IMU数据接收完成时，会向任务发送信号进行异步处理
     */
    class CustomIMU : public bsp::IMU_typeC {
    public:
        /**
         * @brief 构造函数，直接继承父类构造函数
         * @note 使用using声明继承父类的所有构造函数
         */
        using bsp::IMU_typeC::IMU_typeC;

        /**
         * @brief 设置任务句柄用于中断通知
         * @param handle IMU任务的线程句柄
         * @note 设置后，当IMU接收完成时会向此任务发送RX_SIGNAL信号
         */
        void SetTaskHandle(osThreadId_t handle) {
            task_handle_ = handle;
        }

    protected:
        /**
         * @brief IMU数据接收完成回调函数
         * @note 在IMU数据接收完成时由硬件中断调用
         *       向任务发送信号，触发数据更新处理
         */
        void RxCompleteCallback() final {
            if (task_handle_) {
                osThreadFlagsSet(task_handle_, RX_SIGNAL);
            }
        }

    private:
        osThreadId_t task_handle_ = nullptr;  // IMU任务的线程句柄
    };

    /**
     * @brief 默认构造函数
     * @note 初始化时不分配资源，需要调用Init()完成初始化
     */
    IMUController() = default;

    /**
     * @brief 初始化IMU控制器
     * @param imu_init IMU初始化配置结构体
     * @note 配置包括：
     *       - BMI088传感器配置（SPI接口、片选引脚）
     *       - IST8310磁力计配置（I2C接口、中断引脚）
     *       - 加热器配置（PWM定时器、目标温度）
     *       - DMA配置（SPI收发DMA通道）
     */
    void Init(const bsp::IMU_typeC_init_t& imu_init);

    /**
     * @brief 设置任务句柄用于中断回调
     * @param handle IMU任务的线程句柄
     * @note 必须在任务创建后调用，用于建立中断与任务的通信
     */
    void SetTaskHandle(osThreadId_t handle);

    /**
     * @brief 主执行函数，处理IMU数据更新
     * @note 执行流程：
     *       1. 等待RX_SIGNAL信号（由中断触发）
     *       2. 调用IMU的Update()方法处理新数据
     *       3. 更新传感器融合算法
     *       4. 计算姿态角和温度补偿
     */
    void Execute();

    /**
     * @brief 获取IMU对象指针
     * @return 指向CustomIMU对象的指针
     * @note 用于其他模块访问IMU数据，如通信控制器获取数据上报
     */
    CustomIMU* GetIMU() { return imu_.get(); }

private:
    std::unique_ptr<CustomIMU> imu_;  // IMU对象智能指针，管理IMU硬件接口
};