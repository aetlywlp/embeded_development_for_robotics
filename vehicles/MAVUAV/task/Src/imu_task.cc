#include "cmsis_os.h"
#include "imu_task.h"

/**
 * @brief 初始化IMU控制器
 * @param imu_init IMU初始化配置结构体，包含所有硬件配置参数
 * @note 初始化流程：
 *       1. 创建CustomIMU对象并传入配置参数
 *       2. 禁用自动校准（false参数），使用手动校准
 *       3. 执行IMU校准程序，获取零偏和比例因子
 *       4. 启动温度控制系统，确保IMU工作在稳定温度
 */
void IMUController::Init(const bsp::IMU_typeC_init_t& imu_init) {
  // 创建IMU对象，第二个参数false表示禁用自动校准
  imu_ = std::make_unique<CustomIMU>(imu_init, false);

  if (imu_) {
    // 执行IMU校准程序
    // 校准过程包括：加速度计零偏校准、陀螺仪零偏校准、磁力计软硬铁校正
    // 校准时需要保持IMU静止并按要求进行旋转动作
    imu_->Calibrate();
  }
}

/**
 * @brief 设置任务句柄用于中断回调通信
 * @param handle IMU任务的线程句柄
 * @note 此函数必须在任务创建后调用，用于建立中断服务程序与任务的通信
 *       当IMU硬件产生数据就绪中断时，会通过此句柄向任务发送信号
 */
void IMUController::SetTaskHandle(osThreadId_t handle) {
  if (imu_) {
    imu_->SetTaskHandle(handle);
  }
}

/**
 * @brief IMU任务主执行函数
 * @note 执行流程：
 *       1. 等待RX_SIGNAL信号（由IMU硬件中断触发）
 *       2. 当收到信号时，调用IMU的Update()方法
 *       3. Update()方法会：
 *          - 读取最新的传感器原始数据
 *          - 应用校准参数进行数据修正
 *          - 执行传感器融合算法（EKF/互补滤波器）
 *          - 计算姿态角（俯仰、横滚、偏航）
 *          - 更新温度补偿参数
 *       4. 执行任务延时，控制任务执行频率
 */
void IMUController::Execute() {
  // 等待RX_SIGNAL信号，使用osWaitForever表示无限等待
  // 这样可以确保只有在IMU有新数据时才进行处理，提高效率
  uint32_t flags = osThreadFlagsWait(RX_SIGNAL, osFlagsWaitAll, osWaitForever);

  // 检查是否收到了预期的信号且IMU对象有效
  if (flags & RX_SIGNAL && imu_) {
    // 更新IMU数据：读取传感器、融合算法、姿态解算
    imu_->Update();
  }

  // 任务延时，防止任务占用过多CPU时间
  // 即使是中断驱动，也需要适当延时保证系统响应性
  osDelay(TASK_DELAY_MS);
}