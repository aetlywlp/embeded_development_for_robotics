#pragma once

#include <memory>
#include <array>
#include "main.h"
#include "rgb.h"
#include "task_config.h"

//==================================================================================================
// LED控制器类
// 功能描述：
// 1. 管理机器人的RGB LED指示灯显示
// 2. 实现彩色流水灯效果，提供视觉状态指示
// 3. 支持多种颜色渐变和切换效果
// 4. 通过PWM控制RGB三色LED的亮度
// 5. 提供机器人运行状态的视觉反馈
//
// 硬件配置：
// - LED类型：可编程RGB LED
// - 控制接口：PWM定时器输出（通常TIM5）
// - LED数量：可配置，默认支持多个LED组成的灯带
// - 颜色深度：24位（每色8位，支持16777216种颜色）
//
// 显示效果：
// - 流水灯模式：红→绿→蓝循环渐变
// - 渐变时间：每种颜色持续300ms
// - 平滑过渡：颜色间线性插值渐变
//==================================================================================================

class LEDController {
public:
  static constexpr uint32_t TASK_DELAY_MS = LED_TASK_DELAY_MS;         // 任务循环延时，控制LED更新频率
  static constexpr int RGB_FLOW_COLOR_CHANGE_TIME = 200;               // 颜色切换时间，单位：毫秒

  /**
   * @brief 默认构造函数
   * @note 初始化时不分配资源，需要调用Init()完成初始化
   */
  LEDController() = default;

  /**
   * @brief 初始化LED控制器
   * @param htim PWM定时器句柄指针
   * @note 配置参数：
   *       - 定时器：用于生成PWM信号控制LED
   *       - 通道配置：RGB三色通道分别对应定时器的不同通道
   *       - 时钟频率：1MHz，确保PWM信号精度
   *       - LED参数：支持多个LED的级联控制
   */
  void Init(TIM_HandleTypeDef* htim);

  /**
   * @brief 主执行函数，控制LED显示效果
   * @note 执行流程：
   *       1. 检查LED对象是否已初始化
   *       2. 计算当前颜色值和目标颜色值
   *       3. 执行颜色渐变算法（线性插值）
   *       4. 更新LED显示并切换到下一个颜色
   *       5. 循环播放红-绿-蓝流水灯效果
   */
  void Execute();

private:
  std::unique_ptr<display::RGB> led_;      // RGB LED对象智能指针，管理LED硬件接口
  int color_index_ = 0;                    // 当前颜色索引，0=红色，1=绿色，2=蓝色
};