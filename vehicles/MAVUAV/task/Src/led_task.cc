#include "cmsis_os.h"
#include "led_task.h"

/**
 * @brief 初始化LED控制器
 * @param htim PWM定时器句柄指针，用于生成RGB LED控制信号
 * @note 初始化参数说明：
 *       - htim: PWM定时器（通常为TIM5）
 *       - 第2个参数(3): RGB通道数量（红、绿、蓝）
 *       - 第3个参数(2): 可能是LED数量或缓冲区大小
 *       - 第4个参数(1): 可能是控制模式或配置参数
 *       - 第5个参数(1000000): 定时器时钟频率1MHz，确保PWM精度
 */
void LEDController::Init(TIM_HandleTypeDef* htim) {
    // 创建RGB LED对象，配置PWM参数
    led_ = std::make_unique<display::RGB>(htim, 3, 2, 1, 1000000);
}

/**
 * @brief LED控制器主执行函数，实现RGB流水灯效果
 * @note 实现算法：
 *       1. 定义红、绿、蓝三种基础颜色
 *       2. 在当前颜色和下一个颜色之间进行线性插值
 *       3. 每次循环微调颜色值，实现平滑渐变
 *       4. 渐变完成后切换到下一个颜色
 */
void LEDController::Execute() {
    // 检查LED对象是否已成功初始化
    if (!led_) {
        // LED对象未初始化，执行延时后直接返回
        osDelay(TASK_DELAY_MS);
        return;
    }

    // 定义RGB流水灯颜色序列：红 → 绿 → 蓝
    // 颜色格式：0xAARRGGBB（A=Alpha透明度，R=红色，G=绿色，B=蓝色）
    static constexpr std::array<uint32_t, 3> RGB_flow_color = {
            0xFFFF0000,  // 红色：透明度255，红255，绿0，蓝0
            0xFF00FF00,  // 绿色：透明度255，红0，绿255，蓝0
            0xFF0000FF   // 蓝色：透明度255，红0，绿0，蓝255
    };

    // 提取当前颜色的各个分量
    float alpha = (RGB_flow_color[color_index_] & 0xFF000000) >> 24;  // 透明度分量
    float red = ((RGB_flow_color[color_index_] & 0x00FF0000) >> 16);  // 红色分量
    float green = ((RGB_flow_color[color_index_] & 0x0000FF00) >> 8); // 绿色分量
    float blue = ((RGB_flow_color[color_index_] & 0x000000FF) >> 0);  // 蓝色分量

    // 计算到下一个颜色的变化量（用于线性插值）
    // 下一个颜色索引：(当前索引 + 1) % 3，实现循环
    float delta_alpha = static_cast<float>((RGB_flow_color[(color_index_ + 1) % 3] & 0xFF000000) >> 24) -
                        static_cast<float>((RGB_flow_color[color_index_] & 0xFF000000) >> 24);
    float delta_red = static_cast<float>((RGB_flow_color[(color_index_ + 1) % 3] & 0x00FF0000) >> 16) -
                      static_cast<float>((RGB_flow_color[color_index_] & 0x00FF0000) >> 16);
    float delta_green = static_cast<float>((RGB_flow_color[(color_index_ + 1) % 3] & 0x0000FF00) >> 8) -
                        static_cast<float>((RGB_flow_color[color_index_] & 0x0000FF00) >> 8);
    float delta_blue = static_cast<float>((RGB_flow_color[(color_index_ + 1) % 3] & 0x000000FF) >> 0) -
                       static_cast<float>((RGB_flow_color[color_index_] & 0x000000FF) >> 0);

    // 计算每次迭代的颜色变化步长
    // 总变化量除以变化时间，得到每毫秒的变化量
    delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
    delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;

    // 执行颜色渐变循环，持续RGB_FLOW_COLOR_CHANGE_TIME（300ms）
    for (int j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; ++j) {
        // 更新当前颜色值（线性插值）
        alpha += delta_alpha;
        red += delta_red;
        green += delta_green;
        blue += delta_blue;

        // 将浮点颜色值转换为32位整数格式
        uint32_t aRGB = (static_cast<uint32_t>(alpha)) << 24 |    // Alpha通道
                        (static_cast<uint32_t>(red)) << 16 |      // 红色通道
                        (static_cast<uint32_t>(green)) << 8 |     // 绿色通道
                        (static_cast<uint32_t>(blue)) << 0;       // 蓝色通道

        // 显示当前颜色
        led_->Display(aRGB);

        // 延时控制渐变速度，每毫秒更新一次颜色
        osDelay(TASK_DELAY_MS);
    }

    // 渐变完成，切换到下一个颜色
    ++color_index_;
    color_index_ = color_index_ % 3;  // 保持在0-2范围内循环
}