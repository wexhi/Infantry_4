/**
 * @file bsp_pwm.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief  PWM
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"
#include "stdint.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f407xx.h"

#define PWM_DEVICE_MAX_NUM 16 // 支持的最大PWM设备数量

/* PWM实例结构体 */
typedef struct pwm_instance {
    TIM_HandleTypeDef *htim; // 定时器句柄
    uint32_t channel;        // 通道号
    uint32_t tclk;           // 定时器时钟频率
    float period;            // 周期
    float dutycycle;         // 占空比

    void (*callback)(struct pwm_instance *); // 定时器中断回调函数
    void *id;                                // PWN实例ID
} PWM_Instance;

/* PWM初始化配置结构体 */
typedef struct pwm_config {
    TIM_HandleTypeDef *htim; // 定时器句柄
    uint32_t channel;        // 通道号
    float period;            // 周期
    float dutycycle;         // 占空比

    void (*callback)(PWM_Instance *); // 回调函数
    void *id;                         // 实例ID
} PWM_Config_s;

/**
 * @brief 注册pwm实例
 *
 * @param config 初始化配置
 * @return PWM_Instance*
 */
PWM_Instance *PWMRegister(PWM_Config_s *config);

/**
 * @brief 启动pwm
 *
 * @param pwm
 */
void PWMStart(PWM_Instance *pwm);

/**
 * @brief 设置pwm周期
 *
 * @param pwm pwm实例
 * @param period 周期 单位 s
 */
void PWMSetPeriod(PWM_Instance *pwm, float period);

/**
 * @brief 设置pwm占空比
 *
 * @param pwm pwm实例
 * @param dutycycle 占空比 0~1
 */
void PWMSetDutyRatio(PWM_Instance *pwm, float dutycycle);

#endif
