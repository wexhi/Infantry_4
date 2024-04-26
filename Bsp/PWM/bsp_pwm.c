#include "bsp_pwm.h"
#include "stdlib.h"
#include "memory.h"

// 配合中断以及初始化
static uint8_t idx;
static PWM_Instance *pwm_instances[PWM_DEVICE_MAX_NUM] = {NULL}; // 一个指针数组，用于存放PWM实例的指针

/**
 * @brief 设置pwm对应定时器时钟源频率
 *
 * @param htim 定时器句柄
 *
 * @note tim2~7,12~14:APB1  tim1,8~11:APB2
 */
static uint32_t PWMSelectTclk(TIM_HandleTypeDef *htim)
{
    uintptr_t tclk_temp = ((uintptr_t)((htim)->Instance));
    if (
        (tclk_temp <= (APB1PERIPH_BASE + 0x2000UL)) &&
        (tclk_temp >= (APB1PERIPH_BASE + 0x0000UL))) {
        return (HAL_RCC_GetPCLK1Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
    } else if (
        ((tclk_temp <= (APB2PERIPH_BASE + 0x0400UL)) &&
         (tclk_temp >= (APB2PERIPH_BASE + 0x0000UL))) ||
        ((tclk_temp <= (APB2PERIPH_BASE + 0x4800UL)) &&
         (tclk_temp >= (APB2PERIPH_BASE + 0x4000UL)))) {
        return (HAL_RCC_GetPCLK2Freq() * (APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos] == 0 ? 1 : 2));
    }
    return 0;
}

/**
 * @brief pwm dma传输完成回调函数
 *
 * @param htim 发生中断的定时器句柄
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < idx; i++) { // 来自同一个定时器的中断且通道相同
        if (pwm_instances[i]->htim == htim && htim->Channel == (1 << (pwm_instances[i]->channel / 4))) {
            if (pwm_instances[i]->callback) // 如果有回调函数
                pwm_instances[i]->callback(pwm_instances[i]);
            return; // 一次只能有一个通道的中断,所以直接返回
        }
    }
}

/**
 * @brief 注册pwm实例
 *
 * @param config 初始化配置
 * @return PWM_Instance*
 */
PWM_Instance *PWMRegister(PWM_Config_s *config)
{
    if (idx >= PWM_DEVICE_MAX_NUM) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        return NULL;
    PWM_Instance *pwm = (PWM_Instance *)malloc(sizeof(PWM_Instance));
    memset(pwm, 0, sizeof(PWM_Instance)); // 清零,防止原先的地址有脏数据

    pwm->htim      = config->htim;
    pwm->channel   = config->channel;
    pwm->period    = config->period;
    pwm->dutycycle = config->dutycycle;
    pwm->callback  = config->callback;
    pwm->id        = config->id;
    pwm->tclk      = PWMSelectTclk(pwm->htim);

    // 使能定时器
    PWMStart(pwm);
    PWMSetPeriod(pwm, pwm->period);
    PWMSetDutyRatio(pwm, pwm->dutycycle);
    pwm_instances[idx++] = pwm;

    return pwm;
}

/**
 * @brief 启动pwm
 *
 * @param pwm
 */
void PWMStart(PWM_Instance *pwm)
{
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
}

/**
 * @brief 设置pwm周期
 *
 * @param pwm pwm实例
 * @param period 周期 单位 s
 */
void PWMSetPeriod(PWM_Instance *pwm, float period)
{
    __HAL_TIM_SetAutoreload(pwm->htim, period * ((pwm->tclk) / (pwm->htim->Init.Prescaler + 1)));
}

/**
 * @brief 设置pwm占空比
 *
 * @param pwm pwm实例
 * @param dutycycle 占空比 0~1
 */
void PWMSetDutyRatio(PWM_Instance *pwm, float dutycycle)
{
    __HAL_TIM_SetCompare(pwm->htim, pwm->channel, dutycycle * (pwm->htim->Instance->ARR));
}