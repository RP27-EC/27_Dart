/**
 ******************************************************************************
 * @file    loadcell.c
 * @author  RobotPilots 镖架电控
 * @brief   双路测力接口(桩)
 * @version V0.1
 * @date    2026-11-02
 ******************************************************************************
 * @attention
 *  当前所有样本 valid=false；Force_Ready 在模块关闭时返回 true(不参与许可)。
 *
 * Copyright 2026 RobotPilots
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "loadcell.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static loadcell_sample_t s_sample[LOADCELL_CH_NUM];

/* Exported functions --------------------------------------------------------*/
/**
 * @brief  测力模块初始化(桩)
 */
void Loadcell_Init(void)
{
    memset(s_sample, 0, sizeof(s_sample));
}

/**
 * @brief  周期采样入口，CommunityTask 调用(桩)
 * @param  now 当前时刻 ms
 */
void Loadcell_Poll(uint32_t now)
{
    (void)now;
    /* TODO: ADS1232 DRDY 判断与 24bit 读取 */
}

/**
 * @brief  读取一路最近样本
 * @param  ch     通道 0/1
 * @param  sample 输出样本
 * @retval true 样本有效
 */
bool Loadcell_Get(uint8_t ch, loadcell_sample_t *sample)
{
    if ((ch >= LOADCELL_CH_NUM) || (sample == NULL)) {
        return false;
    }
    *sample = s_sample[ch];
    return sample->valid;
}

/**
 * @brief  力就绪判定，供释放许可使用
 * @retval true 力已稳定在目标附近
 */
bool Loadcell_Force_Ready(void)
{
#if DART_LOADCELL_ENABLE
    /* TODO: 绑定目标力、稳定时间与新鲜度 */
    return false;
#else
    return true;
#endif
}
