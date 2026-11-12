#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/*
 * 每圈计数(4倍频) = 磁环每圈7脉冲 × 4倍频(TI1+TI2) × 减速比50 = 1400
 * 说明: 编码器电机磁环一圈7个脉冲, 减速比1:50, 定时器编码器模式4倍频
 */
#define ENC_COUNTS_PER_REV   1400U

/* 初始化 (记录当前计数为基准) */
void  Encoder_Init(void);

/* 由本周期(10ms)增量计算转速, 单位 r/min */
float Encoder_GetRPM(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */
