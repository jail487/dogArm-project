/**
 * @file mainpp.h
 * @brief C 與 C++ 的橋接介面 (Main C++)
 */
#ifndef MAINPP_H
#define MAINPP_H

#include <stdint.h>   // 提供 int32_t 等類型
#include <stdbool.h>  // 提供 bool 類型 (C99)

#ifdef __cplusplus
extern "C" {
#endif

// 初始化機器人 (馬達、PID)
void Robot_Init(void);

// 機器人主迴圈 (需定期呼叫)
// dt_seconds: 距離上次呼叫的時間差 (秒)，例如 1ms = 0.001f
void Robot_Loop(float dt_seconds);

// 設定目標位置 (使用運動學解算)
void Robot_SetTargetPosition(float x, float y);

// 測試模式控制
void Robot_SetTestMode(bool enable);

// 測試模式下直接設定馬達速度 (RPM)
void Robot_SetTestSpeed(int32_t rpm_motor1, int32_t rpm_motor2);

#ifdef __cplusplus
}
#endif

#endif // MAINPP_H
