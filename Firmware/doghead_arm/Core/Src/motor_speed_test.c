/**
 * @file motor_speed_test.c
 * @brief 馬達速度測試與調試範例
 * @details 用於驗證編碼器速度回饋是否正確
 */

#include "main.h"
#include "mainpp.h"
#include "nidec_motor_driver.h"
#include <stdio.h>

// ==========================================================
// UART 重定向設定 (用於 printf 輸出)
// ==========================================================
extern UART_HandleTypeDef huart2;

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// ==========================================================
// 測試函數 1: 讀取並顯示馬達速度
// ==========================================================
void Test_PrintMotorSpeed(void) {
    // 更新編碼器數據（必須先調用才能計算速度）
    Motor_Update(&motor_joint_13pin);
    Motor_Update(&motor_joint_8pin);
    
    // 取得速度數據
    int32_t cmd_rpm1 = motor_joint_13pin.current_rpm_cmd;      // 指令值
    float real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);   // 實際值（馬達軸）
    float angle1 = Motor_GetAngle(&motor_joint_13pin);         // 位置（輸出軸）
    
    int32_t cmd_rpm2 = motor_joint_8pin.current_rpm_cmd;
    float real_rpm2 = Motor_GetVelocity(&motor_joint_8pin);
    float angle2 = Motor_GetAngle(&motor_joint_8pin);
    
    // 計算誤差
    float error1 = (cmd_rpm1 != 0) ? ((real_rpm1 - cmd_rpm1) / (float)cmd_rpm1) * 100.0f : 0.0f;
    float error2 = (cmd_rpm2 != 0) ? ((real_rpm2 - cmd_rpm2) / (float)cmd_rpm2) * 100.0f : 0.0f;
    
    // 輸出到串口
    printf("=== Motor Speed Test ===\r\n");
    printf("Motor 1 (13-Pin):\r\n");
    printf("  CMD: %ld RPM, Real: %.1f RPM, Error: %.1f%%\r\n", 
           cmd_rpm1, real_rpm1, error1);
    printf("  Angle: %.2f deg\r\n", angle1);
    printf("\r\n");
    
    printf("Motor 2 (8-Pin):\r\n");
    printf("  CMD: %ld RPM, Real: %.1f RPM, Error: %.1f%%\r\n", 
           cmd_rpm2, real_rpm2, error2);
    printf("  Angle: %.2f deg\r\n", angle2);
    printf("\r\n");
}

// ==========================================================
// 測試函數 2: 階梯式速度測試
// ==========================================================
void Test_StepResponse(void) {
    printf("開始階梯式速度測試...\r\n");
    
    // 測試速度序列 (RPM)
    int32_t test_speeds[] = {0, 500, 1000, 1500, 2000, 1000, 0};
    int num_steps = sizeof(test_speeds) / sizeof(test_speeds[0]);
    
    for (int i = 0; i < num_steps; i++) {
        printf("\r\n--- 設定目標速度: %ld RPM ---\r\n", test_speeds[i]);
        
        // 設定馬達速度
        Motor_SetSpeed(&motor_joint_13pin, test_speeds[i]);
        Motor_SetSpeed(&motor_joint_8pin, test_speeds[i]);
        
        // 等待 2 秒並持續監測
        for (int t = 0; t < 20; t++) {
            Motor_Update(&motor_joint_13pin);
            Motor_Update(&motor_joint_8pin);
            
            float real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);
            float real_rpm2 = Motor_GetVelocity(&motor_joint_8pin);
            
            printf("  [%d.%ds] M1: %.1f RPM, M2: %.1f RPM\r\n", 
                   t/10, t%10, real_rpm1, real_rpm2);
            
            HAL_Delay(100);
        }
    }
    
    printf("測試完成！\r\n");
}

// ==========================================================
// 測試函數 3: 單馬達調試
// ==========================================================
void Test_SingleMotor(Motor_t *motor, const char *name, int32_t target_rpm) {
    printf("\r\n=== %s 單獨測試 (目標: %ld RPM) ===\r\n", name, target_rpm);
    
    Motor_Start(motor);
    Motor_SetSpeed(motor, target_rpm);
    
    // 監測 5 秒
    for (int i = 0; i < 50; i++) {
        Motor_Update(motor);
        
        float real_rpm = Motor_GetVelocity(motor);
        float angle = Motor_GetAngle(motor);
        int64_t pulses = motor->total_pulse_count;
        
        printf("[%.1fs] RPM: %.1f, Angle: %.2f deg, Pulses: %lld\r\n",
               i * 0.1f, real_rpm, angle, pulses);
        
        HAL_Delay(100);
    }
    
    Motor_Stop(motor);
    printf("%s 測試完成\r\n", name);
}

// ==========================================================
// 測試函數 4: 驗證編碼器方向
// ==========================================================
void Test_EncoderDirection(void) {
    printf("\r\n=== 編碼器方向驗證 ===\r\n");
    printf("馬達將以低速正轉，請觀察：\r\n");
    printf("1. 速度讀數應為正值\r\n");
    printf("2. 角度應該增加\r\n");
    printf("3. 如果相反，請檢查 A/B 相接線或修改代碼\r\n\r\n");
    
    Motor_Start(&motor_joint_13pin);
    Motor_Start(&motor_joint_8pin);
    
    // 低速正轉 (300 RPM)
    Motor_SetSpeed(&motor_joint_13pin, 300);
    Motor_SetSpeed(&motor_joint_8pin, 300);
    
    // 監測 3 秒
    for (int i = 0; i < 30; i++) {
        Motor_Update(&motor_joint_13pin);
        Motor_Update(&motor_joint_8pin);
        
        printf("[%.1fs] M1_RPM: %.1f, M1_Angle: %.2f | M2_RPM: %.1f, M2_Angle: %.2f\r\n",
               i * 0.1f,
               Motor_GetVelocity(&motor_joint_13pin),
               Motor_GetAngle(&motor_joint_13pin),
               Motor_GetVelocity(&motor_joint_8pin),
               Motor_GetAngle(&motor_joint_8pin));
        
        HAL_Delay(100);
    }
    
    Motor_Stop(&motor_joint_13pin);
    Motor_Stop(&motor_joint_8pin);
    printf("方向驗證完成\r\n");
}

// ==========================================================
// 主測試入口
// ==========================================================
void Run_Motor_Speed_Tests(void) {
    printf("\r\n");
    printf("========================================\r\n");
    printf("    馬達速度回饋測試程式 v1.0\r\n");
    printf("========================================\r\n");
    printf("\r\n");
    
    // 測試 1: 編碼器方向驗證
    printf(">>> 測試 1: 編碼器方向驗證\r\n");
    Test_EncoderDirection();
    HAL_Delay(1000);
    
    // 測試 2: 單馬達測試
    printf("\r\n>>> 測試 2: 單馬達測試\r\n");
    Test_SingleMotor(&motor_joint_13pin, "Motor1 (13-Pin)", 1000);
    HAL_Delay(1000);
    Test_SingleMotor(&motor_joint_8pin, "Motor2 (8-Pin)", 1000);
    HAL_Delay(1000);
    
    // 測試 3: 階梯響應測試
    printf("\r\n>>> 測試 3: 階梯響應測試\r\n");
    Test_StepResponse();
    
    printf("\r\n所有測試完成！\r\n");
}

// ==========================================================
// 在 main.c 中調用範例
// ==========================================================
/*
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    
    Robot_Init();  // 初始化馬達系統
    
    printf("\r\n系統啟動完成，按下用戶按鈕開始測試...\r\n");
    
    // 等待按鈕按下
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
        HAL_Delay(100);
    }
    
    // 執行測試
    Run_Motor_Speed_Tests();
    
    // 測試完成後進入循環監測模式
    while (1) {
        Test_PrintMotorSpeed();
        HAL_Delay(500);
    }
}
*/
