/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainpp.h"  // 用於呼叫 C++ 的 Robot_Loop
#include "nidec_motor_driver.h" // 用於存取馬達物件與 API
#include <stdio.h>

// 宣告外部馬達物件 (定義在 nidec_motor_driver.c)
extern Motor_t motor_joint_13pin;
extern Motor_t motor_joint_8pin;

// 全域除錯變數 (可在 Live Watch 中觀察)
volatile float debug_speed_m1 = 0.0f;
volatile float debug_speed_m2 = 0.0f;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MX_FREERTOS_Init(void);  // 宣告初始化函式
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// ==========================================================
// FreeRTOS 任務定義
// ==========================================================

/**
 * @brief 控制任務：高優先級，1kHz (1ms 週期)
 * @note 負責執行 PID 控制、運動學解算、馬達輸出
 */
void ControlTask(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1ms = 1000Hz
  
  for(;;)
  {
    // 呼叫機器手臂核心控制迴圈
    // 傳入精確的時間間隔 (1ms = 0.001s)
    Robot_Loop(0.001f);
    
    // 使用 vTaskDelayUntil 保證精準週期
    // 這會自動補償函式執行時間，確保穩定的 1kHz 控制頻率
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * @brief 通訊任務：低優先級，10Hz (100ms 週期)
 * @note 負責處理 UART 通訊、診斷輸出、未來整合 micro-ROS
 */
void CommTask(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms = 10Hz
  
  uint32_t counter = 0;

  // [測試模式] 啟動測試模式並設定目標轉速
  // 這裡設定為 500 RPM 進行初步驗證
  Robot_SetTestMode(true);
  Robot_SetTestSpeed(500, 500);
  
  for(;;)
  {
    // 讀取並儲存當前馬達速度 (由 ControlTask 中的 Motor_Update 更新)
    debug_speed_m1 = Motor_GetVelocity(&motor_joint_13pin);
    debug_speed_m2 = Motor_GetVelocity(&motor_joint_8pin);

    // 範例：定期輸出診斷資訊
    // 你可以在這裡讀取馬達狀態、編碼器位置等，然後透過 UART 輸出
    counter++;
    
    // 每 1 秒輸出一次 (10Hz * 10 = 1s)
    if (counter % 10 == 0) {
      printf("M1 RPM: %.2f, M2 RPM: %.2f\r\n", debug_speed_m1, debug_speed_m2);
    }
    
    // 未來在這裡處理：
    // - micro-ROS 訊息接收 (Subscriber callback)
    // - 狀態回報 (Publisher)
    // - 參數調整指令解析
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * @brief 在 main() 的 RTOS_THREADS 區段呼叫此函式來建立任務
 */
void MX_FREERTOS_Init(void)
{
  // 建立控制任務 (最高優先級 = 3)
  TaskHandle_t controlTaskHandle = NULL;
  xTaskCreate(
    ControlTask,           // 任務函式
    "ControlTask",         // 任務名稱 (用於除錯)
    256,                   // Stack 大小 (單位: words, 1 word = 4 bytes)
    NULL,                  // 任務參數
    3,                     // 優先級 (數字越大優先級越高)
    &controlTaskHandle     // 任務控制代碼
  );
  
  // 建立通訊任務 (中等優先級 = 2)
  TaskHandle_t commTaskHandle = NULL;
  xTaskCreate(
    CommTask,
    "CommTask",
    256,
    NULL,
    2,
    &commTaskHandle
  );
  
  // defaultTask 已經在 main.c 中由 CubeMX 自動建立
  // 其優先級為 Normal (通常是 1)，低於我們自訂的任務
}

/* USER CODE END Application */

