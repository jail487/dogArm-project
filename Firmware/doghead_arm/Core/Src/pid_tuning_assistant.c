/**
 * @file pid_tuning_assistant.c
 * @brief 工業級 PID 調參輔助系統
 * @details 提供自動化測試、性能評估、參數掃描等功能
 */

#include "mainpp.h"
#include "nidec_motor_driver.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

// ==========================================================
// 配置參數
// ==========================================================
#define MAX_TEST_SAMPLES    1000    // 最大測試樣本數
#define SAMPLE_PERIOD_MS    10      // 採樣週期 (ms)
#define SETTLING_THRESHOLD  2.0f    // 穩定判定閾值 (%)

// ==========================================================
// 數據記錄結構
// ==========================================================
typedef struct {
    uint32_t timestamp_ms;      // 時間戳 (ms)
    float target_position;      // 目標位置 (deg)
    float actual_position;      // 實際位置 (deg)
    float error;                // 誤差 (deg)
    float control_output;       // 控制輸出 (RPM)
    float velocity;             // 實際速度 (RPM)
} DataSample_t;

// 測試結果結構
typedef struct {
    // 時域性能指標
    float IAE;                  // 絕對誤差積分
    float ISE;                  // 誤差平方積分
    float ITAE;                 // 時間加權絕對誤差積分
    float max_error;            // 最大誤差
    float steady_state_error;   // 穩態誤差
    
    // 階躍響應特性
    float overshoot_percent;    // 超調量 (%)
    float rise_time_ms;         // 上升時間 (ms)
    float settling_time_ms;     // 穩定時間 (ms)
    float peak_time_ms;         // 峰值時間 (ms)
    
    // 附加資訊
    bool is_stable;             // 是否穩定
    bool is_oscillating;        // 是否震盪
    uint32_t num_samples;       // 樣本數
} PerformanceMetrics_t;

// 全域數據緩衝區
static DataSample_t g_data_buffer[MAX_TEST_SAMPLES];
static uint32_t g_sample_count = 0;

// ==========================================================
// 1. 數據記錄模組
// ==========================================================

/**
 * @brief 開始新的測試記錄
 */
void TestLog_Start(void) {
    g_sample_count = 0;
    memset(g_data_buffer, 0, sizeof(g_data_buffer));
    printf(">>> 開始記錄測試數據...\r\n");
}

/**
 * @brief 記錄單個樣本
 */
void TestLog_Record(float target, float actual, float control, float velocity) {
    if (g_sample_count >= MAX_TEST_SAMPLES) {
        return;  // 緩衝區已滿
    }
    
    DataSample_t *sample = &g_data_buffer[g_sample_count];
    sample->timestamp_ms = HAL_GetTick();
    sample->target_position = target;
    sample->actual_position = actual;
    sample->error = target - actual;
    sample->control_output = control;
    sample->velocity = velocity;
    
    g_sample_count++;
}

/**
 * @brief 停止記錄並輸出數據（可用於外部分析）
 */
void TestLog_Stop_And_Export(void) {
    printf(">>> 測試數據輸出 (CSV 格式)\r\n");
    printf("Time_ms,Target_deg,Actual_deg,Error_deg,Control_RPM,Velocity_RPM\r\n");
    
    for (uint32_t i = 0; i < g_sample_count; i++) {
        DataSample_t *s = &g_data_buffer[i];
        printf("%lu,%.3f,%.3f,%.3f,%.2f,%.2f\r\n",
               s->timestamp_ms,
               s->target_position,
               s->actual_position,
               s->error,
               s->control_output,
               s->velocity);
    }
    
    printf(">>> 數據輸出完成 (%lu 筆)\r\n", g_sample_count);
}

// ==========================================================
// 2. 性能評估模組
// ==========================================================

/**
 * @brief 計算綜合性能指標
 */
PerformanceMetrics_t Evaluate_Performance(void) {
    PerformanceMetrics_t metrics;
    memset(&metrics, 0, sizeof(metrics));
    
    if (g_sample_count < 10) {
        printf(">>> 警告：樣本數不足，無法評估\r\n");
        return metrics;
    }
    
    metrics.num_samples = g_sample_count;
    float dt = SAMPLE_PERIOD_MS / 1000.0f;  // 轉換為秒
    
    // === 1. 計算積分型指標 ===
    metrics.IAE = 0.0f;
    metrics.ISE = 0.0f;
    metrics.ITAE = 0.0f;
    metrics.max_error = 0.0f;
    
    for (uint32_t i = 0; i < g_sample_count; i++) {
        float err = fabsf(g_data_buffer[i].error);
        float time_s = (g_data_buffer[i].timestamp_ms - g_data_buffer[0].timestamp_ms) / 1000.0f;
        
        metrics.IAE += err * dt;
        metrics.ISE += err * err * dt;
        metrics.ITAE += time_s * err * dt;
        
        if (err > metrics.max_error) {
            metrics.max_error = err;
        }
    }
    
    // === 2. 計算穩態誤差 ===
    // 取最後 10% 的數據平均
    uint32_t steady_start = g_sample_count * 9 / 10;
    float sum_error = 0.0f;
    for (uint32_t i = steady_start; i < g_sample_count; i++) {
        sum_error += fabsf(g_data_buffer[i].error);
    }
    metrics.steady_state_error = sum_error / (g_sample_count - steady_start);
    
    // === 3. 階躍響應特性分析 ===
    // 假設是階躍響應測試
    float target = g_data_buffer[g_sample_count - 1].target_position;
    float initial = g_data_buffer[0].actual_position;
    float final_value = g_data_buffer[g_sample_count - 1].actual_position;
    float step_size = target - initial;
    
    if (fabsf(step_size) > 1.0f) {  // 確實是階躍
        // 找峰值
        float peak_value = final_value;
        uint32_t peak_index = g_sample_count - 1;
        
        for (uint32_t i = 0; i < g_sample_count; i++) {
            if (fabsf(g_data_buffer[i].actual_position - initial) > fabsf(peak_value - initial)) {
                peak_value = g_data_buffer[i].actual_position;
                peak_index = i;
            }
        }
        
        // 超調量
        metrics.overshoot_percent = ((peak_value - final_value) / step_size) * 100.0f;
        metrics.peak_time_ms = g_data_buffer[peak_index].timestamp_ms - g_data_buffer[0].timestamp_ms;
        
        // 上升時間 (10% → 90%)
        float threshold_10 = initial + step_size * 0.1f;
        float threshold_90 = initial + step_size * 0.9f;
        uint32_t rise_start = 0, rise_end = 0;
        
        for (uint32_t i = 0; i < g_sample_count; i++) {
            if (rise_start == 0 && fabsf(g_data_buffer[i].actual_position - threshold_10) < fabsf(step_size * 0.05f)) {
                rise_start = i;
            }
            if (rise_end == 0 && fabsf(g_data_buffer[i].actual_position - threshold_90) < fabsf(step_size * 0.05f)) {
                rise_end = i;
                break;
            }
        }
        
        if (rise_end > rise_start) {
            metrics.rise_time_ms = (g_data_buffer[rise_end].timestamp_ms - g_data_buffer[rise_start].timestamp_ms);
        }
        
        // 穩定時間 (誤差進入 ±2% 後不再出來)
        float settling_band = fabsf(step_size) * SETTLING_THRESHOLD / 100.0f;
        
        for (uint32_t i = g_sample_count - 1; i > 0; i--) {
            if (fabsf(g_data_buffer[i].error) > settling_band) {
                metrics.settling_time_ms = g_data_buffer[i].timestamp_ms - g_data_buffer[0].timestamp_ms;
                break;
            }
        }
    }
    
    // === 4. 穩定性判斷 ===
    metrics.is_stable = (metrics.steady_state_error < 5.0f);  // 穩態誤差 < 5°
    
    // 震盪檢測：檢查最後 20% 的數據是否有持續震盪
    uint32_t osc_start = g_sample_count * 4 / 5;
    int zero_crossings = 0;
    for (uint32_t i = osc_start + 1; i < g_sample_count; i++) {
        if ((g_data_buffer[i-1].error * g_data_buffer[i].error) < 0) {
            zero_crossings++;
        }
    }
    metrics.is_oscillating = (zero_crossings > 5);  // 超過 5 次過零點
    
    return metrics;
}

/**
 * @brief 打印性能指標報告
 */
void Print_Performance_Report(PerformanceMetrics_t *metrics) {
    printf("\r\n");
    printf("╔═══════════════════════════════════════════╗\r\n");
    printf("║       性能評估報告 (Performance Report)   ║\r\n");
    printf("╠═══════════════════════════════════════════╣\r\n");
    
    // 積分型指標
    printf("║ 積分型指標:                               ║\r\n");
    printf("║   IAE  (絕對誤差積分)    : %8.2f      ║\r\n", metrics->IAE);
    printf("║   ISE  (誤差平方積分)    : %8.2f      ║\r\n", metrics->ISE);
    printf("║   ITAE (時間加權誤差)    : %8.2f      ║\r\n", metrics->ITAE);
    printf("║   最大誤差               : %8.2f deg  ║\r\n", metrics->max_error);
    printf("║   穩態誤差               : %8.2f deg  ║\r\n", metrics->steady_state_error);
    printf("╠═══════════════════════════════════════════╣\r\n");
    
    // 階躍響應特性
    printf("║ 階躍響應特性:                             ║\r\n");
    printf("║   超調量                 : %8.1f %%    ║\r\n", metrics->overshoot_percent);
    printf("║   上升時間               : %8.0f ms   ║\r\n", metrics->rise_time_ms);
    printf("║   穩定時間               : %8.0f ms   ║\r\n", metrics->settling_time_ms);
    printf("║   峰值時間               : %8.0f ms   ║\r\n", metrics->peak_time_ms);
    printf("╠═══════════════════════════════════════════╣\r\n");
    
    // 穩定性判斷
    printf("║ 系統狀態:                                 ║\r\n");
    printf("║   穩定性                 : %s         ║\r\n", metrics->is_stable ? "✓ 穩定  " : "✗ 不穩定");
    printf("║   震盪狀態               : %s         ║\r\n", metrics->is_oscillating ? "✗ 震盪  " : "✓ 無震盪");
    printf("║   樣本數                 : %8lu      ║\r\n", metrics->num_samples);
    printf("╚═══════════════════════════════════════════╝\r\n");
    
    // 綜合評分
    float score = 0.0f;
    if (metrics->is_stable && !metrics->is_oscillating) {
        score = 100.0f - metrics->IAE * 0.5f - metrics->steady_state_error * 2.0f;
        if (score < 0) score = 0;
        if (score > 100) score = 100;
        printf("\r\n>>> 綜合評分: %.1f / 100 ", score);
        if (score > 80) printf("(優秀 ⭐⭐⭐)\r\n");
        else if (score > 60) printf("(良好 ⭐⭐)\r\n");
        else if (score > 40) printf("(可接受 ⭐)\r\n");
        else printf("(需改進)\r\n");
    } else {
        printf("\r\n>>> 系統不穩定或震盪，無法評分\r\n");
    }
}

// ==========================================================
// 3. 自動化測試模組
// ==========================================================

/**
 * @brief 階躍響應測試
 */
PerformanceMetrics_t Auto_Test_StepResponse(Motor_t *motor, float step_size, uint32_t duration_ms) {
    printf("\r\n>>> 執行階躍響應測試 (步距: %.1f deg, 時長: %lu ms)\r\n", 
           step_size, duration_ms);
    
    TestLog_Start();
    
    float initial_angle = Motor_GetAngle(motor);
    float target_angle = initial_angle + step_size;
    
    uint32_t start_time = HAL_GetTick();
    uint32_t last_sample_time = start_time;
    
    while ((HAL_GetTick() - start_time) < duration_ms) {
        // 控制週期
        if ((HAL_GetTick() - last_sample_time) >= SAMPLE_PERIOD_MS) {
            Motor_Update(motor);
            
            float actual_angle = Motor_GetAngle(motor);
            float actual_velocity = Motor_GetVelocity(motor);
            
            // 這裡簡化：直接設定速度（實際應該調用 PID 控制器）
            // 您需要改為調用 Robot_Loop 或 PID update
            float error = target_angle - actual_angle;
            int32_t cmd_rpm = (int32_t)(error * 50);  // 簡化的 P 控制
            Motor_SetSpeed(motor, cmd_rpm);
            
            // 記錄數據
            TestLog_Record(target_angle, actual_angle, (float)cmd_rpm, actual_velocity);
            
            last_sample_time = HAL_GetTick();
        }
    }
    
    Motor_SetSpeed(motor, 0);  // 停止
    
    // 評估性能
    PerformanceMetrics_t metrics = Evaluate_Performance();
    Print_Performance_Report(&metrics);
    
    return metrics;
}

/**
 * @brief 正弦波跟隨測試（評估動態性能）
 */
PerformanceMetrics_t Auto_Test_SineTracking(Motor_t *motor, float amplitude, float frequency, uint32_t duration_ms) {
    printf("\r\n>>> 執行正弦波跟隨測試 (幅度: %.1f deg, 頻率: %.2f Hz)\r\n", 
           amplitude, frequency);
    
    TestLog_Start();
    
    float initial_angle = Motor_GetAngle(motor);
    uint32_t start_time = HAL_GetTick();
    uint32_t last_sample_time = start_time;
    
    while ((HAL_GetTick() - start_time) < duration_ms) {
        if ((HAL_GetTick() - last_sample_time) >= SAMPLE_PERIOD_MS) {
            Motor_Update(motor);
            
            float time_s = (HAL_GetTick() - start_time) / 1000.0f;
            float target_angle = initial_angle + amplitude * sinf(2.0f * 3.14159f * frequency * time_s);
            float actual_angle = Motor_GetAngle(motor);
            float actual_velocity = Motor_GetVelocity(motor);
            
            // 簡化控制
            float error = target_angle - actual_angle;
            int32_t cmd_rpm = (int32_t)(error * 50);
            Motor_SetSpeed(motor, cmd_rpm);
            
            TestLog_Record(target_angle, actual_angle, (float)cmd_rpm, actual_velocity);
            
            last_sample_time = HAL_GetTick();
        }
    }
    
    Motor_SetSpeed(motor, 0);
    
    PerformanceMetrics_t metrics = Evaluate_Performance();
    Print_Performance_Report(&metrics);
    
    return metrics;
}

// ==========================================================
// 4. 參數掃描模組
// ==========================================================

/**
 * @brief Kp 參數掃描
 */
void Scan_Kp_Parameter(Motor_t *motor, float kp_start, float kp_end, int steps) {
    printf("\r\n╔═══════════════════════════════════════════╗\r\n");
    printf("║          Kp 參數掃描開始                  ║\r\n");
    printf("╚═══════════════════════════════════════════╝\r\n");
    
    float kp_step = (kp_end - kp_start) / (steps - 1);
    float best_kp = kp_start;
    float best_score = 999999.0f;
    
    printf("\r\nKp,IAE,ISE,Overshoot,SettlingTime,Stable,Score\r\n");
    
    for (int i = 0; i < steps; i++) {
        float kp = kp_start + i * kp_step;
        
        // TODO: 設定 PID 參數
        // joint1_pid.setKp(kp);
        
        // 執行測試
        PerformanceMetrics_t metrics = Auto_Test_StepResponse(motor, 30.0f, 3000);
        
        // 計算綜合分數（IAE 越小越好）
        float score = metrics.IAE + metrics.steady_state_error * 2.0f;
        if (!metrics.is_stable || metrics.is_oscillating) {
            score += 1000.0f;  // 懲罰不穩定
        }
        
        printf("%.2f,%.2f,%.2f,%.1f,%.0f,%d,%.2f\r\n",
               kp, metrics.IAE, metrics.ISE, metrics.overshoot_percent,
               metrics.settling_time_ms, metrics.is_stable, score);
        
        if (score < best_score && metrics.is_stable) {
            best_score = score;
            best_kp = kp;
        }
        
        HAL_Delay(1000);  // 間隔
    }
    
    printf("\r\n>>> 最佳 Kp = %.2f (分數: %.2f)\r\n", best_kp, best_score);
}

// ==========================================================
// 5. 主測試程序
// ==========================================================

/**
 * @brief 完整的自動化調參測試流程
 */
void Run_Comprehensive_Tuning_Test(void) {
    printf("\r\n");
    printf("╔═══════════════════════════════════════════════╗\r\n");
    printf("║      工業級 PID 調參輔助系統 v1.0            ║\r\n");
    printf("║      Industrial PID Tuning Assistant          ║\r\n");
    printf("╚═══════════════════════════════════════════════╝\r\n");
    
    // 測試 1: 基準階躍響應
    printf("\r\n【測試 1/3】基準階躍響應測試\r\n");
    PerformanceMetrics_t step_result = Auto_Test_StepResponse(&motor_joint_13pin, 30.0f, 5000);
    HAL_Delay(2000);
    
    // 測試 2: 正弦跟隨
    printf("\r\n【測試 2/3】正弦波跟隨測試\r\n");
    PerformanceMetrics_t sine_result = Auto_Test_SineTracking(&motor_joint_13pin, 20.0f, 0.5f, 8000);
    HAL_Delay(2000);
    
    // 測試 3: 快速階躍
    printf("\r\n【測試 3/3】快速階躍測試\r\n");
    PerformanceMetrics_t fast_step = Auto_Test_StepResponse(&motor_joint_13pin, 15.0f, 2000);
    
    // 綜合評估
    printf("\r\n╔═══════════════════════════════════════════════╗\r\n");
    printf("║              綜合評估結果                     ║\r\n");
    printf("╠═══════════════════════════════════════════════╣\r\n");
    printf("║ 階躍響應 IAE    : %10.2f                 ║\r\n", step_result.IAE);
    printf("║ 正弦跟隨 IAE    : %10.2f                 ║\r\n", sine_result.IAE);
    printf("║ 快速響應 IAE    : %10.2f                 ║\r\n", fast_step.IAE);
    printf("╚═══════════════════════════════════════════════╝\r\n");
    
    // 輸出原始數據供外部分析
    printf("\r\n>>> 是否需要輸出原始數據？(y/n)\r\n");
    // TestLog_Stop_And_Export();  // 如需要可取消註解
}

/**
 * @brief 互動式調參助手
 */
void Interactive_Tuning_Assistant(void) {
    printf("\r\n=== 互動式調參助手 ===\r\n");
    printf("請選擇測試項目:\r\n");
    printf("1. 階躍響應測試\r\n");
    printf("2. 正弦跟隨測試\r\n");
    printf("3. Kp 參數掃描\r\n");
    printf("4. 完整測試流程\r\n");
    printf("5. 輸出數據 (CSV)\r\n");
    printf(">> 請輸入選項 (1-5): ");
    
    // 這裡可以加入 UART 輸入處理
    // 簡化版：直接執行完整測試
    Run_Comprehensive_Tuning_Test();
}
