# DogHead 書法機械手臂 - 專案完整技術文檔

**版本**: v3.0 (FreeRTOS + micro-ROS 架構版)  
**最後更新**: 2026-01-01  
**微控制器**: STM32F446RET6  
**開發板**: NUCLEO-F446RE  
**作業系統**: FreeRTOS  
**中間件**: micro-ROS (Humble)

---

# 1. 專案狀態更新 (2026-01-01)

## 1.1 重大架構變更
本專案已從裸機 (Bare-Metal) 架構遷移至 **FreeRTOS + micro-ROS** 架構，以支援更複雜的通訊需求與即時控制任務。

### 新舊架構對比
| 特性 | 舊架構 (v2.0) | 新架構 (v3.0) |
|------|--------------|--------------|
| **核心排程** | `while(1)` 主迴圈 + Timer 中斷 | FreeRTOS 搶佔式多工 (Preemptive Multitasking) |
| **通訊協定** | 簡單 UART 字串 | micro-ROS (DDS over UART) |
| **控制頻率** | 依賴迴圈速度 (不穩定) | `ControlTask` 嚴格定時 (1kHz) |
| **模組化** | 函式呼叫耦合 | 獨立任務 (Tasks) 解耦 |

## 1.2 FreeRTOS 任務規劃
目前系統配置了三個主要任務，依優先級排序：

| 任務名稱 | 優先級 | 頻率 | 堆疊大小 | 職責 |
|---------|--------|------|----------|------|
| **ControlTask** | `osPriorityRealtime` (3) | 1000 Hz (1ms) | 512 Words | 負責 PID 計算、運動學解算、軌跡規劃。最高優先級以確保控制穩定性。 |
| **CommTask** | `osPriorityHigh` (2) | 10 Hz (100ms) | 512 Words | 負責處理非即時通訊、系統診斷數據回報。 |
| **defaultTask** | `osPriorityNormal` (1) | - | 3000 Words | 負責 micro-ROS 節點初始化、Executor 運行 (處理訂閱與服務)。 |

## 1.3 micro-ROS 整合狀態
- **Library**: `libmicroros.a` (Humble 版本) 已針對 Cortex-M4 靜態編譯並連結。
- **Transport**: 使用自定義 UART 傳輸層 (`it_transport.c`)，基於中斷模式以避免阻塞。
- **記憶體管理**: 整合了 FreeRTOS 的記憶體分配器 (`microros_allocators.c`)。
- **目前進度**: 
    - [x] 靜態庫編譯完成
    - [x] IDE 專案路徑配置完成
    - [x] 傳輸層與分配器程式碼整合
    - [x] 任務架構重構完成
    - [ ] Publisher/Subscriber 應用邏輯實作 (待辦)

---

# 2. 硬體配置

## 2.1 系統時鐘配置

| 參數 | 設定值 |
|------|--------|
| 系統時鐘源 | HSI (內部振盪器) |
| **SYSCLK** | **84 MHz** |
| AHB 時鐘 | 84 MHz |
| APB1 時鐘 | 42 MHz |
| APB2 時鐘 | 84 MHz |

## 2.2 GPIO 引腳配置

### 系統與調試接口
| Pin | 功能 | 說明 |
|-----|------|------|
| PA13/14 | SWD | 調試接口 |
| PC13 | B1 | 用戶按鈕 |
| PA5 | LD2 | 板載 LED |

### 通訊接口 (micro-ROS)
| Pin | 功能 | 說明 |
|-----|------|------|
| PA2 | USART2_TX | micro-ROS 傳輸 (115200 baud) |
| PA3 | USART2_RX | micro-ROS 接收 (115200 baud) |

## 2.3 馬達驅動系統配置

### 關節 1 - 13-Pin 馬達 (24H702U030)
- **控制模式**: 頻率控制 (Frequency Control)
- **PWM**: PA5 (TIM2_CH1)
- **編碼器**: PB6/PB7 (TIM4_CH1/CH2)
- **減速比**: 50:1
- **頻率公式**: `Freq (Hz) = RPM × 400 / 60`

### 關節 2 - 8-Pin 馬達 (24H220Q231)
- **控制模式**: PWM 佔空比控制 (Low Active)
- **PWM**: PB4 (TIM3_CH1)
- **編碼器**: PA8/PA9 (TIM1_CH1/CH2)
- **減速比**: 30:1
- **速度公式**: `CCR = ARR × (1 - Speed_Ratio)`

---

# 3. 軟體架構與控制流程

## 3.1 系統模組結構
```
doghead_arm/
├── Core/
│   ├── Inc/
│   │   ├── pid_controller.hpp         ← PID+前饋控制器
│   │   ├── kinematics.hpp             ← 運動學解算
│   │   └── nidec_motor_driver.h       ← 馬達驅動 API
│   └── Src/
│       ├── main.c                     ← FreeRTOS 初始化
│       ├── freertos.c                 ← 任務實作 (ControlTask, CommTask)
│       ├── robot_arm_core.cpp         ← 機器人核心邏輯
│       ├── it_transport.c             ← micro-ROS UART 傳輸
│       └── microros_allocators.c      ← 記憶體管理
```

## 3.2 控制流程 (ControlTask)
此任務以 1kHz 頻率運行 (`vTaskDelayUntil`)：

1.  **讀取感測器**: `Motor_Update` 讀取編碼器數值。
2.  **運動學解算 (IK)**: 將目標座標 (X, Y) 轉換為關節角度。
3.  **安全檢查**: 檢查是否超出虛擬圍籬或工作空間。
4.  **軌跡規劃**: 計算目標速度與加速度前饋量。
5.  **PID 控制**: 
    ```cpp
    output = PID(error) + Kv * TargetVel + Ka * TargetAcc
    ```
6.  **馬達輸出**: 更新 PWM 或頻率指令。

---

# 4. PID 前饋控制架構詳解

## 4.1 PositionController 類別
**檔案位置**: `Core/Inc/pid_controller.hpp`

### 參數說明
| 參數 | 說明 | 典型值 | 調整建議 |
|------|------|--------|----------|
| `kp` | 比例增益 | 5.0~10.0 | 基礎響應力道 |
| `ki` | 積分增益 | 0.1~0.5 | 消除穩態誤差 |
| `kd` | 微分增益 | 0.0~1.0 | 抑制震盪 |
| `kv` | 速度前饋 | 1.0 | **關鍵**: 消除動態滯後 |
| `ka` | 加速度前饋 | 0.1 | 補償慣性 |

### 控制算法
```cpp
// 1. 誤差計算
error = target_pos - current_pos;

// 2. PID 反饋
feedback = Kp*error + Ki*integral + Kd*derivative;

// 3. 前饋補償 (Feedforward)
feedforward = (target_vel * Kv) + (target_acc * Ka);

// 4. 總輸出
output_rpm = feedback + feedforward;
```

## 4.2 軌跡規劃器 (TrajectoryPlanner)
為了發揮前饋控制的效果，目標位置不能突變，必須經過平滑處理以產生連續的速度與加速度曲線。
- **輸入**: 階躍目標位置
- **處理**: 一階低通濾波 + 差分計算
- **輸出**: 平滑位置、速度、加速度

---

# 5. 工業級調參工作流程

## 5.1 調參步驟
本專案採用量化數據分析法進行調參，而非憑感覺調整。

### 階段 1: 基準測試
使用 `Run_Comprehensive_Tuning_Test()` 執行自動化測試，採集階躍響應數據。

### 階段 2: 數據分析
觀察以下指標：
- **IAE (積分絕對誤差)**: 評估整體跟隨性能。
- **超調量 (Overshoot)**: 評估穩定性 (<10% 為佳)。
- **穩定時間 (Settling Time)**: 評估收斂速度。

### 階段 3: 參數優化策略
1.  **系統震盪** → 降低 Kp，增加 Kd。
2.  **響應遲緩** → 增加 Kp，啟用 Kv (設為 1.0)。
3.  **穩態誤差** → 增加 Ki。
4.  **啟動抖動** → 降低 Ka 或減小軌跡規劃器的加速度限制。

---

# 6. 待辦事項 (Todo)

## 6.1 近期目標
- [ ] **硬體驗證**: 在實際開發板上測試 FreeRTOS 任務切換是否正常。
- [ ] **micro-ROS 實作**:
    - [ ] 建立 Publisher 發布關節角度 (`sensor_msgs/JointState`)。
    - [ ] 建立 Subscriber 接收目標位置 (`geometry_msgs/Point`)。
- [ ] **歸零程序**: 實作開機自動歸零邏輯。

## 6.2 長期目標
- [ ] 支援 MoveIt! 軌跡控制。
- [ ] 增加電流環控制 (需硬體支援)。

---

**文檔維護者**: GitHub Copilot  
**原始檔案整合**: `完整技術文檔.md`, `Pinout_configuration.md`, `PID_Feedforward_Control_Architecture.md`, `工業級調參工作流程.md`
