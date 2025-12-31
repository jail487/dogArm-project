# DogHead 書法手臂機器人 - Pinout 配置文件

## 專案資訊
- **專案名稱**: 書法手臂機器人 (Calligraphy Robot Arm)
- **微控制器**: STM32F446RET6
- **開發板**: NUCLEO-F446RE
- **機構類型**: 平面五連桿機構 (Five-bar Linkage)
- **馬達類型**: Nidec 無刷直流馬達 (24H702U030 與 24H220Q231)
- **最後更新**: 2025-12-30

---

## 系統時鐘配置

| 參數 | 設定值 |
|------|--------|
| 系統時鐘源 | HSI (內部振盪器) |
| PLL 輸入 | HSI 16MHz |
| PLL_M | 16 |
| PLL_N | 336 |
| PLL_P | 4 |
| SYSCLK | 84 MHz |
| AHB 時鐘 | 84 MHz |
| APB1 時鐘 | 42 MHz |
| APB2 時鐘 | 84 MHz |

---

## GPIO 引腳配置

### 系統與調試接口

| Pin | 功能 | 說明 |
|-----|------|------|
| PA13 | SWDIO | SWD 調試接口 - 數據線 |
| PA14 | SWCLK | SWD 調試接口 - 時鐘線 |
| PB3 | SWO | SWD 追蹤輸出 |
| PC13 | B1 (USER Button) | 板載用戶按鈕 (下降沿觸發中斷) |
| PA5 | LD2 (LED) | 板載綠色 LED (Output) |

### 通訊接口

| Pin | 功能 | 說明 |
|-----|------|------|
| PA2 | USART2_TX | 串口傳輸 (115200 baud) |
| PA3 | USART2_RX | 串口接收 (115200 baud) |

---

## 馬達驅動系統配置

### 關節 1 - 13-Pin 馬達 (型號: 24H702U030)

**控制模式**: 頻率控制 (Frequency Control)

| 功能 | Pin | Timer/GPIO | 說明 |
|------|-----|-----------|------|
| 速度控制 PWM | PA5* | TIM2_CH1 | STMP 引腳，產生頻率信號 (400 pulses/rev) |
| 方向控制 | PC2 | GPIO Output | DIR 引腳 (Low=CW, High=CCW) |
| 啟動控制 | PC1 | GPIO Output | START 引腳 (High=Enable, Low=Disable) |
| 編碼器 A 相 | PB6 | TIM4_CH1 | Encoder Mode (100 PPR) |
| 編碼器 B 相 | PB7 | TIM4_CH2 | Encoder Mode (100 PPR) |

**馬達參數**:
- 額定轉速: 3000 RPM
- 最大轉速: 6000 RPM
- 編碼器解析度: 100 PPR (x4 模式 = 400 pulses/rev)
- 減速比: 50:1 (需依實際機構調整)

**頻率計算公式**: `Freq (Hz) = RPM × 400 / 60`

---

### 關節 2 - 8-Pin 馬達 (型號: 24H220Q231)

**控制模式**: PWM 佔空比控制 (Low Active)

| 功能 | Pin | Timer/GPIO | 說明 |
|------|-----|-----------|------|
| 速度控制 PWM | PB4 | TIM3_CH1 | Low Active (100% Duty = 停止, 0% Duty = 最快) |
| 方向控制 | PC5 | GPIO Output | DIR 引腳 (High=CW, Low=CCW) |
| 煞車控制 | PC4 | GPIO Output | BRAKE 引腳 (High=Run, Low=Brake) |
| 編碼器 A 相 | PA8 | TIM1_CH1 | Encoder Mode (100 PPR) |
| 編碼器 B 相 | PA9 | TIM1_CH2 | Encoder Mode (100 PPR) |

**馬達參數**:
- 額定轉速: 3150 RPM
- 最大轉速: 6300 RPM
- 編碼器解析度: 100 PPR (x4 模式 = 400 pulses/rev)
- 減速比: 30:1 (需依實際機構調整)

**速度計算公式**: `CCR = ARR × (1 - Speed_Ratio)` (Low Active 特性)

---

## Timer 配置

### TIM1 - 編碼器模式 (8-Pin 馬達)
- **模式**: Encoder Mode x4
- **通道**: CH1 (PA8), CH2 (PA9)
- **ARR**: 65535 (最大值)
- **用途**: 讀取關節 2 位置

### TIM2 - PWM 頻率產生 (13-Pin 馬達速度控制)
- **模式**: PWM Generation CH1
- **通道**: CH1 (PA5)
- **用途**: 產生動態頻率信號控制 13-Pin 馬達轉速

### TIM3 - PWM 佔空比控制 (8-Pin 馬達速度控制)
- **模式**: PWM Generation CH1
- **通道**: CH1 (PB4)
- **用途**: 產生 PWM 信號控制 8-Pin 馬達轉速 (Low Active)

### TIM4 - 編碼器模式 (13-Pin 馬達)
- **模式**: Encoder Mode x4
- **通道**: CH1 (PB6), CH2 (PB7)
- **ARR**: 65535 (最大值)
- **用途**: 讀取關節 1 位置

---

## 控制架構說明

### 軟體模組結構

```
main.c (主程式)
├── nidec_motor_driver.c/h (馬達底層驅動 - C)
│   ├── Motor_System_Config() - 硬體初始化
│   ├── Motor_Update() - 編碼器讀取與溢位處理
│   ├── Motor_SetSpeed() - 設定馬達轉速
│   └── Motor_GetAngle() - 取得關節角度
│
├── robot_arm_core.cpp (高層控制邏輯 - C++)
│   ├── Robot_Init() - 系統初始化
│   ├── Robot_Loop() - 主控制迴圈 (PID)
│   └── Robot_SetTarget() - 設定目標位置
│
├── pid_controller.hpp (PID 控制器)
│   └── PositionController::update() - 位置控制演算法
│
└── kinematics.hpp (運動學解算)
    ├── solveIK() - 逆運動學 (座標 → 角度)
    └── solveFK() - 正運動學 (角度 → 座標)
```

### PID 控制參數

| 控制器 | Kp | Ki | Kd | 最大輸出 (RPM) |
|--------|----|----|----| --------------|
| 關節 1 (大臂) | 5.0 | 0.1 | 0.0 | 3000 |
| 關節 2 (小臂) | 8.0 | 0.2 | 0.0 | 4000 |

### 運動學參數 (範例，需依實際機構測量)

| 參數 | 符號 | 數值 (mm) | 說明 |
|------|------|-----------|------|
| 主動臂長度 | L1 | 150 | 連接馬達的短臂 |
| 從動臂長度 | L2 | 200 | 連接末端的長臂 |
| 馬達間距 | D | 100 | 兩馬達軸心距離 |

---

## 控制流程

### 初始化流程
1. HAL 庫初始化 (`HAL_Init()`)
2. 系統時鐘配置 (`SystemClock_Config()`)
3. GPIO 初始化 (`MX_GPIO_Init()`)
4. UART 初始化 (`MX_USART2_UART_Init()`)
5. 馬達系統初始化 (`Robot_Init()`)
   - 配置馬達參數
   - 啟動編碼器 Timer
   - 啟動馬達 Enable

### 主迴圈流程 (建議 1-10ms 週期)
1. 更新編碼器數據 (`Motor_Update()`)
2. 讀取當前關節角度 (`Motor_GetAngle()`)
3. PID 計算目標轉速 (`joint_pid.update()`)
4. 發送轉速命令給馬達 (`Motor_SetSpeed()`)

---

## 重要注意事項

### 硬體連接
1. ⚠️ **編碼器方向**: 安裝時需確認編碼器 A/B 相位與馬達旋轉方向的對應關係
2. ⚠️ **減速比設定**: 代碼中的減速比 (50:1, 30:1) 為範例值，請依實際減速機規格修改
3. ⚠️ **電源供應**: 馬達電源與邏輯電源需分離，共地即可
4. ⚠️ **PWM 頻率**: TIM2/TIM3 的基礎頻率需設定適當以支援馬達控制範圍

### 安全考量
1. 系統啟動時應先確認機械結構在安全位置
2. 建議加入軟體限位保護 (Angle Limits)
3. 緊急停止機制 (可使用板載按鈕 PC13)
4. 過流保護電路 (硬體層面)

### 調試建議
1. 使用 UART2 輸出調試信息 (115200 baud)
2. 可透過 SWO 輸出即時數據
3. 建議先單獨測試每個馬達的編碼器讀數
4. PID 參數需依實際負載調整

---

## 待實作功能

- [ ] 軌跡規劃器 (Trajectory Planning)
- [ ] 速度前饋控制 (Feedforward)
- [ ] 碰撞檢測 (Collision Detection)
- [ ] 示教模式 (Teaching Mode)
- [ ] USB 通訊協定 (USB CDC)
- [ ] 參數儲存至 Flash (Non-volatile Storage)

---

## 參考資料

- STM32F446RE Datasheet
- STM32 HAL Library Documentation
- Nidec 24H702U030 規格書
- Nidec 24H220Q231 規格書
- 五連桿機構運動學理論

---

*此文件根據 Core/Src 與 Core/Inc 中的實際代碼配置自動生成*
