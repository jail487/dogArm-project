# Getting Started with dogArm

[English](#english) | [中文](#chinese)

---

## <a name="english"></a>English

### Prerequisites

#### Hardware
- ESP32 development board or Arduino Mega 2560
- 2x Stepper motors with drivers (e.g., NEMA 17 with A4988/DRV8825)
- 1x Servo motor (for brush control)
- 12V power supply (for motors)
- Double parallel linkage mechanical frame
- Brush holder assembly
- Limit switches (for homing)

#### Software
- Python 3.7 or higher
- PlatformIO (for firmware development)
- USB cable for programming

### Installation

#### 1. Install Python Dependencies

```bash
cd dogArm-project
pip install -r requirements.txt
```

#### 2. Install PlatformIO

Follow the installation guide at: https://platformio.org/install

Or install via pip:
```bash
pip install platformio
```

#### 3. Configure Hardware

Edit `firmware/src/config.h` to match your hardware setup:
- Pin assignments for motors and servo
- Motor steps per revolution and microstepping
- Link lengths and mechanical parameters
- Workspace limits

#### 4. Upload Firmware

```bash
cd firmware
# For ESP32
pio run -e esp32dev -t upload

# For Arduino Mega
pio run -e megaatmega2560 -t upload
```

#### 5. Test Connection

```bash
# Open serial monitor
pio device monitor

# You should see:
# dogArm Firmware v1.0
# Initializing...
# Ready! Waiting for commands...
```

### Quick Test

#### Run Basic Control Example

1. Edit `examples/basic_control.py` and set your serial port:
   ```python
   port = '/dev/ttyUSB0'  # Linux
   # port = 'COM3'        # Windows
   # port = '/dev/cu.usbserial-*'  # macOS
   ```

2. Run the example:
   ```bash
   python examples/basic_control.py
   ```

#### Run Calligraphy Example

1. Edit `examples/write_character.py` and set your serial port
2. Run the example:
   ```bash
   python examples/write_character.py
   ```

### Basic Usage

```python
from software.control import RobotController

# Connect to robot
robot = RobotController('/dev/ttyUSB0')
robot.connect()

# Home the robot
robot.home()

# Move to position
robot.move_to(x=100, y=200, z=10)

# Pen control
robot.pen_down()
robot.pen_up()

# Disconnect
robot.disconnect()
```

### Serial Commands

The firmware accepts the following commands:

- `HOME` - Home the robot
- `MOVE:x,y,z` - Move to Cartesian coordinates (mm)
- `SPEED:value` - Set movement speed (steps/sec)
- `PEN:UP` or `PEN:DOWN` - Control pen position
- `STATUS` - Get current status
- `POS` - Get current position
- `STOP` - Emergency stop

### Troubleshooting

**Problem**: Cannot connect to robot
- Check USB cable connection
- Verify correct serial port
- Check if port is not in use by another program
- On Linux, you may need to add your user to `dialout` group:
  ```bash
  sudo usermod -a -G dialout $USER
  ```

**Problem**: Robot not moving smoothly
- Adjust `MAX_SPEED` and `MAX_ACCELERATION` in `config.h`
- Check motor driver current settings
- Verify mechanical assembly for binding

**Problem**: Position accuracy issues
- Verify link lengths in `config.h` match actual hardware
- Check for mechanical backlash
- Ensure limit switches are properly positioned

---

## <a name="chinese"></a>中文

### 前置需求

#### 硬體
- ESP32 開發板或 Arduino Mega 2560
- 2x 步進馬達及驅動器（例如：NEMA 17 配 A4988/DRV8825）
- 1x 伺服馬達（用於毛筆控制）
- 12V 電源供應器（供馬達使用）
- 雙平行連桿機械框架
- 毛筆固定座組件
- 限位開關（用於歸位）

#### 軟體
- Python 3.7 或更高版本
- PlatformIO（用於韌體開發）
- USB 傳輸線（用於程式上傳）

### 安裝

#### 1. 安裝 Python 相依套件

```bash
cd dogArm-project
pip install -r requirements.txt
```

#### 2. 安裝 PlatformIO

請參閱安裝指南：https://platformio.org/install

或透過 pip 安裝：
```bash
pip install platformio
```

#### 3. 配置硬體

編輯 `firmware/src/config.h` 以符合您的硬體設定：
- 馬達和伺服馬達的腳位分配
- 馬達每轉步數和微步設定
- 連桿長度和機械參數
- 工作空間限制

#### 4. 上傳韌體

```bash
cd firmware
# 針對 ESP32
pio run -e esp32dev -t upload

# 針對 Arduino Mega
pio run -e megaatmega2560 -t upload
```

#### 5. 測試連線

```bash
# 開啟序列埠監視器
pio device monitor

# 您應該會看到：
# dogArm Firmware v1.0
# Initializing...
# Ready! Waiting for commands...
```

### 快速測試

#### 執行基本控制範例

1. 編輯 `examples/basic_control.py` 並設定您的序列埠：
   ```python
   port = '/dev/ttyUSB0'  # Linux
   # port = 'COM3'        # Windows
   # port = '/dev/cu.usbserial-*'  # macOS
   ```

2. 執行範例：
   ```bash
   python examples/basic_control.py
   ```

#### 執行書法範例

1. 編輯 `examples/write_character.py` 並設定您的序列埠
2. 執行範例：
   ```bash
   python examples/write_character.py
   ```

### 基本使用

```python
from software.control import RobotController

# 連接機器人
robot = RobotController('/dev/ttyUSB0')
robot.connect()

# 歸位
robot.home()

# 移動到指定位置
robot.move_to(x=100, y=200, z=10)

# 筆控制
robot.pen_down()
robot.pen_up()

# 斷線
robot.disconnect()
```

### 序列埠指令

韌體接受以下指令：

- `HOME` - 機器人歸位
- `MOVE:x,y,z` - 移動到笛卡爾座標（單位：mm）
- `SPEED:value` - 設定移動速度（步/秒）
- `PEN:UP` 或 `PEN:DOWN` - 控制筆的位置
- `STATUS` - 取得目前狀態
- `POS` - 取得目前位置
- `STOP` - 緊急停止

### 疑難排解

**問題**：無法連接到機器人
- 檢查 USB 傳輸線連接
- 驗證正確的序列埠
- 檢查埠是否未被其他程式使用
- 在 Linux 上，您可能需要將使用者加入 `dialout` 群組：
  ```bash
  sudo usermod -a -G dialout $USER
  ```

**問題**：機器人移動不順暢
- 調整 `config.h` 中的 `MAX_SPEED` 和 `MAX_ACCELERATION`
- 檢查馬達驅動器電流設定
- 驗證機械組件是否有卡住

**問題**：位置精度問題
- 驗證 `config.h` 中的連桿長度是否符合實際硬體
- 檢查機械回差
- 確保限位開關正確定位
