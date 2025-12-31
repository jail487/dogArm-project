# dogArm Project - 書法機械臂專案

[English](#english) | [中文](#chinese)

---

## <a name="english"></a>English

### Overview
dogArm is a robotic arm project designed to write Chinese calligraphy (Kaishu/Regular Script) using a double parallel linkage mechanism. This repository contains both firmware for embedded control and software for motion planning and coordination.

**⚠️ Important**: This is a starter template with placeholder kinematics. You MUST customize the kinematics equations for your specific parallel linkage mechanism. See [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md) for details.

### Project Structure
```
dogArm-project/
├── firmware/           # Embedded system firmware (Arduino/ESP32)
│   ├── src/           # Main firmware source code
│   ├── lib/           # Custom libraries
│   └── platformio.ini # PlatformIO configuration
├── software/          # Control software (Python)
│   ├── control/       # Robot control modules
│   ├── kinematics/    # Inverse kinematics solver
│   ├── calligraphy/   # Calligraphy path planning
│   └── communication/ # Serial/network communication
├── docs/              # Documentation
└── examples/          # Example calligraphy characters
```

### Features
- **Double Parallel Linkage Mechanism**: Provides stable and precise motion control
- **Inverse Kinematics**: Real-time calculation for smooth trajectory tracking
- **Calligraphy Path Planning**: Converts Chinese characters to robot trajectories
- **Multi-platform Support**: Arduino/ESP32 firmware with Python control software

### Hardware Requirements
- Microcontroller: ESP32 or Arduino Mega
- Motors: 2x Stepper motors or Servo motors
- End effector: Brush holder with vertical axis control
- Power supply: 12V DC adapter
- Mechanical frame: Double parallel linkage structure

### Software Requirements
- Python 3.7+
- PlatformIO (for firmware development)
- Required Python packages (see requirements.txt)

### Quick Start
See [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md) for detailed setup instructions.

**Important**: Before using, read [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md) to understand current limitations and required customizations.

---

## <a name="chinese"></a>中文

### 專案概述
dogArm 是一個基於雙平行連桿機構的機械臂專案，設計用於書寫中文書法（楷書）。本儲存庫包含嵌入式控制的韌體以及運動規劃和協調的軟體。

**⚠️ 重要提示**：這是一個包含佔位運動學的起始模板。您必須為您的特定平行連桿機構自訂運動學方程式。詳見 [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md)。

### 專案結構
```
dogArm-project/
├── firmware/           # 嵌入式系統韌體 (Arduino/ESP32)
│   ├── src/           # 主要韌體原始碼
│   ├── lib/           # 自訂函式庫
│   └── platformio.ini # PlatformIO 配置
├── software/          # 控制軟體 (Python)
│   ├── control/       # 機器人控制模組
│   ├── kinematics/    # 逆運動學求解器
│   ├── calligraphy/   # 書法路徑規劃
│   └── communication/ # 串列/網路通訊
├── docs/              # 文件
└── examples/          # 範例書法字元
```

### 功能特色
- **雙平行連桿機構**：提供穩定且精確的運動控制
- **逆運動學**：即時計算以實現平滑的軌跡追蹤
- **書法路徑規劃**：將中文字元轉換為機器人軌跡
- **多平台支援**：Arduino/ESP32 韌體配合 Python 控制軟體

### 硬體需求
- 微控制器：ESP32 或 Arduino Mega
- 馬達：2x 步進馬達或伺服馬達
- 末端執行器：毛筆固定座及垂直軸控制
- 電源供應：12V DC 變壓器
- 機械框架：雙平行連桿結構

### 軟體需求
- Python 3.7+
- PlatformIO（用於韌體開發）
- 所需 Python 套件（參見 requirements.txt）

### 快速開始
詳細的設定說明請參閱 [docs/GETTING_STARTED.md](docs/GETTING_STARTED.md)。

**重要提示**：使用前，請閱讀 [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md) 以了解目前的限制和必要的自訂設定。

---

### License
MIT License

### Contributors
Final Project Team