/**
 * @file robot_arm_core.cpp
 * @brief 機器手臂核心邏輯 (v4.0 - 運動學與驅動整合)
 */

#include "mainpp.h"
#include "pid_controller.hpp"
#include "nidec_motor_driver.h"
#include "kinematics.hpp"
#include <queue>
#include <cmath>

// ==========================================================
// 機構參數設定 (請依照實際硬體測量修改！)
// 單位: mm
// ==========================================================
// 假設五連桿參數
#define LINK_L1  100.0f  // 主動臂 (連接馬達)
#define LINK_L2  150.0f  // 從動臂 (連接末端)
#define MOTOR_DIST_D 60.0f // 兩馬達間距

// 建立運動學解算器實體
FiveBarKinematics kinematics(LINK_L1, LINK_L2, MOTOR_DIST_D);

// ==========================================================
// 軌跡規劃器 (Trajectory Planner) - 產生速度與加速度前饋
// ==========================================================
class TrajectoryPlanner {
public:
    TrajectoryPlanner() : _prev_target(0.0f), _prev_velocity(0.0f), _velocity(0.0f), _acceleration(0.0f) {}

    /**
     * @brief 計算平滑的速度和加速度 (使用一階濾波)
     * @param current_target 當前目標位置 (Degree)
     * @param dt 時間間隔 (s)
     * @param max_velocity 最大速度限制 (Deg/s)
     * @param max_acceleration 最大加速度限制 (Deg/s²)
     */
    void update(float current_target, float dt, float max_velocity = 360.0f, float max_acceleration = 1800.0f) {
        // 計算原始速度 (位置差分)
        float raw_velocity = (current_target - _prev_target) / dt;
        
        // 速度限幅
        if (raw_velocity > max_velocity) raw_velocity = max_velocity;
        else if (raw_velocity < -max_velocity) raw_velocity = -max_velocity;
        
        // 一階低通濾波 (平滑速度，避免抖動)
        const float alpha = 0.7f; // 濾波系數 (0~1, 越小越平滑)
        _velocity = alpha * raw_velocity + (1.0f - alpha) * _prev_velocity;
        
        // 計算加速度 (速度差分)
        _acceleration = (_velocity - _prev_velocity) / dt;
        
        // 加速度限幅
        if (_acceleration > max_acceleration) _acceleration = max_acceleration;
        else if (_acceleration < -max_acceleration) _acceleration = -max_acceleration;
        
        // 更新記錄
        _prev_target = current_target;
        _prev_velocity = _velocity;
    }

    float getVelocity() const { return _velocity; }
    float getAcceleration() const { return _acceleration; }
    
    void reset() {
        _prev_target = 0.0f;
        _prev_velocity = 0.0f;
        _velocity = 0.0f;
        _acceleration = 0.0f;
    }

private:
    float _prev_target;
    float _prev_velocity;
    float _velocity;
    float _acceleration;
};

// 為每個關節建立軌跡規劃器
TrajectoryPlanner traj_joint1;
TrajectoryPlanner traj_joint2;

// ==========================================================
// PID 控制器與變數 (含前饋參數)
// ==========================================================
// 參數說明: (Kp, Ki, Kd, Kv_速度前饋, Ka_加速度前饋, max_rpm)
// Kv: 對於速度控制馬達，通常設為 1.0 (直接對應速度指令)
// Ka: 加速度補償係數，用於補償慣量，建議從 0.05~0.2 開始調整

// 關節 1 (13-Pin 馬達 - 24H702U030)
PositionController joint1_pid(5.0f, 0.1f, 0.0f, 1.0f, 0.1f, 3000.0f);

// 關節 2 (8-Pin 馬達 - 24H220Q231)
PositionController joint2_pid(8.0f, 0.2f, 0.0f, 1.0f, 0.15f, 4000.0f);

// 測試目標 (座標模式)
float target_x = 0.0f;
float target_y = 150.0f; // 預設停在前方
bool ik_mode_enabled = false;

// 測試模式變數
bool test_mode = false;
int32_t test_rpm_motor1 = 0;  // 測試模式下馬達1的目標轉速
int32_t test_rpm_motor2 = 0;  // 測試模式下馬達2的目標轉速

// PVT 緩衝區 (未來使用，目前先留著結構)
struct TrajectoryPoint {
    float theta1, theta2, z, duration;
};
std::queue<TrajectoryPoint> traj_buffer;


// ==========================================================
// 1. 初始化
// ==========================================================
extern "C" void Robot_Init(void) {
    // 呼叫 C 語言底層驅動初始化
    Motor_System_Config();

    // 重置 PID 狀態
    joint1_pid.reset();
    joint2_pid.reset();
    
    // 重置軌跡規劃器
    traj_joint1.reset();
    traj_joint2.reset();

    // 預設目標設為當前位置 (防止開機暴衝)
    // 注意：這裡假設開機時已經在某個合理位置，且已手動歸零
    // 如果沒有歸零，Encoder 值會是 0，IK 可能解不出來
    target_x = 0.0f;
    target_y = 150.0f;
}

// ==========================================================
// 2. 設定目標 API (給 main.c 測試用)
// ==========================================================
extern "C" void Robot_SetTargetPosition(float x, float y) {
    target_x = x;
    target_y = y;
    ik_mode_enabled = true;
}

// ==========================================================
// 測試模式 API
// ==========================================================
extern "C" void Robot_SetTestMode(bool enable) {
    test_mode = enable;
    if (enable) {
        ik_mode_enabled = false;  // 測試模式下關閉運動學控制
    }
}

extern "C" void Robot_SetTestSpeed(int32_t rpm_motor1, int32_t rpm_motor2) {
    test_rpm_motor1 = rpm_motor1;
    test_rpm_motor2 = rpm_motor2;
}
// ==========================================================
// 3. 核心控制迴圈 (請在 Timer 中斷或 main loop 固定呼叫)
// ==========================================================
extern "C" void Robot_Loop(float dt_seconds) {
    // --- 測試模式：直接控制馬達速度 ---
    if (test_mode == true) {
        // 更新編碼器數據（仍需讀取位置回饋）
        Motor_Update(&motor_joint_13pin);
        Motor_Update(&motor_joint_8pin);
        
        // 確保馬達啟動
        Motor_Start(&motor_joint_13pin);
        Motor_Start(&motor_joint_8pin);
        
        // 直接設定速度，不經過 PID 和運動學
        Motor_SetSpeed(&motor_joint_13pin, test_rpm_motor1);
        Motor_SetSpeed(&motor_joint_8pin, test_rpm_motor2);
        
        return;  // 測試模式下不執行後續的運動學和 PID 控制
    }
    // --- 步驟 A: 讀取感測器 (Feedback) ---
    // 呼叫底層驅動讀取 Encoder
    Motor_Update(&motor_joint_13pin);
    Motor_Update(&motor_joint_8pin);

    // 取得真實角度 (Degree)
    // 注意：這裡可能需要加上歸零後的 Offset
    // 例如: float real_theta1 = Motor_GetAngle(&motor_joint_13pin) + JOINT1_OFFSET;
    float real_theta1 = Motor_GetAngle(&motor_joint_13pin);
    float real_theta2 = Motor_GetAngle(&motor_joint_8pin);

    // --- 步驟 B: 計算目標角度 (Setpoint) ---
    float target_angle1_deg = real_theta1; // 預設保持現狀
    float target_angle2_deg = real_theta2;

    if (ik_mode_enabled) {
        // 使用運動學解算 (IK)
        MotorAngles solution = kinematics.solveIK({target_x, target_y});

        if (solution.is_reachable) {
            // IK 算出來是 Radian，轉成 Degree 給 PID 用
            target_angle1_deg = FiveBarKinematics::rad2deg(solution.theta1);
            target_angle2_deg = FiveBarKinematics::rad2deg(solution.theta2);
        } else {
            // 目標點超出工作範圍 (Unreachable)
            // 策略：保持在最後一個有效位置，或報錯
            // 這裡簡單處理：保持不動
        }
    }

    // --- 步驟 C: 安全檢查 (FK) ---
    // 利用正向運動學檢查當前位置是否撞機
    Point2D current_pos = kinematics.solveFK(
        FiveBarKinematics::deg2rad(real_theta1),
        FiveBarKinematics::deg2rad(real_theta2)
    );

    // 虛擬圍籬範例：如果 Y < 10mm (太靠近底座)，強制停止
    if (current_pos.y < 10.0f && ik_mode_enabled) {
        Motor_Stop(&motor_joint_13pin);
        Motor_Stop(&motor_joint_8pin);
        return; // 跳過 PID 計算
    }

    // --- 步驟 D: 軌跡規劃 (Trajectory Planning) ---
    // 根據目標位置變化，計算速度與加速度前饋
    traj_joint1.update(target_angle1_deg, dt_seconds);
    traj_joint2.update(target_angle2_deg, dt_seconds);
    
    float target_vel1 = traj_joint1.getVelocity();      // Deg/s
    float target_acc1 = traj_joint1.getAcceleration(); // Deg/s²
    float target_vel2 = traj_joint2.getVelocity();
    float target_acc2 = traj_joint2.getAcceleration();

    // --- 步驟 E: PID 計算 (Control with Feedforward) ---
    // 確保馬達處於啟動狀態
    Motor_Start(&motor_joint_13pin);
    Motor_Start(&motor_joint_8pin);

    // 計算速度命令 (RPM) - 現在包含前饋項
    // update(目標位置, 目標速度, 目標加速度, 當前位置, 時間間隔)
    float cmd_rpm1 = joint1_pid.update(target_angle1_deg, target_vel1, target_acc1, real_theta1, dt_seconds);
    float cmd_rpm2 = joint2_pid.update(target_angle2_deg, target_vel2, target_acc2, real_theta2, dt_seconds);

    // --- 步驟 F: 輸出到底層 (Output) ---
    // 將 float RPM 轉為 int32 傳給底層驅動
    Motor_SetSpeed(&motor_joint_13pin, (int32_t)cmd_rpm1);
    Motor_SetSpeed(&motor_joint_8pin, (int32_t)cmd_rpm2);
}
