/**
 * @file pid_controller.hpp
 * @brief 支援前饋控制的 PID 控制器
 */
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PositionController {
public:
    // 建構子新增 Kv (速度前饋 gain) 和 Ka (加速度前饋 gain)
    // 對於 Nidec 這類本身就是速度控制的馬達，Kv 通常設為 1.0 (直接對應)
    PositionController(float kp, float ki, float kd, float kv, float ka, float max_rpm)
        : _kp(kp), _ki(ki), _kd(kd), _kv(kv), _ka(ka), _max_output(max_rpm) {
        reset();
    }

    void reset() {
        _integral = 0.0f;
        _prev_error = 0.0f;
    }

    /**
     * @brief 更新控制器
     * @param target_pos 目標位置 (Degree)
     * @param target_vel 目標速度 (Degree/s) - 來自 PVT
     * @param target_acc 目標加速度 (Degree/s^2) - 來自 PVT (可選)
     * @param current_pos 實際位置 (Degree)
     * @param dt 時間差 (s)
     * @return 馬達轉速命令 (RPM)
     */
    float update(float target_pos, float target_vel, float target_acc, float current_pos, float dt) {
        // 1. 回授控制 (Feedback): 處理位置誤差
        float error = target_pos - current_pos;
        
        float p_out = _kp * error;
        
        _integral += error * dt;
        float i_out = _ki * _integral;
        
        float derivative = (error - _prev_error) / dt;
        float d_out = _kd * derivative;
        
        float feedback_rpm = p_out + i_out + d_out;

        // 2. 前饋控制 (Feedforward): 預測需要的輸出
        // 將角速度 (Deg/s) 轉換為 RPM:  (Deg/s) / 6.0 = RPM
        float ff_vel_rpm = (target_vel / 360.0f) * 60.0f * _kv;
        
        // 加速度前饋 (簡單模擬慣量補償)
        // 這是一個經驗值，當需要高加速度時，額外增加一點 RPM 指令讓馬達反應更快
        float ff_acc_rpm = target_acc * _ka;

        // 3. 總輸出
        float output = feedback_rpm + ff_vel_rpm + ff_acc_rpm;

        // 4. 限制與保存
        if (output > _max_output) output = _max_output;
        else if (output < -_max_output) output = -_max_output;

        _prev_error = error;
        return output;
    }

private:
    float _kp, _ki, _kd, _kv, _ka;
    float _max_output;
    float _integral;
    float _prev_error;
};

#endif // PID_CONTROLLER_HPP