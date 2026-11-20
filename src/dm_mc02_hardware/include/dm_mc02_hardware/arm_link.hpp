#pragma once
#include "arm_protocol.hpp"
#include "frame_codec.hpp"
#include "serial_port.hpp"
#include <array>
#include <cmath>
#include <limits>
#include <string>

namespace dm_mc02 {

struct JointConfig {
    double sign = 1.0;
    double offset = 0.0;
    double mcu_min = -3.2;
    double mcu_max = 3.2;
    double max_step_rad = 0.01;
};

struct GripConfig {
    double q_close_m = -0.000523;
    double q_open_m = 0.009530;
    // Unknown until measured. Hard travel bounds are NOT calibration endpoints.
    double mcu_closed = std::numeric_limits<double>::quiet_NaN();
    double mcu_open = std::numeric_limits<double>::quiet_NaN();
    double mcu_min = -12.0;
    double mcu_max = 0.0;
    bool calibrated = false;
    double max_step = 0.01;
    bool command_enable = false;
};

struct ArmLinkConfig {
    std::array<JointConfig, 6> joints;
    GripConfig grip;
    uint16_t valid_time_ms = 100;
    uint32_t state_timeout_ms = 100;
    bool allow_enable = false;
    bool mapping_calibrated = false;

    ArmLinkConfig() {
        joints[0].mcu_min = -2.96054893; joints[0].mcu_max = 2.81695309;
        joints[1].mcu_min = -2.05669899; joints[1].mcu_max = 0.0;
        joints[2].mcu_min = -3.00016972; joints[2].mcu_max = 0.0;
        joints[3].mcu_min = 0.0; joints[3].mcu_max = 3.00567291;
        joints[4].mcu_min = -0.904316101; joints[4].mcu_max = 1.40938965;
        joints[5].mcu_min = -0.650879326; joints[5].mcu_max = 2.32408657;
    }
    std::string validation_error() const;
};

enum class ArmState { IDLE, WAIT_ENABLE_ACK, WAIT_BASELINE, ARMED, FAULT };

struct ArmStatus {
    ArmState state = ArmState::IDLE;
    uint8_t system_state = SYS_IDLE;
    uint8_t arm_mode = 0;
    uint16_t fault_code = 0;
    uint8_t flags = 0;
    uint8_t feedback_valid = 0;
    std::array<uint16_t, NUM_JOINTS> feedback_age_ms{};
    bool any_joint_claimed = false;
    uint32_t last_state_ms = 0;
    uint32_t frames_sent = 0;
    uint32_t frames_received = 0;
    uint32_t crc_errors = 0;
    uint32_t ack_errors = 0;
    uint8_t last_ack_result = ACK_OK;
    std::string reason;
    int32_t grip_motor_urad = 0;
};

class ArmLink {
public:
    explicit ArmLink(const ArmLinkConfig& cfg) : cfg_(cfg), state_(ArmState::IDLE) {
        joint_pos_.fill(std::numeric_limits<double>::quiet_NaN());
        joint_vel_.fill(0.0);
        joint_eff_.fill(0.0);
        joint_cmd_.fill(std::numeric_limits<double>::quiet_NaN());
        last_sent_.fill(0);
        claimed_.fill(false);
        feedback_valid_ = false;
        grip_pos_ = grip_vel_ = grip_eff_ = std::numeric_limits<double>::quiet_NaN();
        grip_cmd_ = std::numeric_limits<double>::quiet_NaN();
        last_sent_grip_ = 0;
    }

    std::vector<uint8_t> step(uint32_t now_ms, const uint8_t* rx_data, size_t rx_len);
    // read() updates feedback only; commands are generated after controller update().
    void receive(uint32_t now_ms, const uint8_t* rx_data, size_t rx_len);
    void set_claimed(size_t joint_idx, bool claimed);

    void get_joint_state(size_t idx, double& pos, double& vel, double& eff) const {
        pos = joint_pos_[idx]; vel = joint_vel_[idx]; eff = joint_eff_[idx];
    }
    void get_grip_state(double& pos, double& vel, double& eff) const {
        pos = grip_pos_; vel = grip_vel_; eff = grip_eff_;
    }

    void set_joint_command(size_t idx, double pos) { if (idx < NUM_JOINTS - 1) joint_cmd_[idx] = pos; }
    void set_grip_command(double pos) { grip_cmd_ = pos; }
    ArmStatus status() const { return status_; }

private:
    ArmLinkConfig cfg_;
    FrameEncoder encoder_;
    FrameDecoder decoder_;
    ArmState state_;
    ArmStatus status_;

    std::array<double, 6> joint_pos_, joint_vel_, joint_eff_, joint_cmd_;
    std::array<int32_t, 6> last_sent_;
    std::array<bool, 7> claimed_;
    double grip_pos_, grip_vel_, grip_eff_, grip_cmd_;
    int32_t last_sent_grip_;
    std::array<int32_t, NUM_JOINTS> motor_position_{};

    bool feedback_valid_;
    bool stop_pending_ = false;
    uint32_t last_cmd_ms_ = 0;
    uint32_t last_state_rx_ms_ = 0;
    uint32_t enable_request_ms_ = 0;
    uint32_t now_ms_ = 0;
    bool have_state_ = false;
    uint8_t enable_sequence_ = 0;

    void handle_state_frame(const StateFrame& sf, uint32_t now_ms);
    void handle_ack_frame(const AckFrame& af);
    std::vector<uint8_t> try_arm(uint32_t now_ms);
    std::vector<uint8_t> send_baseline(uint32_t now_ms);
    std::vector<uint8_t> send_command(uint32_t now_ms);
    bool check_limits(const int32_t target_urad[NUM_JOINTS]) const;
    bool feedback_ready() const;
    void disarm(const char* reason);
    std::vector<uint8_t> stop_frame();
    void invalidate_feedback();
};

}  // namespace dm_mc02



