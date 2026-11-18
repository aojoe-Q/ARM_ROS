#include "dm_mc02_hardware/arm_link.hpp"
#include <algorithm>

namespace dm_mc02 {
namespace {
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
bool in_range(double v, double lo, double hi) {
    return std::isfinite(v) && v >= lo && v <= hi;
}
}

std::string ArmLinkConfig::validation_error() const {
    const ArmLinkConfig firmware;
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& j = joints[i];
        if ((j.sign != 1.0 && j.sign != -1.0) || !std::isfinite(j.offset) ||
            !std::isfinite(j.mcu_min) || !std::isfinite(j.mcu_max) ||
            j.mcu_min >= j.mcu_max || j.mcu_min < firmware.joints[i].mcu_min ||
            j.mcu_max > firmware.joints[i].mcu_max ||
            !in_range(j.max_step_rad, 0.000001, 0.01)) return "invalid joint mapping/limits/step";
    }
    if (!std::isfinite(grip.mcu_min) || !std::isfinite(grip.mcu_max) ||
        grip.mcu_min < -12.0 || grip.mcu_max > 0.0 || grip.mcu_min >= grip.mcu_max ||
        !in_range(grip.max_step, 0.000001, 0.01)) return "invalid gripper hard limits/step";
    if (grip.calibrated &&
        (!in_range(grip.mcu_closed, grip.mcu_min, grip.mcu_max) ||
         !in_range(grip.mcu_open, grip.mcu_min, grip.mcu_max) ||
         std::abs(grip.mcu_closed - grip.mcu_open) < 0.000001 ||
         !std::isfinite(grip.q_close_m) || !std::isfinite(grip.q_open_m) ||
         grip.q_open_m <= grip.q_close_m)) return "invalid gripper calibration";
    if (grip.command_enable && !grip.calibrated) return "gripper command requires calibration";
    if (valid_time_ms < 40 || valid_time_ms > 300 || state_timeout_ms == 0 ||
        state_timeout_ms > 100) return "invalid timing configuration";
    if (allow_enable && !mapping_calibrated) return "arm mapping has not been calibrated";
    return {};
}

void ArmLink::invalidate_feedback() {
    joint_pos_.fill(nan); joint_vel_.fill(nan); joint_eff_.fill(nan);
    grip_pos_ = grip_vel_ = grip_eff_ = nan;
    feedback_valid_ = false;
}

void ArmLink::receive(uint32_t now_ms, const uint8_t* rx_data, size_t rx_len) {
    now_ms_ = now_ms;
    // Handle every complete frame in wire order, including multiple ACKs in one USB read.
    for (size_t i = 0; rx_data && i < rx_len; ++i) {
        decoder_.feed(rx_data + i, 1);
        StateFrame sf{};
        AckFrame af{};
        if (decoder_.take_state(sf)) {
            handle_state_frame(sf, now_ms);
            ++status_.frames_received;
        }
        if (decoder_.take_ack(af)) {
            handle_ack_frame(af);
            ++status_.frames_received;
        }
    }
    status_.crc_errors += decoder_.take_bad_crc_count();
    if (have_state_ && now_ms - last_state_rx_ms_ > cfg_.state_timeout_ms) {
        invalidate_feedback();
        if (state_ != ArmState::IDLE && state_ != ArmState::FAULT) disarm("STATE timeout");
    } else if (have_state_) {
        const uint32_t elapsed = now_ms - last_state_rx_ms_;
        for (size_t i = 0; i < NUM_JOINTS; ++i) {
            if (elapsed + status_.feedback_age_ms[i] <= 100) continue;
            feedback_valid_ = false;
            if (i < 6) joint_pos_[i] = joint_vel_[i] = joint_eff_[i] = nan;
            else grip_pos_ = grip_vel_ = grip_eff_ = nan;
        }
    }
}

std::vector<uint8_t> ArmLink::step(uint32_t now_ms, const uint8_t* rx_data, size_t rx_len) {
    receive(now_ms, rx_data, rx_len);
    encoder_.update_host_time(now_ms);
    status_.any_joint_claimed = std::any_of(claimed_.begin(), claimed_.end(), [](bool c) { return c; });
    if (stop_pending_) return stop_frame();
    if (state_ == ArmState::FAULT) return {};
    if (state_ == ArmState::IDLE) {
        return status_.any_joint_claimed ? try_arm(now_ms) : std::vector<uint8_t>{};
    }
    if (status_.arm_mode != 10 || !feedback_ready()) {
        disarm("PC mode or fresh feedback lost");
        return stop_frame();
    }
    if (state_ == ArmState::WAIT_ENABLE_ACK || state_ == ArmState::WAIT_BASELINE) {
        // ENABLE clears historical faults; the old SYS_FAULT may persist until its ACK.
        if (now_ms - enable_request_ms_ > 200) {
            disarm("ENABLE handshake timeout");
            return stop_frame();
        }
        if (state_ == ArmState::WAIT_BASELINE && status_.fault_code == 0 &&
            (status_.system_state == SYS_ENABLED || status_.system_state == SYS_RUNNING)) {
            return send_baseline(now_ms);
        }
        return {};
    }
    if (status_.system_state != SYS_ENABLED && status_.system_state != SYS_RUNNING) {
        disarm("MCU disabled or faulted");
        return stop_frame();
    }
    if (status_.fault_code != 0) {
        disarm("MCU fault");
        return stop_frame();
    }
    return now_ms - last_cmd_ms_ >= 20 ? send_command(now_ms) : std::vector<uint8_t>{};
}

void ArmLink::set_claimed(size_t joint_idx, bool claimed) {
    if (joint_idx >= claimed_.size()) return;
    claimed_[joint_idx] = claimed;
    // Do not replay an old controller command when an interface is reclaimed.
    if (joint_idx < 6) joint_cmd_[joint_idx] = nan;
    else grip_cmd_ = nan;
    if (!std::any_of(claimed_.begin(), claimed_.end(), [](bool c) { return c; })) {
        if (state_ != ArmState::IDLE && state_ != ArmState::FAULT) stop_pending_ = true;
        state_ = status_.state = ArmState::IDLE;
        // Releasing all command interfaces is the explicit fault recovery boundary.
    }
}

void ArmLink::handle_state_frame(const StateFrame& sf, uint32_t now_ms) {
    have_state_ = true;
    last_state_rx_ms_ = status_.last_state_ms = now_ms;
    status_.system_state = sf.system_state;
    status_.arm_mode = sf.arm_mode;
    status_.fault_code = sf.fault_code;
    status_.flags = sf.flags;
    status_.feedback_valid = sf.feedback_valid;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        status_.feedback_age_ms[i] = sf.feedback_age_ms[i];
        motor_position_[i] = sf.position_urad[i];
    }
    status_.grip_motor_urad = sf.position_urad[6];
    feedback_valid_ = true;
    feedback_valid_ = feedback_ready();
    for (size_t i = 0; i < 6; ++i) {
        const bool valid = (sf.feedback_valid & (1U << i)) && sf.feedback_age_ms[i] <= 100;
        joint_pos_[i] = valid ? cfg_.joints[i].sign * (urad_to_rad(sf.position_urad[i]) - cfg_.joints[i].offset) : nan;
        joint_vel_[i] = valid ? cfg_.joints[i].sign * urads_to_rads(sf.velocity_urad_s[i]) : nan;
        joint_eff_[i] = valid ? cfg_.joints[i].sign * mnm_to_nm(sf.torque_mNm[i]) : nan;
    }
    grip_pos_ = grip_vel_ = grip_eff_ = nan;
    if (cfg_.grip.calibrated && (sf.feedback_valid & 0x40) && sf.feedback_age_ms[6] <= 100) {
        const double scale = (cfg_.grip.q_open_m - cfg_.grip.q_close_m) /
                             (cfg_.grip.mcu_open - cfg_.grip.mcu_closed);
        grip_pos_ = cfg_.grip.q_close_m + (urad_to_rad(sf.position_urad[6]) - cfg_.grip.mcu_closed) * scale;
        grip_vel_ = urads_to_rads(sf.velocity_urad_s[6]) * scale;
        // joint8 is prismatic: equivalent force (N), not motor torque (Nm).
        grip_eff_ = mnm_to_nm(sf.torque_mNm[6]) / scale;
    }
    // Latch an unsafe STATE even if a later frame in this USB read reports recovery.
    if (state_ == ArmState::ARMED && (sf.arm_mode != 10 || !feedback_ready() ||
        sf.fault_code != 0 || (sf.system_state != SYS_ENABLED && sf.system_state != SYS_RUNNING))) {
        disarm("unsafe MCU state");
    }
}

void ArmLink::handle_ack_frame(const AckFrame& af) {
    // Preserve the rejection code if a later benign ACK shares the same USB transfer.
    if (af.result != ACK_OK) {
        status_.last_ack_result = af.result;
        ++status_.ack_errors;
    }
    if (state_ == ArmState::WAIT_ENABLE_ACK && af.acked_type == TYPE_ENABLE &&
        af.acked_sequence == enable_sequence_) {
        if (af.result == ACK_OK) state_ = status_.state = ArmState::WAIT_BASELINE;
        else disarm("ENABLE refused (see ack result)");
    } else if (af.acked_type == TYPE_POSITION_CMD && af.result != ACK_OK &&
               (state_ == ArmState::ARMED || state_ == ArmState::WAIT_BASELINE)) {
        disarm("POSITION_CMD rejected (see ack result)");
    }
}

std::vector<uint8_t> ArmLink::try_arm(uint32_t now_ms) {
    if (!cfg_.allow_enable || !cfg_.mapping_calibrated || status_.arm_mode != 10 || !feedback_ready()) return {};
    if (status_.flags & (FLAG_CAN_FAULT | FLAG_DM_FAILED)) return {};
    if (!check_limits(motor_position_.data())) {
        disarm("measured baseline outside configured motor limits");
        return stop_frame();
    }
    state_ = status_.state = ArmState::WAIT_ENABLE_ACK;
    status_.reason.clear();
    status_.last_ack_result = ACK_OK;
    enable_request_ms_ = now_ms;
    auto frame = encoder_.encode_enable(1);
    enable_sequence_ = frame[3];
    ++status_.frames_sent;
    return frame;
}

std::vector<uint8_t> ArmLink::send_baseline(uint32_t now_ms) {
    // Preserve all seven original fixed-point positions, including an uncalibrated gripper.
    if (!check_limits(motor_position_.data())) {
        disarm("baseline outside motor limits");
        return stop_frame();
    }
    std::copy_n(motor_position_.begin(), 6, last_sent_.begin());
    last_sent_grip_ = motor_position_[6];
    last_cmd_ms_ = now_ms;
    state_ = status_.state = ArmState::ARMED;
    ++status_.frames_sent;
    return encoder_.encode_position_cmd(motor_position_.data(), now_ms, cfg_.valid_time_ms);
}

std::vector<uint8_t> ArmLink::send_command(uint32_t now_ms) {
    int32_t target[NUM_JOINTS];
    for (size_t i = 0; i < 6; ++i) {
        target[i] = last_sent_[i];
        if (!claimed_[i] || std::isnan(joint_cmd_[i])) continue;
        const double mcu = cfg_.joints[i].sign * joint_cmd_[i] + cfg_.joints[i].offset;
        if (!in_range(mcu, cfg_.joints[i].mcu_min, cfg_.joints[i].mcu_max)) {
            disarm("non-finite or out-of-range joint command"); return stop_frame();
        }
        target[i] = rad_to_urad(mcu);
        if (std::abs(static_cast<double>(target[i]) - last_sent_[i]) > cfg_.joints[i].max_step_rad * 1e6) {
            disarm("joint max_step exceeded"); return stop_frame();
        }
    }
    target[6] = last_sent_grip_;
    if (claimed_[6] && cfg_.grip.command_enable && cfg_.grip.calibrated && !std::isnan(grip_cmd_)) {
        if (!in_range(grip_cmd_, cfg_.grip.q_close_m, cfg_.grip.q_open_m)) {
            disarm("non-finite or out-of-range gripper command"); return stop_frame();
        }
        const double t = (grip_cmd_ - cfg_.grip.q_close_m) / (cfg_.grip.q_open_m - cfg_.grip.q_close_m);
        target[6] = rad_to_urad(cfg_.grip.mcu_closed + t * (cfg_.grip.mcu_open - cfg_.grip.mcu_closed));
        if (std::abs(static_cast<double>(target[6]) - last_sent_grip_) > cfg_.grip.max_step * 1e6) {
            disarm("gripper max_step exceeded"); return stop_frame();
        }
    }
    if (!check_limits(target)) { disarm("motor limit violation"); return stop_frame(); }
    std::copy_n(target, 6, last_sent_.begin());
    last_sent_grip_ = target[6];
    last_cmd_ms_ = now_ms;
    ++status_.frames_sent;
    return encoder_.encode_position_cmd(target, now_ms, cfg_.valid_time_ms);
}

bool ArmLink::check_limits(const int32_t target[NUM_JOINTS]) const {
    for (size_t i = 0; i < 6; ++i) {
        if (!in_range(urad_to_rad(target[i]), cfg_.joints[i].mcu_min, cfg_.joints[i].mcu_max)) return false;
    }
    return in_range(urad_to_rad(target[6]), cfg_.grip.mcu_min, cfg_.grip.mcu_max);
}

bool ArmLink::feedback_ready() const {
    if (!have_state_ || !feedback_valid_ || now_ms_ - last_state_rx_ms_ > cfg_.state_timeout_ms ||
        !(status_.flags & FLAG_ARM_READY) || status_.feedback_valid != 0x7F ||
        (status_.flags & (FLAG_CAN_FAULT | FLAG_DM_FAILED))) return false;
    for (const auto age : status_.feedback_age_ms)
        if (static_cast<uint32_t>(age) + (now_ms_ - last_state_rx_ms_) > 100) return false;
    return true;
}

void ArmLink::disarm(const char* reason) {
    if (state_ != ArmState::FAULT) {
        stop_pending_ = true;
        status_.reason = reason;
    }
    state_ = status_.state = ArmState::FAULT;
}

std::vector<uint8_t> ArmLink::stop_frame() {
    stop_pending_ = false;
    ++status_.frames_sent;
    return encoder_.encode_stop();
}
}  // namespace dm_mc02

