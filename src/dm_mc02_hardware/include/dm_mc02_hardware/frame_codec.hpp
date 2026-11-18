#pragma once

#include "arm_protocol.hpp"
#include <cstring>
#include <vector>

namespace dm_mc02 {

inline void put_u16(uint8_t *p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

inline void put_u32(uint8_t *p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
    p[2] = static_cast<uint8_t>(v >> 16);
    p[3] = static_cast<uint8_t>(v >> 24);
}

inline uint16_t get_u16(const uint8_t *p) {
    return static_cast<uint16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}

inline uint32_t get_u32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

inline int32_t get_i32(const uint8_t *p) {
    return static_cast<int32_t>(get_u32(p));
}

class FrameEncoder {
public:
    FrameEncoder() : seq_(0), host_time_ms_(0) {}
    void update_host_time(uint32_t now_ms) { host_time_ms_ = now_ms; }

    std::vector<uint8_t> encode_enable(uint8_t enable_val) {
        std::vector<uint8_t> frame(FRAME_HEADER_SIZE + 4 + 2, 0);
        fill_header(frame.data(), TYPE_ENABLE, 4);
        frame[FRAME_HEADER_SIZE] = enable_val;
        finish_crc(frame);
        return frame;
    }

    std::vector<uint8_t> encode_stop() {
        std::vector<uint8_t> frame(FRAME_HEADER_SIZE + 2, 0);
        fill_header(frame.data(), TYPE_STOP, 0);
        finish_crc(frame);
        return frame;
    }

    std::vector<uint8_t> encode_heartbeat() {
        std::vector<uint8_t> frame(FRAME_HEADER_SIZE + 2, 0);
        fill_header(frame.data(), TYPE_HEARTBEAT, 0);
        finish_crc(frame);
        return frame;
    }

    std::vector<uint8_t> encode_position_cmd(
        const int32_t target_urad[NUM_JOINTS],
        uint32_t cmd_timestamp_ms,
        uint16_t valid_time_ms) {
        std::vector<uint8_t> frame(FRAME_HEADER_SIZE + POSITION_CMD_PAYLOAD_SIZE + 2, 0);
        fill_header(frame.data(), TYPE_POSITION_CMD, POSITION_CMD_PAYLOAD_SIZE);
        uint8_t *p = frame.data() + FRAME_HEADER_SIZE;
        put_u32(p, cmd_timestamp_ms); p += 4;
        put_u16(p, valid_time_ms); p += 2;
        *p++ = 1;
        *p++ = 0;
        for (size_t i = 0; i < NUM_JOINTS; ++i, p += 4) {
            put_u32(p, static_cast<uint32_t>(target_urad[i]));
        }
        finish_crc(frame);
        return frame;
    }

private:
    uint8_t seq_;
    uint32_t host_time_ms_;

    void fill_header(uint8_t *p, uint8_t type, uint16_t payload_len) {
        p[0] = FRAME_SOF;
        p[1] = FRAME_VERSION;
        p[2] = type;
        p[3] = seq_++;
        put_u16(p + 4, payload_len);
        put_u32(p + 6, host_time_ms_);
    }

    static void finish_crc(std::vector<uint8_t> &frame) {
        put_u16(frame.data() + frame.size() - 2,
                crc16_ccitt(frame.data(), frame.size() - 2));
    }
};

class FrameDecoder {
public:
    FrameDecoder() : state_(WAIT_SOF), buf_pos_(0), state_ready_(false),
                     ack_ready_(false), bad_crc_count_(0) {}

    enum ParseResult { NEED_MORE, GOT_STATE, GOT_ACK, BAD_CRC, BAD_FRAME };

    ParseResult feed(const uint8_t *data, size_t len) {
        ParseResult result = NEED_MORE;
        for (size_t i = 0; i < len; ++i) {
            const ParseResult one = feed_byte(data[i]);
            if (one == GOT_STATE || one == GOT_ACK) result = one;
            else if (one == BAD_CRC && result == NEED_MORE) result = BAD_CRC;
            else if (one == BAD_FRAME && result == NEED_MORE) result = BAD_FRAME;
        }
        return result;
    }

    bool take_state(StateFrame &out) {
        if (!state_ready_) return false;
        out = last_state_;
        state_ready_ = false;
        return true;
    }

    bool take_ack(AckFrame &out) {
        if (!ack_ready_) return false;
        out = last_ack_;
        ack_ready_ = false;
        return true;
    }

    uint32_t take_bad_crc_count() {
        const uint32_t n = bad_crc_count_;
        bad_crc_count_ = 0;
        return n;
    }

    const StateFrame& last_state() const { return last_state_; }
    const AckFrame& last_ack() const { return last_ack_; }

private:
    enum State { WAIT_SOF, IN_FRAME };
    State state_;
    uint8_t buf_[256];
    size_t buf_pos_;
    StateFrame last_state_{};
    AckFrame last_ack_{};
    bool state_ready_;
    bool ack_ready_;
    uint32_t bad_crc_count_;

    ParseResult feed_byte(uint8_t b) {
        if (state_ == WAIT_SOF) {
            if (b == FRAME_SOF) {
                buf_[0] = b;
                buf_pos_ = 1;
                state_ = IN_FRAME;
            }
            return NEED_MORE;
        }

        if (buf_pos_ >= sizeof(buf_)) {
            state_ = WAIT_SOF;
            buf_pos_ = 0;
            return BAD_FRAME;
        }
        buf_[buf_pos_++] = b;
        if (buf_pos_ < FRAME_HEADER_SIZE) return NEED_MORE;

        const uint16_t payload_len = get_u16(buf_ + 4);
        const size_t total_size = FRAME_HEADER_SIZE + payload_len + 2;
        if (total_size > sizeof(buf_)) {
            state_ = WAIT_SOF;
            buf_pos_ = 0;
            return BAD_FRAME;
        }
        if (buf_pos_ < total_size) return NEED_MORE;

        const uint16_t expected = get_u16(buf_ + total_size - 2);
        const uint16_t computed = crc16_ccitt(buf_, total_size - 2);
        state_ = WAIT_SOF;
        buf_pos_ = 0;
        if (computed != expected) {
            ++bad_crc_count_;
            return BAD_CRC;
        }
        if (buf_[1] != FRAME_VERSION) return BAD_FRAME;

        if (buf_[2] == TYPE_STATE && payload_len == STATE_PAYLOAD_SIZE) {
            decode_state(buf_);
            state_ready_ = true;
            return GOT_STATE;
        }
        if (buf_[2] == TYPE_ACK && payload_len == ACK_PAYLOAD_SIZE) {
            decode_ack(buf_);
            ack_ready_ = true;
            return GOT_ACK;
        }
        return BAD_FRAME;
    }

    void decode_state(const uint8_t *frame) {
        StateFrame sf{};
        sf.header.sof = frame[0];
        sf.header.version = frame[1];
        sf.header.type = frame[2];
        sf.header.seq = frame[3];
        sf.header.length = get_u16(frame + 4);
        sf.header.timestamp_ms = get_u32(frame + 6);
        const uint8_t *p = frame + FRAME_HEADER_SIZE;
        sf.timestamp_ms = get_u32(p); p += 4;
        sf.arm_mode = *p++;
        sf.system_state = *p++;
        sf.fault_code = get_u16(p); p += 2;
        sf.flags = *p++;
        sf.feedback_valid = *p++;
        sf.reserved = get_u16(p); p += 2;
        for (size_t i = 0; i < NUM_JOINTS; ++i, p += 2) sf.feedback_age_ms[i] = get_u16(p);
        for (size_t i = 0; i < NUM_JOINTS; ++i, p += 4) sf.position_urad[i] = get_i32(p);
        for (size_t i = 0; i < NUM_JOINTS; ++i, p += 4) sf.velocity_urad_s[i] = get_i32(p);
        for (size_t i = 0; i < NUM_JOINTS; ++i, p += 4) sf.torque_mNm[i] = get_i32(p);
        for (size_t i = 0; i < NUM_JOINTS; ++i, p += 4) sf.target_position_urad[i] = get_i32(p);
        sf.crc16 = get_u16(p);
        last_state_ = sf;
    }

    void decode_ack(const uint8_t *frame) {
        AckFrame af{};
        af.header.sof = frame[0];
        af.header.version = frame[1];
        af.header.type = frame[2];
        af.header.seq = frame[3];
        af.header.length = get_u16(frame + 4);
        af.header.timestamp_ms = get_u32(frame + 6);
        const uint8_t *p = frame + FRAME_HEADER_SIZE;
        af.acked_type = p[0];
        af.acked_sequence = p[1];
        af.result = p[2];
        af.reserved = p[3];
        af.crc16 = get_u16(p + 4);
        last_ack_ = af;
    }
};

}  // namespace dm_mc02

