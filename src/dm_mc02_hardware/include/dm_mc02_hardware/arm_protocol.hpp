#pragma once
#include <cstdint>
#include <cstring>
#include <array>

namespace dm_mc02 {

// Protocol v2 frame types
constexpr uint8_t FRAME_SOF = 0xA5;
constexpr uint8_t FRAME_VERSION = 0x02;

constexpr uint8_t TYPE_STATE = 0x01;
constexpr uint8_t TYPE_POSITION_CMD = 0x02;
constexpr uint8_t TYPE_ENABLE = 0x03;
constexpr uint8_t TYPE_STOP = 0x04;
constexpr uint8_t TYPE_HEARTBEAT = 0x05;
constexpr uint8_t TYPE_ACK = 0x06;
constexpr uint8_t TYPE_FAULT = 0x07;
constexpr uint8_t TYPE_DIAGNOSTIC = 0x08;

// ACK result codes
constexpr uint8_t ACK_OK = 0x00;
constexpr uint8_t ACK_BAD_LENGTH = 0x01;
constexpr uint8_t ACK_BAD_VERSION = 0x02;
constexpr uint8_t ACK_NOT_ENABLED = 0x03;
constexpr uint8_t ACK_OUT_OF_RANGE = 0x04;
constexpr uint8_t ACK_UNKNOWN_TYPE = 0x05;
constexpr uint8_t ACK_BAD_VALUE = 0x06;
constexpr uint8_t ACK_STALE = 0x07;
constexpr uint8_t ACK_NOT_READY = 0x08;
constexpr uint8_t ACK_NO_SESSION = 0x09;
constexpr uint8_t ACK_WRONG_MODE = 0x0A;

// System state
constexpr uint8_t SYS_IDLE = 0x00;
constexpr uint8_t SYS_ENABLED = 0x01;
constexpr uint8_t SYS_RUNNING = 0x02;
constexpr uint8_t SYS_FAULT = 0x03;

// Fault codes
constexpr uint16_t FAULT_LINK_TIMEOUT = 0x0001;
constexpr uint16_t FAULT_USB_DISCONNECT = 0x0002;
constexpr uint16_t FAULT_RX_OVERFLOW = 0x0004;
constexpr uint16_t FAULT_FEEDBACK_LOST = 0x0008;
constexpr uint16_t FAULT_CAN_BUS = 0x0010;
constexpr uint16_t FAULT_DM_ENABLE = 0x0020;
constexpr uint8_t FLAG_ARM_READY = 0x01;
constexpr uint8_t FLAG_CAN_FAULT = 0x04;
constexpr uint8_t FLAG_DM_FAILED = 0x10;

constexpr size_t NUM_JOINTS = 7;
constexpr size_t FRAME_HEADER_SIZE = 10;
constexpr size_t STATE_PAYLOAD_SIZE = 138;
constexpr size_t POSITION_CMD_PAYLOAD_SIZE = 64;
constexpr size_t ACK_PAYLOAD_SIZE = 4;

// CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no XOR out)
inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; ++j) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}

struct FrameHeader {
    uint8_t sof;
    uint8_t version;
    uint8_t type;
    uint8_t seq;
    uint16_t length;
    uint32_t timestamp_ms;
} __attribute__((packed));

struct StateFrame {
    FrameHeader header;
    uint32_t timestamp_ms;
    uint8_t arm_mode;
    uint8_t system_state;
    uint16_t fault_code;
    uint8_t flags;
    uint8_t feedback_valid;
    uint16_t reserved;
    uint16_t feedback_age_ms[7];
    int32_t position_urad[7];      // microradians
    int32_t velocity_urad_s[7];    // microradians/s
    int32_t torque_mNm[7];         // milliNewton·m
    int32_t target_position_urad[7];
    uint16_t crc16;
} __attribute__((packed));

struct PositionCmdFrame {
    FrameHeader header;
    uint32_t command_timestamp_ms;
    uint16_t valid_time_ms;
    uint8_t enable;
    uint8_t reserved;
    int32_t target_position_urad[7];
    int32_t velocity_limit_urad_s[7];  // 0 = firmware default (0.5 rad/s)
    uint16_t crc16;
} __attribute__((packed));

struct EnableFrame {
    FrameHeader header;
    uint8_t enable;
    uint8_t reserved[3];
    uint16_t crc16;
} __attribute__((packed));

struct StopFrame {
    FrameHeader header;
    uint16_t crc16;
} __attribute__((packed));

struct HeartbeatFrame {
    FrameHeader header;
    uint16_t crc16;
} __attribute__((packed));

struct AckFrame {
    FrameHeader header;
    uint8_t acked_type;
    uint8_t acked_sequence;
    uint8_t result;
    uint8_t reserved;
    uint16_t crc16;
} __attribute__((packed));

static_assert(sizeof(StateFrame) == FRAME_HEADER_SIZE + STATE_PAYLOAD_SIZE + 2, "STATE size");
static_assert(sizeof(PositionCmdFrame) == FRAME_HEADER_SIZE + POSITION_CMD_PAYLOAD_SIZE + 2, "POSITION_CMD size");

// Unit conversions
constexpr int32_t RAD_TO_URAD = 1000000;
constexpr int32_t RADS_TO_URADS = 1000000;
constexpr int32_t NM_TO_MNM = 1000;

inline int32_t rad_to_urad(double rad) {
    return static_cast<int32_t>(rad * RAD_TO_URAD);
}

inline double urad_to_rad(int32_t urad) {
    return static_cast<double>(urad) / RAD_TO_URAD;
}

inline int32_t rads_to_urads(double rads) {
    return static_cast<int32_t>(rads * RADS_TO_URADS);
}

inline double urads_to_rads(int32_t urads) {
    return static_cast<double>(urads) / RADS_TO_URADS;
}

inline int32_t nm_to_mnm(double nm) {
    return static_cast<int32_t>(nm * NM_TO_MNM);
}

inline double mnm_to_nm(int32_t mnm) {
    return static_cast<double>(mnm) / NM_TO_MNM;
}

}  // namespace dm_mc02
