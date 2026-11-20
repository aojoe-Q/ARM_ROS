#include <gtest/gtest.h>
#include "dm_mc02_hardware/arm_protocol.hpp"
#include "dm_mc02_hardware/frame_codec.hpp"

using namespace dm_mc02;

TEST(ProtocolTest, CRC16) {
    const uint8_t test[] = {'1','2','3','4','5','6','7','8','9'};
    EXPECT_EQ(crc16_ccitt(test, sizeof(test)), 0x29B1);
}

TEST(ProtocolTest, EncodeEnable) {
    FrameEncoder enc;
    enc.update_host_time(1000);
    auto frame = enc.encode_enable(1);
    ASSERT_EQ(frame.size(), FRAME_HEADER_SIZE + 4 + 2);
    EXPECT_EQ(frame[0], FRAME_SOF);
    EXPECT_EQ(frame[1], FRAME_VERSION);
    EXPECT_EQ(frame[2], TYPE_ENABLE);
    EXPECT_EQ(get_u32(frame.data() + 6), 1000u);
    EXPECT_EQ(frame[FRAME_HEADER_SIZE], 1);
    EXPECT_EQ(get_u16(frame.data() + frame.size() - 2),
              crc16_ccitt(frame.data(), frame.size() - 2));
}

TEST(ProtocolTest, EncodePositionCmdUsesFirmwareOrder) {
    FrameEncoder enc;
    enc.update_host_time(2000);
    int32_t target[NUM_JOINTS] = {100000,-200000,300000,-400000,500000,-600000,700000};
    auto frame = enc.encode_position_cmd(target, 2000, 100);
    ASSERT_EQ(frame.size(), FRAME_HEADER_SIZE + POSITION_CMD_PAYLOAD_SIZE + 2);
    const uint8_t *p = frame.data() + FRAME_HEADER_SIZE;
    EXPECT_EQ(get_u32(p), 2000u);
    EXPECT_EQ(get_u16(p + 4), 100u);
    EXPECT_EQ(p[6], 1);
    EXPECT_EQ(get_i32(p + 8), 100000);
    EXPECT_EQ(get_i32(p + 8 + 5 * 4), -600000);
    EXPECT_EQ(get_i32(p + 8 + 6 * 4), 700000);
    EXPECT_EQ(get_u16(frame.data() + frame.size() - 2),
              crc16_ccitt(frame.data(), frame.size() - 2));
}

TEST(ProtocolTest, DecodeStateAndAckInOneTransfer) {
    StateFrame sf{};
    sf.header = {FRAME_SOF, FRAME_VERSION, TYPE_STATE, 0, STATE_PAYLOAD_SIZE, 3000};
    sf.timestamp_ms = 3000;
    sf.arm_mode = 10;
    sf.system_state = SYS_RUNNING;
    sf.fault_code = 0;
    sf.flags = 1;
    sf.feedback_valid = 0x7F;
    sf.reserved = 0;
    for (size_t i = 0; i < NUM_JOINTS; ++i) {
        sf.feedback_age_ms[i] = 5;
        sf.position_urad[i] = static_cast<int32_t>((i + 1) * 100000);
        sf.velocity_urad_s[i] = static_cast<int32_t>((i + 1) * 50000);
        sf.torque_mNm[i] = static_cast<int32_t>((i + 1) * 1000);
        sf.target_position_urad[i] = sf.position_urad[i];
    }
    sf.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&sf), sizeof(sf) - 2);

    AckFrame af{};
    af.header = {FRAME_SOF, FRAME_VERSION, TYPE_ACK, 1, ACK_PAYLOAD_SIZE, 4000};
    af.acked_type = TYPE_ENABLE;
    af.acked_sequence = 7;
    af.result = ACK_OK;
    af.reserved = 0;
    af.crc16 = crc16_ccitt(reinterpret_cast<const uint8_t*>(&af), sizeof(af) - 2);

    std::vector<uint8_t> transfer(sizeof(sf) + sizeof(af));
    std::memcpy(transfer.data(), &af, sizeof(af));
    std::memcpy(transfer.data() + sizeof(af), &sf, sizeof(sf));

    FrameDecoder dec;
    dec.feed(transfer.data(), transfer.size());
    StateFrame decoded_state{};
    AckFrame decoded_ack{};
    ASSERT_TRUE(dec.take_state(decoded_state));
    ASSERT_TRUE(dec.take_ack(decoded_ack));
    EXPECT_EQ(decoded_state.arm_mode, 10);
    EXPECT_EQ(decoded_state.feedback_age_ms[6], 5);
    EXPECT_EQ(decoded_state.position_urad[6], 700000);
    EXPECT_EQ(decoded_ack.acked_type, TYPE_ENABLE);
    EXPECT_EQ(decoded_ack.acked_sequence, 7);
    EXPECT_EQ(decoded_ack.result, ACK_OK);
}

TEST(ProtocolTest, UnitConversions) {
    EXPECT_DOUBLE_EQ(urad_to_rad(1000000), 1.0);
    EXPECT_EQ(rad_to_urad(1.0), 1000000);
    EXPECT_DOUBLE_EQ(urads_to_rads(2000000), 2.0);
    EXPECT_EQ(rads_to_urads(2.0), 2000000);
    EXPECT_DOUBLE_EQ(mnm_to_nm(3000), 3.0);
    EXPECT_EQ(nm_to_mnm(3.0), 3000);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

