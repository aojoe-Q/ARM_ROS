// Optional cross-codec check, compiled against the actual unmodified firmware source.
#include "dm_mc02_hardware/frame_codec.hpp"
#include "arm_usb_protocol.h"
#include <iostream>
#include <stdexcept>

int main() {
    ArmStatePayload_t s{};
    s.timestamp_ms=123; s.arm_mode=10; s.flags=1; s.feedback_valid=127;
    for (unsigned i=0;i<7;++i) {
        s.feedback_age_ms[i]=static_cast<uint16_t>(i+1);
        s.position_urad[i]=-100000 * static_cast<int32_t>(i+1);
        s.velocity_urad_s[i]=1000+i; s.torque_mNm[i]=-20;
        s.target_position_urad[i]=s.position_urad[i];
    }
    uint8_t payload[160], frame[172];
    auto n=arm_proto_encode_state(payload,&s);
    auto length=arm_proto_build_frame(frame,sizeof(frame),ARM_MSG_STATE,255,456,payload,n);
    dm_mc02::FrameDecoder decoder; decoder.feed(frame,length); dm_mc02::StateFrame decoded{};
    if (!decoder.take_state(decoded) || decoded.timestamp_ms!=123 || decoded.header.timestamp_ms!=456)
        throw std::runtime_error("firmware STATE header mismatch");
    for (unsigned i=0;i<7;++i) {
        if (decoded.position_urad[i]!=s.position_urad[i] || decoded.feedback_age_ms[i]!=i+1 ||
            decoded.velocity_urad_s[i]!=s.velocity_urad_s[i] || decoded.torque_mNm[i]!=-20 ||
            decoded.target_position_urad[i]!=s.position_urad[i]) throw std::runtime_error("STATE field mismatch");
    }
    dm_mc02::FrameEncoder encoder; encoder.update_host_time(123);
    int32_t target[7]={100000,-200000,-300000,400000,500000,-600000,-700000};
    auto cmd=encoder.encode_position_cmd(target,999,100);
    if (!arm_proto_frame_crc_ok(cmd.data(),ARM_POS_CMD_PAYLOAD_SIZE)) throw std::runtime_error("command CRC mismatch");
    ArmPositionCommand_t parsed{};arm_proto_decode_position_cmd(cmd.data()+10,&parsed);
    if (parsed.command_timestamp_ms!=999 || parsed.valid_time_ms!=100 || parsed.enable!=1)
        throw std::runtime_error("command header mismatch");
    for (unsigned i=0;i<7;++i)
        if (parsed.position_urad[i]!=target[i] || parsed.velocity_limit_urad_s[i]!=0)
            throw std::runtime_error("command field mismatch");
    std::cout<<"PASS: current firmware STATE -> ROS, ROS POSITION_CMD -> current firmware\n";
}
