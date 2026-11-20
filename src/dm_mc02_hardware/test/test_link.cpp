// Standalone regression suite: no ROS, hardware, or GoogleTest dependency.
#include "dm_mc02_hardware/arm_link.hpp"
#include <iostream>
#include <stdexcept>
#include <functional>

using namespace dm_mc02;
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
std::vector<uint8_t> frame(uint8_t type, std::vector<uint8_t> payload) {
    std::vector<uint8_t> f(12 + payload.size(), 0);
    f[0]=FRAME_SOF; f[1]=FRAME_VERSION; f[2]=type;
    put_u16(f.data()+4, static_cast<uint16_t>(payload.size()));
    std::copy(payload.begin(),payload.end(),f.begin()+10);
    put_u16(f.data()+f.size()-2,crc16_ccitt(f.data(),f.size()-2)); return f;
}
std::vector<uint8_t> state(uint8_t sys=SYS_IDLE, uint8_t mask=127, uint16_t fault=0,
                           int32_t grip=-1000000, uint8_t flags=1, uint8_t mode=10) {
    std::vector<uint8_t> p(138,0); p[4]=mode; p[5]=sys; put_u16(p.data()+6,fault);
    p[8]=flags; p[9]=mask;
    int32_t pos[7]={100000,-200000,-300000,400000,100000,-100000,grip};
    for (size_t i=0;i<7;++i) { put_u16(p.data()+12+i*2,5); put_u32(p.data()+26+i*4,pos[i]); }
    return frame(TYPE_STATE,p);
}
std::vector<uint8_t> ack(uint8_t seq=0,uint8_t result=ACK_OK,uint8_t type=TYPE_ENABLE) {
    return frame(TYPE_ACK,{type,seq,result,0});
}
ArmLinkConfig config() { ArmLinkConfig c; c.allow_enable=true; c.mapping_calibrated=true; return c; }
std::vector<uint8_t> tick(ArmLink& link,uint32_t t,const std::vector<uint8_t>& rx={}) {
    return link.step(t,rx.data(),rx.size());
}
void type_is(const std::vector<uint8_t>& f,uint8_t type) { require(f.size()>=12 && f[2]==type,"wrong output frame"); }
int32_t target(const std::vector<uint8_t>& f,int i) { type_is(f,TYPE_POSITION_CMD); return get_i32(f.data()+18+i*4); }
std::vector<uint8_t> arm(ArmLink& link,uint32_t t=100,int32_t grip=-1000000) {
    link.set_claimed(0,true); auto enable=tick(link,t,state(SYS_IDLE,127,0,grip)); type_is(enable,TYPE_ENABLE);
    auto reply=ack(enable[3]); auto s=state(SYS_ENABLED,127,0,grip); reply.insert(reply.end(),s.begin(),s.end());
    auto base=tick(link,t+1,reply); type_is(base,TYPE_POSITION_CMD); return base;
}
int main() {
    unsigned passed=0,failed=0;
    auto test=[&](const char* name,std::function<void()> fn) {
        try { fn(); ++passed; std::cout<<"PASS "<<name<<'\n'; }
        catch(const std::exception& e) { ++failed; std::cerr<<"FAIL "<<name<<": "<<e.what()<<'\n'; }
    };
    test("CRC and field offsets",[]{
        const uint8_t s[]={'1','2','3','4','5','6','7','8','9'};
        require(crc16_ccitt(s,9)==0x29B1,"CRC vector"); FrameEncoder e;
        int32_t q[7]={1,2,3,4,5,-600000,-700000}; auto f=e.encode_position_cmd(q,123,100);
        require(target(f,5)==-600000 && target(f,6)==-700000,"position offsets");
        require(get_i32(f.data()+46)==0,"velocity default");
    });
    test("baseline preserves negative gripper exactly",[]{ ArmLink l(config()); auto b=arm(l,100,-1234567);
        require(target(b,6)==-1234567,"gripper changed in baseline");
        require(target(tick(l,121),6)==-1234567,"disabled gripper moved"); });
    test("fresh frame required before enable",[]{ ArmLink l(config()); tick(l,0,state()); tick(l,101);
        l.set_claimed(0,true); require(tick(l,102).empty(),"stale enable"); });
    test("feedback stream timeout invalidates all units",[]{ ArmLink l(config()); arm(l);
        type_is(tick(l,202),TYPE_STOP); double p,v,e; l.get_joint_state(0,p,v,e);
        require(std::isnan(p)&&std::isnan(v)&&std::isnan(e),"stale state still finite"); });
    test("invalid individual feedback becomes NaN",[]{ ArmLink l(config()); tick(l,100,state(SYS_IDLE,126));
        double p,v,e;l.get_joint_state(0,p,v,e);require(std::isnan(p)&&std::isnan(v)&&std::isnan(e),"invalid joint finite"); });
    test("feedback age includes time since STATE reception",[]{ ArmLink l(config());arm(l);
        auto s=state(SYS_RUNNING);put_u16(s.data()+22,90);
        put_u16(s.data()+s.size()-2,crc16_ccitt(s.data(),s.size()-2));tick(l,110,s);
        type_is(tick(l,121),TYPE_STOP);double p,v,e;l.get_joint_state(0,p,v,e);
        require(std::isnan(p),"aged motor sample stayed valid"); });
    test("historical fault clears through enable handshake",[]{ ArmLink l(config()); l.set_claimed(0,true);
        auto en=tick(l,100,state(SYS_FAULT,127,FAULT_LINK_TIMEOUT)); type_is(en,TYPE_ENABLE);
        require(tick(l,101,ack(en[3])).empty(),"baseline before cleared STATE");
        type_is(tick(l,102,state(SYS_ENABLED)),TYPE_POSITION_CMD); });
    test("active bus fault blocks enable",[]{ ArmLink l(config());l.set_claimed(0,true);
        require(tick(l,100,state(SYS_FAULT,127,FAULT_CAN_BUS,-1000000,5)).empty(),"enabled bus fault"); });
    test("wrong enable ACK sequence ignored",[]{ ArmLink l(config());l.set_claimed(0,true);tick(l,100,state());
        tick(l,101,ack(99));require(l.status().state==ArmState::WAIT_ENABLE_ACK,"wrong ACK accepted"); });
    test("multiple ACKs do not hide rejection",[]{ ArmLink l(config());arm(l);
        auto a=ack(1,ACK_OUT_OF_RANGE,TYPE_POSITION_CMD);auto b=ack(2,ACK_OK,TYPE_STOP);a.insert(a.end(),b.begin(),b.end());
        type_is(tick(l,110,a),TYPE_STOP);require(l.status().state==ArmState::FAULT,"rejection lost");
        require(l.status().last_ack_result==ACK_OUT_OF_RANGE,"rejection code overwritten"); });
    test("unsafe STATE cannot be overwritten by healthy STATE",[]{ ArmLink l(config());arm(l);
        auto a=state(SYS_FAULT,127,FAULT_FEEDBACK_LOST);auto b=state(SYS_RUNNING);a.insert(a.end(),b.begin(),b.end());
        type_is(tick(l,110,a),TYPE_STOP); });
    test("explicit controller stop permits recovery",[]{ ArmLink l(config());arm(l);
        type_is(tick(l,110,state(SYS_FAULT,127,FAULT_LINK_TIMEOUT)),TYPE_STOP);
        require(tick(l,120,state()).empty(),"automatic re-arm"); l.set_claimed(0,false);l.set_claimed(0,true);
        type_is(tick(l,121,state(SYS_FAULT,127,FAULT_LINK_TIMEOUT)),TYPE_ENABLE); });
    test("stop is emitted once",[]{ ArmLink l(config());arm(l);l.set_joint_command(0,1.0);
        type_is(tick(l,121),TYPE_STOP);require(tick(l,122).empty(),"duplicate STOP"); });
    test("unclaimed joints ignore stale commands",[]{ ArmLink l(config());arm(l);l.set_joint_command(1,100.0);
        require(target(tick(l,121),1)==-200000,"unclaimed command used"); });
    test("infinity rejected before integer conversion",[]{ ArmLink l(config());arm(l);
        l.set_joint_command(0,std::numeric_limits<double>::infinity());type_is(tick(l,121),TYPE_STOP); });
    test("current firmware rejects positive J2 target on host",[]{ ArmLink l(config());arm(l);l.set_claimed(1,true);
        l.set_joint_command(1,0.2);type_is(tick(l,121),TYPE_STOP); });
    test("sign and offset round trip",[]{ auto c=config();c.joints[1].sign=-1;c.joints[1].offset=-0.1;ArmLink l(c);arm(l);
        double p,v,e;l.get_joint_state(1,p,v,e);require(std::abs(p-0.1)<1e-9,"mapping feedback");
        l.set_claimed(1,true);l.set_joint_command(1,0.105);require(target(tick(l,121),1)==-205000,"mapping command"); });
    test("uncalibrated gripper has no fake metre value",[]{ ArmLink l(config());arm(l);double p,v,e;
        l.get_grip_state(p,v,e);require(std::isnan(p),"invented gripper position"); });
    test("calibrated gripper moves within step",[]{ auto c=config();c.grip.calibrated=true;c.grip.command_enable=true;
        c.grip.mcu_closed=-2;c.grip.mcu_open=0;c.grip.q_close_m=0;c.grip.q_open_m=0.01;
        ArmLink l(c);arm(l);l.set_claimed(6,true);l.set_grip_command(0.005025);
        require(std::abs(target(tick(l,121),6)+995000)<=1,"gripper conversion"); });
    test("calibration and hard limits validated",[]{ auto c=config();require(c.validation_error().empty(),"valid config");
        c.grip.command_enable=true;require(!c.validation_error().empty(),"missing grip calibration accepted");
        c=config();c.joints[1].mcu_max=1;require(!c.validation_error().empty(),"unsafe limits accepted");
        c=config();c.mapping_calibrated=false;require(!c.validation_error().empty(),"unknown mapping enabled"); });
    test("read does not generate commands",[]{ ArmLink l(config());arm(l);auto s=state(SYS_RUNNING);
        l.receive(121,s.data(),s.size());l.set_joint_command(0,0.105);
        require(target(tick(l,121),0)==105000,"write did not use current command"); });
    test("handshake timeout",[]{ ArmLink l(config());l.set_claimed(0,true);tick(l,100,state());
        type_is(tick(l,301,state()),TYPE_STOP); });
    test("time wraparound",[]{ ArmLink l(config());arm(l,0xfffffff0U);
        type_is(tick(l,10,state(SYS_RUNNING)),TYPE_POSITION_CMD); });
    test("out-of-range measured baseline rejected",[]{ ArmLink l(config());l.set_claimed(0,true);
        type_is(tick(l,100,state(SYS_IDLE,127,0,100000)),TYPE_STOP); });
    test("telemetry cannot arm",[]{ auto c=config();c.allow_enable=false;ArmLink l(c);l.set_claimed(0,true);
        require(tick(l,100,state()).empty(),"telemetry sent enable"); });
    std::cout<<passed<<" passed, "<<failed<<" failed\n";return failed?1:0;
}
