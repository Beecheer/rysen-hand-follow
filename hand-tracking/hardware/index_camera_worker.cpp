// Native index/full-hand follow worker. stdin is a private local pipe.
// TARGET degrees / TARGET21 radians, START, STOP. Starts disabled.
#include "rysen_apexhand_sdk.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <fstream>
#include <mutex>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
constexpr double RAD=3.14159265358979323846/180.;
volatile std::sig_atomic_t stop_signal=0;
struct Feedback{std::array<double,21> q{},v{};Clock::time_point at{},stamp{};bool valid=false;};
int main(int argc,char**argv){
    const bool full=argc==2&&std::string(argv[1])=="--full-hand";
    if(argc>2||(argc==2&&!full))return 2;
    const double speed=6*RAD,accel=10*RAD; // Index trial only; full-hand retains SDK parameters.
    std::array<double,21> lower{},upper{},full_goal{},full_command{};
    if(full){
        std::ifstream limits("/opt/hand-follow/full-hand-limits.txt");
        for(int i=0,id;i<21;++i)if(!(limits>>id>>lower[i]>>upper[i])||id!=i||!std::isfinite(lower[i])||!std::isfinite(upper[i])||lower[i]>upper[i])return 3;
    }
    std::signal(SIGTERM,[](int){stop_signal=1;});std::signal(SIGINT,[](int){stop_signal=1;});
    rysen::Rysen bot;std::mutex mutex;Feedback latest;bool connected=false,subscribed=false,active=false;
    bool enable_attempted=false,limits_changed=false;rysen::ParamInfo original;
    const auto ok=rysen::ErrorCode::ERROR_CODE_OK;const auto index=rysen::FingerId::FINGER_ID_INDEX;
    std::vector<rysen::FingerId> fingers;
    if(full)for(int i=0;i<5;++i)fingers.push_back(static_cast<rysen::FingerId>(i));else fingers={index};
    Clock::time_point target_at{},armed_at{},sent_at{},printed_at{};double goal=0,command=0;
    std::array<double,21> hold{};std::string state="IDLE",reason="ready_disabled",last_stop_reason,buffer;
    int motion_calls=0,fault=0;std::array<bool,5> enabled{};
    std::array<double,21> accepted_command{};
    bool have_accepted=false,thumb_range_held=false;int thumb_range_rejections=0;
    std::string sdk_version,firmware_version;
    auto snapshot=[&](){std::lock_guard<std::mutex> guard(mutex);return latest;};
    auto fresh=[&](const Feedback&f){return f.valid && Clock::now()-f.at<250ms;};
    auto disable=[&](const std::string&why){
        if(active || why!="manual_or_source_stop")last_stop_reason=why;
        active=false;target_at={};state="IDLE";reason=why;
        if(enable_attempted){
            const auto result=bot.SetFingerDisabled(fingers);
            bool any=false;for(auto finger:fingers)any|=bot.IsFingerEnabled(finger);
            if(result!=ok || any){state="FAULT";reason="disable_not_confirmed";return false;}
            enable_attempted=false;
        }return true;
    };
    auto restore=[&](){
        if(!limits_changed)return;
        std::vector<rysen::MaxJointSpeed> speeds;std::vector<rysen::MaxJointAccel> accels;
        for(int i=0;i<21;++i){speeds.push_back({static_cast<rysen::JointId>(i),original.max_speed[i]});
            accels.push_back({static_cast<rysen::JointId>(i),original.max_accel[i]});}
        if(bot.SetMaxJointSpeed(speeds)!=ok || bot.SetMaxJointAccel(accels)!=ok)
            std::cerr<<"LIMIT_RESTORE_NOT_CONFIRMED"<<std::endl;
        limits_changed=false;
    };
    try{
        if(bot.Connect("192.168.0.102",rysen::ConnectionType::CONNECTION_TYPE_ETHERNET)!=ok)throw std::runtime_error("connect_failed");
        connected=true;
        auto version=bot.GetVersionInfo();sdk_version=version.sdk_version;firmware_version=version.hand_firmware_version;
        if(bot.GetHandDir()!=rysen::HandDir::LEFT)throw std::runtime_error("not_left");
        for(int i=0;i<5;++i)if(bot.IsFingerEnabled(static_cast<rysen::FingerId>(i)))throw std::runtime_error("another_controller_enabled");
        if(bot.SetRobotModelFromURDF("/workspace/install/rysen_apexhand_description/share/rysen_apexhand_description/urdf/apex_hand_left.urdf")!=ok)
            throw std::runtime_error("model_load_failed");
        original=bot.GetParameters();
        if(original.max_speed.size()!=21 || original.max_accel.size()!=21)throw std::runtime_error("invalid_device_parameters");
        if(bot.RegisterGetJointStatesCallback([&](const rysen::JointStates&s){
            Feedback f;std::array<bool,21> seen{};
            for(const auto&j:s.joint_states){int i=static_cast<int>(j.joint_id);
                if(i<0||i>=21||seen[i]||!std::isfinite(j.position)||!std::isfinite(j.velocity))return;
                seen[i]=true;f.q[i]=j.position;f.v[i]=j.velocity;}
            if(!std::all_of(seen.begin(),seen.end(),[](bool v){return v;}))return;
            f.at=Clock::now();f.stamp=s.timestamp;f.valid=true;
            std::lock_guard<std::mutex> guard(mutex);if(!latest.valid||f.stamp!=latest.stamp)latest=f;
        },50)!=ok)throw std::runtime_error("subscribe_failed");
        subscribed=true;
        while(!stop_signal){
            pollfd fd{STDIN_FILENO,POLLIN,0};int polled=poll(&fd,1,10);
            if(polled>0 && (fd.revents&(POLLIN|POLLHUP))){
                char bytes[2048];ssize_t count=::read(STDIN_FILENO,bytes,sizeof bytes);
                if(count<=0)break;
                buffer.append(bytes,count);if(buffer.size()>8192)throw std::runtime_error("input_overflow");
                for(size_t e;(e=buffer.find('\n'))!=std::string::npos;){
                    const auto line=buffer.substr(0,e);buffer.erase(0,e+1);
                    if(line=="STOP"){disable("manual_or_source_stop");continue;}
                    if(full&&line.rfind("TARGET21 ",0)==0){
                        std::istringstream input(line.substr(9));std::array<double,21> values{};bool valid=true;std::string extra;
                        for(int i=0;i<21;++i)if(!(input>>values[i])||!std::isfinite(values[i])||values[i]<lower[i]-1e-7||values[i]>upper[i]+1e-7)valid=false;
                        if(input>>extra)valid=false;
                        for(auto pair:std::array<std::pair<int,int>,5>{{{4,3},{8,7},{12,11},{16,15},{20,19}}})
                            if(std::abs(values[pair.first]-values[pair.second])>1e-6)valid=false;
                        if(!valid){disable("invalid_full_hand_target");continue;}
                        full_goal=values;target_at=Clock::now();continue;
                    }
                    if(line.rfind("TARGET ",0)==0){
                        if(full){disable("wrong_target_profile");continue;}
                        std::istringstream input(line.substr(7));double value;std::string extra;
                        if(!(input>>value)||input>>extra||!std::isfinite(value)||value<0||value>10){disable("invalid_target");continue;}
                        goal=value*RAD;target_at=Clock::now();continue;
                    }
                    if(line!="START"){disable("invalid_command");continue;}
                    auto f=snapshot();
                    if(active)continue;
                    if(enable_attempted || !fresh(f) || target_at==Clock::time_point{} || Clock::now()-target_at>200ms ||
                       bot.GetHardwareErrorCode()!=ok){reason="start_requires_fresh_target_feedback";continue;}
                    bool all_off=true;for(int i=0;i<5;++i)all_off&=!bot.IsFingerEnabled(static_cast<rysen::FingerId>(i));
                    if(!all_off){reason="start_requires_all_disabled";continue;}
                    bool pose_ok=true;
                    // Full-hand targets remain bounded. Actual relaxed positions
                    // are handled by the SDK model, not the camera target envelope.
                    if(!full)pose_ok=f.q[6]>=-1*RAD&&f.q[6]<=11*RAD&&std::abs(f.q[5])<=2*RAD&&std::abs(f.q[7])<=2*RAD;
                    if(!pose_ok){reason="pose_outside_start_envelope";continue;}
                    hold=f.q;command=std::clamp(f.q[6],0.,10*RAD);
                    full_command=f.q;
                    if(!full){
                    std::vector<rysen::MaxJointSpeed> speeds;std::vector<rysen::MaxJointAccel> accels;
                    for(int i=0;i<21;++i){speeds.push_back({static_cast<rysen::JointId>(i),full||i>=5&&i<=8?speed:original.max_speed[i]});
                        accels.push_back({static_cast<rysen::JointId>(i),full||i>=5&&i<=8?accel:original.max_accel[i]});}
                    limits_changed=true;
                    if(bot.SetMaxJointSpeed(speeds)!=ok||bot.SetMaxJointAccel(accels)!=ok){state="FAULT";reason="limit_configuration_failed";continue;}
                    auto configured=bot.GetParameters();
                    bool limits_ok=configured.max_speed.size()==21&&configured.max_accel.size()==21;
                    if(limits_ok)for(int i=full?0:5;i<=(full?20:8);++i)limits_ok&=configured.max_speed[i]>0&&configured.max_accel[i]>0&&configured.max_speed[i]<=speed+1e-6&&configured.max_accel[i]<=accel+1e-6;
                    if(!limits_ok){state="FAULT";reason="limit_readback_failed";continue;}
                    }
                    enable_attempted=true;
                    if(bot.SetFingerEnabled(fingers)!=ok){disable("enable_failed");continue;}
                    bool all_on=true;for(auto finger:fingers){bool on=bot.IsFingerEnabled(finger);enabled[static_cast<int>(finger)]=on;all_on&=on;}
                    if(!all_on){disable("enable_failed");continue;}
                    active=true;have_accepted=false;thumb_range_held=false;state="FOLLOWING";last_stop_reason.clear();reason=full?"full_hand_following":"index_only_0_to_10_deg";armed_at=sent_at=Clock::now();
                }
            }
            auto now=Clock::now();auto f=snapshot();
            if(now-printed_at>=50ms){
                fault=static_cast<int>(bot.GetHardwareErrorCode());
                for(int i=0;i<5;++i)enabled[i]=bot.IsFingerEnabled(static_cast<rysen::FingerId>(i));
            }
            if(active){
                if(fault!=0)disable("sdk_hardware_fault");
                else if(!bot.IsConnected())disable("sdk_disconnected");
                else if(!fresh(f))disable("sdk_feedback_timeout");
                else if(target_at==Clock::time_point{}||now-target_at>250ms)disable("camera_target_timeout");
                else if(!full&&now-armed_at>30s)disable("session_30_second_limit");
                else if(!full&&(f.q[6]<-1.5*RAD||f.q[6]>12*RAD))disable("index_position_outside_envelope");
                else{
                    bool other=false;
                    if(full){for(int i=0;i<5;++i)other|=!enabled[i];}
                    else{for(int i=0;i<21;++i)if(i!=6&&std::abs(f.q[i]-hold[i])>2*RAD)other=true;
                        for(int i:{0,2,3,4})other|=enabled[i];}
                    if(other)disable("other_joint_or_finger_changed");
                    else if(now-sent_at>=50ms){
                        double dt=std::min(.1,std::chrono::duration<double>(now-sent_at).count());sent_at=now;
                        command+=std::clamp(goal-command,-speed*dt,speed*dt);
                        if(full){full_command=full_goal;
                            for(auto pair:std::array<std::pair<int,int>,5>{{{4,3},{8,7},{12,11},{16,15},{20,19}}})full_command[pair.first]=full_command[pair.second];}
                        std::vector<rysen::MoveJPositionFollowParam> params;
                        for(int i=0;i<21;++i){rysen::MoveJPositionFollowParam p;
                            p.id=static_cast<rysen::JointId>(i);p.position=full?full_command[i]:i==6?command:hold[i];params.push_back(p);}
                        ++motion_calls;
                        auto follow_began=Clock::now();
                        auto follow_result=bot.MoveJPositionFollow(params);
                        auto follow_ms=std::chrono::duration<double,std::milli>(Clock::now()-follow_began).count();
                        thumb_range_held=false;
                        // OUT_OF_RANGE is an argument rejection, not a hardware
                        // fault. One bounded retry changes only the thumb target
                        // back to its last accepted tuple; never re-enable here.
                        if(full&&static_cast<int>(follow_result)==3&&have_accepted&&fresh(snapshot())&&
                           bot.IsConnected()&&bot.GetHardwareErrorCode()==ok){
                            auto attempted=full_command;
                            for(int i=0;i<5;++i){full_command[i]=accepted_command[i];params[i].position=accepted_command[i];}
                            bool different=false;for(int i=0;i<5;++i)different|=std::abs(attempted[i]-full_command[i])>1e-7;
                            if(different){
                                ++motion_calls;++thumb_range_rejections;
                                follow_result=bot.MoveJPositionFollow(params);
                                if(follow_result==ok){thumb_range_held=true;
                                    std::cerr<<"THUMB_TARGET_HELD_AFTER_RANGE_REJECT count="<<thumb_range_rejections<<std::endl;}
                            }
                        }
                        if(full&&follow_result==ok){accepted_command=full_command;have_accepted=true;}
                        if(follow_result!=ok){std::cerr<<"FOLLOW_REJECT code="<<static_cast<int>(follow_result)<<" duration_ms="<<follow_ms<<std::endl;
                            disable("follow_command_rejected_code_"+std::to_string(static_cast<int>(follow_result)));}
                    }
                }
            }
            if(now-printed_at>=50ms){
                printed_at=now;
                std::cout<<"STATE {\"state\":\""<<state<<"\",\"reason\":\""<<reason<<"\",\"active\":"<<(active?"true":"false")
                         <<",\"last_stop_reason\":\""<<last_stop_reason<<"\""
                         <<",\"thumb_range_held\":"<<(thumb_range_held?"true":"false")<<",\"thumb_range_rejections\":"<<thumb_range_rejections
                         <<",\"connected\":"<<(bot.IsConnected()?"true":"false")<<",\"fault_code\":"<<fault<<",\"motion_calls\":"<<motion_calls
                         <<",\"feedback_age_ms\":"<<(f.valid?std::chrono::duration<double,std::milli>(now-f.at).count():1e9)
                         <<",\"full_hand\":"<<(full?"true":"false")<<",\"speed_policy\":\""<<(full?"sdk_original":"index_limited")<<"\",\"speed_deg_s\":"<<(full?"null":std::to_string(speed/RAD))
                         <<",\"goal_deg\":"<<(full?full_goal[6]:goal)/RAD<<",\"command_deg\":"<<(full?full_command[6]:command)/RAD<<",\"commands\":[";
                for(int i=0;i<21;++i)std::cout<<(i?",":"")<<(full?full_command[i]:i==6?command:hold[i]);
                std::cout<<"],\"positions\":[";
                for(int i=0;i<21;++i)std::cout<<(i?",":"")<<f.q[i];
                std::cout<<"],\"enabled\":[";for(int i=0;i<5;++i)std::cout<<(i?",":"")<<(bot.IsFingerEnabled(static_cast<rysen::FingerId>(i))?"true":"false");
                std::cout<<"],\"version\":{\"sdk_version\":\""<<sdk_version<<"\",\"hand_firmware_version\":\""<<firmware_version<<"\"}}"<<std::endl;
            }
        }
    }catch(const std::exception&e){std::cerr<<"WORKER_ERROR "<<e.what()<<std::endl;}
    bool disabled=disable("worker_exit");
    if(disabled)restore();
    if(subscribed)bot.UnregisterGetJointStatesCallback();
    if(connected)bot.Disconnect();
    return disabled?0:2;
}
