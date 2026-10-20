// Test double only. This directory must never be on the hardware build path.
#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
namespace rysen {
enum class ErrorCode{ERROR_CODE_OK=0,ERROR=1};
enum class ConnectionType{CONNECTION_TYPE_ETHERNET=1};
enum class HandDir{LEFT=0};
enum class FingerId{FINGER_ID_INDEX=1};
enum class JointId{JOINT_ID_INDEX_J1=6};
struct JointState{JointId joint_id;double position=0,velocity=0;};
struct JointStates{std::chrono::steady_clock::time_point timestamp;std::vector<JointState> joint_states;};
struct JointControlParam{JointId joint_id;double position=0,velocity=0,acceleration=0;};
struct MoveJPositionFollowParam{JointId id;double position=0;};
struct MaxJointSpeed{JointId joint_id;double speed;};
struct MaxJointAccel{JointId joint_id;double accel;};
struct VersionInfo{std::string sdk_version="fake",hand_firmware_version="fake";};
struct ParamInfo{std::vector<double>max_speed=std::vector<double>(21,1),max_accel=std::vector<double>(21,2);};
class Rysen {
    std::atomic<bool> live{false},enabled{false},moving{false};
    std::atomic<double> q{0};std::thread stream,async_motion;
    std::string mode=std::getenv("TRIAL_TEST_MODE")?std::getenv("TRIAL_TEST_MODE"):"normal";
    std::chrono::steady_clock::time_point start;
    int move_count=0;
    ParamInfo params;
    std::array<std::atomic<bool>,5> finger_states{};
public:
    Rysen(){if(mode=="return_zero")q=7*3.14159265358979323846/180.;}
    ~Rysen(){Disconnect();}
    ErrorCode Connect(const std::string&,ConnectionType){return ErrorCode::ERROR_CODE_OK;}
    HandDir GetHandDir(){return HandDir::LEFT;}
    bool IsConnected(){return true;}
    ErrorCode GetHardwareErrorCode(){return moving && mode=="fault"?ErrorCode::ERROR:ErrorCode::ERROR_CODE_OK;}
    bool IsFingerEnabled(FingerId f)const{
        if(mode=="other_enabled" && static_cast<int>(f)==2)return true;
        return static_cast<int>(f)==1?enabled.load():finger_states[static_cast<int>(f)].load();
    }
    ParamInfo GetParameters(){return params;}
    VersionInfo GetVersionInfo(){return {};}
    ErrorCode SetMaxJointSpeed(const std::vector<MaxJointSpeed>&values){std::cout<<"FAKE_SET_SPEED"<<std::endl;for(auto&p:values)params.max_speed[static_cast<int>(p.joint_id)]=p.speed;return ErrorCode::ERROR_CODE_OK;}
    ErrorCode SetMaxJointAccel(const std::vector<MaxJointAccel>&values){std::cout<<"FAKE_SET_ACCEL"<<std::endl;for(auto&p:values)params.max_accel[static_cast<int>(p.joint_id)]=p.accel;return ErrorCode::ERROR_CODE_OK;}
    ErrorCode MoveJPositionFollow(const std::vector<MoveJPositionFollowParam>&values){
        if(values.size()!=21)return ErrorCode::ERROR;
        if((mode=="thumb_range"||mode=="nonthumb_range")&&values.at(0).position>.3)return static_cast<ErrorCode>(3);
        if(mode=="nonthumb_range"&&values.at(6).position>.3)return static_cast<ErrorCode>(3);
        if(mode=="follow_comm_error"&&values.at(0).position>.3)return ErrorCode::ERROR;
        std::cout<<"FAKE_FOLLOW"<<std::endl;moving=true;
        const double goal=values.at(6).position;
        q=q.load()+std::clamp(goal-q.load(),-.3*3.14159265358979323846/180.,.3*3.14159265358979323846/180.);
        return ErrorCode::ERROR_CODE_OK;
    }
    ErrorCode SetRobotModelFromURDF(const std::string&){return ErrorCode::ERROR_CODE_OK;}
    ErrorCode SetFingerEnabled(const std::vector<FingerId>&fingers){
        for(auto f:fingers){finger_states[static_cast<int>(f)]=true;if(static_cast<int>(f)==1)enabled=true;}
        std::cout<<"FAKE_ENABLE"<<std::endl;enabled=true;
        return mode=="enable_error"?ErrorCode::ERROR:ErrorCode::ERROR_CODE_OK;
    }
    ErrorCode SetFingerDisabled(const std::vector<FingerId>&fingers){
        for(auto f:fingers){finger_states[static_cast<int>(f)]=false;if(static_cast<int>(f)==1)enabled=false;}
        std::cout<<"FAKE_DISABLE"<<std::endl;return ErrorCode::ERROR_CODE_OK;
    }
    ErrorCode RegisterGetJointStatesCallback(std::function<void(const JointStates&)> callback,unsigned){
        live=true;stream=std::thread([this,callback](){
            while(live){
                if(!(moving && mode=="stale")){
                    JointStates states;states.timestamp=std::chrono::steady_clock::now();
                    for(int i=0;i<21;++i)states.joint_states.push_back({static_cast<JointId>(i),i==6?q.load():mode=="relaxed"&&(i==7||i==8)?-.143:0,0});
                    callback(states);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });return ErrorCode::ERROR_CODE_OK;
    }
    void UnregisterGetJointStatesCallback(){live=false;if(stream.joinable())stream.join();}
    void Disconnect(){UnregisterGetJointStatesCallback();if(async_motion.joinable())async_motion.join();}
    ErrorCode MoveJoint(const std::vector<JointControlParam>&commands){
        ++move_count;
        if(async_motion.joinable())async_motion.join();
        std::cout<<"FAKE_MOVE count="<<commands.size()<<" id="<<static_cast<int>(commands.at(0).joint_id)
                 <<" target="<<commands.at(0).position<<std::endl;
        if(mode=="move_rejected" || (mode=="return_rejected" && move_count==2))return ErrorCode::ERROR;
        moving=true;
        const double initial=q.load();
        if(mode=="early_return"){
            const double target=commands[0].position;
            async_motion=std::thread([this,target,initial](){
                for(int n=1;n<=30 && enabled;++n){
                    q=initial+(target-initial)*n/30.;std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                moving=false;
            });return ErrorCode::ERROR_CODE_OK;
        }
        for(int n=1;n<=30 && enabled;++n){
            q=initial+(commands[0].position-initial)*n/30.;std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // Keep fault/stale mode active until main thread disables the finger.
        if(mode=="fault" || mode=="stale")while(enabled)std::this_thread::sleep_for(std::chrono::milliseconds(10));
        moving=false;return ErrorCode::ERROR_CODE_OK;
    }
};
}
