// One bounded, explicitly authorized index MCP trial. No camera command path.
// Default is preflight only. --execute performs one +3 degree target.
// --cycle is a separately authorized fixed 8 degree target then 0 degree return.
#include "rysen_apexhand_sdk.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

using Clock=std::chrono::steady_clock;
using namespace std::chrono_literals;
constexpr double RAD=3.14159265358979323846/180.;
volatile std::sig_atomic_t interrupted=0;
void signal_handler(int){interrupted=1;}

struct Sample {
    std::array<double,21> q{},v{};
    Clock::time_point received{},stamp{};
    bool valid=false;
};

int main(int argc,char**argv){
    const std::string mode=argc==2?argv[1]:"--preflight";
    const bool cycle=mode=="--cycle" || mode=="--preflight-cycle";
    const bool recovery=mode=="--return-zero" || mode=="--preflight-return-zero";
    const bool execute=mode=="--execute" || mode=="--cycle" || mode=="--return-zero";
    if(argc>2 || (!execute && mode!="--preflight" && mode!="--preflight-cycle" && mode!="--preflight-return-zero"))return 2;
    // Recovery acceptance is explicitly reported, never applied as an offset.
    const double arrival_tolerance=(recovery?1.2:.8)*RAD;
    std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
    rysen::Rysen bot;bool connected=false,subscribed=false,enable_attempted=false;
    std::mutex mutex;Sample latest;
    std::atomic<bool> motion_done{false};std::atomic<int> motion_result{-1};
    std::thread motion;
    const auto index=rysen::FingerId::FINGER_ID_INDEX;
    const auto ok=rysen::ErrorCode::ERROR_CODE_OK;
    std::ofstream journal(recovery?"/tmp/index-return-zero.csv":cycle?"/tmp/index-cycle-trial.csv":"/tmp/index-step-trial.csv",std::ios::trunc);
    journal<<"elapsed_s,event";for(int i=0;i<21;++i)journal<<",q"<<i<<"_rad";journal<<"\n";
    auto began=Clock::now();
    auto read=[&](){
        std::lock_guard<std::mutex> guard(mutex);
        if(!latest.valid || Clock::now()-latest.received>250ms)throw std::runtime_error("feedback stale or incomplete");
        return latest;
    };
    auto record=[&](const std::string&event,const Sample&s){
        journal<<std::chrono::duration<double>(Clock::now()-began).count()<<","<<event;
        for(double q:s.q)journal<<","<<q;journal<<"\n";journal.flush();
    };
    auto disable=[&](){
        if(!enable_attempted)return true;
        for(int attempt=0;attempt<2;++attempt){
            const auto result=bot.SetFingerDisabled({index});
            if(result==ok && !bot.IsFingerEnabled(index)){
                enable_attempted=false;std::cout<<"INDEX_DISABLED_CONFIRMED"<<std::endl;return true;
            }
        }
        std::cerr<<"INDEX_DISABLE_NOT_CONFIRMED: use device physical stop"<<std::endl;return false;
    };
    int exit_code=0;
    try {
        if(bot.Connect("192.168.0.102",rysen::ConnectionType::CONNECTION_TYPE_ETHERNET)!=ok)
            throw std::runtime_error("connect failed");
        connected=true;
        if(bot.GetHandDir()!=rysen::HandDir::LEFT)throw std::runtime_error("not LEFT hand");
        if(bot.GetHardwareErrorCode()!=ok)throw std::runtime_error("device fault");
        for(int i=0;i<5;++i)if(bot.IsFingerEnabled(static_cast<rysen::FingerId>(i)))
            throw std::runtime_error("preflight requires all five fingers disabled");
        if(bot.SetRobotModelFromURDF("/workspace/install/rysen_apexhand_description/share/rysen_apexhand_description/urdf/apex_hand_left.urdf")!=ok)
            throw std::runtime_error("SDK model load failed");
        auto callback=[&](const rysen::JointStates&states){
            Sample s;std::array<bool,21> seen{};
            for(const auto&joint:states.joint_states){
                const int id=static_cast<int>(joint.joint_id);
                if(id<0 || id>=21 || seen[id] || !std::isfinite(joint.position) || !std::isfinite(joint.velocity))return;
                seen[id]=true;s.q[id]=joint.position;s.v[id]=joint.velocity;
            }
            if(!std::all_of(seen.begin(),seen.end(),[](bool value){return value;}))return;
            s.received=Clock::now();s.stamp=states.timestamp;s.valid=true;
            std::lock_guard<std::mutex> guard(mutex);
            if(!latest.valid || s.stamp!=latest.stamp)latest=s;
        };
        if(bot.RegisterGetJointStatesCallback(callback,50)!=ok)throw std::runtime_error("subscribe failed");
        subscribed=true;
        for(int n=0;n<50;++n){
            {std::lock_guard<std::mutex> guard(mutex);if(latest.valid)break;}
            std::this_thread::sleep_for(20ms);
        }
        const auto baseline=read();record("baseline",baseline);
        for(int id:{5,6,7,8})if(std::abs(baseline.q[id])>(recovery && id==6?10:cycle && id==6?4:2)*RAD)
            throw std::runtime_error("index outside initial posture envelope");
        const double target=recovery?0.:cycle?8*RAD:baseline.q[6]+3*RAD;
        // Additional fixed trial envelope; no model-wide hardware limits inferred.
        if(!cycle && !recovery && (target<0 || target>5*RAD))throw std::runtime_error("trial target outside 0..5 degree envelope");
        const auto parameters=bot.GetParameters();
        if(parameters.max_speed.size()!=21 || parameters.max_accel.size()!=21 ||
           parameters.max_speed[6]<6*RAD || parameters.max_accel[6]<10*RAD)
            throw std::runtime_error("device parameter limits incompatible with trial");
        std::cout<<"PREFLIGHT_OK baseline_deg="<<baseline.q[6]/RAD<<" target_deg="<<target/RAD
                 <<" speed_deg_s=6 accel_deg_s2=10 cycle="<<cycle<<" recovery="<<recovery
                 <<" arrival_tolerance_deg="<<arrival_tolerance/RAD<<std::endl;
        if(execute){
            if(interrupted)throw std::runtime_error("interrupted before enable");
            enable_attempted=true; // Cleanup also runs if enabling partly fails.
            if(bot.SetFingerEnabled({index})!=ok || !bot.IsFingerEnabled(index))
                throw std::runtime_error("index enable failed");
            std::this_thread::sleep_for(150ms);
            const auto enabled=read();record("enabled",enabled);
            for(int id:{5,6,7,8})if(std::abs(enabled.q[id]-baseline.q[id])>1.5*RAD)
                throw std::runtime_error("index moved unexpectedly during enabling");
            for(int i:{0,2,3,4})if(bot.IsFingerEnabled(static_cast<rysen::FingerId>(i)))
                throw std::runtime_error("another finger is enabled");
            auto run_leg=[&](double leg_target,const std::string&leg){
            if(interrupted)throw std::runtime_error("interrupted before command");
            if(motion.joinable())motion.join();
            motion_done=false;motion_result=-1;
            rysen::JointControlParam command;
            command.joint_id=rysen::JointId::JOINT_ID_INDEX_J1;
            command.position=leg_target;command.velocity=6*RAD;command.acceleration=10*RAD;
            const auto motion_began=Clock::now();
            motion=std::thread([&,command](){
                try {motion_result=static_cast<int>(bot.MoveJoint({command}));}
                catch(...){motion_result=-2;}
                motion_done=true;
            });
            std::cout<<"ONE_MOVE_SUBMITTED joint_id=6 leg="<<leg<<" target_deg="<<leg_target/RAD<<std::endl;
            double peak=baseline.q[6],peak_speed=0.;
            Clock::time_point returned_at{},stable_at{};
            while(true) {
                std::this_thread::sleep_for(20ms);
                const auto current=read();record(leg,current);
                peak=std::max(peak,current.q[6]);peak_speed=std::max(peak_speed,std::abs(current.v[6]));
                if(interrupted)throw std::runtime_error("interrupted");
                if(!bot.IsConnected() || bot.GetHardwareErrorCode()!=ok)throw std::runtime_error("connection/fault stop");
                if(Clock::now()-motion_began>4s)throw std::runtime_error("motion exceeded 4 second deadline");
                if(((cycle || recovery) && (current.q[6]>10*RAD || current.q[6]<-1.5*RAD)) ||
                   (!cycle && !recovery && (current.q[6]-baseline.q[6]>5*RAD || current.q[6]-baseline.q[6]<-2*RAD)))
                    throw std::runtime_error("MCP exceeded trial displacement envelope");
                for(int i=0;i<21;++i)if(i!=6 && std::abs(current.q[i]-baseline.q[i])>2*RAD)
                    throw std::runtime_error("unexpected movement in another joint");
                // SDK success can precede measured arrival. Continue bounded
                // feedback monitoring without submitting another command.
                if(motion_done){
                    if(motion_result!=0)break;
                    if(returned_at==Clock::time_point{})returned_at=Clock::now();
                    if(std::abs(current.q[6]-leg_target)<=arrival_tolerance){
                        if(stable_at==Clock::time_point{})stable_at=Clock::now();
                        if(Clock::now()-stable_at>=150ms)break;
                    }else stable_at={};
                    if(Clock::now()-returned_at>800ms)
                        throw std::runtime_error("SDK returned but feedback did not settle; no retry");
                }
            }
            const auto end=read();record(leg+"_settled",end);
            std::cout<<"MOVE_RESULT="<<motion_result<<" leg="<<leg<<" end_deg="<<end.q[6]/RAD
                     <<" delta_deg="<<(end.q[6]-baseline.q[6])/RAD<<" peak_deg="<<peak/RAD
                     <<" peak_speed_deg_s="<<peak_speed/RAD<<std::endl;
            if(motion_result!=0 || std::abs(end.q[6]-leg_target)>arrival_tolerance)
                throw std::runtime_error("target/result validation failed; no retry");
            };
            run_leg(target,recovery?"return_zero":cycle?"outbound":"moving");
            if(cycle){
                for(int n=0;n<15;++n){
                    std::this_thread::sleep_for(20ms);const auto held=read();record("hold",held);
                    if(interrupted || bot.GetHardwareErrorCode()!=ok || !bot.IsConnected())
                        throw std::runtime_error("hold interrupted or device fault");
                    if(std::abs(held.q[6]-target)>1.5*RAD)throw std::runtime_error("hold position drifted");
                }
                run_leg(0.,"return_zero");
            }
            if(!disable())throw std::runtime_error("disable not confirmed");
            std::this_thread::sleep_for(300ms);record("after_disable",read());
            std::cout<<"TRIAL_COMPLETE "<<(recovery?"return_zero_only":cycle?"cycle_8_deg_return_zero":"single_target_no_return")<<std::endl;
        }
    }catch(const std::exception&error){
        std::cerr<<"TRIAL_ABORTED "<<error.what()<<std::endl;exit_code=1;
    }
    const bool disabled=disable();
    if(!disabled)exit_code=2;
    if(motion.joinable()){
        const auto deadline=Clock::now()+2s;
        while(!motion_done && Clock::now()<deadline)std::this_thread::sleep_for(20ms);
        if(!motion_done){
            std::cerr<<"BLOCKING_CALL_DID_NOT_RETURN disabled_confirmed="<<disabled<<std::endl;
            journal.flush();std::_Exit(3);
        }
        motion.join();
    }
    if(subscribed)bot.UnregisterGetJointStatesCallback();
    if(connected)bot.Disconnect();
    return exit_code;
}
