/* SPDX-License-Identifier: MIT */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "vive_vr_ros2/external_control.hpp"
#include "vive_vr_ros2/policy_guard.hpp"
namespace vive_vr_ros2 {
class SmolVLAControl final : public ExternalControl {
    using Clock=std::chrono::steady_clock;
public:
    SmolVLAControl(rclcpp::Node &node, ExternalControlHooks hooks):node_(node),hooks_(std::move(hooks)) {
        policy_status_=node_.create_publisher<std_msgs::msg::String>("~/policy_status",rclcpp::QoS(1).transient_local());
        policy_service_=node_.create_service<std_srvs::srv::SetBool>("~/policy_enable",
            [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
                   std_srvs::srv::SetBool::Response::SharedPtr res) {
                if (!req->data) { hooks_.stop("policy stopped by operator"); res->success=true; res->message="Policy stopped; VR remains disarmed"; return; }
                if (policy_active_) { res->message="Policy already active"; return; }
                if (const auto why=hooks_.blockers(); !why.empty()) { res->message=why; return; }
                const auto snapshot=hooks_.snapshot();
                names_=snapshot.names;
                policy_lower_=snapshot.lower; policy_upper_=snapshot.upper;
                policy_command_=snapshot.measured;
                for (size_t i=0;i<8;++i) {
                    if (!std::isfinite(policy_command_[i]) || policy_command_[i]<policy_lower_[i] || policy_command_[i]>policy_upper_[i]) {
                        res->message="Measured pose outside policy limits: "+names_[i]+"; return to zero first"; return;
                    }
                }
                hooks_.stop("policy acquiring control");
                policy_velocity_.fill(0); policy_plan_.clear(); policy_source_ns_=0;
                policy_active_=true; policy_received_=Clock::now();
                policy_token_=std::to_string(node_.now().nanoseconds())+"_"+std::to_string(++policy_generation_);
                std_msgs::msg::String status; status.data="active:"+policy_token_; policy_status_->publish(status);
                res->success=true; res->message=policy_token_;
            });
        policy_sub_=node_.create_subscription<trajectory_msgs::msg::JointTrajectory>("~/policy_plan",rclcpp::QoS(1),
            [this](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { receive_policy(*msg); });
    }
    bool active() const override { return policy_active_; }
    void tick(double dt) override { tick_policy(dt); }
    void stop(const char *reason) override {
        if (!policy_active_) return;
        hooks_.hold();
        policy_active_=false; policy_plan_.clear(); policy_velocity_.fill(0);
        std_msgs::msg::String status; status.data=std::string("stopped:")+reason;
        policy_status_->publish(status);
        RCLCPP_WARN(node_.get_logger(),"Policy stopped: %s; explicit restart required",reason);
    }
private:
    void receive_policy(const trajectory_msgs::msg::JointTrajectory &msg) {
        if (!policy_active_ || msg.header.frame_id!=policy_token_) return;
        const int64_t source=rclcpp::Time(msg.header.stamp).nanoseconds();
        const double age=(node_.now().nanoseconds()-source)*1e-9;
        if (source<=policy_source_ns_) return; // delayed/replayed chunk cannot renew ownership
        if (age<0 || age>.65) {
            hooks_.stop(("policy plan timestamp rejected: input age at bridge="+std::to_string(age*1000)+
                   " ms; allowed 0 to 650 ms. Check inference/transport delay and system clock").c_str()); return;
        }
        if (msg.joint_names.size()!=8 || msg.points.size()!=50) {
            hooks_.stop(("policy plan shape rejected: received "+std::to_string(msg.joint_names.size())+
                   " joints and "+std::to_string(msg.points.size())+" points; expected 8 joints and 50 points").c_str()); return;
        }
        std::vector<PolicyPose> plan(50);
        for (size_t i=0;i<8;++i) if (msg.joint_names[i]!=names_[i]) { hooks_.stop("policy joint order mismatch"); return; }
        for (size_t k=0;k<50;++k) {
            const auto &point=msg.points[k];
            const double t=point.time_from_start.sec+point.time_from_start.nanosec*1e-9;
            if (point.positions.size()!=8 || std::abs(t-k/30.0)>1e-6 ||
                !point.velocities.empty() || !point.accelerations.empty() || !point.effort.empty()) {
                hooks_.stop("policy plan dimensions/timing invalid"); return;
            }
            std::copy(point.positions.begin(),point.positions.end(),plan[k].begin());
        }
        if (!bound_policy_plan(plan,policy_lower_,policy_upper_)) { hooks_.stop("policy target exceeds clipping allowance"); return; }
        policy_plan_=std::move(plan); policy_source_ns_=source; policy_received_=Clock::now();
        policy_source_age_=age;
    }
    void tick_policy(double dt) {
        if (!hooks_.healthy()) { hooks_.stop("policy feedback or driver health lost"); return; }
        const double silence=std::chrono::duration<double>(Clock::now()-policy_received_).count();
        if (silence>.75) { hooks_.stop("policy prediction watchdog expired"); return; }
        if (policy_plan_.empty()) return;
        const double index=(policy_source_age_+silence)*30.0;
        if (index>=49) { hooks_.stop("policy trajectory exhausted"); return; }
        const size_t first=static_cast<size_t>(index);
        const double mix=index-first;
        PolicyPose goal{}, measured{};
        const auto snapshot=hooks_.snapshot();
        for (size_t i=0;i<8;++i) {
            goal[i]=policy_plan_[first][i]*(1-mix)+policy_plan_[first+1][i]*mix;
            measured[i]=snapshot.measured[i];
        }
        if (!policy_step(goal,measured,policy_lower_,policy_upper_,dt,policy_command_,policy_velocity_)) {
            std::string reason="policy step rejected: timer interval "+std::to_string(dt)+" s";
            for (size_t i=0;i<8;++i) {
                const double margin=i==7 ? .002 : .05;
                const double lead=i==7 ? (measured[i]>=policy_command_[i] ? .006 : .004) : .15;
                if (!std::isfinite(measured[i]) || !std::isfinite(goal[i]) ||
                    measured[i]<policy_lower_[i]-margin || measured[i]>policy_upper_[i]+margin ||
                    std::abs(policy_command_[i]-measured[i])>lead) {
                    const double scale=i==7 ? 1000.0 : 180.0/3.141592653589793;
                    const std::string unit=i==7 ? " mm" : " deg";
                    reason="policy stopped at "+names_[i]+
                        ": measured="+std::to_string(measured[i]*scale)+unit+
                        ", last command="+std::to_string(policy_command_[i]*scale)+unit+
                        ", gap="+std::to_string(std::abs(policy_command_[i]-measured[i])*scale)+unit+
                        ", allowed gap="+std::to_string(lead*scale)+unit+
                        ", measured range=["+std::to_string((policy_lower_[i]-margin)*scale)+
                        ", "+std::to_string((policy_upper_[i]+margin)*scale)+"]"+unit;
                    break;
                }
            }
            hooks_.stop(reason.c_str()); return;
        }
        hooks_.send(policy_command_);
    }
    bool policy_active_=false;
    uint64_t policy_generation_=0;
    std::string policy_token_;
    int64_t policy_source_ns_=0;
    double policy_source_age_=0;
    Clock::time_point policy_received_{};
    PolicyPose policy_command_{}, policy_velocity_{}, policy_lower_{}, policy_upper_{};
    std::vector<PolicyPose> policy_plan_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr policy_service_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr policy_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr policy_status_;
    rclcpp::Node &node_;
    ExternalControlHooks hooks_;
    std::array<std::string,8> names_;
};
}
extern "C" vive_vr_ros2::ExternalControl *openarm_create_external_control_v1(
    rclcpp::Node *node, vive_vr_ros2::ExternalControlHooks hooks) {
    return new vive_vr_ros2::SmolVLAControl(*node,std::move(hooks));
}
