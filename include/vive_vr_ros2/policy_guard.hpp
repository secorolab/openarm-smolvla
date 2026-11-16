/* SPDX-License-Identifier: MIT */
#pragma once
#include <array>
#include <vector>
#include "vive_vr_ros2/hardware_guard.hpp"

namespace vive_vr_ros2 {
using PolicyPose = std::array<double,8>;
// Policy rates are independent of teleoperation and homing settings.
inline constexpr double policy_joint_speed=0.30, policy_finger_speed=0.010;
inline bool bound_policy_plan(std::vector<PolicyPose> &plan, const PolicyPose &lower,
                              const PolicyPose &upper) {
    if (plan.size()!=50) return false;
    for (auto &pose:plan) for (size_t i=0;i<8;++i) {
        const double lower_allowance=i==7 ? 0.003 : 0.05;
        const double upper_allowance=i==7 ? 0.005 : 0.05;
        if (!std::isfinite(pose[i]) || pose[i]<lower[i]-lower_allowance || pose[i]>upper[i]+upper_allowance)
            return false;
        pose[i]=std::clamp(pose[i],lower[i],upper[i]);
    }
    return true;
}
inline bool policy_step(const PolicyPose &goal, const PolicyPose &measured,
                        const PolicyPose &lower, const PolicyPose &upper, double dt,
                        PolicyPose &command, PolicyPose &velocity) {
    if (!std::isfinite(dt) || dt<=0 || dt>0.1) return false;
    const double step_dt=std::min(dt,0.02);
    auto next=command, speeds=velocity;
    for (size_t i=0;i<8;++i) {
        if (!std::isfinite(measured[i]) || !std::isfinite(goal[i]) ||
            measured[i]<lower[i]-(i==7 ? .002 : .05) || measured[i]>upper[i]+(i==7 ? .002 : .05)) return false;
        const double lead=i==7 ? .004 : .15;
        const double bounded=std::clamp(goal[i],lower[i],upper[i]);
        if (i==7) {
            // Position-only contact tolerance: stop adding squeeze before the
            // old 4 mm edge. A small rebound may hold, but cannot add closure.
            const double closing_gap=measured[i]-command[i];
            if (closing_gap>0.006 || closing_gap < -lead) return false;
            if (bounded<=command[i] && closing_gap>=0.003) {
                speeds[i]=0;
                continue;
            }
        } else if (std::abs(command[i]-measured[i])>lead) return false;
        next[i]=smooth_bounded_step(bounded,command[i],speeds[i],
            i==7 ? policy_finger_speed : policy_joint_speed, i==7 ? .05 : .6,step_dt);
        next[i]=std::clamp(next[i],lower[i],upper[i]);
        const bool beyond_lead=i==7
            ? ((next[i]<command[i] && measured[i]-next[i]>.003) || next[i]-measured[i]>lead)
            : std::abs(next[i]-measured[i])>lead;
        if (beyond_lead) { next[i]=command[i]; speeds[i]=0; }
    }
    command=next; velocity=speeds;
    return true;
}
}
