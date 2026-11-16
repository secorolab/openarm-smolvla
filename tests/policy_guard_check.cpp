#include <cassert>
#include <limits>
#include "vive_vr_ros2/policy_guard.hpp"
using namespace vive_vr_ros2;
int main() {
    PolicyPose lo{},hi{},q{},v{},goal{},measured{};
    hi.fill(1); hi[7]=.044;
    std::vector<PolicyPose> plan(50);
    for(auto &p:plan) { p[3]=-.03; p[7]=-.0003; }
    assert(bound_policy_plan(plan,lo,hi));
    assert(plan[0][3]==0 && plan[0][7]==0);
    plan[47][7]=0.04621828347444534;
    assert(bound_policy_plan(plan,lo,hi));
    assert(plan[47][7]==hi[7]);
    plan[47][7]=-.0025;
    assert(bound_policy_plan(plan,lo,hi));
    assert(plan[47][7]==lo[7]);
    plan[47][7]=.04733394831418991;
    assert(bound_policy_plan(plan,lo,hi));
    assert(plan[47][7]==hi[7]);
    plan[47][7]=hi[7]+.005;
    assert(bound_policy_plan(plan,lo,hi));
    assert(plan[47][7]==hi[7]);
    plan[47][7]=.0491;
    assert(!bound_policy_plan(plan,lo,hi));
    plan[47][7]=-.0031;
    assert(!bound_policy_plan(plan,lo,hi));
    plan[47][7]=0;
    plan[0][3]=-.06; assert(!bound_policy_plan(plan,lo,hi));
    plan[0][3]=std::numeric_limits<double>::quiet_NaN(); assert(!bound_policy_plan(plan,lo,hi));
    goal.fill(1); goal[7]=.044;
    for(int k=0;k<400;++k) {
        const auto prev=q;
        measured=q;
        assert(policy_step(goal,measured,lo,hi,.02,q,v));
        for(size_t i=0;i<8;++i) {
            assert(q[i]>=lo[i] && q[i]<=hi[i]);
            assert(std::abs(q[i]-prev[i])<=(i==7?.010:.30)*.02+1e-12);
        }
        if(k==100) goal.fill(0);
    }
    measured=q; measured[0]-=.16;
    assert(!policy_step(goal,measured,lo,hi,.02,q,v));
    measured=q; assert(!policy_step(goal,measured,lo,hi,.2,q,v));
    // Reproduce the reported contact stop; the arm can continue while fingers hold.
    q.fill(0); v.fill(0); measured.fill(0); goal.fill(.1);
    q[7]=.022085063; measured[7]=.026086196; goal[7]=0;
    const double hold=q[7];
    for (int k=0;k<100;++k) {
        assert(policy_step(goal,measured,lo,hi,.02,q,v));
        assert(q[7]==hold && v[7]==0);
        for(size_t j=0;j<7;++j) measured[j]=q[j];
    }
    assert(q[0]>0);
    // Opening must be able to recover from the held closing gap at bounded speed.
    goal[7]=.044;
    for(int k=0;k<100;++k) {
        const auto previous=q;
        assert(policy_step(goal,measured,lo,hi,.02,q,v));
        assert(q[7]>=previous[7] && q[7]-previous[7]<=.000200001);
        measured=q;
    }
    assert(q[7]>hold);
    measured=q; measured[7]=q[7]-.0041;
    assert(!policy_step(goal,measured,lo,hi,.02,q,v)); // opening failure
    q[7]=.022; measured=q; measured[7]=q[7]+.0061;
    assert(!policy_step(goal,measured,lo,hi,.02,q,v)); // excessive closing discrepancy
    q.fill(0); measured.fill(0); v.fill(0); goal.fill(0);
    q[7]=measured[7]=.026;
    for(int k=0;k<500;++k) {
        const double previous=q[7];
        assert(policy_step(goal,measured,lo,hi,.02,q,v));
        assert(q[7]<=previous && previous-q[7]<=.000200001);
        assert(measured[7]-q[7]<=.003000001);
    }
    assert(measured[7]-q[7]>.0025); // stationary object causes a bounded squeeze
}
