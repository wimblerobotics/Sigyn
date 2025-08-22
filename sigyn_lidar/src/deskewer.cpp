#include "sigyn_lidar/deskew.hpp"
#include <algorithm>

namespace sigyn_lidar {
    void YawInterpolator::addSample(double t, double yaw, double yaw_rate) {
        samples_.push_back({ t,yaw,yaw_rate });
        // keep last ~200 samples
        if (samples_.size() > 200) samples_.erase(samples_.begin(), samples_.end() - 200);
    }

    bool YawInterpolator::interpolate(double t, double& yaw_out) const {
        if (samples_.size() < 2) return false;
        auto it = std::lower_bound(samples_.begin(), samples_.end(), t, [](const Sample& s, double val) {return s.t < val;});
        if (it == samples_.begin()) { yaw_out = it->yaw; return true; }
        if (it == samples_.end()) { yaw_out = samples_.back().yaw; return true; }
        const Sample& s1 = *(it - 1); const Sample& s2 = *it;
        double u = (t - s1.t) / (s2.t - s1.t);
        yaw_out = s1.yaw + u * (s2.yaw - s1.yaw);
        return true;
    }
}
