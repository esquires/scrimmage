#include <scrimmage/viewer/ScriptedCamera.h>
#include <scrimmage/common/Utilities.h>

#include <stdexcept>
#include <iostream>

namespace scrimmage {

ScriptedCamera::ScriptedCamera() :
        phase_(0),
        t_start_(std::chrono::high_resolution_clock::now()) {
    skip_phase0_ = true;
}

ScriptedCameraOutput ScriptedCamera::step(
        double sim_time,
        double follow_offset,
        const std::vector<double> &current_camera_pos,
        const std::vector<double> &current_camera_focal_point) {
    // ScriptedCameraOutput out;
    // out.do_view_update = false;
    // return out;

    if (phase_ == 0) {
        return calc_phase0();
    } else if (phase_ == 1) {
        return calc_phase1(
            sim_time, follow_offset,
            current_camera_pos, current_camera_focal_point);
    } else if (phase_ == 2) {
        return calc_phase2(current_camera_pos, current_camera_focal_point);
    } else if (phase_ == 3) {
        return calc_phase3(sim_time);
    } else if (phase_ == 4) {
        return calc_phase4(
            follow_offset, current_camera_pos, current_camera_focal_point);
    } else if (phase_ == 5) {
        return calc_phase5(sim_time);
    } else {
        ScriptedCameraOutput out;
        out.do_view_update = false;
        return out;
    }
}

std::vector<double> ScriptedCamera::interp_vec(
        const std::vector<double> &v1,
        const std::vector<double> &v2,
        double pct) {
    if (v1.size() != 3 || v2.size() != 3) {
        throw std::runtime_error("vector sizes are not correct");
    }

    return {interp(v1[0], v2[0], pct),
            interp(v1[1], v2[1], pct),
            interp(v1[2], v2[2], pct)};
}

ScriptedCameraOutput ScriptedCamera::calc_phase5(double sim_time) {
    double t0 = 61.25;

    ScriptedCameraOutput out;
    if (sim_time < t0) {
        double pct = (sim_time - phase5_start_sim_time_) / (t0 - phase5_start_sim_time_);
        out.follow_offset = interp(phase5_start_follow_offset_, 35.0, pct);
        if (paused_) {
            std::cout << "toggling paused" << std::endl;
            out.toggle_pause = true;
            paused_ = false;
        }
        if (warp_ < -2) {
            std::cout << "inc warp" << std::endl;
            warp_++;
            out.inc_warp = true;
        }
    }
    return out;
}

ScriptedCameraOutput ScriptedCamera::calc_phase3(double sim_time) {
    ScriptedCameraOutput out;
    double t0 = 60.25;

    if (sim_time < t0) {
        return out;
    } else {
        if (!paused_) {
            out.toggle_pause = true;
            paused_ = true;
        }
        t_start_ = std::chrono::high_resolution_clock::now();
        phase_++;
    }
    return out;
}

ScriptedCameraOutput ScriptedCamera::calc_phase4(
        double follow_offset,
        const std::vector<double> &current_camera_pos,
        const std::vector<double> &current_camera_focal_point) {
    double t0 = 1;
    double t1 = 6;
    double t2 = 11;
    double t3 = 16;
    double t4 = 16.5;

    std::vector<double> p1 {-23.0962, 17.0384, 84.6828};
    std::vector<double> p2 {-31.127, -83.7342, 3.58879};

    std::vector<double> f1 {0, 0, 0};


    const double diff = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - t_start_).count();
    ScriptedCameraOutput out;

    if (diff < t0) {
        phase3_start_circle_camera_pos_ = current_camera_pos;
        phase3_start_circle_camera_focal_point_ = current_camera_focal_point;
    } else if (diff < t1) {
        double pct = (diff - t0) / (t1 - t0);
        out.camera_focal_point = interp_vec(
            phase3_start_circle_camera_focal_point_, {0, 0, 0}, pct);
        out.camera_pos = interp_vec(
            phase3_start_circle_camera_pos_, p1, pct);
        out.do_view_update = true;
        out.mode = ViewMode::FREE;
    } else if (diff < t2) {
        double pct = (diff - t1) / (t2 - t1);
        out.camera_focal_point = {0, 0, 0};
        out.camera_pos = interp_vec(p1, p2, pct);
        out.do_view_update = true;
        out.mode = ViewMode::FREE;
    } else if (diff < t3 + 0.1) {
        auto dist = [&](const auto &v) {return std::sqrt(pow(v[0], 2) + pow(v[1], 2));};
        auto yaw = [&](const auto &v){return std::atan2(v[1], v[0]);};
        double beg_dist = dist(p2);
        double end_dist = dist(phase3_start_circle_camera_pos_);

        double beg_yaw = yaw(p2);
        double end_yaw = yaw(phase3_start_circle_camera_pos_);

        double pct = (diff - t2) / (t3 - t2);
        double interp_yaw = interp(beg_yaw, end_yaw, pct);
        double interp_dist = interp(beg_dist, end_dist, pct);

        out.camera_pos = {
            interp_dist * std::cos(interp_yaw),
            interp_dist * std::sin(interp_yaw),
            interp(p2[2], phase3_start_circle_camera_pos_[2], pct)};
        out.camera_focal_point =
            interp_vec(f1, phase3_start_circle_camera_focal_point_, pct);
        out.do_view_update = true;
        out.mode = ViewMode::FREE;
    } else if (diff < t4) {
        out.do_view_update = true;
        out.mode = ViewMode::FOLLOW;
        phase5_start_follow_offset_ = follow_offset;
        phase_++;
    }
    return out;
}

ScriptedCameraOutput ScriptedCamera::calc_phase2(
        const std::vector<double> &current_camera_pos,
        const std::vector<double> &current_camera_focal_point) {
    double t0 = 1;
    double t1 = 6;
    double t2 = 11;
    double t3 = 12;

    const double diff = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - t_start_).count();

    ScriptedCameraOutput out;

    double start_yaw = std::atan2(
        phase2_start_circle_camera_pos_[1],
        phase2_start_circle_camera_pos_[0]);

    auto calc_camera_pos = [&](
            double beg_yaw, double end_yaw, double beg_z, double end_z, double pct) {
        double curr_yaw = interp(beg_yaw, end_yaw, pct);
        auto pos = phase2_start_circle_camera_pos_;

        double xy_dist = std::sqrt(
            pow(pos[0], 2) + pow(pos[1], 2));

        pos[0] = xy_dist * std::cos(curr_yaw);
        pos[1] = xy_dist * std::sin(curr_yaw);
        pos[1] = xy_dist * std::sin(curr_yaw);
        return pos;
    };

    if (diff < t0) {
        phase2_start_circle_camera_pos_ = current_camera_pos;
        phase2_start_circle_camera_focal_point_ = current_camera_focal_point;
    } else if (diff < t1) {
        double pct = (diff - t0) / (t1 - t0);
        out.do_view_update = true;
        out.mode = ViewMode::FREE;

        out.camera_focal_point = interp_vec(
            phase2_start_circle_camera_focal_point_, {0, 0, 0}, pct);
        out.camera_pos = calc_camera_pos(
            start_yaw, start_yaw + M_PI,
            phase2_start_circle_camera_pos_[2], 0, pct);

    } else if (diff < t2) {

        double pct = (diff - t1) / (t2 - t1);
        out.do_view_update = true;
        out.mode = ViewMode::FREE;

        out.camera_focal_point = interp_vec(
            {0, 0, 0}, phase2_start_circle_camera_focal_point_, pct);
        out.camera_pos = calc_camera_pos(
            start_yaw + M_PI, start_yaw + 2 * M_PI,
            0, phase2_start_circle_camera_pos_[2], pct);
    } else if (diff < t3) {
        out.do_view_update = true;
        out.mode = ViewMode::FOLLOW;
        if (warp_ > -5) {
            warp_--;
            out.dec_warp = true;
        }
    } else {
        if (paused_) {
            out.toggle_pause = true;
            paused_ = false;
        }
        phase_++;
    }

    return out;
}

ScriptedCameraOutput ScriptedCamera::calc_phase1(
        double sim_time,
        double follow_offset,
        const std::vector<double> &current_camera_pos,
        const std::vector<double> &current_camera_focal_point) {
    double t0 = 52;
    double t1 = 52.5;
    double t2 = 52.7;
    double t3 = 54.0;
    double t4 = 59.2;
    ScriptedCameraOutput out;

    if (sim_time < t0) {
        out.do_view_update = true;
        out.mode = ViewMode::FOLLOW;
        phase1_start_follow_offset_ = follow_offset;
    } else if (sim_time < t1) {
        if (warp_ > -1) {
            warp_--;
            out.dec_warp = true;
        }
    } else if (sim_time < t2) {
        if (warp_ > -2) {
            warp_--;
            out.dec_warp = true;
        }
    } else if (sim_time < t3 + 1) { // +1 ensures the final pct is achieved
        double pct = (sim_time - t2) / (t3 - t2);
        if (pct > 1) {
            pct = 1;
        }
        out.follow_offset = interp(phase1_start_follow_offset_, 75.0, pct);
        out.veh_scale = interp(10.0, 1.0, pct);
    } else if (t4 < sim_time) {
        if (!paused_) {
            out.toggle_pause = true;
            paused_ = true;
        }
        phase2_start_circle_camera_pos_ = current_camera_pos;
        phase2_start_circle_camera_focal_point_ = current_camera_focal_point;
        phase_ = 2;
        t_start_ = std::chrono::high_resolution_clock::now();
    }
    return out;
}

ScriptedCameraOutput ScriptedCamera::calc_phase0() {
    const double diff = std::chrono::duration<double>(
        std::chrono::high_resolution_clock::now() - t_start_).count();
    double t0 = 2;
    double t1 = 4;
    double t2 = 22;
    double t3 = 24;
    double t4 = 25;

    if (skip_phase0_) {
        t0 = 3.1;
        t1 = 3.2;
        t2 = 3.3;
        t3 = 3.4;
        t4 = 4;
    }

    std::vector<double> p0 {-2272, 0, 720};
    std::vector<double> p1 {-2896.04, 0, 1310.92};
    std::vector<double> p2 {-2217.46, 0, 200};

    std::vector<double> f0 {-1500, 0, 0};
    std::vector<double> f1 {-1200.59, 0, 353.345};
    std::vector<double> f2 {-1500, 0, 0};

    ScriptedCameraOutput out;
    out.do_view_update = true;
    out.veh_scale = 10;
    out.toggle_pause = false;

    if (diff < t0) {
        out.mode = ViewMode::FREE;
        out.camera_pos = p0;
        out.camera_focal_point = f0;
    } else if (diff < t1) {
        out.mode = ViewMode::FREE;
        double pct = (diff - t0) / (t1 - t0);
        out.camera_pos = interp_vec(p0, p1, pct);
        out.camera_focal_point = interp_vec(f0, f1, pct);
    } else if (diff < t2) {
        out.mode = ViewMode::FREE;
        out.camera_pos = p1;
        out.camera_focal_point = f1;
    } else if (diff < t3) {
        out.mode = ViewMode::FREE;
        double pct = (diff - t2) / (t3 - t2);
        out.camera_pos = interp_vec(p1, p2, pct);
        out.camera_focal_point = interp_vec(f1, f2, pct);
    } else if (diff < t4) {
        out.mode = ViewMode::FREE;
        out.camera_pos = p2;
        out.camera_focal_point = f2;
    } else {
        out.mode = ViewMode::FOLLOW;
        out.do_view_update = false;
        out.toggle_pause = true;
        paused_ = false;
        phase_ = 1;
    }

    return out;
}
} // namespace scrimmage
