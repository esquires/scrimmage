
#ifndef SUBMODULES_SCRIMMAGE_INCLUDE_SCRIMMAGE_VIEWER_SCRIPTEDCAMERA_H_
#define SUBMODULES_SCRIMMAGE_INCLUDE_SCRIMMAGE_VIEWER_SCRIPTEDCAMERA_H_

#include <vector>
#include <memory>
#include <limits>
#include <chrono>

namespace scrimmage {

enum class ViewMode {FOLLOW = 0, FREE, OFFSET, FPV};

struct ScriptedCameraOutput {
    bool do_view_update = false;
    ViewMode mode = ViewMode::FOLLOW;
    double veh_scale = std::numeric_limits<double>::quiet_NaN();
    std::vector<double> camera_pos {0, 0, 0};
    std::vector<double> camera_focal_point {0, 0, 0};
    bool toggle_pause = false;
    bool inc_warp = false;
    bool dec_warp = false;
    double follow_offset = std::numeric_limits<double>::quiet_NaN();
};

class ScriptedCamera {
 public:
    ScriptedCamera();
    ScriptedCameraOutput step(
        double sim_time,
        double follow_offset,
        const std::vector<double> &current_camera_pos,
        const std::vector<double> &current_camera_focal_point);

 protected:
    int phase_ = 0;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_start_;

    std::vector<double> interp_vec(
        const std::vector<double> &v1,
        const std::vector<double> &v2,
        double pct);

    ScriptedCameraOutput calc_phase0();
    ScriptedCameraOutput calc_phase1(
        double sim_time,
        double follow_offset,
        const std::vector<double> &current_camera_pos,
        const std::vector<double> &current_camera_focal_point);
    ScriptedCameraOutput calc_phase2(
            const std::vector<double> &current_camera_pos,
            const std::vector<double> &current_camera_focal_point);
    ScriptedCameraOutput calc_phase3(double sim_time);
    ScriptedCameraOutput calc_phase4(
            double follow_offset,
            const std::vector<double> &current_camera_pos,
            const std::vector<double> &current_camera_focal_point);
    ScriptedCameraOutput calc_phase5(double sim_time);

    double dist(const std::vector<double> &v);

    int warp_ = 0;
    double phase1_start_follow_offset_ = std::numeric_limits<double>::quiet_NaN();
    bool skip_phase0_ = false;

    std::vector<double> phase2_start_circle_camera_pos_;
    std::vector<double> phase2_start_circle_camera_focal_point_;

    std::vector<double> phase3_start_circle_camera_pos_;
    std::vector<double> phase3_start_circle_camera_focal_point_;
    double phase5_start_follow_offset_ = std::numeric_limits<double>::quiet_NaN();
    double phase5_start_sim_time_ = std::numeric_limits<double>::quiet_NaN();
    bool paused_ = true;
};

} // namespace scrimmage

#endif // SUBMODULES_SCRIMMAGE_INCLUDE_SCRIMMAGE_VIEWER_SCRIPTEDCAMERA_H_
