/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_FIXEDWING6DOFCONTROLLERROS_FIXEDWING6DOFCONTROLLERROS_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_FIXEDWING6DOFCONTROLLERROS_FIXEDWING6DOFCONTROLLERROS_H_

#include <scrimmage/plugins/motion/FixedWing6DOF/FixedWing6DOF.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>

namespace scrimmage {
namespace controller {
class FixedWing6DOFControllerROS : public Controller {
 public:
    virtual void init(std::map<std::string, std::string>& params);
    virtual bool step(double t, double dt);

 protected:
    Eigen::Vector4d u_;

    void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::Subscriber cmd_vel_sub_;
    geometry_msgs::Twist cmd_vel_;

    int throttle_idx_ = 0;
    int elevator_idx_ = 0;
    int aileron_idx_ = 0;
    int rudder_idx_ = 0;

 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace controller
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_CONTROLLER_FIXEDWING6DOFCONTROLLERROS_FIXEDWING6DOFCONTROLLERROS_H_
