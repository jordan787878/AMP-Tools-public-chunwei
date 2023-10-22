#pragma once

#include "AMPCore.h"
#include <cmath>

//namespace amp {

class MyLinkManipulator2D : public amp::LinkManipulator2D {
public:
    MyLinkManipulator2D();
    MyLinkManipulator2D(const amp::LinkManipulator2D& baseobj);
    MyLinkManipulator2D(const std::vector<double>& link_lengths);
    MyLinkManipulator2D(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths);

    Eigen::Vector2d getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const override;
    amp::ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

private:
    //std::vector<Eigen::Matrix3d> matrixVector;
    amp::ManipulatorState2Link inverseKinematics(const Eigen::Vector2d& endEffectorPos, 
                                            double l1, double l2, double l3,
                                            const Eigen::Vector2d& baseLocation) const;

};

//} // namespace amp
