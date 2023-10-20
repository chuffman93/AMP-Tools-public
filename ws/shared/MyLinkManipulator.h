#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include "tools/LinkManipulator.h"
#include <cmath>
using namespace std;
using namespace amp;

class MyLinkManipulator : public LinkManipulator2D {
    
    public:
        MyLinkManipulator()
        : LinkManipulator2D(){};

        MyLinkManipulator(const std::vector<double>& link_lengths)
        : LinkManipulator2D(link_lengths){};

        MyLinkManipulator(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths)
        : LinkManipulator2D(base_location, link_lengths){};

        struct linkerState
        {
            vector<double> lengths;
            ManipulatorState angles;
            int numLink;
            Eigen::Vector2d endPoint;
            vector<Eigen::Matrix4d> jointPts;
        };
        /// @brief Get the location of the nth joint using the current link attributes using Forward Kinematics
        /// @param state Joint angle state (radians). Must have size() == nLinks()
        /// @param joint_index Joint index in order of base to end effector 
        /// (joint_index = 0 should return the base location, joint_index = nLinks() should return the end effector location)
        /// @return Joint coordinate
        virtual Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;

        /// @brief Set the configuration (link attributes) give an end effector location using Inverse Kinematics
        /// @param end_effector_location End effector coordinate
        /// @return Joint angle state (radians) in increasing joint index order. Must have size() ==nLinks()
        virtual ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;

        linkerState FK(vector<double> lengths, ManipulatorState angles) const;
        linkerState IK(vector<double> lengths, Eigen::Vector2d target) const;

    private:
        Eigen::Matrix4d rotateZ(double ang) const;
        Eigen::Matrix4d translate(double dx, double dy, double dz) const;

};