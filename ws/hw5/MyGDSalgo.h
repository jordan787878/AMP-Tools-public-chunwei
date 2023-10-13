#pragma once

#include "AMPCore.h"

#include "hw/HW5.h"

#include "Eigen/Core"

class MyGDSalgo : public amp::GDAlgorithm{
    public:
        amp::Path2D plan(const amp::Problem2D& problem) override;
        double xi; // scaling factor of the att. potential
        double eta; // scaling factor of the rep. potentail
        double d_star_goal; // distance criteria for the att. potential
        double Q_star; // distance of influence of obstacle
        std::vector<double> Q_star_list;
        double stepsize; // gradient descent stepsize
        Eigen::Vector2d pos;
        int steps;
    
    private:
        Eigen::Vector2d cal_gradient(const Eigen::Vector2d, const Eigen::Vector2d, const amp::Problem2D&);
        double potential_att(const Eigen::Vector2d, const Eigen::Vector2d);
        double potential_rep(const Eigen::Vector2d, const amp::Problem2D&);
        double cal_distance(const Eigen::Vector2d, const Eigen::Vector2d);
        double pointToLineDistance(Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d);

};