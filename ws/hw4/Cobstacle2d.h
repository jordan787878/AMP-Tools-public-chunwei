#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "AMPCore.h"

// Structure to represent a 2D point
typedef Eigen::Vector2d Point;

class Cobstacle2d {
    public:
        std::vector<amp::Polygon> Cobstacle_polygons;
        std::vector<double> heights;
        // [debug]
        std::vector<amp::Polygon> Wrobot_rotate;
        
        std::vector<Point> convexHull(std::vector<Point>& points);
        void create_cobstacle_rotate_rob(const amp::Obstacle2D &obs, 
                                         const amp::Obstacle2D &rob);
        std::vector<Point> create_cobstacle(const amp::Obstacle2D &obs, 
                                            const amp::Obstacle2D &rob);

    private:
        int orientation(const Point& p, const Point& q, const Point& r);
        Eigen::Vector2d rotatePoint(const Eigen::Vector2d& point, double theta, const Eigen::Vector2d& pivot);
};

