#pragma once

#include<Eigen/Dense>

// Structure to represent a 2D point
typedef Eigen::Vector2d Point;

int orientation(const Point& p, const Point& q, const Point& r);
std::vector<Point> convexHull(std::vector<Point>& points);


