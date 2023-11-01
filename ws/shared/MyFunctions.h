#pragma once

#include <iostream>
#include<Eigen/Dense>
#include "AMPCore.h"

// Structure to represent a 2D point
typedef Eigen::Vector2d Point;


int orientation(const Point& p, const Point& q, const Point& r);
std::vector<Point> convexHull(std::vector<Point>& points);

double cal_distance(Eigen::Vector2d, Eigen::Vector2d);

bool isPointCollision(const Eigen::Vector2d points, const std::vector<amp::Polygon> obstacles);
bool isEdgeCollision(const Eigen::Vector2d p1, const Eigen::Vector2d p2, const std::vector<amp::Polygon> obstacles);
bool isPointInConvexPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);

double crossProduct(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);
bool doLineSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
bool onSegment(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q);

