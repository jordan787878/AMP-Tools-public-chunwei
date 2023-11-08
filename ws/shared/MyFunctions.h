#pragma once

#include <iostream>
#include <fstream>
#include<Eigen/Dense>
#include "AMPCore.h"

// Structure to represent a 2D point
typedef Eigen::Vector2d Point;


int orientation(const Point& p, const Point& q, const Point& r);
std::vector<Point> convexHull(std::vector<Point>& points);

double cal_distance(Eigen::Vector2d, Eigen::Vector2d);

bool isCollisionWithConvexPolygon(const Eigen::Vector2d& p, double radius, const std::vector<Eigen::Vector2d>& obstacle);
bool isCircleRobotCollision(const Eigen::Vector2d point, double radius, const std::vector<amp::Polygon> obstacles);
bool isPointCollision(const Eigen::Vector2d points, const std::vector<amp::Polygon> obstacles);
bool isEdgeCollision(const Eigen::Vector2d p1, const Eigen::Vector2d p2, const std::vector<amp::Polygon> obstacles);
bool isPointInConvexPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);

double crossProduct(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3);
bool doLineSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2);
bool onSegment(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q);

void log_vector(Eigen::VectorXd v);
void log_vector(std::vector<double> v);
void log_node_path(std::vector<int> node_path);
void write_x_traj(std::vector<Eigen::VectorXd>& data, std::string filename);
Eigen::VectorXd vector_from_list(std::vector<double> list);

void writeListVectorToCSV(const std::list<std::vector<double>>& data, const std::string& filename);

