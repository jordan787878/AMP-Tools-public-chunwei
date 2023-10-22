#pragma once

#include "AMPCore.h"

#include "MyLinkManipulator2D.h"

//namespace amp {
    
// Structure to represent a 2D point
typedef Eigen::Vector2d Point;
// struct Point {
//     double x;
//     double y;
// };

class MyGridCSpace : public amp::GridCSpace2D {
public:
    MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, 
                 double x0_min, double x0_max, double x1_min, double x1_max, const amp::Environment2D &env);

    MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, 
                 double x0_min, double x0_max, double x1_min, double x1_max,
                 const MyLinkManipulator2D, const amp::Environment2D);

    // Implement the virtual method from the base class
    
    // Given a point in continuous space that is between the bounds, determine what cell (i, j) that the point is in
    std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;
    Eigen::Vector2d getPointFromCell(int i, int j);
    bool inCollision(double x0, double x1) const override;

    void setCollisions();

    // members
    MyLinkManipulator2D robot;
    std::vector<amp::Polygon> obstacles;
    bool isRobot = false;

    // Set the robot
    void setRobot(const MyLinkManipulator2D&);

    // Set the obstacles
    void setObstacles(const std::vector<amp::Polygon>);

    double crossProduct(const Point& p1, const Point& p2, const Point& p3) const;
    bool doLineSegmentsIntersect(const Point& p1, const Point& p2, const Point& q1, const Point& q2) const;
    bool onSegment(const Point& p1, const Point& p2, const Point& q) const;
    bool isPointInConvexPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) const;


};

//} // namespace amp