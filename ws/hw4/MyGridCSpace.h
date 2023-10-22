#pragma once

#include "AMPCore.h"
#include "MyLinkManipulator2D.h"

namespace amp {

struct Point {
    double x;
    double y;
};

class MyGridCSpace : public GridCSpace2D {
public:
    MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, 
                 double x0_min, double x0_max, double x1_min, double x1_max,
                 const MyLinkManipulator2D, const Environment2D);

    // Implement the virtual method from the base class
    bool inCollision(double x0, double x1) const override;

    void setCollisions();

    // members
    MyLinkManipulator2D robot;
    std::vector<Polygon> obstacles;

    // Set the robot
    void setRobot(const MyLinkManipulator2D&);

    // Set the obstacles
    void setObstacles(const std::vector<Polygon>);

    double crossProduct(const Point& p1, const Point& p2, const Point& p3) const;
    bool doLineSegmentsIntersect(const Point& p1, const Point& p2, const Point& q1, const Point& q2) const;
    bool onSegment(const Point& p1, const Point& p2, const Point& q) const;


};

} // namespace amp