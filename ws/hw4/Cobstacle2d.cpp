#include "Cobstacle2d.h"
#include <vector>

void Cobstacle2d::create_cobstacle_rotate_rob(const amp::Obstacle2D &obs, 
                                              const amp::Obstacle2D &rob){
    Point pivot = {0.0,0.0};
    for(int i=0; i<12; i++){
        //rotate the rob w.r.t its pivot point
        double theta_i = (double)i*(360/12)*3.14/180;
        std::vector<Point> rob_rotate;
        for(int i=0; i<rob.verticesCCW().size(); i++){
            rob_rotate.push_back(Cobstacle2d::rotatePoint(rob.verticesCCW()[i], theta_i, pivot));
        }
        amp::Obstacle2D rob_rotate_amp(rob_rotate);

        std::vector<Point> c_obs_i = Cobstacle2d::create_cobstacle(obs,rob_rotate);

        //Store to member variable
        Cobstacle_polygons.push_back(c_obs_i);
        heights.push_back(theta_i);
        Wrobot_rotate.push_back(rob_rotate_amp);
    }
}

// Create a C-space obstacle
std::vector<Point> Cobstacle2d::create_cobstacle(const amp::Obstacle2D &obs, const amp::Obstacle2D &rob){
    //Minkovski Difference
    std::vector<Point> min_diff_set;
    for(int i=0; i<obs.verticesCCW().size(); i++){
        for(int j=0; j<rob.verticesCCW().size(); j++){
            min_diff_set.push_back(obs.verticesCCW()[i]-rob.verticesCCW()[j]);
        }
    }
    //Convex Hull
    std::vector<Point> convexHullPoints = Cobstacle2d::convexHull(min_diff_set);

    return convexHullPoints;
}


// Gift Wrapping Algorithm for finding Convex Hull
std::vector<Point> Cobstacle2d::convexHull(std::vector<Point>& points) {
    int n = points.size();
    if (n < 3) return {}; // Convex hull is not possible with less than 3 points

    // Find the leftmost point
    int leftmost = 0;
    for (int i = 1; i < n; i++) {
        if (points[i].x() < points[leftmost].x())
            leftmost = i;
    }

    std::vector<Point> hull;
    int p = leftmost, q;
    do {
        hull.push_back(points[p]);

        // Find the next point on the convex hull
        q = (p + 1) % n;
        for (int i = 0; i < n; i++) {
            if (Cobstacle2d::orientation(points[p], points[i], points[q]) == 2)
                q = i;
        }

        p = q;
    } while (p != leftmost);

    return hull;
}

// Function to find the orientation of three points (p, q, r)
// Returns 0 if collinear, 1 if clockwise, and 2 if counterclockwise
int Cobstacle2d::orientation(const Point& p, const Point& q, const Point& r) {
    double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
    if (val == 0.0) return 0;  // Collinear
    return (val > 0.0) ? 1 : 2; // Clockwise or Counterclockwise
}

// Function to perform 2D rotation
Eigen::Vector2d Cobstacle2d::rotatePoint(const Eigen::Vector2d& point, double theta, const Eigen::Vector2d& pivot) {
    double cosTheta = cos(theta);
    double sinTheta = sin(theta);

    // Translate the point to the origin, rotate, and then translate it back
    double translatedX = point.x() - pivot.x();
    double translatedY = point.y() - pivot.y();

    double rotatedX = translatedX * cosTheta - translatedY * sinTheta;
    double rotatedY = translatedX * sinTheta + translatedY * cosTheta;

    rotatedX += pivot.x();
    rotatedY += pivot.y();

    return Eigen::Vector2d(rotatedX, rotatedY);
}

