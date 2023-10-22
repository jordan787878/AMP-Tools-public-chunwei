#include "MyFunctions.h"
#include <vector>

// Function to find the orientation of three points (p, q, r)
// Returns 0 if collinear, 1 if clockwise, and 2 if counterclockwise
int orientation(const Point& p, const Point& q, const Point& r) {
    double val = (q.y() - p.y()) * (r.x() - q.x()) - (q.x() - p.x()) * (r.y() - q.y());
    if (val == 0.0) return 0;  // Collinear
    return (val > 0.0) ? 1 : 2; // Clockwise or Counterclockwise
}

// Gift Wrapping Algorithm for finding Convex Hull
std::vector<Point> convexHull(std::vector<Point>& points) {
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
            if (orientation(points[p], points[i], points[q]) == 2)
                q = i;
        }

        p = q;
    } while (p != leftmost);

    return hull;
}