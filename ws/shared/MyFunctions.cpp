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

double cal_distance(Eigen::Vector2d p1, Eigen::Vector2d p2){
    double r = pow(pow(p1[0]-p2[0],2) + pow(p1[1]-p2[1],2),0.5);
    return r;
}

bool isPointCollision(const Eigen::Vector2d point, const std::vector<amp::Polygon> obstacles){
    for(int i = 0; i < obstacles.size(); i++){
        amp::Polygon obs = obstacles[i];

        std::vector<Eigen::Vector2d> obs_points;

        for(int j=0; j<obs.verticesCCW().size(); j++){
            obs_points.push_back(obs.verticesCCW()[j]);
        }

        if(isPointInConvexPolygon(point, obs_points)){
            return true;
        }

    }
    return false;
}

bool isPointInConvexPolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon){
    int n = polygon.size();

    // Check if the polygon has at least 3 vertices
    if (n < 3) {
        return false;
    }

    bool isInside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
        if ((polygon[i].y() > point.y()) != (polygon[j].y() > point.y()) &&
            (point.x() < (polygon[j].x() - polygon[i].x()) * (point.y() - polygon[i].y()) / (polygon[j].y() - polygon[i].y()) + polygon[i].x())) {
            isInside = !isInside;
        }
    }

    return isInside;
}

double crossProduct(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3) {
    return (p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0]);
}

// Function to check if two line segments intersect
bool doLineSegmentsIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q1, const Eigen::Vector2d& q2) {
    // Calculate the orientations of the points
    double orientation1 = crossProduct(p1, p2, q1);
    double orientation2 = crossProduct(p1, p2, q2);
    double orientation3 = crossProduct(q1, q2, p1);
    double orientation4 = crossProduct(q1, q2, p2);
    // Check if the orientations are different and the segments are not collinear
    if (((orientation1 > 0 && orientation2 < 0) || (orientation1 < 0 && orientation2 > 0)) &&
        ((orientation3 > 0 && orientation4 < 0) || (orientation3 < 0 && orientation4 > 0))) {
        return true; // Segments intersect
    }
    // Special cases for collinear segments
    if (orientation1 == 0 && onSegment(p1, p2, q1)) return true;
    if (orientation2 == 0 && onSegment(p1, p2, q2)) return true;
    if (orientation3 == 0 && onSegment(q1, q2, p1)) return true;
    if (orientation4 == 0 && onSegment(q1, q2, p2)) return true;
    return false; // Segments do not intersect
}

// Function to check if a point q lies on a line segment defined by p1 and p2
bool onSegment(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q) {
    return (q[0] >= std::min(p1[0], p2[0]) && q[0] <= std::max(p1[0], p2[0]) &&
            q[1] >= std::min(p1[1], p2[1]) && q[1] <= std::max(p1[1], p2[1]));
}

bool isEdgeCollision(const Eigen::Vector2d p1, const Eigen::Vector2d p2, const std::vector<amp::Polygon> obstacles){
    for(int j=0; j<obstacles.size(); j++){
        amp::Obstacle2D obs_j = obstacles[j];
        for(int k=0; k<obs_j.verticesCCW().size(); k++){
            Point p3;
            Point p4;
            if(k==obs_j.verticesCCW().size()-1){
                p3 = {obs_j.verticesCCW()[k][0], obs_j.verticesCCW()[k][1]};
                p4 = {obs_j.verticesCCW()[0][0], obs_j.verticesCCW()[0][1]};
            }
            else{
                p3 = {obs_j.verticesCCW()[k][0], obs_j.verticesCCW()[k][1]};
                p4 = {obs_j.verticesCCW()[k+1][0], obs_j.verticesCCW()[k+1][1]};
            }
            bool intersection = doLineSegmentsIntersect(p1, p2, p3, p4);
            if(intersection){
                return true;
            }
        }
    }
            return false;
}

void log_vector(Eigen::VectorXd v){
    std::cout << "vector: ";
    for(int i = 0; i < v.size(); i ++){
        std::cout << v[i] << " ";
        }
    // std::cout << "\n";
}

void write_x_traj(std::vector<Eigen::VectorXd>& data, std::string filename){
    // Open a CSV file for writing
    std::ofstream outputFile(filename);

    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the output file." << std::endl;
    }
    else{
        std::cout << "Write data... ";
        // Write data to the CSV file
        for (const Eigen::VectorXd& vec : data) {
            for (int i = 0; i < vec.size(); i++) {
                outputFile << vec(i);
                if (i < vec.size() - 1) {
                    outputFile << ",";
                }
            }
            outputFile << "\n";
        }

        // Close the file
        outputFile.close();
        std::cout << "Save file to: " << filename << "\n";
    }
}