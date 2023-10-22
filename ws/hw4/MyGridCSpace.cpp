#include "MyGridCSpace.h"

namespace amp{


MyGridCSpace::MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, 
                           double x0_min, double x0_max, double x1_min, double x1_max,
                           const MyLinkManipulator2D r, const Environment2D env)
: GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max){
    MyGridCSpace::setRobot(r);
    MyGridCSpace::setObstacles(env.obstacles);
    MyGridCSpace::setCollisions();
}

void MyGridCSpace::setCollisions(){
   std::pair<std::size_t, std::size_t> dimensions = size();
   std::pair<double, double> x0s = x0Bounds();
   std::pair<double, double> x1s = x0Bounds();
   double dx0 = (x0s.second-x0s.first)/(dimensions.first-1);
   double dx1 = (x1s.second-x1s.first)/(dimensions.second-1);
   //std::cout << "dx0 " << dx0 << "\n";
   for(int i=0; i<dimensions.first; i++){
       for(int j=0; j<dimensions.second; j++){
            double x0 = (double)i*dx0+x0s.first;
            double x1 = (double)j*dx1+x1s.first;
            //std::cout << "x0,x1: " << x0 << " " << x1 << "\n";
            (*this)(i,j) = inCollision(x0,x1);
       }
   }
   std::cout << "finish setCollisions to DenseArray\n";
}

bool MyGridCSpace::inCollision(double x0, double x1) const {

    // Get the Joint Location for given a configuration
    ManipulatorState state = {x0,x1};
    
    // loop over joints:
    std::vector<Eigen::Vector2d> jointlocations;
    for(int i =0; i<robot.nLinks()+1; i++){
       Eigen::Vector2d loc = robot.getJointLocation(state, i);
       jointlocations.push_back(loc);
       //std::cout << "debug: " << i << " " << loc[0] << " " << loc[1] << "\n";
    }

    // loop over locations to get the line segment
    // inside the line segment, do line segment and polygon intersection check
    // if collide return true
    // else return false
    
    for(int i=0; i<robot.nLinks();i++){
        Point p1 = {jointlocations[i][0], jointlocations[i][1]};
        Point p2 = {jointlocations[i+1][0], jointlocations[i+1][1]};
        //std::cout << "i: " << i << "\n";
        //std::cout << "segment: \n";
        //std::cout << segment.start.x << " " << segment.start.y << "\n";
        //std::cout << segment.end.x << " " << segment.end.y << "\n";

        for(int j=0; j<obstacles.size(); j++){
            Obstacle2D obs_j = obstacles[j];
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
                //[debug]
                //if(i==1){
                //    std::cout << "p1: " << p1.x << " " << p1.y << ", ";
                //    std::cout << "p2: " << p2.x << " " << p2.y << "\n";
                //    std::cout << "p3: " << p3.x << " " << p3.y << ", ";
                //    std::cout << "p4: " << p4.x << " " << p4.y << "\n";
                //}
                bool intersection = MyGridCSpace::doLineSegmentsIntersect(p1, p2, p3, p4);
                if(intersection){
                    // [debug]
                    //if(x0==0){
                    //   std::cout << "Collision: " << x0*180/3.14 << " " << x1*180/3.14 << "\n";
                    //}
                    return true;
                }
                
            }

        }
    }
    return false;
}

void MyGridCSpace::setRobot(const MyLinkManipulator2D& r){
    robot = r;
    std::cout << "Construct Robot: ";
    robot.print();
}

void MyGridCSpace::setObstacles(const std::vector<Polygon> os){
    obstacles = os;
    std::cout << "Construct " << obstacles.size() << " obstacles\n";
}


// Helper function to calculate the cross product of two vectors (p1p2 and p1p3)
double MyGridCSpace::crossProduct(const Point& p1, const Point& p2, const Point& p3) const {
    return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
}

// Function to check if two line segments intersect
bool MyGridCSpace::doLineSegmentsIntersect(const Point& p1, const Point& p2, const Point& q1, const Point& q2) const {
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
bool MyGridCSpace::onSegment(const Point& p1, const Point& p2, const Point& q) const {
    return (q.x >= std::min(p1.x, p2.x) && q.x <= std::max(p1.x, p2.x) &&
            q.y >= std::min(p1.y, p2.y) && q.y <= std::max(p1.y, p2.y));
}


}


