// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

#include "Cobstacle2d.h"

#include "MyLinkManipulator2D.h"

#include "MyGridCSpace.h"

#include "MyGridCSpace2DConstructor.h"

using namespace amp;

void Exe1(){
    // Init Obstacle and Robot
    amp::Obstacle2D Obs1 = HW4::getEx1TriangleObstacle();
    amp::Obstacle2D Rob1 = HW4::getEx1TriangleObstacle();
    // Cspace Obstacle: using the class
    Cobstacle2d Cobs;
    Cobs.create_cobstacle_rotate_rob(Obs1, Rob1);
    // Visualize
    amp::Visualizer::makeFigure(Cobs.Cobstacle_polygons, Cobs.heights);
    // [debug: plot the rotation robot]
    // amp::Visualizer::makeFigure(Cobs.Wrobot_rotate, Cobs.heights);
    Visualizer::showFigures();
    std::cout<< "Visual Obs1";
}

void Exe2(){
    //Prob(1)
    std::vector<double> link_lengths = {0.5, 1.0, 0.5};
    Eigen::Vector2d base_location = {0.0, 0.0};
    MyLinkManipulator2D mylink1(base_location, link_lengths);
    //mylink1.print();
    ManipulatorState state1 = {3.14/6, 3.14/3, 7*3.14/4};
    Visualizer::makeFigure(mylink1, state1);
    Visualizer::showFigures();
    //Prob(2)
    link_lengths = {1.0, 0.5, 1.0};
    base_location = {0.0, 0.0};
    MyLinkManipulator2D mylink2(base_location, link_lengths);
    //mylink2.print();
    Eigen::Vector2d end_effector_location = {2.0, 0.0};
    ManipulatorState state2 = mylink2.getConfigurationFromIK(end_effector_location);
    Visualizer::makeFigure(mylink2, state2);
    Visualizer::showFigures();
}

void Exe3(){
    // Define CSpace (S x S)
    int Ncells = 360;
    double x0_min = 0; double x0_max = 2*3.141;
    double x1_min = 0; double x1_max = 2*3.141;
    
    // WorkSpace (getEx3Workspace <1,2,3>)
    Environment2D Exe3_World = HW4::getEx3Workspace3();

    std::vector<double> link_lengths_3 = {1.0, 1.0};
    Eigen::Vector2d base_location_3 = {0.0, 0.0};
    MyLinkManipulator2D mylink3(base_location_3, link_lengths_3);
    ManipulatorState state_3 = {0*3.14/180, 30*3.14/180};
    Visualizer::makeFigure(Exe3_World, mylink3, state_3);
    Visualizer::showFigures();
    // CSpaceObstacle
    MyGridCSpace grid(Ncells, Ncells, x0_min, x0_max, x1_min, x1_max,
                      mylink3, Exe3_World);
    // Visualize CSpace 
    Visualizer::makeFigure(grid);
    Visualizer::showFigures();
}


int main(int argc, char** argv) {

    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Exe1();

    //Exe2();

    Exe3();

    // Grade method
    //MyGridCSpace2DConstructor constructor;
    //amp::HW4::grade<MyLinkManipulator2D>(constructor, "chko1829@colorado.edu", argc, argv);
    return 0;
}