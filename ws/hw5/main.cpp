// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"
#include "hw/HW5.h"

#include "MyGDSalgo.h"

void Problem1(){
    // [work]
    //amp::Problem2D P1 = amp::HW5::getWorkspace1();
    //amp::Problem2D P1 = amp::HW2::getWorkspace1();
    amp::Problem2D P1 = amp::HW2::getWorkspace2();

    MyGDSalgo algo;

    //Visualize Potential
    //amp::Visualizer::makeFigure(algo.potential, 0, 10, 0, 10);
    //amp::Visualizer::showFigures();
    
    amp::Path2D path = algo.plan(P1);
    bool success = amp::HW5::check(path, P1);
    LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));
    amp::Visualizer::makeFigure(P1, path);
    
    amp::Visualizer::showFigures();
}



int main(int argc, char** argv) {

    //Problem1();

    amp::HW5::grade<MyGDSalgo>("Chun-Wei.Kong@colorado.edu", argc, argv);


    return 0;
}