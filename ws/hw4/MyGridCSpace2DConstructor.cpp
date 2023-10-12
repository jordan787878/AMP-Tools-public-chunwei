#include "MyGridCSpace2DConstructor.h"
#include "MyLinkManipulator2D.h"
#include "MyGridCSpace.h"
#include "AMPCore.h"
//#include "MyGridCSpace.h"

namespace amp{

MyGridCSpace2DConstructor::MyGridCSpace2DConstructor(): GridCSpace2DConstructor(){}

std::unique_ptr<amp::GridCSpace2D> MyGridCSpace2DConstructor::construct(
    const amp::LinkManipulator2D& manipulator, 
    const amp::Environment2D& env){

    int Ncells = 360;
    double x0_min = 0; double x0_max = 2*3.141;
    double x1_min = 0; double x1_max = 2*3.141;
    MyLinkManipulator2D mylink(manipulator);
    
    std::unique_ptr<GridCSpace2D> ptr = std::make_unique<MyGridCSpace>(
    Ncells, Ncells, x0_min, x0_max, x1_min, x1_max, manipulator, env);
    return ptr;
}


}

