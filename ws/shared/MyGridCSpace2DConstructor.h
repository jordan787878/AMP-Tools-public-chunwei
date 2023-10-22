#include "hw/HW4.h"

//namespace amp{

class MyGridCSpace2DConstructor : public amp::GridCSpace2DConstructor{
    public:
        MyGridCSpace2DConstructor();
        
        std::unique_ptr<amp::GridCSpace2D> construct(
            const amp::LinkManipulator2D& manipulator, 
            const amp::Environment2D& env) override;
};

//}
