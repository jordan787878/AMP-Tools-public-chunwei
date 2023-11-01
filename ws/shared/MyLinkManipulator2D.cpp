#include "MyLinkManipulator2D.h"

#include "MyFunctions.h"

#include <cmath>

//namespace amp {

MyLinkManipulator2D::MyLinkManipulator2D() : amp::LinkManipulator2D() {}

MyLinkManipulator2D::MyLinkManipulator2D(const amp::LinkManipulator2D& baseobj): amp::LinkManipulator2D(baseobj){}

MyLinkManipulator2D::MyLinkManipulator2D(const std::vector<double>& link_lengths)
    : amp::LinkManipulator2D(link_lengths) {}

MyLinkManipulator2D::MyLinkManipulator2D(const Eigen::Vector2d& base_location, const std::vector<double>& link_lengths)
    : amp::LinkManipulator2D(base_location, link_lengths) {}

Eigen::Vector2d MyLinkManipulator2D::getJointLocation(const amp::ManipulatorState& state, uint32_t joint_index) const {
    // Implement your forward kinematics here
    // This method should return the joint location for the given joint_index
    //Eigen::Vector2d jointLocation(0.0, 0.0); // Replace with actual calculation
    //return jointLocation;
    
    //std::cout << "joint_index " << joint_index << ": ";
    if(joint_index == 0){
        //std::cout << "\n";
        return getBaseLocation();
    }
    else{
        //std::cout << "\n";
        //init vector
        Eigen::Vector3d v(getLinkLengths()[joint_index-1], 0.0, 1.0);
        //std::cout << v << "\n";
        
        //construct the transformation matrix
        for(int i=0;i<joint_index; i++){
            int j = joint_index-i; // T2
            double a = 0;
            if(j==1){
                a = 0;
            }
            else{
                a = getLinkLengths()[j-2]; // a1
            }
            double angle = state[j-1];
            //std::cout << "a: " << a << "angle: " << angle << "  ";

            Eigen::Matrix3d T;
            T << std::cos(angle), -std::sin(angle), a,
                 std::sin(angle),  std::cos(angle), 0.0,
                 0.0,              0.0,             1.0;
            v = T*v;
        }
        //std::cout << "\nv: " << v[0] << " " << v[1];
        //std::cout << "\n\n";
        
        return Eigen::Vector2d(v[0]+getBaseLocation()[0], v[1]+getBaseLocation()[1]);
    }


}

amp::ManipulatorState MyLinkManipulator2D::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    amp::ManipulatorState jointAngles;
    double l1 = getLinkLengths()[0];
    double l2 = getLinkLengths()[1];
    double l3 = getLinkLengths()[2];

    jointAngles = MyLinkManipulator2D::inverseKinematics(end_effector_location, 
                                         l1, l2, l3,
                                         getBaseLocation());
    //std::cout << "angles: " << jointAngles[0] << " " << jointAngles[1] << " " << jointAngles[2] << "\n";
    return jointAngles;
}

amp::ManipulatorState2Link MyLinkManipulator2D::inverseKinematics(const Eigen::Vector2d& endEffectorPos, 
                                            double l1, double l2, double l3,
                                            const Eigen::Vector2d& baseLocation) const{
    
    // amp::ManipulatorState jointAngles;
    // //Init JointAngles
    // for(int i=0; i<nLinks(); i++){
    //     jointAngles.push_back(0.0);
    // }

    //amp::ManipulatorState2Link jointAngles;
    
    // Calculate the end effector position relative to the base
    Eigen::Vector2d relativePos = endEffectorPos - baseLocation;

    // Calculate the distance from the base to the end effector
    double d = relativePos.norm();

    amp::ManipulatorState2Link jointAngles = Eigen::Vector2d({0.0, 0.0});
    if(nLinks() == 2){
        std::cout << "2 Links IK\n";
        // Check if the end effector is within reach
        if (d > l1 + l2|| d < fabs(l1 - l2)) {
            std::cerr << "The end effector is out of reach." << std::endl;
            jointAngles = Eigen::Vector2d({-999.0, -999.0});
            return jointAngles;
        }

        // double x0 = relativePos[0]; // Replace with your values
        // double y0 = relativePos[1]; // Replace with your values

        // jointAngles = solveEquations(l1, l2, x0, y0);

        double x = relativePos[0];
        double y = relativePos[1];
        double z = (x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2);
        std::cout << " (debug) z: " << z << "\n";

        if(z > 1 || z < -1){
            return jointAngles;
        }
        double t2 = acos(z);
        std::cout << " (debug) t2: " << t2 << "\n";

        double tol = 0.000001;
        double t1 = 0.0;
        int refine_factor = 10000;
        for(int i =0; i<360*refine_factor; i++){
            t1 = (double)i* 1.0 *M_PI/180.0/(double)refine_factor;
            double rx = l1*cos(t1) + l2*cos(t1+t2) - relativePos[0];
            double ry = l1*sin(t1) + l2*sin(t1+t2) - relativePos[1];
            if( abs(rx) < tol && abs(ry) < tol){
                jointAngles[0] = t1;
                jointAngles[1] = t2;
                std::cout << "t1: " << t1 << " , t2: " << t2 << "\n";
            }
        }
        
        // // discretize the configuraiton space
        // double t1 = 0.0;
        // double t2 = 0.0;
        // double tol = 0.00001;
        // int refine_factor = 50;
        // for(int i =0; i<360*refine_factor; i++){
        //     t1 = (double)i* 1.0 *M_PI/180.0/(double)refine_factor;
        //     for(int j=0; j< 360*refine_factor; j++){
        //         t2= (double)j* 1.0 *M_PI/180.0/(double)refine_factor;
        //         double rx = l1*cos(t1)+l2*cos(t2)-relativePos[0];
        //         double ry = l1*sin(t1)+l2*sin(t2)-relativePos[1];
        //         if( abs(rx) < tol && abs(ry) < tol){
        //             std::cout << t1 << " " << t2-t1<< "\n";
        //             jointAngles[0] = t1;
        //             jointAngles[1] = t2-t1;
        //             return jointAngles;
        //         }
        //     }
        // }
        return jointAngles;
    }

    // Check if the end effector is within reach
    if (d > l1 + l2 + l3 || d < fabs(l1 - l2 - l3)) {
        std::cerr << "The end effector is out of reach." << std::endl;
        return jointAngles;
    }

    // discretize the configuraiton space
    double t1 = 0;
    double t2 = 0;
    double t3 = 0;
    double tol = 0.001;
    for(int i =0; i<360; i++){
        t1 = (double)i*3.14/180;
        for(int j=0; j<360; j++){
            t2= (double)j*3.14/180;
            for(int k=0; k<360; k++){
                t3 = (double)k*3.14/180;

                double rx = l1*cos(t1)+l2*cos(t2)+l3*cos(t3)-relativePos[0];
                double ry = l1*sin(t1)+l2*sin(t2)+l3*sin(t3)-relativePos[1];
                if( abs(rx) < tol && abs(ry) < tol){
                    //std::cout << rx << " " << ry << "   ";
                    //std::cout << t1 << " " << t2 << " " << t3 << "\n";
                    jointAngles[0] = t1;
                    jointAngles[1] = t2-t1;
                    jointAngles[1] = t3-t2;
                    return jointAngles;
                }

            }
        }
    }
    return jointAngles;
}


//} // namespace amp
