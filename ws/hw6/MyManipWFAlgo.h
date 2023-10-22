#pragma once

#include "AMPCore.h"

#include "MyGridCSpace2DConstructor.h"

#include <cmath>

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        MyLinkManipulator2D robot;
        amp::Path2D cpath;

        // Default ctor
        MyManipWFAlgo(MyLinkManipulator2D& link)
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>()) {robot = link;}

        // You can have custom ctor params for all of these classes
        // MyManipWFAlgo(const std::string& beep) 
        //     : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) 
        //     {LOG("construcing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::ManipulatorTrajectory2Link plan(const amp::LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override 
        {
            return amp::ManipulatorTrajectory2Link();
        }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override 
        {
            std::cout << "planInCSpace\n";

            // init and goal in CSpace
            //Eigen::Vector2d c_init = grid_cspace.robot. mylink.getConfigurationFromIK(end_effector_i);

            // std::make_shared<MyGridCSpace> *ptr = grid_cspace;
            Eigen::Vector2d c_init = robot.getConfigurationFromIK(q_init);
            Eigen::Vector2d c_goal = robot.getConfigurationFromIK(q_goal);


            amp::Path2D path;
            
            // add the init wspace
            path.waypoints.push_back(q_init);
            // cpath.waypoints.push_back(Eigen::Vector2d{3.141592,0.0});
            cpath.waypoints.push_back(c_init);

            // init the wavefront 2D array
            std::pair<std::size_t, std::size_t> dimensions = grid_cspace.size();
            const int rows = dimensions.first;
            const int cols = dimensions.second;
            // Dynamically allocate a 2D array
            int** waveFrontArray = new int*[rows];
            for (int i = 0; i < rows; i++) {
                waveFrontArray[i] = new int[cols];
            }
            // set all elements to zero
            for (int i=0; i < rows; i++){
                for (int j=0; j < cols; j++){
                    waveFrontArray[i][j] = 0;
                }
            }

            // [debug cout for Cspace]
            // std::cout << "(debug) print CSpace\n";
            // for (int j=0; j < cols; j++){
            //     for (int i=0; i < rows; i++){
            //         int index_i = i;//rows-i-1;
            //         int index_j = rows - j -1;
            //         std::cout << grid_cspace(index_i, index_j) << " ";
            //     }
            //     std::cout<< "\n";
            // }
            // std:: cout << "\n";

            // set obstacle to 1
            for(int i=0; i<rows; i++){
                for(int j=0; j<cols; j++){
                    if(grid_cspace(i, j)){
                        waveFrontArray[i][j] = 1;
                    }
                }
            }

            // get (i,j) of q_start
            std::pair<std::size_t, std::size_t> ij_init = grid_cspace.getCellFromPoint(c_init[0], c_init[1]);
            std::cout << ij_init.first << " " << ij_init.second << "\n";
            std::pair<std::size_t, std::size_t> ij_goal = grid_cspace.getCellFromPoint(c_goal[0], c_goal[1]);
            std::cout << ij_goal.first << " " << ij_goal.second << "\n";

            // set q_goal to 2
            waveFrontArray[ij_goal.first][ij_goal.second] = 2;

            // update until ij_init is not zero
            int check_value = 2;
            while(true){
                //std::cout << "check value: " << check_value << "\n";
                if(waveFrontArray[ij_init.first][ij_init.second] > 0){
                    break;
                }
                for(int i=0; i<rows; i++){
                    for(int j =0; j<cols; j++){
                        if(waveFrontArray[i][j] == check_value){
                            // update its 0 neighbor
                            int ni;
                            int nj;

                            // n1: left
                            ni = i-1; nj = j;
                            assignValue(ni, nj, rows, cols, check_value, waveFrontArray);

                            // n2: right
                            ni = i+1; nj = j;
                            assignValue(ni, nj, rows, cols, check_value, waveFrontArray);

                            // n3: down
                            ni = i; nj = j-1;
                            assignValue(ni, nj, rows, cols, check_value, waveFrontArray);

                            // n4: top
                            ni = i; nj = j+1;
                            assignValue(ni, nj, rows, cols, check_value, waveFrontArray);
                        }
                    }
                }
                // [debug cout for waveFrontArray]
                // std::cout << "(debug) print waveFrontArray\n";
                // for (int j=0; j < cols; j++){
                //     for (int i=0; i < rows; i++){
                //         int index_i = i;//rows-i-1;
                //         int index_j = rows - j -1;
                //         std::cout << waveFrontArray[index_i][index_j] << " ";
                //     }
                //     std::cout<< "\n";
                // }

                // int xxx;
                // std::cin >> xxx;
                // iterate to next value
                check_value = check_value + 1;


            }

            // [debug cout for waveFrontArray]
            // std::cout << "(debug) print waveFrontArray\n";
            // for (int j=0; j < cols; j++){
            //     for (int i=0; i < rows; i++){
            //         int index_i = i;//rows-i-1;
            //         int index_j = rows - j -1;
            //         std::cout << std::setw(2) << std::setfill('0') << waveFrontArray[index_i][index_j] << " ";
            //     }
            //     std::cout<< "\n";
            // }

            // generate path from the waveFrontArray
            int waveFrontValue = waveFrontArray[ij_init.first][ij_init.second];
            int index_x = ij_init.first;
            int index_y = ij_init.second;
            while(true){
                if(waveFrontValue <= 2){
                    if(waveFrontValue == 2){
                        std::cout << "reach goal\n";
                        path.waypoints.push_back(q_goal);
                        cpath.waypoints.push_back(c_goal);
                        //cpath.waypoints.push_back(Eigen::Vector2d{0.0,0.0});
                    }
                    else{
                        std::cout << "cannot reach goal\n";
                    }
                    break;
                }

                // init bool flag
                bool findnextstep = false;

                // int xxx;
                // std::cin >> xxx;
                // std::cout << "path index: " << index_x << " " << index_y << " " << waveFrontValue << "\n";
                // std::cout << "WaveVale: " << waveFrontValue << " flag: " << findnextstep << "\n";

                // check all 4 neighbors of index_x, index_y
                int ni; int nj;
                // n1: left
                ni = index_x - 1; nj = index_y;
                updatepath(ni, nj, rows, cols,
                           grid_cspace, waveFrontArray, 
                           index_x, index_y, waveFrontValue, findnextstep,
                           path);
                // n2: right
                ni = index_x + 1; nj = index_y;
                updatepath(ni, nj, rows, cols,
                           grid_cspace, waveFrontArray, 
                           index_x, index_y, waveFrontValue, findnextstep,
                           path);
                // n3: down
                ni = index_x; nj = index_y-1;
                updatepath(ni, nj, rows, cols,
                           grid_cspace, waveFrontArray, 
                           index_x, index_y, waveFrontValue, findnextstep,
                           path);
                // n4: top
                ni = index_x; nj = index_y+1;
                updatepath(ni, nj, rows, cols,
                           grid_cspace, waveFrontArray, 
                           index_x, index_y, waveFrontValue, findnextstep,
                           path);
            }

            // Visualization before returning the path
            amp::Visualizer::makeFigure(grid_cspace, cpath);
            amp::Visualizer::showFigures();

            std::cout << "planInCSpace done\n";
            // (debugiing)
            // path.waypoints.push_back(Eigen::Vector2d{0.0,0.0});
            return cpath;
        }

        Eigen::Vector2d getPointFromCell(const amp::GridCSpace2D& grid_cspace, const int i, const int j){
            std::pair<std::size_t, std::size_t> dimensions = grid_cspace.size();
            std::pair<double, double> x0s = grid_cspace.x0Bounds();
            std::pair<double, double> x1s = grid_cspace.x1Bounds();
            double dx0 = (x0s.second-x0s.first)/(dimensions.first-1);
            double dx1 = (x1s.second-x1s.first)/(dimensions.second-1);
            double x0 = (double)i*dx0+x0s.first;
            double x1 = (double)j*dx1+x1s.first;
            Eigen::Vector2d result ={x0,x1};
            return result;    
        }

        void assignValue(int ni, int nj, const int rows, const int cols, int check_value, int** waveFrontArray){
            //std::cout << "n top: " << ni << " " << nj << "\n";
            if(ni < 0){ni = ni+rows;} if(ni >= rows){ni = 0;}
            if(nj < 0){nj = nj+cols;} if(nj >= cols){nj = 0;}
            if(waveFrontArray[ni][nj] == 0){
                waveFrontArray[ni][nj] = check_value + 1;
                //std::cout << "set: " << ni << " " << nj << "\n";
            }
        }

        void updatepath(int ni, int nj, const int rows, const int cols,
                        const amp::GridCSpace2D& grid_cspace, int** waveFrontArray, 
                        int &index_x, int &index_y, int &waveFrontValue, bool &findnextstep,
                        amp::Path2D &path){
            if(ni < 0){ni = ni+rows;} if(ni >= rows){ni = 0;}
            if(nj < 0){nj = nj+cols;} if(nj >= cols){nj = 0;}
            //std::cout << "current index: " << index_x << " " << index_y << "\n";
            //std::cout << "n       index: " << ni << " " << nj << "\n";
            if(waveFrontArray[ni][nj] == waveFrontValue-1 && findnextstep == false){
                // update
                index_x = ni; index_y = nj; waveFrontValue = waveFrontValue - 1;
                findnextstep = true;
                // push state
                Eigen::Vector2d q = getPointFromCell(grid_cspace, index_x, index_y);
                //std::cout << "add to path: " << q[0] << " " << q[1] << "\n";
                cpath.waypoints.push_back(q);
                Eigen::Vector2d end_effector_loc = robot.getJointLocation(q, 2);
                path.waypoints.push_back(end_effector_loc);
            }
        }
};