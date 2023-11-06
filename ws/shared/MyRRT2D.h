#pragma once

#include "AMPCore.h"
#include "Eigen/Dense"
#include "MyFunctions.h"


class MyRRT2D: public amp::GoalBiasRRT2D{
    public:
        std::vector<int> Tree;
        amp::Graph<double> graph;
        std::map<amp::Node, Eigen::Vector2d> node_to_coord;
        amp::RNG rng;
        double step_size = 0.5;
        int N_iter = 5000;
        double tol_goal = 0.25;

        bool isSuccess = false;
        double pathLength = 0.0;

        void init(){
            pathLength = 0.0;
            isSuccess = false;
            Tree.clear();
            graph.clear();
            node_to_coord.clear();
        }

        amp::Path2D plan(const amp::Problem2D& problem){

            init();

            double x_min = problem.x_min;
            double x_max = problem.x_max;
            double y_min = problem.y_min;
            double y_max = problem.y_max;

            amp::Path2D path;

            Tree.push_back(0);
            node_to_coord[0] = problem.q_init;

            // Sampling a Q rand
            int index_nodes = 1;
            while(true){
                if(Tree.size() >= N_iter){
                    // Fail
                    //std::cout << "EXCEED MAX ITERATION\n";
                    break;
                }

                // Sample Q_rand
                Eigen::Vector2d q_rand = get_q_sample(problem, x_min, x_max, y_min, y_max);
                //std::cout << "q_rand: " << q_rand[0] << " " << q_rand[1] << "\n";

                // Extend
                int node_near = search_node_near(q_rand);
                Eigen::Vector2d q_near = node_to_coord[Tree[node_near]];
                Eigen::Vector2d q_new = get_q_new(q_near, q_rand);
    
                // Connect
                if(!isEdgeCollision(q_near, q_new, problem.obstacles) && !isPointCollision(q_new, problem.obstacles)){
                    //std::cout << "q_near: " << Tree[node_near] << " : " << q_near[0] << " " << q_near[1] << "\n";
                    //std::cout << "q_new:  " << index_nodes << " : " << q_new[0] << " " << q_new[1] << "\n\n";
                    Tree.push_back(index_nodes);
                    node_to_coord[index_nodes] = q_new;
                    graph.connect(Tree[node_near], index_nodes, step_size);
                    index_nodes = index_nodes + 1;

                    //Success
                    if(cal_distance(q_new, problem.q_goal) < tol_goal){
                        //std::cout << "SUCCESS\n";
                        isSuccess = true;
                        break;
                    }
                }
            }

            // Visualization
            // graph.print();
            // std::cout << "visualization\n";
            // amp::Visualizer::makeFigure(problem, graph, node_to_coord);
            // amp::Visualizer::showFigures();

            if(isSuccess){
                // generate success path
                path.waypoints.insert(path.waypoints.begin(), problem.q_goal);

                int node = Tree.back();
                while(true){
                    Eigen::Vector2d q = node_to_coord[node];
                    //std::cout << "current node: " << node << " " << q[0] << " " << q[1] << "\n";
                    path.waypoints.insert(path.waypoints.begin(), q);
                    pathLength = pathLength + step_size;

                    std::vector<unsigned int> node_parent_vector = graph.parents(node);
                    if(node_parent_vector.size() == 0){
                        break;
                    }

                    int node_parent = node_parent_vector[0];
                    //std::cout << "its parent node: " << node_parent << "\n";
                    node = node_parent;
                }

                return path;
            }
            else{
                return path;
            }
        }

        Eigen::Vector2d get_q_sample(const amp::Problem2D& problem, 
                                     const double x_min, const double x_max,
                                     const double y_min, const double y_max){
            Eigen::Vector2d p;
            int random = rng.randi(1,100);
            if(random <= 5){
                return problem.q_goal;
            }
            else{
                while(true){
                    double x = rng.srandd(x_min, x_max);
                    double y = rng.srandd(y_min, y_max);
                    p[0] = x; p[1] = y;
                    if(!isPointCollision(p, problem.obstacles)){
                        break;
                    }
                }
                return p;
            }
        }

        int search_node_near(Eigen::Vector2d& q_rand){
            //std::cout << "\n";
            double d_ref = 999;
            double n_ref = -1;
            for(int i=0; i<Tree.size(); i++){
                Eigen::Vector2d q = node_to_coord[Tree[i]];
                double d = cal_distance(q, q_rand);
                //std::cout << "node: " << Tree[i] <<  " " << q[0] <<  " " << q[1] << " d: " << d << "\n";
                if(d<d_ref){
                    d_ref = d;
                    n_ref = i;
                }
            }
            //std::cout << "node (near): " << n_ref << "\n";
            return n_ref;
        }

        Eigen::Vector2d get_q_new(Eigen::Vector2d& q_near, Eigen::Vector2d& q_rand){
            double dx = q_rand[0] - q_near[0];
            double dy = q_rand[1] - q_near[1];
            double l = pow(dx*dx + dy*dy, 0.5);
            Eigen::Vector2d step_dir = {dx/l, dy/l};

            Eigen::Vector2d q_new = {step_dir[0]*step_size, step_dir[1]*step_size};
            //Eigen::Vector2d q_new = {dx*step_size, dy*step_size};
            q_new = q_new + q_near;
            return q_new;
        }



};