#pragma once

#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyFunctions.h"
#include <chrono>

class MyDecentMAgentRRT : public amp::DecentralizedMultiAgentRRT{
    public:
        std::vector<int> Tree;
        amp::Graph<double> graph;
        std::map<amp::Node, Eigen::Vector2d> node_to_coord;
        bool isSuccess;
        amp::RNG rng;
        double step_size = 0.5;
        int N_iter = 7500;
        double tol_goal = 0.25;
        double max_exec_time = 10.0;

        amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem){

            std::cout << " === Num Agents: " << problem.numAgents() << " === \n";

            amp::MultiAgentPath2D result;

            result.valid = true;

            // Putting q_inits to the result
            for(auto prop : problem.agent_properties){
                amp::Path2D path_i = plan_for_one_agent(problem, prop, result);
                result.agent_paths.push_back(path_i);
                if(path_i.waypoints.size() < 1){
                    result.valid = false;
                }
            }

            // Debugging
            if(result.valid){
                std::cout << "Success\n";
            }
            else{
                std::cout << "Fail\n";
            }
            return result;
        }

        amp::Path2D plan_for_one_agent(const amp::MultiAgentProblem2D& problem, auto prop, amp::MultiAgentPath2D& result){
            // amp::Path2D path;
            // path.waypoints.push_back(prop.q_init);

            // Construct the subproblem
            amp::Problem2D subproblem;
            subproblem.obstacles = problem.obstacles;
            subproblem.x_min = problem.x_min;
            subproblem.x_max = problem.x_max;
            subproblem.y_min = problem.y_min;
            subproblem.y_max = problem.y_max;
            subproblem.q_init = prop.q_init;
            subproblem.q_goal = prop.q_goal;

            // Clean up
            Tree.clear();
            graph.clear();
            node_to_coord.clear();
            isSuccess = false;

            amp::Path2D path = return_rrt_path(subproblem, result);

            return path;
        }

        amp::Path2D return_rrt_path(const amp::Problem2D& problem, amp::MultiAgentPath2D& result){

            // Timer
            auto start = std::chrono::high_resolution_clock::now();

            // init
            amp::Path2D path;

            // Root node
            Tree.push_back(0);
            node_to_coord[0] = problem.q_init;

            // Sampling a Q rand
            int index_nodes = 1; // this is also the "time"
            while(true){

                // Check the elapsed time
                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end - start;

                if (elapsed.count() > max_exec_time) {
                    std::cout << "... Function exceeded {max_exec_time} second. Breaking the loop." << std::endl;
                    break;
                }

                if(Tree.size() >= N_iter){
                    std::cout << "... exceed {N_iter} nodes\n";
                    break;
                }

                // Sample Q_rand
                Eigen::Vector2d q_rand = get_q_sample(problem, problem.x_min, problem.x_max, problem.y_min, problem.y_max);

                // Extend
                int node_near = search_node_near(q_rand);
                int time_index = get_timing(Tree[node_near]);

                // std::cout << "time index: " << time_index << "\n";
                Eigen::Vector2d q_near = node_to_coord[Tree[node_near]];
                Eigen::Vector2d q_new = get_q_new(q_near, q_rand);

                // Connect
                if(!isEdgeCollision(q_near, q_new, problem.obstacles) && !isCircleRobotCollision(q_new, 0.5, problem.obstacles)){
                    
                    if(!isOverlap(q_new, time_index, result)){

                        Tree.push_back(index_nodes);
                        node_to_coord[index_nodes] = q_new;
                        graph.connect(Tree[node_near], index_nodes, step_size);
                        index_nodes = index_nodes + 1;

                        //Success
                        if(cal_distance(q_new, problem.q_goal) < tol_goal){
                            isSuccess = true;
                            break;
                        }
                    }
                }
            }

            if(isSuccess){
                // generate success path
                path.waypoints.insert(path.waypoints.begin(), problem.q_goal);

                int node = Tree.back();
                while(true){
                    Eigen::Vector2d q = node_to_coord[node];
                    //std::cout << "current node: " << node << " " << q[0] << " " << q[1] << "\n";
                    path.waypoints.insert(path.waypoints.begin(), q);

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

        bool isRobotsEdgeCollision(const amp::Problem2D& problem, 
                                   const Eigen::VectorXd& q_near, 
                                   const Eigen::VectorXd& q_new){
            Eigen::VectorXd q_diff = q_new - q_near;
            double d = q_diff.norm();
            q_diff = q_diff/d;
            int N = (int)(d/step_size);
            for(int i=0; i<N; i++){
                Eigen::VectorXd qi = q_near + q_diff*step_size*i;
                if(isCircleRobotCollision(qi, 0.5, problem.obstacles)){
                    return true;
                }
            }
            return false;
        }

        bool isOverlap(Eigen::Vector2d q_new, const int time_index, amp::MultiAgentPath2D& result){
            std::vector<Eigen::Vector2d> q_others = get_q_other_agents(time_index, result);
            if(q_others.size() == 0){
                return false;
            }
            for(auto q : q_others){
                Eigen::Vector2d q_diff = q - q_new;
                if(q_diff.norm() < 0.5 + 0.5 + 0.2){
                    return true;
                }
            }
            return false;
        }

        std::vector<Eigen::Vector2d> get_q_other_agents(const int time_index, amp::MultiAgentPath2D& result){
            std::vector<Eigen::Vector2d> q_list;

            for(auto path : result.agent_paths){
                // Path is a amp::Path2D
                if(time_index < path.waypoints.size()){
                    Eigen::Vector2d q = {path.waypoints[time_index][0], path.waypoints[time_index][1]};
                    q_list.push_back(q);
                    // std::cout << "q_others: ";
                    // log_vector(q);
                }
                
            }
            // for(auto trajectory : trajectory_list){
            //     if(time_index < trajectory.waypoints.size()){
            //         Eigen::Vector2d q = {trajectory.waypoints[time_index][0], trajectory.waypoints[time_index][1]};
            //         q_list.push_back(q);
            //         std::cout << "q_others: ";
            //         log_vector(q);
            //     }
            // }
            return q_list;
        }

        int get_timing(const int node_near){
            int timing = 0;

            int node = node_near;

            // std::cout << "get timing of node: " << node << " ";
            
            while(true){
                if(node == 0){
                    // std::cout << "timing: " << timing << "\n";
                    return timing;
                }

                std::vector<unsigned int> node_parent_vector = graph.parents(node);

                int node_parent = node_parent_vector[0];
                node = node_parent;
                timing = timing + 1;
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
                    if(!isCircleRobotCollision(p, 0.5, problem.obstacles)){
                    //if(!isPointCollision(p, problem.obstacles)){
                        return p;
                    }
                }
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