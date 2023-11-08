#pragma once
#include <iostream>
#include "AMPCore.h"
#include "Eigen/Dense"
#include "MyFunctions.h"

class MyCentMAgentRRT : public amp::CentralizedMultiAgentRRT{
    public:
        amp::RNG rng;
        std::vector<int> Tree;
        amp::Graph<double> graph;
        std::map<amp::Node, Eigen::VectorXd> node_to_coord;
        std::vector<double> radius_list;

        int dimension;
        int dimension_meta;
        int N_iter;
        double step_size;
        double epsilon;
        bool isSuccess;
        double treesize;

        int whileloopcount;
        int max_whileloops;

        void init(const amp::MultiAgentProblem2D& problem){
            dimension = 2;
            dimension_meta = dimension*problem.numAgents();
            N_iter = 7500;
            step_size = 0.5;
            epsilon = 0.25;
            isSuccess = false;
            treesize = 0;
            whileloopcount = 0;
            max_whileloops = 10;

            Tree.clear();
            graph.clear();
            node_to_coord.clear();
            radius_list.clear();
            for(auto agent_property : problem.agent_properties){
                radius_list.push_back(agent_property.radius);
            }

            std::cout << "Num Agemts: " << problem.numAgents() << " ";
        }

        amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem){

            init(problem);

            amp::MultiAgentPath2D result;

            //[TEST]
            // std::cout << isCircleRobotCollision(Eigen::Vector2d(4.92895, 7.24022), 0.5, problem.obstacles) << "\n";
            // return result;

            // [x1, y1, x2, y2, ... xn, yn]
            // Set sampling bound
            Eigen::VectorXd q_min = get_meta_q_min(problem, 0);
            Eigen::VectorXd q_max = get_meta_q_min(problem, 1);
            // log_vector(q_min);
            // log_vector(q_max);

            // Meta-agent configuration space
            Eigen::VectorXd q_init = get_meta_q_init_or_goal(problem, 0);
            Eigen::VectorXd q_goal = get_meta_q_init_or_goal(problem, 1);
            // log_vector(q_init);
            // log_vector(q_goal);

            // set the root node
            Tree.push_back(0);
            node_to_coord[0] = q_init;

            // RRT loop
            int index_node = 1;
            while(true){
                if(Tree.size() >= N_iter){
                    std::cout << "EXCEED MAX Nodes\n\n"; // Fail
                    break;
                }
                // Just for debugging
                if(Tree.size()%1000 == 0){
                    std::cout << Tree.size() << " ";
                }
                if(whileloopcount > max_whileloops){
                    std::cout << "EXCEED MAX WhileLoops\n\n";
                    break;
                }
                whileloopcount = whileloopcount + 1;
                if( whileloopcount% 1000 == 0){
                    std::cout << whileloopcount << " Tree Size: " << Tree.size() << "\n";
                }


                // Sample Q_rand
                Eigen::VectorXd q_rand = get_q_sample(problem, q_min, q_max, q_goal);
                //std::cout << "q_rand: "; log_vector(q_rand);

                // Extend
                int node_near = search_node_near(q_rand);
                Eigen::VectorXd q_near = node_to_coord[Tree[node_near]];
                // std::cout << "q_near: "; log_vector(q_near);
                Eigen::VectorXd q_new = get_q_new(q_near, q_rand);
                // std::cout << "q_new: "; log_vector(q_new);
                // std::cout << "\n";

                // Connect [TEMP]
                if(!isRobotsCollision(problem, q_new) && !isRobotsEdgeCollision(problem, q_near, q_new)){
                    
                    Tree.push_back(index_node);
                    node_to_coord[index_node] = q_new;
                    graph.connect(Tree[node_near], index_node, step_size);
                    index_node = index_node + 1;

                    //Success
                    if(isGoal(q_new, q_goal)){
                        isSuccess = true;
                        break;
                    }
                }
            }

            // Reconstruct path
            if(isSuccess){
                // Node path
                std::cout << "Success\n";
                std::vector<int> node_path = reconstruct_node_path();
                // log_node_path(node_path);
                // for(int i=0; i < node_path.size(); i++){
                //     log_vector(node_to_coord[Tree[node_path[i]]]);
                // }
                
                // Coordinate Path
                for(int i=0; i<problem.numAgents(); i++){
                    amp::Path2D path_i;
                    for(int k=0; k < node_path.size(); k++){
                        Eigen::VectorXd q_k = node_to_coord[Tree[node_path[k]]];
                        Eigen::Vector2d q_ik = {q_k[dimension*i], q_k[dimension*i+1]};
                        path_i.waypoints.push_back(q_ik);
                        // log_vector(q_ik);
                    }
                    Eigen::Vector2d q_ik = {q_goal[dimension*i], q_goal[dimension*i+1]};
                    path_i.waypoints.push_back(q_ik);
                    result.agent_paths.push_back(path_i);
                }
            }
            else{
                for(int i=0; i < problem.numAgents(); i++){
                    amp::Path2D path_i;
                    Eigen::Vector2d q_i = {q_init[dimension*i], q_init[dimension*i+1]};
                    path_i.waypoints.push_back(q_i);
                    result.agent_paths.push_back(path_i);
                }
            }

            treesize = (double)(Tree.size());
            return result;
        }

        Eigen::VectorXd get_q_sample(const amp::MultiAgentProblem2D& problem, 
                                     Eigen::VectorXd q_min,
                                     Eigen::VectorXd q_max,
                                     Eigen::VectorXd q_goal){
            Eigen::VectorXd p(dimension_meta);
            // goal-biased
            int random = rng.randi(1,100);
            if(random <= 5){
                return q_goal;
            }
            // random nodes [q_min, q_max]
            else{
                while(true){
                    for(int i=0; i<dimension_meta; i ++){
                        p[i] = rng.srandd(q_min[i], q_max[i]);
                    }
                    if(!isRobotsCollision(problem, p)){
                        break;
                    }
                    //std::cout << "\nCOLLIDE!!!\n";
                }
                return p;
            }
        }

        bool isRobotsCollision(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& p){
            // Reconstruct xi, yi
            const int n_robots = dimension_meta/dimension;
            for(int i=0; i < n_robots; i++){

                // Robot to obstacle collision
                Eigen::Vector2d qi = {p[dimension*i], p[dimension*i+1]};
                double ri = radius_list[i];
                if(isCircleRobotCollision(qi, ri, problem.obstacles)){
                    return true;
                }

                // Robot to robot collision
                for(int j=0; j < n_robots; j++){
                    if(i != j){
                        Eigen::Vector2d qi = {p[dimension*i], p[dimension*i+1]};
                        double ri = radius_list[i];
                        Eigen::Vector2d qj = {p[dimension*j], p[dimension*j+1]};
                        double rj = radius_list[j];
                        Eigen::Vector2d r_diff = qi - qj;
                        if(r_diff.norm() < (ri + rj)){
                            return true;
                        }
                    }
                }
            }
            return false;
        }

        bool isRobotsEdgeCollision(const amp::MultiAgentProblem2D& problem, 
                                   const Eigen::VectorXd& q_near, 
                                   const Eigen::VectorXd& q_new){
            Eigen::VectorXd q_diff = q_new - q_near;
            double d = q_diff.norm();
            q_diff = q_diff/d;
            int N = (int)(d/step_size);
            for(int i=0; i<N; i++){
                Eigen::VectorXd qi = q_near + q_diff*step_size*i;
                if(isRobotsCollision(problem, qi)){
                    return true;
                }
            }
            return false;
        }



        int search_node_near(Eigen::VectorXd& q_rand){
            double d_ref = 999;
            double n_ref = -1;

            for(int i=0; i<Tree.size(); i++){
                Eigen::VectorXd q = node_to_coord[Tree[i]];
                Eigen::VectorXd q_diff = q - q_rand;
                double d = q_diff.norm();
                //std::cout << "node: " << Tree[i] <<  " " << q[0] <<  " " << q[1] << " d: " << d << "\n";
                if(d<d_ref){
                    d_ref = d;
                    n_ref = i;
                }
            }
            return n_ref;
        }

        Eigen::VectorXd get_q_new(Eigen::VectorXd& q_near, Eigen::VectorXd& q_rand){
            // std::cout << "Connecting: ";
            // log_vector(q_near);

            Eigen::VectorXd diff = q_rand - q_near;
            Eigen::VectorXd q_new = diff * step_size / diff.norm();
            q_new = q_new + q_near;
            // log_vector(q_new);

            return q_new;
        }

        bool isGoal(Eigen::VectorXd& q, Eigen::VectorXd& q_goal){
            Eigen::VectorXd diff = q - q_goal;
            double d = diff.norm();
            if(d < epsilon){
                return true;
            }
            return false;
        }

        std::vector<int> reconstruct_node_path(){
            std::vector<int> node_path;

            int node = Tree.back();

            while(true){
                node_path.insert(node_path.begin(), node);

                std::vector<unsigned int> node_parent_vector = graph.parents(node);
                
                if(node_parent_vector.size() == 0){
                    return node_path;
                }

                int node_parent = node_parent_vector[0];
                node = node_parent;
            }
        }

        void print_problem(const amp::MultiAgentProblem2D& problem){
            std::cout << "Problem: \n";
            std::cout << "x_min_max: " << problem.x_min << " " << problem.x_max << "\n";
            std::cout << "y_min_max: " << problem.y_min << " " << problem.y_max << "\n";
            std::vector<amp::CircularAgentProperties> agent_properties = problem.agent_properties;
            for(auto agent_property : agent_properties){
                std::cout << "agent r: " << agent_property.radius << "\n";
                std::cout << "start: " << agent_property.q_init[0] << " " <<  agent_property.q_init[1] <<  "\n";
                std::cout << "end: "   <<  agent_property.q_goal[0] << " " << agent_property.q_goal[1] <<  "\n";
            }
        }

        Eigen::VectorXd get_meta_q_init_or_goal(const amp::MultiAgentProblem2D& problem, int index){
            std::vector<double> q_list;
            for(auto agent_property : problem.agent_properties){
                if (index == 0){
                    q_list.push_back(agent_property.q_init[0]);
                    q_list.push_back(agent_property.q_init[1]);
                }
                else{
                    q_list.push_back(agent_property.q_goal[0]);
                    q_list.push_back(agent_property.q_goal[1]);
                }
            }
            Eigen::VectorXd q = vector_from_list(q_list);
            return q;
        }

        Eigen::VectorXd get_meta_q_min(const amp::MultiAgentProblem2D& problem, int index){
            std::vector<double> q;
            const int n_robots = dimension_meta/dimension;
            for(int i = 0; i < n_robots; i++){
                if(index == 0){
                    q.push_back(problem.x_min);
                    q.push_back(problem.y_min);
                }
                else{
                    q.push_back(problem.x_max);
                    q.push_back(problem.y_max);
                }
            }
            return vector_from_list(q);
        }

};
