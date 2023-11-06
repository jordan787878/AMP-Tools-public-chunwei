#pragma once

#include "AMPCore.h"
#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include "MyODE.h"
#include "MyFunctions.h"
#include <algorithm>

struct Element
{
    int node;
    double h;
};


class MyRRT{
    public:
        amp::RNG rng;
        double tstep_Traj = 0.0001;
        MyODE ode;
        Eigen::VectorXd x_init;
        Eigen::VectorXd x_goal;
        double d_init;
        std::vector<std::pair<double, double>> U_bounds;
        
        // Hyperparameter
        const int max_nodes = 50000;
        std::vector<Element> elements;
        int max_elements = 20;     // the number of min distance node stored in memory (serve as the "goal-biased nodes" to grow from)
        double orbit_factor = 1.2; // the factor of the trajectory's orbit with respect to the initial orbit
        int max_elements_percentage = 25;

        std::vector<int> Tree;
        amp::Graph<double> graph;
        std::map<amp::Node, Eigen::VectorXd> node_to_coord;
        std::map<std::vector<int>, std::vector<double>> edge_to_control;

        // Results
        std::vector<Eigen::VectorXd> graph_plot;
        double time_of_flight;


        void problem_setup(std::string config_file){
            // ODE
            ode.init_params(config_file);

            // inital and goal states
            x_init.resize(6);
            x_init << ode.x_init_config[0], ode.x_init_config[1], ode.x_init_config[2], 
                      ode.x_init_config[3], ode.x_init_config[4], ode.x_init_config[5];
            // x_init << 0.0019, 0.0488, 0.0, 0.0, 0.0, 4.7163;                     // 4. orbit
            
            x_goal.resize(6);
            x_goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            // Control bounds (u_min, u_max)
            std::pair<double, double> U1_bounds(-0.5*M_PI, 0.5*M_PI);
            U_bounds.push_back(U1_bounds);
            std::pair<double, double> U2_bounds(0, 2*M_PI);
            U_bounds.push_back(U2_bounds);
            std::pair<double, double> U3_bounds(0.001, 0.05);
            U_bounds.push_back(U3_bounds);

            // d_init
            d_init = distance_Eu(x_init, x_goal);
            std::cout << "d_init: " << d_init << "\n";
            elements.push_back({0, d_init});

            // time_of_flight init
            time_of_flight = 0.0;

            // clean
            Tree.clear();
            node_to_coord.clear();

        }

        std::vector<Eigen::VectorXd> plan(){
            // init path
            std::vector<Eigen::VectorXd> path;

            // init tree
            Tree.push_back(0);
            node_to_coord[0] = x_init;
            int node_count = 1;

            while(true){
                if(node_count > max_nodes){
                    std::cout << "[DEBUG] exceed max nodes\n";
                    break;
                }
                
                // Choose x0 to grow
                // (max_elements_percentage)%: chance to select the closer-to-goal nodes to grow
                // (100-max_elements_percentage)%: chance to select any nodes in the Tree
                Eigen::VectorXd x0(6);
                // random select node to grow
                int random1 = rng.randi(0, elements.size());
                int node = elements[random1].node;
                int random = rng.randi(0, 100);
                if(random > max_elements_percentage){
                    node = rng.randi(0, Tree.size());
                }
                x0 = node_to_coord[node];

                // Sample U (u1, u2, tau)
                std::vector<double> U_sample;
                for(int i=0; i<U_bounds.size(); i++){
                    U_sample.push_back(get_control_sample(i));
                    // std::cout << "u" << i << "_sample: " << U_sample[i] << "\n";
                }

                // Propagate state
                Eigen::VectorXd* state_sample_ptr = get_state_sample_ptr(x0, U_sample, node_count);

                // Connect to Tree (x0 to state_sample_ptr)
                connect_node(state_sample_ptr, node, node_count, U_sample);

                // Terminal condition (defined in the ODE)
                if(state_sample_ptr){
                    Eigen::VectorXd x_new = (*state_sample_ptr);
                    if(ode.isTerminal(x_new)){
                        break;
                    }
                }

            }

            // Reconstruct path
            std::vector<int> node_path;
            construct_path(path, node_path);

            // Recontrcut Traj
            // std::cout << "[RESULT] Transition:\n";
            std::vector<Eigen::VectorXd> trajectory;
            std::vector<Eigen::VectorXd> control_signals;
            trajectory.push_back(path[0]);
            for(int i=0; i < path.size()-1; i++){
                int node_i = node_path[i];
                //std::cout << node_i << " ";
                std::vector<int> edge_i = {node_i, node_path[i+1]};
                std::vector<double> U_i = edge_to_control[edge_i];
                std::vector<double> u_control_i = {U_i[0], U_i[1]};
                std::vector<Eigen::VectorXd> trajectory_i = ode.rungeKutta(0.0, trajectory.back(), u_control_i, tstep_Traj, U_i[2]);

                Eigen::VectorXd control_signals_i(3);
                control_signals_i << u_control_i[0] , u_control_i[1] , U_i[2];
                control_signals.push_back(control_signals_i);

                // std::cout << node_i << "->" << node_path[i+1] << ": "
                // << u_control_i[0] << " " << u_control_i[1] << " " << U_i[2] << "\n";

                trajectory.insert(trajectory.end(), trajectory_i.begin(), trajectory_i.end());

                // time of flight
                time_of_flight = time_of_flight + U_i[2];
            }
            std::cout << "\n";
            write_x_traj(graph_plot, "/Users/chko1829/src/SolarSailLanding/file_dump/graph.csv");
            write_x_traj(control_signals, "/Users/chko1829/src/SolarSailLanding/file_dump/control_signals.csv");
            write_x_traj(trajectory, "/Users/chko1829/src/SolarSailLanding/file_dump/trajectory.csv");
            // Validatet collision free trajectory
            for(auto& q: trajectory){
                if(isCollision(q)){
                    std::cout << "[DEBUG] trajectory collide\n";
                    break;
                }
                if(ode.isTerminal(q)){
                    std::cout << "[RESULT] trajectory land success\n";
                    break;
                }
            }
            print_terminal_state(path.back());

            return path;
        }

        Eigen::VectorXd* get_state_sample_ptr(Eigen::VectorXd& x0, std::vector<double>& U_sample, int node){            
            const double u1 = U_sample[0];
            const double u2 = U_sample[1];
            const double t_end = U_sample[2];
            const std::vector<double> u_control = {u1, u2};
            std::vector<Eigen::VectorXd> x_traj;
            x_traj = ode.rungeKutta(0.0, x0, u_control, tstep_Traj, t_end);
            // write_x_traj(x_traj, "/Users/chko1829/src/SolarSailLanding/file_dump/traj_sample.csv");

            // Check the validity of the edge (the x_traj)
            for(auto& q: x_traj){
                if(isCollision(q)){
                    return nullptr;
                }
            }
            
            // Get the node ptr
            Eigen::VectorXd x_new = x_traj.back();
            Eigen::VectorXd* state_sample_ptr = new Eigen::VectorXd(6);
            (*state_sample_ptr) = x_new;

            // Store the 20-minimal element list
            double d_new = distance_Eu(x_new, x_goal);
            // Sort the vector based on the h value using a lambda function
            std::sort(elements.begin(), elements.end(), [](const Element& a, const Element& b) {
                return a.h < b.h;
            });
            Element e = elements.back();
            double d_thres = e.h;
            // std::cout << "size: " << elements.size() << " ";
            // std::cout << "d_thres: " << d_thres << "\n";
            if(d_new < d_thres){
                if(elements.size() >= max_elements){
                    elements.pop_back();
                }
                elements.push_back({node, d_new});
                std::cout << "d_new: " << d_new << "\n";
            }

            return state_sample_ptr;
        }

        double get_control_sample(const int control_signal){
            std::pair<double, double> bound = U_bounds[control_signal];
            return rng.srandd(bound.first, bound.second);
        }

        double distance_Eu(Eigen::VectorXd& a, Eigen::VectorXd& b){
            Eigen::VectorXd pos(3);
            pos << a[0] - b[0], a[1] - b[1], a[2] - b[2];
            //std::cout << "unit_length: " << ode.unit_length << "\n";
            return pos.norm();
        }

        void connect_node(Eigen::VectorXd* state_sample_ptr, const int node, int& node_count, const std::vector<double>& U_sample){
            if(state_sample_ptr){
                Eigen::VectorXd x0 = node_to_coord[node];
                Eigen::VectorXd x_new = (*state_sample_ptr);
                // std::cout << "[DEBUG] Valid state sample: ";
                // log_vector(x_new);
                Tree.push_back(node_count);
                node_to_coord[node_count] = x_new;
                std::vector<int> edge = {node, node_count};
                edge_to_control[edge] = U_sample;
                graph.connect(node, node_count, 0.0);
                // std::cout << node << "->" << node_count << ": " 
                // << U_sample[0] << " " << U_sample[1] << " " << U_sample[2] << "\n";
                // std::cout << "    ";
                // log_vector(x0);
                // std::cout << " -> ";
                // log_vector(x_new);
                // std::cout << "\n";

                // write the data
                Eigen::VectorXd node_pair(12);
                node_pair << x0[0] , x0[1] , x0[2] , x0[3] , x0[4] , x0[5],
                             x_new[0] , x_new[1] , x_new[2], x_new[3] , x_new[4] , x_new[5];
                graph_plot.push_back(node_pair);

                node_count = node_count + 1;
            }
        }

        void construct_path(std::vector<Eigen::VectorXd>& path, std::vector<int>& node_path){
            // Sort the vector based on the h value using a lambda function
            std::sort(elements.begin(), elements.end(), [](const Element& a, const Element& b) {
                return a.h < b.h;
            });
            Element e = elements.front();
            int n = e.node;
            while(true){
                std::cout << n << " ";
                Eigen::VectorXd q = node_to_coord[n];
                path.insert(path.begin(), q);
                node_path.insert(node_path.begin(), n);

                std::vector<unsigned int> node_parent_vector = graph.parents(n);
                if(node_parent_vector.size() == 0){
                    break;
                }

                int node_parent = node_parent_vector[0];
                n = node_parent;
            }
            std::cout << "\n";
        }

        bool isCollision(Eigen::VectorXd& s){
            double km2m = 1000.0;
            double x =  abs(s[0]*ode.unit_length*km2m);
            double y =  abs(s[1]*ode.unit_length*km2m);
            double z =  abs(s[2]*ode.unit_length*km2m);
            double r = pow(x*x + y*y + z*z, 0.5);
            double vx = abs(s[3]*ode.unit_vel*km2m);
            double vy = abs(s[4]*ode.unit_vel*km2m);
            double vz = abs(s[5]*ode.unit_vel*km2m);
            // out of bound condition
            if(distance_Eu(s, x_goal) > orbit_factor * d_init){
                return true;
            }
            // crashing condition
            // if( std::max({vx, vy, vz}) > 2.0 && std::min({x, y, z}) < 500.0){
            if( std::max({vx, vy, vz}) > 2.0 && r < 500.0){
                return true;
            }
            return false;
        }

        void print_terminal_state(Eigen::VectorXd s){
            double km2m = 1000.0;
            std::cout << "[RESULT] terminal at: " << time_of_flight << "\n";
            std::cout << "[RESULT] terminal state (m, m/s): ";
            std::cout << s[0]*ode.unit_length*km2m << " " 
                      << s[1]*ode.unit_length*km2m << " " 
                      << s[2]*ode.unit_length*km2m << " "
                      << s[3]*ode.unit_vel*km2m << " "
                      << s[4]*ode.unit_vel*km2m << " "
                      << s[5]*ode.unit_vel*km2m << "\n";
        }

        // Custom comparison function to compare Element structures based on h
        bool compareByH(const Element& a, const Element& b) {
            return a.h < b.h;
        }

};