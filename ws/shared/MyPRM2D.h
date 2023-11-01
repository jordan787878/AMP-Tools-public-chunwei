#pragma once

#include "AMPCore.h"
#include "hw/HW7.h"
#include "MyFunctions.h"
#include "MyAStarAlgo.h"
#include <iostream>

struct LookupSearchHeuristic : public amp::SearchHeuristic {
	/// @brief Get the heuristic value stored in `heuristic_values`. 
	/// @param node Node to get the heuristic value h(node) for. 
	/// @return Heuristic value
	virtual double operator()(amp::Node node) const override {return heuristic_values.at(node);}

    /// @brief Store the heursitic values for each node in a map
    std::map<amp::Node, double> heuristic_values; 
};


class MyPRM2D: public amp::PRM2D{
    public:
        bool usePathSmooth = true;

        double path_length;
        bool findPath;
        int N_sample;
        double R_radius;
        amp::Graph<double> graph;
        std::map<amp::Node, Eigen::Vector2d> node_to_coord;

        void init(int N, double r){
            N_sample = N;
            R_radius = r;
            path_length = 0;
            findPath = false;
            graph.clear();
            node_to_coord.clear();
        }

        amp::Path2D plan(const amp::Problem2D& problem){
            
            // init return variable
            amp::Path2D path;
            path_length = 0;
            findPath = false;
            graph.clear();
            node_to_coord.clear();
            
            path.waypoints.push_back(problem.q_init);

            //print_problem_setup(problem);
            
            // Get sample points
            std::vector<Eigen::Vector2d> sample_points_coord = get_sample_points(problem, 
                problem.x_min, problem.x_max, problem.y_min, problem.y_max);

            // Connecting sample points and map node(int) to coordinate (x,y)
            connect_sample_points(problem, sample_points_coord);
            
            // Store heuristic, loop over connecting nodes
            LookupSearchHeuristic heuristic = get_heuristic();

            // Connect q_init and q_goal to graph
            connect_qinit_qgoal(problem);

            // Apply A-star algorithm
            MyAStarAlgo::GraphSearchResult result = get_Astar_result(heuristic);

            // Generate Path
            generate_path(problem, result, path);

            // Return Smooth Path (or not)
            if(usePathSmooth){
                // Generate Smooth Path
                amp::Path2D path_smooth = generate_path_smooth(problem, result);
                return path_smooth;
            }
            else{
                return path;
            }
        }


        bool isEdgeCollision(const Eigen::Vector2d p1, const Eigen::Vector2d p2, const std::vector<amp::Polygon> obstacles){
            for(int j=0; j<obstacles.size(); j++){
                amp::Obstacle2D obs_j = obstacles[j];
                for(int k=0; k<obs_j.verticesCCW().size(); k++){
                    Point p3;
                    Point p4;
                    if(k==obs_j.verticesCCW().size()-1){
                        p3 = {obs_j.verticesCCW()[k][0], obs_j.verticesCCW()[k][1]};
                        p4 = {obs_j.verticesCCW()[0][0], obs_j.verticesCCW()[0][1]};
                    }
                    else{
                        p3 = {obs_j.verticesCCW()[k][0], obs_j.verticesCCW()[k][1]};
                        p4 = {obs_j.verticesCCW()[k+1][0], obs_j.verticesCCW()[k+1][1]};
                    }
                    bool intersection = doLineSegmentsIntersect(p1, p2, p3, p4);
                    if(intersection){
                        return true;
                    }
                }
            }
            return false;
        }

        void print_problem_setup(const amp::Problem2D& problem){
            std::cout << problem.x_min << " " << problem.x_max << "\n";
            std::cout << problem.y_min << " " << problem.y_max << "\n";
            std::cout << "q_init " << problem.q_init[0] << " " << problem.q_init[1] << "\n";
            std::cout << "q_goal " << problem.q_goal[0] << " " << problem.q_goal[1] << "\n";
        }

        std::vector<Eigen::Vector2d> get_sample_points(const amp::Problem2D& problem, 
                                                       const double x_min, const double x_max,
                                                       const double y_min, const double y_max){
            int pts_sample = 0;
            std::vector<Eigen::Vector2d> sample_points_coord;
            amp::RNG rng;
            while(pts_sample < N_sample){
                double x = rng.srandd(x_min, x_max);
                double y = rng.srandd(y_min, y_max);
                Eigen::Vector2d p ={x,y};
                if(!isPointCollision(p, problem.obstacles)){
                    //std::cout << "No Collision, add sample: " << pts_sample << "\n";
                    // update counts
                    sample_points_coord.push_back(p);
                    pts_sample = pts_sample + 1;
                }
            }
            return sample_points_coord;
        }

        void connect_sample_points(const amp::Problem2D& problem, std::vector<Eigen::Vector2d>& sample_points_coord){
            for(int i=0; i<N_sample; i++){
                // Map node to coordinate
                node_to_coord[i] = sample_points_coord[i];
                for(int j=i+1; j<N_sample; j++){
                    Eigen::Vector2d pi = sample_points_coord[i];
                    Eigen::Vector2d pj = sample_points_coord[j];
                    double edge_distance_L2 = cal_distance(pi, pj);
                    if(edge_distance_L2 < R_radius){
                        bool edgecollision = MyPRM2D::isEdgeCollision(pi, pj, problem.obstacles);
                        //std::cout << "edge collision check: " << edgecollision << "\n";
                        if(!edgecollision){
                            graph.connect(i, j, edge_distance_L2);
                            graph.connect(j, i, edge_distance_L2);
                        }
                    }
                }
            }
        }

        LookupSearchHeuristic get_heuristic(){
            LookupSearchHeuristic heuristic;
            for(int i = 0; i < graph.nodes().size(); i++){
                //double h_i = cal_distance(node_to_coord[nodes[i]], problem.q_goal);
                //std::cout << nodes[i] << " , h: " << h_i << "\n";
                heuristic.heuristic_values[graph.nodes()[i]] = 0.0;
            }
            return heuristic;
        }

        void connect_qinit_qgoal(const amp::Problem2D& problem){
            double d_ref_init = 999; 
            double d_ref_goal = 999;
            int n_ref_init = -1; 
            int n_ref_goal = -1;
            for(int i=0; i<N_sample; i++){
                Eigen::Vector2d p = node_to_coord[i];
                double d_to_init = cal_distance(p, problem.q_init);
                if(d_to_init < d_ref_init){
                    d_ref_init = d_to_init;
                    n_ref_init = i;
                }
                double d_to_goal = cal_distance(p, problem.q_goal);
                if(d_to_goal < d_ref_goal){
                    d_ref_goal = d_to_goal;
                    n_ref_goal = i;
                }
            }
            graph.connect(N_sample, n_ref_init, d_ref_init);
            node_to_coord[N_sample] = problem.q_init;
            graph.connect(n_ref_goal, N_sample+1, d_ref_goal);
            node_to_coord[N_sample+1] = problem.q_goal;

            // [Debug]
            //std::cout << n_ref_init << " " << N_sample   << " "<< node_to_coord[n_ref_init][0] << " " << node_to_coord[n_ref_init][1] << "\n";
            //std::cout << n_ref_goal << " " << N_sample+1 << " " << node_to_coord[n_ref_goal][0] << " " << node_to_coord[n_ref_goal][1] << "\n";
            // amp::Visualizer::makeFigure(problem, graph, node_to_coord);
            // amp::Visualizer::showFigures();
        }

        MyAStarAlgo::GraphSearchResult get_Astar_result(const amp::SearchHeuristic& heuristic){
            amp::ShortestPathProblem graphproblem;
            graphproblem.graph = std::make_shared<amp::Graph<double>>(graph);
            graphproblem.init_node = N_sample;
            graphproblem.goal_node = N_sample+1;
            MyAStarAlgo astar_algo;
            MyAStarAlgo::GraphSearchResult result = astar_algo.search(graphproblem, heuristic);
            return result;
        }

        void generate_path(const amp::Problem2D& problem, MyAStarAlgo::GraphSearchResult& result, amp::Path2D& path){
            if(result.success){
                //std::cout << "Find a Path\n";
                findPath = true;
                Eigen::Vector2d p = problem.q_init;
                
                // std::cout << "Node Path:\n";
                for(auto it = result.node_path.begin(); it != result.node_path.end(); ++it){
                    path.waypoints.push_back(node_to_coord[*it]);
                    double d = cal_distance(p, node_to_coord[*it]);
                    path_length = path_length + d;
                    p = node_to_coord[*it];
                }

                //path.waypoints.push_back(problem.q_goal);
            }
        }

        amp::Path2D generate_path_smooth(const amp::Problem2D& problem, MyAStarAlgo::GraphSearchResult& result){
            amp::Path2D path_smooth;

            std::vector<int> nodes_in_path;
            if(result.success){
                for(auto it = result.node_path.begin(); it != result.node_path.end(); ++it){
                    nodes_in_path.push_back(*it);
                    // std::cout << *it << " (" << std::fixed << std::setprecision(1)
                    //           << node_to_coord[*it][0] << " " <<node_to_coord[*it][1] << ")\n";
                }

                for(int i=1; i<50; i++){
                    int N_nodes_in_path = nodes_in_path.size();
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    std::uniform_int_distribution<int> dist(0, N_nodes_in_path-1);
                    int random1 = dist(gen);
                    int random2 = dist(gen);
                    if(random1 != random2){
                        //std::cout << N_nodes_in_path << ": " << random1 << " " << random2 << " ";
                        int n1 = nodes_in_path[random1];
                        int n2 = nodes_in_path[random2];
                        Eigen::Vector2d p1 = node_to_coord[n1];
                        Eigen::Vector2d p2 = node_to_coord[n2];
                        bool edgecollision = MyPRM2D::isEdgeCollision(p1, p2, problem.obstacles);
                        if(!edgecollision){
                            int j = random1 + 1;
                            if(j < random2){
                                auto it = nodes_in_path.begin() + j;
                                //std::cout << ", erase: " << j << " ";
                                nodes_in_path.erase(it);
                            }
                        }
                    }
                    // std::cout << "\n";
                }

                Eigen::Vector2d p = problem.q_init;
                path_length = 0;
                amp::Path2D path_smooth;
                for(int i=0; i<nodes_in_path.size(); i++){
                    int n = nodes_in_path[i];
                    path_smooth.waypoints.push_back(node_to_coord[n]);
                    double d = cal_distance(p, node_to_coord[n]);
                    path_length = path_length + d;
                    p = node_to_coord[n];
                }
                return path_smooth;
            }
            else{
                return path_smooth;
            }
        }

};
