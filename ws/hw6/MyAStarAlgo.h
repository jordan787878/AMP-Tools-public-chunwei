#include "AMPCore.h"


struct Element{
    uint32_t n;
    double h_value;
    uint32_t n_source;

    // Custom comparison function based on valueB
    bool operator<(const Element& other) const {
        return h_value < other.h_value;
    }
};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {

            GraphSearchResult result;
            result.success = false;
            result.path_cost = 0.0;
            Element current;
            // result.node_path is the close set?

            // Init an open (augmented) set of nodes: with (node index, priority)
            std::list<Element> open_set;
            std::list<Element> close_set;
            // Push the first node to the openset
            // A-star
            open_set.push_back({problem.init_node, heuristic(problem.init_node)});
            // Dijkstar's
            //open_set.push_back({problem.init_node, 0.0});

            int cal_steps = 0;

            // While loop until the open set is not empty
            while(!open_set.empty()){
                cal_steps = cal_steps + 1;
                // Get the top node from the open set
                current = open_set.front();
                // Remove it from the open set
                open_set.pop_front();
                // Add it to the close set
                close_set.push_back(current);
                // (debug)
                // std::cout << "current node: " << current.n << "\n";

                if(current.n == problem.goal_node){
                    // find and return the path
                    std::cout << "!!!!! Find a Path !!!!!\n";
                    // print_openset(close_set);
                    std::cout << "init node: " << problem.init_node << "\n";
                    std::cout << "goal node: " << problem.goal_node << "\n"; 

                    // Reconstruct a Path
                    // target node
                    result.success = true;
                    double pathcost = current.h_value;         
                    result.path_cost = pathcost;

                    std::cout << "Reconstructing the path...\n";
                    result.node_path.push_front(current.n);
                    // std::cout << "node: " << current.n << " ";

                    while(current.n != problem.init_node){

                        std::cout << "n : " << current.n << 
                                     " h:  " << current.h_value << 
                                     " source: " << current.n_source << "\n";

                        // loop over close set
                        for (auto it = close_set.begin(); it != close_set.end(); ++it) {
                            Element e_close = *it;
                            if(e_close.n == current.n_source){
                                current = e_close;
                                break;
                            }
                        }

                        result.node_path.push_front(current.n);
                        //std::cout << "node: " << e_parent.n << " ";
                        //current = e_parent;

                        // (debug)
                        // int xxx;
                        // std::cout << "reconstruct next? ";
                        // std::cin >> xxx;
                    }
                    
                    std::cout << "total calculation steps: " << cal_steps << "\n";
                    return result;
                }

                // (debug)
                // std::cout << "open set before adding neighbors: ";
                //print_openset(open_set);

                // get neighbor nodes (children for the directed graph)
                std::vector<uint32_t> neighbors = problem.graph->children(current.n);
                // (debug)
                // std::cout << "neighbors nodes: ";
                // for(int i=0; i<neighbors.size(); i++){
                //     std::cout << neighbors[i] << " ";
                // }
                // std::cout << "\n";

                // Calculate element information for neighbors
                //std::cout << "with priority: ";
                const std::vector<double>& edge_cost = problem.graph->outgoingEdges(current.n);
                std::vector<double> priority_neighbors;
                int i = 0;
                for (uint32_t neighbor : neighbors){
                    // A-star
                    double p = current.h_value - heuristic(current.n) + edge_cost[i] + heuristic(neighbor);
                    // Dijkstar's
                    //double p = current.h_value - 0*heuristic(current.n) + edge_cost[i] + 0*heuristic(neighbor);

                    priority_neighbors.push_back(p);
                    //std::cout << p << " ";
                    i = i+1;
                }
                //std::cout << "\n";
                
                // Add neighbors to open set
                // create an temp list
                std::list<uint32_t> sub_open_set;
                for (auto it = open_set.begin(); it != open_set.end(); ++it) {
                    Element e = *it;
                    sub_open_set.push_back(e.n);
                }
                std::list<uint32_t> sub_close_set;
                for (auto it = close_set.begin(); it != close_set.end(); ++it) {
                    Element e = *it;
                    sub_close_set.push_back(e.n);
                }

                // Adding neighbors
                i = 0;
                for (uint32_t neighbor : neighbors) {
                    // Check if the element is not in the 'Open' list and not in the 'Close' list

                    if (std::find(sub_close_set.begin(), sub_close_set.end(), neighbor) == sub_close_set.end()){
                        if(std::find(sub_open_set.begin(), sub_open_set.end(), neighbor) == sub_open_set.end()){
                            // direct add
                            open_set.push_back({neighbor, priority_neighbors[i], current.n});  
                        }
                        else{
                            // [temp] update value
                            std::list<Element>::iterator it;
                            //std::cout << "node: " << neighbor << " exists in open set at node: ";
                            for (it = open_set.begin(); it != open_set.end(); ++it) {
                                //Element e = &(*it);
                                if(it->n == neighbor){
                                    //std::cout << it->n << " " << it->h_value 
                                    //<< " (" << priority_neighbors[i] << " " << current.n << ")";
                                    if(it->h_value > priority_neighbors[i]){
                                        it->h_value = priority_neighbors[i];
                                        it->n_source = current.n;
                                        // std::cout << " update it.\n";
                                    }
                                    break;
                                }
                            }
                            
                        }
                    }

                    i = i + 1;
                }

                // (debug)
                // print_openset(open_set);
                

                // Sort the open set
                open_set.sort();
    //             //(debug) display the sorted list_aug
    //             for (const amp::Element& element : list_aug) {
    //                 std::cout << "Node: " << element.n <<
    //                 ", h value: " << element.h_value << std::endl;
    //             }
    //             // Update Open Set
    //             open_set.clear();
    //             for (const amp::Element& element : list_aug) {
    //                 open_set.push_back(element.n);
    //             }
    //             std::cout << "Open Set after Sorting the Priority ";
                // print_openset(open_set);

                // (debug)
                // int kkk;
                // std::cout << "Next Iteration?  ";
                // std::cin >> kkk;
             }

            std::cout << "CANNOT FIND PATH... (open set is empty)\n";
            return result;
        }

        // debug function
        void print_openset(std::list<Element> set){
            std::cout << "open set after adding neighbors:\n";
            for (auto it = set.begin(); it != set.end(); ++it) {
                Element e = *it;
                std::cout << "node: " << e.n << " , h: " << e.h_value << " , n source: " << e.n_source << "\n";
            }
            std::cout << "\n";
        }

};

// struct GraphSearchResult {
//             /// @brief Set to `true` if path was found, `false` if no path exists
//             bool success = false;

//             /// @brief Sequence of nodes where `node_path.front()` must contain init node, and `node_path.back()` must contain the goal node
//             std::list<amp::Node> node_path;

//             /// @brief Path cost (must equal sum of edge weights along node_path)
//             double path_cost;
// };