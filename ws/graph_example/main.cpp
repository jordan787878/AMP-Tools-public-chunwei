#include <cstdlib> 
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW8.h"
#include <fstream>
#include <vector>
#include <list>

#include "MyCentMAgentRRT.h"
#include "MyDecentMAgentRRT.h"

// NOTE: just for reference
// struct CircularAgentProperties {
//     /// @brief Radius of the circular agent
//     double radius = 1.0;

//     /// @brief Initial location
//     Eigen::Vector2d q_init;
//     /// @brief Goal location
//     Eigen::Vector2d q_goal;
// };

// /// @brief Environment with a set of agent properties
// struct MultiAgentProblem2D : Environment2D {
//     /// @brief Properties of each agent
//     std::vector<CircularAgentProperties> agent_properties;

//     /// @brief Number of agents is given by the number of agent properties
//     /// @return Number of circular agents
//     inline std::size_t numAgents() const {return agent_properties.size();}
// };

void Problem1(){
    MyCentMAgentRRT centmagentrrtalgo;

    uint32_t n_agents = 2;
    amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(n_agents);

    centmagentrrtalgo.print_problem(problem);
    amp::MultiAgentPath2D path = centmagentrrtalgo.plan(problem);

    amp::Visualizer::makeFigure(problem, path);
    amp::Visualizer::showFigures();

    amp::HW8::check(path, problem);

}

void BenchMarkCentRRT_1c(){

    bool isShowResult = false;
    MyCentMAgentRRT centmagentrrtalgo;

    std::cout << "Start Benchmarking...\n";

    std::vector<std::string> labels = {"2", "3", "4", "5", "6"};
    // std::vector<std::string> labels = {"2"};

    std::list<std::vector<double>> data_TreeSize;
    std::list<std::vector<double>> data_CompTime;

    for(int i=2; i<7; i++){

        uint32_t n_agents = i;
        amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(n_agents);

        std::cout << " ==== Agents: " << i << " ==== \n";

    
        std::vector<double> TreeSize;
        std::vector<double> CompTime;

        for(int j=0; j<100; j++){
            auto start_time = std::chrono::high_resolution_clock::now();

            amp::MultiAgentPath2D path = centmagentrrtalgo.plan(problem);

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end_time - start_time;
            double duration_ms = duration.count();
            
            TreeSize.push_back(centmagentrrtalgo.treesize);
            CompTime.push_back(duration_ms);

            // if(i==3 && centmagentrrtalgo.isSuccess && !isShowResult){
            //     amp::Visualizer::makeFigure(problem, path);
            //     amp::Visualizer::showFigures();
            //     amp::HW8::check(path, problem);
            //     isShowResult = true;
            // }
        }
        std::cout << "\n";

        data_TreeSize.push_back(TreeSize);
        data_CompTime.push_back(CompTime);
    }

    writeListVectorToCSV(data_TreeSize, "/Users/chko1829/src/ASEN5254_Motion_Planning/AMP-Tools-public-chunwei/file_dump/data_TreeSize.csv");
    writeListVectorToCSV(data_CompTime, "/Users/chko1829/src/ASEN5254_Motion_Planning/AMP-Tools-public-chunwei/file_dump/data_CompTime.csv");

    amp::Visualizer::makeBoxPlot(data_TreeSize, labels);
    amp::Visualizer::makeBoxPlot(data_CompTime, labels);
    amp::Visualizer::showFigures();
}

void Problem2(){
    MyDecentMAgentRRT decentmagentrrtalgo;

    uint32_t n_agents = 2;
    amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(n_agents);

    amp::MultiAgentPath2D path = decentmagentrrtalgo.plan(problem);

    amp::Visualizer::makeFigure(problem, path);
    amp::Visualizer::showFigures();

    amp::HW8::check(path, problem);
}

void GradeHW8(int argc, char** argv){

    MyCentMAgentRRT centmagentrrtalgo;

    MyDecentMAgentRRT decentmagentrrtalgo;

    amp::HW8::grade(centmagentrrtalgo, decentmagentrrtalgo, "chko1829@colorado.edu", argc, argv);

}

int main(int argc, char** argv) {

    // Problem1();

    // BenchMarkCentRRT_1c();

    // Problem2();

    GradeHW8(argc, argv);

    std::cout << "[ (hw8) RUN COMPLETE ]\n";
    
    

    return 0;


    //system("/opt/homebrew/Cellar/graphviz/9.0.0/bin/dot -Tpng /Users/chko1829/src/ASEN5254_Motion_Planning/AMP-Tools-public-chunwei/ws/graph_example/my_graph.dot -o /Users/chko1829/src/ASEN5254_Motion_Planning/AMP-Tools-public-chunwei/ws/graph_example/my_graph.png");
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // // Let's connect some nodes with some edges
    // amp::Graph<double> graph;
    // graph.connect(0, 1, 1.0);
    // graph.connect(1, 2, 1.0);
    // graph.connect(1, 0, 1.0);
    // graph.connect(1, 6, 1.0);
    // graph.connect(6, 2, 1.0);
    // graph.connect(6, 1, 1.0);

    // // Make sure the graph is correct by printing it
    // graph.print();

    // // Try mapping node to coordinate and visualization
    // std::map<amp::Node, Eigen::Vector2d> node_to_coord;
    // node_to_coord[0] = Eigen::Vector2d(0.0, 0.0);
    // node_to_coord[1] = Eigen::Vector2d(0.0, 1.0);
    // node_to_coord[2] = Eigen::Vector2d(1.0, 0.0);
    // node_to_coord[6] = Eigen::Vector2d(1.0, 1.0);
    // amp::Problem2D prob = amp::HW2::getWorkspace1();
    // amp::Visualizer::makeFigure(prob, graph, node_to_coord);
    // amp::Visualizer::showFigures();

    // Print the nodes inside the graph 
    // std::vector<amp::Node> nodes = graph.nodes();
    // NEW_LINE;
    // INFO("Nodes in graph:");
    // for (amp::Node n : nodes)
    //     INFO(" - " << n);

    // // Look at the children of node `1`
    // amp::Node node = 1;
    // const std::vector<amp::Node>& children = graph.children(node);

    // // Print the children
    // NEW_LINE;
    // INFO("Children of node " << node << ":");
    // for (amp::Node n : children)
    //     INFO(" - " << n);

    // // Look at the outgoing edges of node `1`
    // const auto& outgoing_edges = graph.outgoingEdges(node);

    // // Print the outgoing edges (notice they are always in the same order as the children nodes)
    // NEW_LINE;
    // INFO("Outgoing edges of node " << node << ":");
    // for (const auto& edge : outgoing_edges)
    //     INFO(" - " << edge);

    // // Ok let's get funky and disconnect some nodes!
    // graph.disconnect(1, 0); // disconnect any edge between 1 and 0
    // graph.disconnect(6, 2, "do"); // disconnect only the edge "do" going from 6 to 2
    // NEW_LINE;
    // graph.print();

    // // Random graph
    // std::shared_ptr<amp::Graph<double>> random_graph = amp::GraphTools::generateRandomGraphDouble(10, 0.0, 1.0, 3);
    // NEW_LINE;
    // random_graph->print("Random Graph");

    // return 0;
}


