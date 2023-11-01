#include "AMPCore.h"
#include "hw/HW2.h"

int main() {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    // Let's connect some nodes with some edges
    amp::Graph<double> graph;
    graph.connect(0, 1, 1.0);
    graph.connect(1, 2, 1.0);
    graph.connect(1, 0, 1.0);
    graph.connect(1, 6, 1.0);
    graph.connect(6, 2, 1.0);
    graph.connect(6, 1, 1.0);

    // Make sure the graph is correct by printing it
    graph.print();

    // Try mapping node to coordinate and visualization
    std::map<amp::Node, Eigen::Vector2d> node_to_coord;
    node_to_coord[0] = Eigen::Vector2d(0.0, 0.0);
    node_to_coord[1] = Eigen::Vector2d(0.0, 1.0);
    node_to_coord[2] = Eigen::Vector2d(1.0, 0.0);
    node_to_coord[6] = Eigen::Vector2d(1.0, 1.0);
    amp::Problem2D prob = amp::HW2::getWorkspace1();
    amp::Visualizer::makeFigure(prob, graph, node_to_coord);
    amp::Visualizer::showFigures();

    return 0;


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