#include "AMPCore.h"
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
			std::cout << j << "\t";
            auto start_time = std::chrono::high_resolution_clock::now();

            amp::MultiAgentPath2D path = centmagentrrtalgo.plan(problem);

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end_time - start_time;
            double duration_ms = duration.count();
            
            TreeSize.push_back(centmagentrrtalgo.treesize);
            CompTime.push_back(duration_ms);

            // if(i==3 && centmagentrrtalgo.isSuccess && !isShowResult){
            //     amp::Visualizer::makeFigure(problem, path);
            //     amp::HW8::check(path, problem);
            //     isShowResult = true;
            // }
        }
        std::cout << "\n";

        data_TreeSize.push_back(TreeSize);
        data_CompTime.push_back(CompTime);
    }

    //writeListVectorToCSV(data_TreeSize, "/Users/chko1829/src/ASEN5254_Motion_Planning/AMP-Tools-public-chunwei/file_dump/data_TreeSize.csv");
    //writeListVectorToCSV(data_CompTime, "/Users/chko1829/src/ASEN5254_Motion_Planning/AMP-Tools-public-chunwei/file_dump/data_CompTime.csv");

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

void BenchMarkDecentRRT(){

    bool isShowResult = false;
    MyDecentMAgentRRT decentrrtalgo;

    std::cout << "Start Benchmarking...\n";

    std::vector<std::string> labels = {"2", "3", "4", "5", "6"};
    //std::vector<std::string> labels = {"2"};

    std::list<std::vector<double>> data_TreeSize;
    std::list<std::vector<double>> data_CompTime;

    for(int i=2; i<7; i++){

        uint32_t n_agents = i;
        amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(n_agents);

        std::cout << " ==== Agents: " << i << " ==== \n";

        //std::vector<double> TreeSize;
        std::vector<double> CompTime;

        for(int j=0; j<100; j++){
			std::cout << j << " ";
            auto start_time = std::chrono::high_resolution_clock::now();

            amp::MultiAgentPath2D path = decentrrtalgo.plan(problem);

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end_time - start_time;
            double duration_ms = duration.count();
            
            //TreeSize.push_back(decentrrtalgo.treesize);
            CompTime.push_back(duration_ms);

            if(i==2 && decentrrtalgo.isSuccess && !isShowResult){
                amp::Visualizer::makeFigure(problem, path);
                amp::HW8::check(path, problem);
                isShowResult = true;
            }
        }
        std::cout << "\n";

        //data_TreeSize.push_back(TreeSize);
        data_CompTime.push_back(CompTime);
    }

    //writeListVectorToCSV(data_TreeSize, "/Users/chko1829/src/ASEN5254_Motion_Planning/AMP-Tools-public-chunwei/file_dump/data_TreeSize.csv");
    //writeListVectorToCSV(data_CompTime, "/Users/chko1829/src/ASEN5254_Motion_Planning/AMP-Tools-public-chunwei/file_dump/data_CompTime.csv");

    //amp::Visualizer::makeBoxPlot(data_TreeSize, labels);
    amp::Visualizer::makeBoxPlot(data_CompTime, labels);
    amp::Visualizer::showFigures();
}

void GradeHW8(int argc, char** argv){

    MyCentMAgentRRT centmagentrrtalgo;

    MyDecentMAgentRRT decentmagentrrtalgo;

    amp::HW8::grade(centmagentrrtalgo, decentmagentrrtalgo, "chko1829@colorado.edu", argc, argv);

}

int main(int argc, char** argv) {

	amp::RNG::seed(amp::RNG::randiUnbounded());

    // Problem1();

    // BenchMarkCentRRT_1c();

    // Problem2();

	// BenchMarkDecentRRT();

    GradeHW8(argc, argv);

    std::cout << "[ (hw8) RUN COMPLETE ]\n";
    

    return 0;
}
