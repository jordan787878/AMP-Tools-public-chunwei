#include <chrono>
#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"

#include "MyPRM2D.h"
#include "MyRRT2D.h"

void BenchMarkPRM(){
    bool isShowFigure = false;

    amp::Problem2D prob = amp::HW5::getWorkspace1();

    MyPRM2D rpmalgo;

    std::cout << "Start Benchmarking...\n";
    std::vector<std::pair<int, double>> N_Rs = {{200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0},
                                                {500, 0.5}, {500, 1.0}, {500, 1.5}, {500, 2.0}};
    // std::vector<std::pair<int, double>> N_Rs = { {200, 1.0}, {200, 2.0}, {500, 1.0}, {500, 2.0},
    //                                              {1000, 1.0}, {1000, 2.0} };
    std::vector<std::string> labels = {"{200, 0.5}", "{200, 1}", "{200, 1.5}" , "{200, 2.0}",
                                       "{500, 0.5}", "{500, 1}", "{500, 1.5}" , "{500, 2.0}"};
    // std::vector<std::string> labels = {"{200, 1}", "{200, 2}", "{500, 1}" , "{500, 2}",
    //                                    "{1000, 1}", "{1000, 2}" };
    
    std::list<std::vector<double>> data_PathLength;
    // std::list<std::vector<double>> data_Success;
    std::vector<double> data_Success;
    std::list<std::vector<double>> data_CompTime;
    
    int i = 0;
    for(const std::pair<int, double> i_NR : N_Rs){
        std::cout << "\n === Label === " << labels[i] << "\n";
        int N_samples = i_NR.first;
        double R_radious = i_NR.second;
        double success_count = 0.0;

        std::vector<double> PathLengths;
        std::vector<double> isSuccess;
        std::vector<double> CompTime;

        for(int j=0; j<100; j++){
            // std::cout << j;

            auto start_time = std::chrono::high_resolution_clock::now();
            
            rpmalgo.init(N_samples, R_radious);
            amp::Path2D path = rpmalgo.plan(prob);

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end_time - start_time;
            double duration_ms = duration.count();
            double findPath = static_cast<double>(rpmalgo.findPath);

            if(i == 1 && findPath == 1.0 && isShowFigure == false){
                std::cout << "[LOG] Path Length: " << rpmalgo.path_length << "\n";
                amp::Visualizer::makeFigure(prob, rpmalgo.graph, rpmalgo.node_to_coord);
                amp::Visualizer::makeFigure(prob, path);
                //amp::Visualizer::showFigures();
                isShowFigure = true;
            }

            // std::cout << " (" << rpmalgo.path_length << ",";
            // std::cout << findPath << ",";
            // std::cout << duration_ms << ")\n";

            success_count = success_count + findPath;
            PathLengths.push_back(rpmalgo.path_length);
            CompTime.push_back(duration_ms);
        }
        data_PathLength.push_back(PathLengths);
        data_Success.push_back(success_count);
        data_CompTime.push_back(CompTime);
        std::cout << "\nsuccess count: " << success_count << "\n";
        i = i + 1;
    }

    // amp::Visualizer::makeBoxPlot(data_PathLength, labels);
    // amp::Visualizer::makeBarGraph(data_Success, labels);
    // amp::Visualizer::makeBoxPlot(data_CompTime, labels);
    amp::Visualizer::showFigures();

}

void Problem1(){
    //amp::Problem2D prob = amp::HW5::getWorkspace1();
    amp::Problem2D prob = amp::HW2::getWorkspace2();
    MyPRM2D rpmalgo;

    rpmalgo.init(1000, 2.0);
    amp::Path2D path = rpmalgo.plan(prob);
    double path_length = rpmalgo.path_length;
    bool findPath = rpmalgo.findPath;
    std::cout << "path length: " << path_length << "\n";
    std::cout << "find path? " << findPath << "\n";
    amp::HW7::check(path, prob);
    amp::Visualizer::makeFigure(prob, rpmalgo.graph, rpmalgo.node_to_coord);
    amp::Visualizer::makeFigure(prob, path);
    amp::Visualizer::showFigures();
}

void Problem2(){
    //amp::Problem2D prob = amp::HW5::getWorkspace1();
    amp::Problem2D prob = amp::HW2::getWorkspace2();
    
    MyRRT2D rrtalgo;

    amp::Path2D path = rrtalgo.plan(prob);
    std::cout << "Path Length: " << rrtalgo.pathLength << "\n";
    amp::HW7::check(path, prob);
    amp::Visualizer::makeFigure(prob, rrtalgo.graph, rrtalgo.node_to_coord);
    amp::Visualizer::makeFigure(prob, path);
    amp::Visualizer::showFigures();

    // [Remember to run init() in plan()]
    amp::HW7::generateAndCheck(rrtalgo);
}

void BenchMarkRRT(){
    bool isShowFigure = false;

    MyRRT2D rrtalgo;

    std::cout << "Start Benchmarking...\n";

    std::vector<std::string> labels = {"HW5", "HW2 W1", "HW2 W2"};
    std::list<std::vector<double>> data_PathLength;
    std::vector<double> data_Success;
    std::list<std::vector<double>> data_CompTime;
    std::vector<amp::Problem2D> problem_set;
    problem_set.push_back(amp::HW5::getWorkspace1());
    problem_set.push_back(amp::HW2::getWorkspace1());
    problem_set.push_back(amp::HW2::getWorkspace2());
    
    for(int i = 0; i < labels.size(); i++){
        std::cout << "\n === Label === " << labels[i] << "\n";
        double success_count = 0.0;

        std::vector<double> PathLengths;
        std::vector<double> isSuccess;
        std::vector<double> CompTime;
        amp::Problem2D prob = problem_set[i];

        for(int j=0; j<100; j++){
            auto start_time = std::chrono::high_resolution_clock::now();
            
            amp::Path2D path = rrtalgo.plan(prob);

            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end_time - start_time;
            double duration_ms = duration.count();
            double findPath = static_cast<double>(rrtalgo.isSuccess);

            // if(i == 1 && findPath == 1.0 && isShowFigure == false){
            //     std::cout << "[LOG] Path Length: " << rrtalgo.path_length << "\n";
            //     amp::Visualizer::makeFigure(prob, rrtalgo.graph, rrtalgo.node_to_coord);
            //     amp::Visualizer::makeFigure(prob, path);
            //     //amp::Visualizer::showFigures();
            //     isShowFigure = true;
            // }
            // std::cout << " (" << rpmalgo.path_length << ",";
            // std::cout << findPath << ",";
            // std::cout << duration_ms << ")\n";

            success_count = success_count + findPath;
            PathLengths.push_back(rrtalgo.pathLength);
            CompTime.push_back(duration_ms);
        }
        data_PathLength.push_back(PathLengths);
        data_Success.push_back(success_count);
        data_CompTime.push_back(CompTime);
        std::cout << "\nsuccess count: " << success_count << "\n";
    }

    amp::Visualizer::makeBoxPlot(data_PathLength, labels);
    amp::Visualizer::makeBarGraph(data_Success, labels);
    amp::Visualizer::makeBoxPlot(data_CompTime, labels);
    amp::Visualizer::showFigures();
    

}

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    //std::mt19937 rng(42);

    //Problem1();

    BenchMarkPRM();

    //Problem2();

    //BenchMarkRRT();

    // For Grading
    // MyPRM2D prmalgo;
    // prmalgo.init(500, 2.0);
    // MyRRT2D rrtalgo;
    // amp::HW7::grade(prmalgo, rrtalgo, "chko1829@colorado.edu", argc, argv);

    std::cout << "[RUN COMPLETE]\n";
    return 0;
}

// Use random.h to generate random points