#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"
#include "hw/HW4.h"

#include "MyPointWFAlgo.h"
#include "MyManipWFAlgo.h"
#include "MyAStarAlgo.h"

#include "MyLinkManipulator2D.h"
#include "MyGridCSpace.h"

void Exe1(){
    amp::Problem2D problem = amp::HW2::getWorkspace1();
    amp::Environment2D env;
    env.x_max = problem.x_max; env.x_min = problem.x_min; env.y_max = problem.y_max; env.y_min = problem.y_min;
    env.obstacles = problem.obstacles;

    MyPointWFAlgo algo;
    std::unique_ptr<amp::GridCSpace2D> gridSpace = algo.constructDiscretizedWorkspace(env);
    amp::Path2D path = algo.planInCSpace(problem.q_init, problem.q_goal, *gridSpace);
    amp::HW6::checkPointAgentPlan(path, problem);
}

void Exe2(){
    // Define CSpace (S x S)
    int Ncells = 30;
    double x0_min = 0; double x0_max = 2*3.141;
    double x1_min = 0; double x1_max = 2*3.141;
    
    // WorkSpace (getEx3Workspace <1,2,3>)
    amp::Environment2D wspace = amp::HW4::getEx3Workspace3();

    std::vector<double> link_lengths = {1.0, 1.0};
    Eigen::Vector2d base_location = {0.0, 0.0};
    MyLinkManipulator2D mylink(base_location, link_lengths);

    // Initial Configuration
    Eigen::Vector2d end_effector_i = {-2.0, 0.0};
    amp::ManipulatorState state_i = mylink.getConfigurationFromIK(end_effector_i);
    // Final Configuration
    Eigen::Vector2d end_effector_f = {2.0, 0.0};
    amp::ManipulatorState state_f = mylink.getConfigurationFromIK(end_effector_f);
    
    // Visualization
    amp::ManipulatorTrajectory2Link state_traj;
    state_traj.waypoints.push_back({state_i[0], state_i[1]});
    state_traj.waypoints.push_back({state_f[0], state_f[1]});
    amp::Problem2D problem;
    problem.q_init = Eigen::Vector2d{state_i[0], state_i[1]};
    problem.q_goal = Eigen::Vector2d{state_f[0], state_f[1]};
    problem.obstacles = wspace.obstacles;
    problem.x_max = wspace.x_max; problem.x_min = wspace.x_min;
    problem.y_max = wspace.y_max; problem.y_min = wspace.y_min;
    //amp::Visualizer::makeFigure(problem, mylink, state_traj);
    //amp::Visualizer::showFigures();
    
    // CSpaceObstacle
    MyGridCSpace grid(Ncells, Ncells, x0_min, x0_max, x1_min, x1_max,
                      mylink, wspace);
    
    // plan in Cspace
    MyManipWFAlgo wfalgo;
    amp::Path2D path = wfalgo.planInCSpace(problem.q_init, problem.q_goal, grid);

    // Visualize Workspace
    amp::Visualizer::makeFigure(problem, mylink, path);
    amp::Visualizer::showFigures();
    amp::HW6::checkLinkManipulatorPlan(path, mylink, problem);
}

void Exe3(){
    // Set up
    amp::LookupSearchHeuristic heuristic = amp::HW6::getEx3Heuristic();
    amp::ShortestPathProblem graphproblem = amp::HW6::getEx3SPP();

    // Init Aglo
    MyAStarAlgo algo;
    MyAStarAlgo::GraphSearchResult result = algo.search(graphproblem, heuristic);
    amp::HW6::checkGraphSearchResult(result, graphproblem);
    amp::HW6::generateAndCheck(algo);
}

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());

    //Exe1();

    //Exe2();
    
    Exe3();

    // HW6 grading
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, 
    // std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    std::cout << "run complete";
    return 0;
}