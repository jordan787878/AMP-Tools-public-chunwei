// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem = HW2::getWorkspace1();

    // Use WO1 from Exercise 2
    //Problem2D problem = HW2::getWorkspace2();

    // Make a random environment spec, edit properties about it such as the number of obstacles
    /*
    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 5.0;
    spec.n_obstacles = 2;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.01;

    //Randomly generate the environment;
    Problem2D problem = EnvironmentTools::generateRandom(spec); // Random environment
    */

    // Declare your algorithm object 
    MyBugAlgorithm algo;
    // Choose Bug1 (1) or Bug2 (2)
    int bug1orbug2 = 2;
    algo.bu1orbug2 = bug1orbug2;
    {
        amp::Path2D path = algo.plan(problem);

        bool success = HW2::check(path, problem);
        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));
        Visualizer::makeFigure(problem, path);
    }

    algo.bu1orbug2 = bug1orbug2;
    // Let's get crazy and generate a random environment and test your algorithm
    {
        amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
        amp::Problem2D random_prob; 
        std::vector<Eigen::Vector2d> collision_points;
        bool random_trial_success = HW2::generateAndCheck(algo, path, random_prob, collision_points);
        LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));

        LOG("path length: " << path.length());

        // Visualize the path environment, and any collision points with obstacles
        Visualizer::makeFigure(random_prob, path, collision_points);
    }

    Visualizer::showFigures();

    //HW2::grade(algo, "Chun-Wei.Kong@colorado.edu", argc, argv);
    
    /* If you want to reconstruct your bug algorithm object every trial (to reset member variables from scratch or initialize), use this method instead*/
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, constructor_parameter_1, constructor_parameter_2, etc...);
    
    // This will reconstruct using the default constructor every trial
    //HW2::grade<MyBugAlgorithm>("Chun-Wei.Kong@colorado.edu", argc, argv);

    return 0;
}