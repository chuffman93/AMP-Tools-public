// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBug1Algorithm.h"
#include "MyBug2Algorithm.h"

using namespace amp;

int main(int argc, char** argv) {

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem1 = HW2::getWorkspace1();

    // Use WO1 from Exercise 2
    Problem2D problem2 = HW2::getWorkspace2();


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
    MyBug1Algorithm algo1;
    MyBug2Algorithm algo2;
    
    // Bug 1
    {
        // Call your algorithm on the problem
        amp::Path2D path_1_w1 = algo1.plan(problem1);

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW2::check(path_1_w1, problem1);

        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem1, path_1_w1);

        // // Call your algorithm on the problem
        // amp::Path2D path_1_w2 = algo1.plan(problem2);

        // // Check your path to make sure that it does not collide with the environment 
        // success = HW2::check(path_1_w2, problem2);

        // LOG("Found valid solution to workspace 2: " << (success ? "Yes!" : "No :("));

        // // Visualize the path and environment
        // Visualizer::makeFigure(problem2, path_1_w2);
    }

    // // Bug 2
    // {
    //     // Call your algorithm on the problem
    //     amp::Path2D path_2_w1 = algo2.plan(problem1);

    //     // Check your path to make sure that it does not collide with the environment 
    //     bool success = HW2::check(path_2_w1, problem1);

    //     LOG("Found valid solution to workspace 2: " << (success ? "Yes!" : "No :("));

    //     // Visualize the path and environment
    //     Visualizer::makeFigure(problem1, path_2_w1);

    //     // Call your algorithm on the problem
    //     amp::Path2D path_2_w2 = algo2.plan(problem2);

    //     // Check your path to make sure that it does not collide with the environment 
    //     success = HW2::check(path_2_w2, problem2);

    //     LOG("Found valid solution to workspace 2: " << (success ? "Yes!" : "No :("));

    //     // Visualize the path and environment
    //     Visualizer::makeFigure(problem2, path_2_w2);
    // }

    // // Let's get crazy and generate a random environment and test your algorithm
    // {
    //     amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
    //     amp::Problem2D random_prob; 
    //     std::vector<Eigen::Vector2d> collision_points;
    //     bool random_trial_success = HW2::generateAndCheck(algo2, path, random_prob, collision_points);
    //     LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));

    //     LOG("path length: " << path.length());

    //     // Visualize the path environment, and any collision points with obstacles
    //     Visualizer::makeFigure(random_prob, path, collision_points);
    // }

    Visualizer::showFigures();

    // HW2::grade(algo2, "cohu8717@colorado.edu", argc, argv);

    return 0;
}