// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBug1Algorithm.h"
#include "MyBug2Algorithm.h"

using namespace amp;

int main(int argc, char** argv) {
    /*    Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());

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
        
        // // Call your algorithm on the problem
        // amp::Path2D path_1_w1 = algo1.plan(problem1);

        // // Check your path to make sure that it does not collide with the environment 
        // bool success = HW2::check(path_1_w1, problem1);

        // LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));
        // LOG("path length: " << path_1_w1.length()); // 104.634

        // // Visualize the path and environment
        // Visualizer::makeFigure(problem1, path_1_w1);

        // // Call your algorithm on the problem
        // amp::Path2D path_1_w2 = algo1.plan(problem2);

        // // Check your path to make sure that it does not collide with the environment 
        // success = HW2::check(path_1_w2, problem2);

        // LOG("Found valid solution to workspace 2: " << (success ? "Yes!" : "No :("));
        // LOG("path length: " << path_1_w2.length()); // 315.04

        // // Visualize the path and environment
        //  Visualizer::makeFigure(problem2, path_1_w2);
    }

    // Bug 2
    {
        // // Call your algorithm on the problem
        // amp::Path2D path_2_w1 = algo2.plan(problem1);

        // // Check your path to make sure that it does not collide with the environment 
        // bool success = HW2::check(path_2_w1, problem1);

        // LOG("Found valid solution to workspace 2: " << (success ? "Yes!" : "No :("));
        // LOG("path length: " << path_2_w1.length()); // 52,5887


        // // Visualize the path and environment
        // Visualizer::makeFigure(problem1, path_2_w1);

        // // Call your algorithm on the problem
        // amp::Path2D path_2_w2 = algo2.plan(problem2);

        // // Check your path to make sure that it does not collide with the environment 
        // success = HW2::check(path_2_w2, problem2);

        // LOG("Found valid solution to workspace 2: " << (success ? "Yes!" : "No :("));
        // LOG("path length: " << path_2_w2.length()); // 422.92


        // // Visualize the path and environment
        // Visualizer::makeFigure(problem2, path_2_w2);
    }

    // Let's get crazy and generate a random environment and test your algorithm
    {
        amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
        amp::Problem2D random_prob; 
        std::vector<Eigen::Vector2d> collision_points;
        bool random_trial_success = HW2::generateAndCheck(algo1, path, random_prob, collision_points);
        LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));

        LOG("path length: " << path.length()); // 17.9824 ~ 15.9129

        // Visualize the path environment, and any collision points with obstacles
        Visualizer::makeFigure(random_prob, path, collision_points);
    }

    Visualizer::showFigures();

    // HW2::grade(algo1, "cohu8717@colorado.edu", argc, argv);
    
    /* If you want to reconstruct your bug algorithm object every trial (to reset member variables from scratch or initialize), use this method instead*/
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv, constructor_parameter_1, constructor_parameter_2, etc...);
    
    // This will reconstruct using the default constructor every trial
    //HW2::grade<MyBugAlgorithm>("nonhuman.biologic@myspace.edu", argc, argv);

    return 0;
}