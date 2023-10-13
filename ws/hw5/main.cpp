// I'm going to remove this project eventually, but here is a simple example for how to visualize a potential function

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "MyGDAlgorithm.h"
#include "MyPotentialFunction.h"
#include "hw/HW2.h"


int main(int argc, char** argv) {
    MyGDAlgorithm planner;
    bool graded = true;
    if(!graded)
    {
        Problem2D env1 = HW5::getWorkspace1();
        Path2D e1Path = planner.plan(env1);
        printf("Path length of %.2f\n", e1Path.length());
        bool e1 = HW5::check(e1Path, env1, true);

        Problem2D env2 = HW2::getWorkspace1();
        Path2D e2Path = planner.plan(env2);
        printf("Path length of %.2f\n", e2Path.length());
        bool e2 = HW5::check(e2Path, env2, true);

        Problem2D env3 = HW2::getWorkspace2();
        printf("Building Potential Field\n");
        amp::Visualizer::makeFigure(MyPotentialFunction{env3}, -6, 36, -6, 6);
        printf("Getting Path\n");
        Path2D e3Path = planner.plan(env3);
        printf("Making Figure with length of %.2f\n", e3Path.length());
        amp::Visualizer::makeFigure(env3, e3Path);
        bool e3 = HW5::check(e3Path, env3, true);

        bool ranEnv = HW5::generateAndCheck(planner, true, 0U);

        printf("Env1 %s\nEnv2 %s\nEnv3 %s\nRanEnv %s\n",
                e1 ? "Passed" : "Failed",
                e2 ? "Passed" : "Failed",
                e3 ? "Passed" : "Failed",
                ranEnv ? "Passed" : "Failed");
        sleep(10);
        amp::Visualizer::showFigures();
    }
    else
    {
        HW5::grade(planner, "corey.huffman@colorado.edu", argc, argv);
    }

    return 0;
}