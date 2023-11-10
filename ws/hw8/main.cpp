#include <queue>
#include <chrono>

#include "AMPCore.h"
#include "hw/HW8.h"
#include "myCentralizedMultiAgentRRT.h"
#include "myDecentralizedMultiAgentRRT.h"

#ifndef MYUTILS_H
#include "myUtils.h"
#endif

using namespace amp;
using namespace std;
using namespace chrono;

myUtils hw8U;


int main(int argc, char** argv)
{
    MultiAgentProblem2D prob1 = HW8::getWorkspace1(2);
    // Visualizer::makeFigure(prob1);

    myCentralizedMultiAgentRRT p1;
    myDecentralizedMultiAgentRRT p2;

    double rad;

    MultiAgentPath2D cmaPath = p1.plan(prob1);
    Visualizer::makeFigure(prob1, cmaPath);
    cmaPath = p1.plan(prob1);
    Visualizer::makeFigure(prob1, cmaPath);
    cmaPath = p1.plan(prob1);
    Visualizer::makeFigure(prob1, cmaPath);
    Visualizer::showFigures();

    return 0;
}