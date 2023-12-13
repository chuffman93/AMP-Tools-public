#include <queue>
#include <chrono>

#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW8.h"
#include "MyGBRRT.h"
#include "myDecentralizedMultiAgentRRT.h"

#ifndef MYUTILS_H
#include "myUtils.h"
#endif

using namespace amp;
using namespace std;
using namespace chrono;

int main(int argc, char** argv)
{
    // Simple Workspaces
    Problem2D simple1 = HW5::getWorkspace1();
    Problem2D simple2 = HW2::getWorkspace1();
    Problem2D simple3 = HW2::getWorkspace2();
    // 2-Link Manipulator Workspace
    Problem2D linkMan1 = HW6::getHW4Problem1();
    Problem2D linkMan2 = HW6::getHW4Problem2();
    Problem2D linkMan3 = HW6::getHW4Problem3();
    // Multi-Agent Workspace
    MultiAgentProblem2D multi2 = HW8::getWorkspace1(2);
    MultiAgentProblem2D multi3 = HW8::getWorkspace1(3);
    MultiAgentProblem2D multi4 = HW8::getWorkspace1(4);
    MultiAgentProblem2D multi5 = HW8::getWorkspace1(5);
    MultiAgentProblem2D multi6 = HW8::getWorkspace1(6);

    // GBRRT
    {
        MyGBRRT gbrrt;
        // Simple Workspace
        {
            Path2D gbPathSim1 = gbrrt.plan(simple1);
            HW2::check(gbPathSim1,simple1);
            Path2D gbPathSim2 = gbrrt.plan(simple2);
            HW2::check(gbPathSim2,simple2);
            Path2D gbPathSim3 = gbrrt.plan(simple3);
            HW2::check(gbPathSim3,simple3);
        }
        // 2-Link Manipulator Workspace
        {

        }
        // Multi-Agent Workspace
        {

        }
    }

    // RRT*
    {
        // Simple Workspace
        {

        }
        // 2-Link Manipulator Workspace
        {

        }
        // Multi-Agent Workspace
        {
            
        }
    }

    printf("End of Main Project\n");
}