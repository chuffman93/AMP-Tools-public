#include <queue>
#include <chrono>

#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW6.h"
#include "hw/HW8.h"

#include "MyLinkManipulator.h"
#include "MyGBRRT.h"
#include "MyRRTStar.h"
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
    list<Problem2D> simpProbs{simple1, simple2, simple3};
    vector<string> wsSp{"W1 HW5", "W1 HW2", "W2 HW2"};
    // 2-Link Manipulator Workspace
    MyLinkManipulator man(vector<double>{1.0, 1.0});
    Problem2D linkMan1 = HW6::getHW4Problem1();
    Problem2D linkMan2 = HW6::getHW4Problem2();
    Problem2D linkMan3 = HW6::getHW4Problem3();
    list<Problem2D> linkProbs{linkMan1, linkMan2, linkMan3};
    vector<string> wsLm{"W1 HW6", "W2 HW6", "W3 HW6"};
    // Multi-Agent Workspace
    MultiAgentProblem2D multi2 = HW8::getWorkspace1(2);
    MultiAgentProblem2D multi3 = HW8::getWorkspace1(3);
    MultiAgentProblem2D multi4 = HW8::getWorkspace1(4);
    MultiAgentProblem2D multi5 = HW8::getWorkspace1(5);
    MultiAgentProblem2D multi6 = HW8::getWorkspace1(6);
    list<MultiAgentProblem2D> mAProbs{multi2, multi3, multi4, multi5, multi6};
    vector<string> wsMa{"MA2", "MA3", "MA4", "MA5", "MA6"};

    bool TODO = true;
    tuple<double, vector<double>, vector<double>> tmpPair;
    list<vector<double>> maTimes;
    list<vector<double>> maDist;

    // GBRRT
    if(!TODO){
        MyGBRRT gbrrt;
        myDecentralizedMultiAgentRRT gbrrtMA;
        // Simple Workspace
        if(!TODO){
            Path2D gbPathSim1 = gbrrt.plan(simple1);
            HW2::check(gbPathSim1,simple1);
            Visualizer::makeFigure(simple1,gbPathSim1);

            Path2D gbPathSim2 = gbrrt.plan(simple2);
            HW2::check(gbPathSim2,simple2);
            Visualizer::makeFigure(simple2,gbPathSim2);            
            
            Path2D gbPathSim3 = gbrrt.plan(simple3);
            HW2::check(gbPathSim3,simple3);
            Visualizer::makeFigure(simple3,gbPathSim3);

            Visualizer::showFigures();

        }
        // 2-Link Manipulator Workspace
        if(!TODO){
            Path2D lPath1 = gbrrt.plan(man, linkMan1);
            HW6::checkLinkManipulatorPlan(lPath1, man, linkMan1);
            GridCSpace2D * lp1 = gbrrt.construct(man, linkMan1).release();
            Visualizer::makeFigure(linkMan1, man, lPath1);
            Visualizer::makeFigure(*lp1, lPath1);

            Path2D lPath2 = gbrrt.plan(man, linkMan2);
            HW6::checkLinkManipulatorPlan(lPath2, man, linkMan2);
            GridCSpace2D * lp2 = gbrrt.construct(man, linkMan2).release();
            Visualizer::makeFigure(linkMan2, man, lPath2);
            Visualizer::makeFigure(*lp2, lPath2);

            Path2D lPath3 = gbrrt.plan(man, linkMan3);
            HW6::checkLinkManipulatorPlan(lPath3, man, linkMan3);
            GridCSpace2D * lp3 = gbrrt.construct(man, linkMan3).release();
            Visualizer::makeFigure(linkMan3, man, lPath3);
            Visualizer::makeFigure(*lp3, lPath3);

            Visualizer::showFigures();
        }
        // Multi-Agent Workspace
        if(!TODO){
            MultiAgentPath2D gbma2 = gbrrtMA.plan(multi2);
            HW8::check(gbma2, multi2);
            Visualizer::makeFigure(multi2,gbma2);

            MultiAgentPath2D gbma3 = gbrrtMA.plan(multi3);
            HW8::check(gbma3, multi3);
            Visualizer::makeFigure(multi3,gbma3);

            MultiAgentPath2D gbma4 = gbrrtMA.plan(multi4);
            HW8::check(gbma4, multi4);
            Visualizer::makeFigure(multi4,gbma4);

            MultiAgentPath2D gbma5 = gbrrtMA.plan(multi5);
            HW8::check(gbma5, multi5);
            Visualizer::makeFigure(multi5,gbma5);

            MultiAgentPath2D gbma6 = gbrrtMA.plan(multi6);
            HW8::check(gbma6, multi6);
            Visualizer::makeFigure(multi6,gbma6);
            

            Visualizer::showFigures();
        }
        // Benchmarking
        if(!TODO){
            gbrrt.benchMarkAlgo(200, false, simpProbs, wsSp);
            gbrrt.benchMarkAlgo(200, false, linkProbs, man, wsLm);
            maTimes.clear();
            maDist.clear();
            for(auto prob : mAProbs)
            {
                tmpPair = gbrrtMA.benchMarkAlgo(200, prob);
                maTimes.push_back(get<1>(tmpPair));
                maDist.push_back(get<2>(tmpPair));
            }
            Visualizer::makeBoxPlot(maTimes, wsMa, "Computation Time for Goal Biased RRT", "Environments", "Time [ms]");
            Visualizer::makeBoxPlot(maDist, wsMa, "Total Distance for Goal Biased RRT", "Environments", "Distance [Units]");
            Visualizer::showFigures();
        }
    }

    // RRT*
    if(TODO){
        MyRRTStar rrt;
        myDecentralizedMultiAgentRRT rrtMA;
        // Simple Workspace
        if(!TODO){
            Path2D rrtPathSim1 = rrt.plan(simple1);
            HW2::check(rrtPathSim1,simple1);
            Visualizer::makeFigure(simple1,rrtPathSim1);

            Path2D rrtPathSim2 = rrt.plan(simple2);
            HW2::check(rrtPathSim2,simple2);
            Visualizer::makeFigure(simple2,rrtPathSim2);            
            
            Path2D rrtPathSim3 = rrt.plan(simple3);
            HW2::check(rrtPathSim3,simple3);
            Visualizer::makeFigure(simple3,rrtPathSim3);

            Visualizer::showFigures();
        }
        // 2-Link Manipulator Workspace
        if(!TODO){
            Path2D Path1 = rrt.plan(man, linkMan1);
            HW6::checkLinkManipulatorPlan(Path1, man, linkMan1);
            GridCSpace2D * p1 = rrt.construct(man, linkMan1).release();
            Visualizer::makeFigure(linkMan1, man, Path1);
            Visualizer::makeFigure(*p1, Path1);


            Visualizer::showFigures();
        }
        // Multi-Agent Workspace
        if(!TODO){
            MultiAgentPath2D ma2 = rrtMA.planStar(multi2);
            HW8::check(ma2, multi2);
            Visualizer::makeFigure(multi2,ma2);

            MultiAgentPath2D ma3 = rrtMA.planStar(multi3);
            HW8::check(ma3, multi3);
            Visualizer::makeFigure(multi3,ma3);

            MultiAgentPath2D ma4 = rrtMA.planStar(multi4);
            HW8::check(ma4, multi4);
            Visualizer::makeFigure(multi4,ma4);

            MultiAgentPath2D ma5 = rrtMA.planStar(multi5);
            HW8::check(ma5, multi5);
            Visualizer::makeFigure(multi5,ma5);

            MultiAgentPath2D ma6 = rrtMA.planStar(multi6);
            HW8::check(ma6, multi6);
            Visualizer::makeFigure(multi6,ma6);

            
            Visualizer::showFigures();
        }
        // Benchmarking
        if(TODO){
            // rrt.benchMarkAlgo(200, false, simpProbs, wsSp);
            
            // maTimes.clear();
            // maDist.clear();
            // for(auto prob : mAProbs)
            // {
            //     tmpPair = rrtMA.benchMarkAlgoSt(200, prob);
            //     maTimes.push_back(get<1>(tmpPair));
            //     maDist.push_back(get<2>(tmpPair));
            // }
            // Visualizer::makeBoxPlot(maTimes, wsMa, "Computation Time for RRT*", "Environments", "Time [ms]");
            // Visualizer::makeBoxPlot(maDist, wsMa, "Total Distance for RRT*", "Environments", "Distance [Units]");
            // Visualizer::showFigures();
        }
    }

    printf("End of Main Project\n");
}