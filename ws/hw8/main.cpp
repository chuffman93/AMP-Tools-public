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

    vector<string> labs{"1 Agent", "2 Agents", "3 Agents", "4 Agents", "5 Agents", "6 Agents"};


    double rad;

    // vector<Eigen::Vector2d> wo1{Eigen::Vector2d{3.5, 5.5}, Eigen::Vector2d{6.5, 5.5}, Eigen::Vector2d{6.5, 7.5}, Eigen::Vector2d{3.5, 7.5}};
    // vector<Eigen::Vector2d> wo2{Eigen::Vector2d{3.5, 5.5}, Eigen::Vector2d{5.5, 5.5}, Eigen::Vector2d{5.5, 10.5}, Eigen::Vector2d{3.5, 10.5}};
    // vector<Eigen::Vector2d> wo3{Eigen::Vector2d{3.5, 8.5}, Eigen::Vector2d{6.5, 8.5}, Eigen::Vector2d{6.5, 10.5}, Eigen::Vector2d{3.5, 10.5}};

    // vector<Eigen::Vector2d> wo4{Eigen::Vector2d{9.5, 5.5}, Eigen::Vector2d{12.5, 5.5}, Eigen::Vector2d{12.5, 7.5}, Eigen::Vector2d{9.5, 7.5}};
    // vector<Eigen::Vector2d> wo5{Eigen::Vector2d{10.5, 5.5}, Eigen::Vector2d{12.5, 5.5}, Eigen::Vector2d{12.5, 10.5}, Eigen::Vector2d{10.5, 10.5}};
    // vector<Eigen::Vector2d> wo6{Eigen::Vector2d{9.5, 8.5}, Eigen::Vector2d{12.5, 8.5}, Eigen::Vector2d{12.5, 10.5}, Eigen::Vector2d{9.5, 10.5}};

    // Visualizer::makeFigure(vector<Polygon>{Polygon(wo1), Polygon(wo2), Polygon(wo3), Polygon(wo4), Polygon(wo5), Polygon(wo6)});
    // Visualizer::showFigures();

    // MultiAgentPath2D cmaPath = p1.plan(prob1);
    // myCentralizedMultiAgentRRT::benchRes a1 = p1.benchMarkAlgo(100, HW8::getWorkspace1(1));
    // myCentralizedMultiAgentRRT::benchRes a2 = p1.benchMarkAlgo(100, HW8::getWorkspace1(2));
    // myCentralizedMultiAgentRRT::benchRes a3 = p1.benchMarkAlgo(100, HW8::getWorkspace1(3));
    // myCentralizedMultiAgentRRT::benchRes a4 = p1.benchMarkAlgo(100, HW8::getWorkspace1(4));
    // myCentralizedMultiAgentRRT::benchRes a5 = p1.benchMarkAlgo(100, HW8::getWorkspace1(5));
    // myCentralizedMultiAgentRRT::benchRes a6 = p1.benchMarkAlgo(100, HW8::getWorkspace1(6));
    // Visualizer::showFigures();
    // vector<double> cmaTime{a1.at, a2.at, a3.at, a4.at, a5.at, a6.at};
    // vector<double> cmaSize{a1.as, a2.as, a3.as, a4.as, a5.as, a6.as};
    // Visualizer::makeBarGraph(cmaTime, labs, "Average Time vs # Agents for CMA", "Number of Agents", "Average Time[ms]");
    // Visualizer::makeBarGraph(cmaSize, labs, "Average Tree Size vs # Agents for CMA", "Number of Agents", "Average Tree Size [Nodes]");
    // Visualizer::makeFigure(prob1,cmaPath);
    // Visualizer::showFigures();

    // MultiAgentPath2D dmaPath = p2.plan(prob1);
    // Visualizer::makeFigure(prob1, dmaPath);
    // double b1 = p2.benchMarkAlgo(100, HW8::getWorkspace1(1));
    // double b2 = p2.benchMarkAlgo(100, HW8::getWorkspace1(2));
    // double b3 = p2.benchMarkAlgo(100, HW8::getWorkspace1(3));
    // double b4 = p2.benchMarkAlgo(100, HW8::getWorkspace1(4));
    // double b5 = p2.benchMarkAlgo(100, HW8::getWorkspace1(5));
    // double b6 = p2.benchMarkAlgo(100, HW8::getWorkspace1(6));
    // vector<double> dmaTime{b1, b2, b3, b4, b5, b6};
    // Visualizer::makeBarGraph(dmaTime, labs, "Average Time vs # Agents for DMA", "Number of Agents", "Average Time[ms]");
    // Visualizer::showFigures();

    // p1.setSampleSize(30000);

    HW8::grade(p1, p2,"corey.huffman@colorado.edu", argc, argv);

    return 0;
}