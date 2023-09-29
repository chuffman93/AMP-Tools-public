#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <cmath>
using namespace std;

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        // Add any other methods here...
        double step = 0.01;
        double eps = 1e-3;
        double m;
        double b;
        const char *  turn = "left";
        bool debug = true;
    
        struct qH
        {
            int objNum;
            int vertDir;
            bool hit;
            Eigen::Vector2d qh;
        };
    
    private:
        // Add any member variables here...
        Eigen::Vector2d nextStep(Eigen::Vector2d ptA, Eigen::Vector2d ptB, double stepSize);
        qH collisionCheck(vector<amp::Obstacle2D> obstacles, Eigen::Vector2d qL, Eigen::Vector2d qLNext, int currObj);
        double calcDist(Eigen::Vector2d pt, Eigen::Vector2d goal);
        double lineDirection(Eigen::Vector2d ptA, Eigen::Vector2d ptB, Eigen::Vector2d ptC);
        bool collinearAndOverlapping(Eigen::Vector2d ptA1, Eigen::Vector2d ptA2, Eigen::Vector2d ptB1,  Eigen::Vector2d ptB2);
        bool intersectCheck(Eigen::Vector2d ptA1, Eigen::Vector2d ptA2, Eigen::Vector2d ptB1,  Eigen::Vector2d ptB2);
        bool goalOverreach(Eigen::Vector2d ptA1, Eigen::Vector2d ptA2, Eigen::Vector2d goal);
        amp::Path2D objTraverse(vector<amp::Obstacle2D> obstacles, qH travStart, Eigen::Vector2d goal);
        qH followObstacle(Eigen::Vector2d q, amp::Obstacle2D obstacle, int verItr, int objNumber);
        Eigen::Vector2d interCeptPt(Eigen::Vector2d mx1, Eigen::Vector2d mx2);
        Eigen::Vector2d calTravLine(Eigen::Vector2d ptA1, Eigen::Vector2d ptB1, Eigen::Vector2d qL);
        Eigen::Vector2d calLine(Eigen::Vector2d ptA1, Eigen::Vector2d ptB1);

};