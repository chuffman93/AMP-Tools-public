#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
#include <cmath>
using namespace std;


/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBug1Algorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        double step = 0.01;
        double buffer = 0.01;
        double eps = 1e-3;
        double m;
        double b;
        string turn = "left";
    
        struct qH
        {
            int objNum;
            int hitItr;
            bool hit;
            Eigen::Vector2d qh;
        };
    private:
        // Add any member variables here...
        amp::Path2D lines(Eigen::Vector2d ptA, Eigen::Vector2d ptB);
        vector<amp::Path2D> obsPaths(const amp::Problem2D& problem);
        qH collisionCheck(int numObs, vector<amp::Path2D> obsPaths, Eigen::Vector2d qL);
        qH collisionCheck(int numObs, vector<amp::Path2D> obsPaths, Eigen::Vector2d qL, int currObj);
        Eigen::Vector2d nextStep(Eigen::Vector2d ptA, Eigen::Vector2d ptB);
        amp::Path2D objTraverse(int numObs, vector<amp::Path2D> obsPaths, qH travStart, Eigen::Vector2d goal);
        double calcDist(Eigen::Vector2d pt, Eigen::Vector2d goal);
        Eigen::Vector2d objBuff(Eigen::Vector2d ptA, Eigen::Vector2d ptB, Eigen::Vector2d ptC);
        vector<Eigen::Vector2d> expandObstacle(vector<Eigen::Vector2d> verts);

};