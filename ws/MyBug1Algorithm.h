#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"
using namespace std;

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBug1Algorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        double step = 0.01;
        double eps = 0.0001;
        string turn = "left";
    
    private:
        // Add any member variables here...
        amp::Path2D lines(Eigen::Vector2d ptA, Eigen::Vector2d ptB);
        vector<amp::Path2D> obsPaths(const amp::Problem2D& problem);
};