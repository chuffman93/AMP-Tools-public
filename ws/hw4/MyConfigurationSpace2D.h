#pragma once

#include "AMPCore.h"
#include "hw/HW4.h"
#include "tools/ConfigurationSpace.h"
#include <cmath>
using namespace std;
using namespace amp;

class MyConfigurationSpace2D : public ConfigurationSpace2D
{
    public:
        MyConfigurationSpace2D(double x0_min, double x0_max, double x1_min, double x1_max)
        : ConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max) {};

        /// @brief Access the C-space with continuous variables (interpolation between cells)
        /// @param x0 Value of the first configuration variable
        /// @param x1 Value of the second configuration variable
        /// @return `true` if the the point is in collision, `false` if it is not
        virtual bool inCollision(double x0, double x1) const override;

    private:
        

};


