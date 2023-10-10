#pragma once

#include <memory>
#include "tools/ConfigurationSpace.h"

using namespace std;
using namespace amp;


class MyGridCSpace2D : public GridCSpace2D{
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max)
            { }
            
    
        virtual bool inCollision(double x0, double x1) const override;

};