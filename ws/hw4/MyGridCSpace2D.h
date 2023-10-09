#pragma once

#include <memory>
#include "MyConfigurationSpace2D.h"


class MyGridCSpace2D : public MyConfigurationSpace2D, public DenseArray2D<bool> {
    public:
        MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : MyConfigurationSpace2D(x0_min, x0_max, x1_min, x1_max),
            DenseArray2D(x0_cells, x1_cells)
            {}

};