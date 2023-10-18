#pragma once

#include <memory>
#include "tools/ConfigurationSpace.h"

using namespace std;
using namespace amp;


class MyGridCSpace : public GridCSpace2D {
    public:
        //MyGridCSpace()
        //    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {
            return {0, 0};
        }
};