#pragma once

#include <memory>
#include "tools/ConfigurationSpace.h"

using namespace std;
using namespace amp;


class MyGridCSpace : public amp::GridCSpace2D {
    public:
        //MyGridCSpace()
        //    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
        MyGridCSpace(double a, std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {
                xMin = x0_min;
                yMin = x1_min;
                dis = a;
            }

        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {
                xMin = x0_min;
                yMin = x1_min;
                dis = 0.25;
            }

        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {
            return make_pair(floor( (x0 - this->xMin)/this->dis ), floor( (x1 - this->yMin)/this->dis ));
        }
        double dis;
        double xMin;
        double yMin;
};