#pragma once

#include "hw/HW5.h"

using namespace amp;

/// @brief Gradient Descent planning algorithm. Derives a MotionPlanner2D algorithm with a "point" agent type
class MyGDAlgorithm : public GDAlgorithm {
    public:
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Motion planning problem
        /// @return Path solution of the point agent
        virtual Path2D plan(const Problem2D& problem) override;

};