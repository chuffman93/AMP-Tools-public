#include "AMPCore.h"

using namespace amp;

class MyPotentialFunction : public PotentialFunction2D {
    public:
        virtual double operator()(const Eigen::Vector2d& q) const override; 
        double attractivePotential(Eigen::Vector2d q, Eigen::Vector2d goal);
        double repulsivePotential(Eigen::Vector2d q, Eigen::Vector2d objPt);
};