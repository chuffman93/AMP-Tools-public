// I'm going to remove this project eventually, but here is a simple example for how to visualize a potential function

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include "hw/HW5.h"

class MyPotentialFunction : public amp::PotentialFunction2D {
    public:
        MyPotentialFunction(amp::Problem2D& a){
            env = a;
        };
        virtual double operator()(const Eigen::Vector2d& q) const override {
            Eigen::Vector2d objVert= env.obstacles[0].verticesCCW()[0];
            return (q[0] * q[0])*objVert[0] + (q[1] * q[1])*objVert[1];
        }
        amp::Problem2D env;
};

int main(int argc, char** argv) {
    amp::Problem2D env = amp::HW5::getWorkspace1();
    amp::Visualizer::makeFigure(MyPotentialFunction{env}, -10.0, 10.0, -10.0, 10.0);
    amp::Visualizer::showFigures();
    return 0;
}