#include <math.h>
#include "AMPCore.h"
#include <chrono>

using namespace amp;
using namespace std;
using namespace chrono;

class MyPotentialFunction : public PotentialFunction2D {
    public:
        MyPotentialFunction(Problem2D& a){
            env = a;
            obs = env.obstacles;
            nObs = obs.size();
            goal = env.q_goal;

            // Homework 2 WS 1
            if(nObs == 5 && env.q_init.isApprox(Eigen::Vector2d{0,0}),goal.isApprox(Eigen::Vector2d{10,10})){
                qstar = 0.3;
                dStar = 20;
                nu = 0.4;
                xi = 40;
                timeOut = 30.0;
            }
            // Homwork 2 WS 2
            else if(nObs == 9 && env.q_init.isApprox(Eigen::Vector2d{0,0}),goal.isApprox(Eigen::Vector2d{35,0}))
            {
                qstar = 1;
                dStar = 10;
                nu = 0.2;
                xi = 9;
                timeOut = 30.0;
            }
            // Default
            else
            {
                qstar = 0.5;
                dStar = 10;
                nu = 3;
                xi = 5;
                timeOut = 15.0;
            }

            for(int i = 0; i < nObs; i++)
            {
                qStar.push_back(qstar);
            }
        };

        double timeOut;

        virtual double operator()(const Eigen::Vector2d& q) const override; 
        Eigen::Vector2d attractivePotential(Eigen::Vector2d q) const;
        Eigen::Vector2d repulsivePotential(Eigen::Vector2d q) const;
        double distance(Eigen::Vector2d ptA, Eigen::Vector2d ptB) const;
        void addxyGrad(Eigen::Vector2d U,vector<Eigen::Vector2d>& grads) const;
        vector<Eigen::Vector2d> getxyGrad();
        

    private:
        amp::Problem2D env;
        vector<Obstacle2D> obs;
        const vector<Eigen::Vector2d> xyGrad;
        vector<double> qStar;
        Eigen::Vector2d goal;
        int nObs;
        double obLineRes = 0.01;
        double qstar;
        double dStar;
        double nu;
        double xi;
        
};