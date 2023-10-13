#include "MyGDAlgorithm.h"
#include "MyPotentialFunction.h"

Path2D MyGDAlgorithm::plan(const Problem2D& problem)
{
    Path2D ret;
    Problem2D env = problem;
    MyPotentialFunction pf{env};
    Eigen::Vector2d q = problem.q_init;
    Eigen::Vector2d qNext;
    double alph = 0.01;
    Eigen::Vector2d grad;
    ret.waypoints.push_back(q);
    double gd = pf(q);
    // printf("Beginning Planning!!\n");
    bool timeout = false;

    auto stTime = high_resolution_clock::now();
    auto stpTime = high_resolution_clock::now();

    while(!(gd <= eps) && !timeout)
    {
        // printf("q = (%.2f, %.2f) gd = %.2f\n",q[0], q[1], gd);
        grad = pf.getxyGrad().back();
        if(sqrt(pow(grad[0],2) + pow(grad[1],2)) > 25.0)
        {
            alph = 0.0001;
        }
        else if(sqrt(pow(grad[0],2) + pow(grad[1],2)) < 0.5)
        {
            alph = 0.001;
        }
        else if(env.obstacles.size() > 3)
        {
            alph = 0.01;
        }
        else 
        {
            alph = 0.1;
        }
        // sleep(0.7);
        qNext = q - alph*grad;
        q = qNext;
        ret.waypoints.push_back(q);
        gd = pf(q);
        stpTime = high_resolution_clock::now();
        if(duration_cast<seconds>(stpTime-stTime).count() > pf.timeOut)
        {
            // printf("Algorithm Timed out!\n");
            timeout = true;
        }
        
    }
    ret.waypoints.push_back(problem.q_goal);
    
    return ret;
}