#include "MyPotentialFunction.h"

double MyPotentialFunction::operator()(const Eigen::Vector2d& q) const
{
    double ret;
    Eigen::Vector2d Uatt = attractivePotential(q);
    Eigen::Vector2d Urep = repulsivePotential(q);
    Eigen::Vector2d U = Uatt + Urep;
    // printf("Uatt (%.2f, %.2f) Urep (%.2f, %.2f) ", Uatt[0], Uatt[1], Urep[0], Urep[1]);
    const_cast<vector<Eigen::Vector2d>&>(xyGrad).push_back(U); 
    ret = sqrt(pow(U[0],2) + pow(U[1],2));
    return ret;
} 

Eigen::Vector2d MyPotentialFunction::attractivePotential(Eigen::Vector2d q) const
{
    Eigen::Vector2d ret;
    double dist = distance(q,goal);
    if (dist <= dStar)
    {
        // printf("Closing in on Goal\n");
        ret[0] = xi*(q[0] - goal[0]);
        ret[1] = xi*(q[1] - goal[1]);
    }
    else
    {
        ret[0] = (dStar*xi*(q[0] - goal[0]))/dist;
        ret[1] = (dStar*xi*(q[1] - goal[1]))/dist;
    }
    return ret;
}

Eigen::Vector2d MyPotentialFunction::repulsivePotential(Eigen::Vector2d q) const
{
    Eigen::Vector2d ret = {0.0,0.0};
    vector<Eigen::Vector2d> obverts;
    double m;
    double b;
    double dis;
    double minDis = 10000;
    bool negX;
    bool negY;
    Eigen::Vector2d stpVec;
    Eigen::Vector2d stVec;
    Eigen::Vector2d tmpPt;
    Eigen::Vector2d compPt;
    for(int i = 0; i < obs.size(); i++)
    {
        obverts = obs[i].verticesCCW();
        obverts.push_back(obverts[0]);
        // printf("Object %d out of %ld\n", i+1, obs.size());
        for(int j = 0; j < obverts.size()-1; j++)
        {
            stVec = obverts[j];
            stpVec = obverts[j+1];
            negX = stVec[0] > stpVec[0];
            negY = stVec[1] > stpVec[1];
            m = (stpVec[1] - stVec[1]) / (stpVec[0] - stVec[0]);
            b = (stpVec[1] - (m*stpVec[0]));
            compPt = stVec;
            tmpPt = stVec;
            // printf("Start Vrt (%.2f, %.2f), Stop Vrt (%.2f, %.2f) with m = %.2f and b = %.2f\n", stVec[0], stVec[1], stpVec[0], stpVec[1], m, b);
            // Avoid Point overshoot
            bool timeout = false;
            auto stTime = high_resolution_clock::now();
            auto stpTime = high_resolution_clock::now();
            while(!(tmpPt.isApprox(stpVec,1e-2)) && !timeout)
            {
                dis = distance(q,tmpPt);
    
                if(dis < minDis)
                {
                    minDis = dis;
                    compPt = tmpPt;
                }
                if(m == INFINITY)
                {
                    if(negY)
                    {
                        tmpPt[1] -= obLineRes;
                    }
                    else
                    {
                        tmpPt[1] += obLineRes;
                    }
                }
                else if(abs(m) < 1e-3)
                {
                    if(negX)
                    {
                        tmpPt[0] -= obLineRes;
                    }
                    else
                    {
                        tmpPt[0] += obLineRes;
                    }
                }
                else
                {
                    double rad = atan(m);
                    double xStep = obLineRes*cos(rad);
                    double yStep = obLineRes*sin(rad);
                    if((xStep > 0 && negX) || (xStep < 0 && !negX))
                    {
                        tmpPt[0] -= xStep;
                    }
                    else
                    {
                        tmpPt[0] += xStep;
                    }

                    if((yStep > 0 && negY) || (yStep < 0 && !negY))
                    {
                        tmpPt[1] -= yStep;
                    }
                    else
                    {
                        tmpPt[1] += yStep;
                    }  
                }
                stpTime = high_resolution_clock::now();
                if(duration_cast<seconds>(stpTime-stTime).count() > 2.0)
                {
                    printf("Algorithm Timed out!\n");
                    timeout = true;
                }
            }
        }
        // printf("Comp Pt (%.2f, %.2f), Stop Vrt (%.2f, %.2f)\n", compPt[0], compPt[1], stpVec[0], stpVec[1]);
        if(minDis <= qStar[i])
        {
            ret[0] += nu*( -1*(1/(minDis+0.1)) + (1/qStar[i])) * (((q[0] - compPt[0])/(minDis)) * 1/pow((minDis+0.1),2)); 
            ret[1] += nu*( -1*(1/(minDis+0.1)) + (1/qStar[i])) * (((q[1] - compPt[1])/(minDis)) * 1/pow((minDis+0.1),2));
        }
    }
    return ret;
}

double MyPotentialFunction::distance(Eigen::Vector2d ptA, Eigen::Vector2d ptB) const
{
    return sqrt(pow(ptB[0]-ptA[0],2) + pow(ptB[1]-ptA[1],2));
}

vector<Eigen::Vector2d> MyPotentialFunction::getxyGrad()
{
    return xyGrad;
}