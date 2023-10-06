// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"
#include <cmath>
// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

using namespace amp;
using namespace std;

struct linkerState
{
    vector<double> lengths;
    vector<double> angles;
    int numLink;
    Eigen::Vector2d endPoint;
    vector<Eigen::Matrix4d> jointPts;
};

Eigen::Matrix4d rotateZ(double ang)
{
    Eigen::Matrix4d ret;
    ret << cos(ang), -sin(ang), 0, 0,
           sin(ang), cos(ang), 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    return ret;
}

Eigen::Matrix4d translate(double dx, double dy, double dz)
{
    Eigen::Matrix4d ret;
    ret << 1, 0, 0, dx,
           0, 1, 0, dy,
           0, 0, 1, dz,
           0, 0, 0, 1;
    return ret;
}

linkerState FK(vector<double> lengths, vector<double> angles)
{
    linkerState newState;
    newState.lengths = lengths;
    newState.angles = angles;
    newState.numLink = lengths.size();
    vector<Eigen::Matrix4d> P;
    P.push_back(Eigen::Matrix4d::Identity()); 
    Eigen::Matrix4d R;
    Eigen::Matrix4d T;
    for(int i = 0; i < newState.numLink; i++)
    {
        R = rotateZ(angles[i]);
        T = translate(lengths[i], 0, 0);
        P.push_back(P.back()*R*T);
    }
    newState.jointPts = P;
    return newState;
}

linkerState IK(vector<double> lengths, Eigen::Vector2d target)
{
    int max_itr = 1000;
    double err = 1e-12;

    bool solved = false;

    Eigen::Vector2d endToTarget;
    double errEndToTarget;
    Eigen::Vector2d currToEnd;
    double errCurrToEnd;
    Eigen::Vector2d currtoTarget;
    double currToTargetMag;
    double endTargetMag;
    double cosRotAng;
    double sinRotAng;
    double rotAng;

    linkerState curr;
    linkerState ret;

    vector<double> Ang{0.0,0.0,0.0};


    while(!solved && max_itr > 0)
    {
        for(int i = lengths.size()-1; i >= 0; i--)
        {
            curr = FK(lengths, Ang);
            endToTarget = target - Eigen::Vector2d{curr.jointPts.back().col(3)(0),curr.jointPts.back().col(3)(1)};
            errEndToTarget = sqrt(pow(endToTarget[0],2) + pow(endToTarget[1],2));
            if(errEndToTarget < err)
            {
                solved = true;
            }
            else
            {
                currToEnd = Eigen::Vector2d{curr.jointPts.back().col(3)(0),curr.jointPts.back().col(3)(1)} - 
                            Eigen::Vector2d{curr.jointPts[i].col(3)(0),curr.jointPts[i].col(3)(1)};
                errCurrToEnd = sqrt(pow(currToEnd[0],2) + pow(currToEnd[1],2));
                currtoTarget = target - Eigen::Vector2d{curr.jointPts[i].col(3)(0),curr.jointPts[i].col(3)(1)};
                currToTargetMag = sqrt(pow(currtoTarget[0],2) + pow(currtoTarget[1],2));
                
                endTargetMag = errCurrToEnd*currToTargetMag;

                if(endTargetMag <= 0.0001)
                {
                    cosRotAng = 1.0;
                    sinRotAng = 0.0;
                }
                else
                {
                    cosRotAng = (currToEnd[0]*currtoTarget[0] + currToEnd[1]*currtoTarget[1])/endTargetMag;
                    sinRotAng = (currToEnd[0]*currtoTarget[1] - currToEnd[1]*currtoTarget[0])/endTargetMag;
                }

                rotAng = acos(max(-1.0, min(1.0,cosRotAng)));

                if(sinRotAng < 0.0)
                {
                    rotAng = -rotAng;
                }

                Ang[i] = Ang[i] + rotAng;

                if(Ang[i] >= 2.0*M_PI)
                {
                    Ang[i] = Ang[i] -(2.0*M_PI);
                }
                if(Ang[i] < 0.0)
                {
                    Ang[i] = (2.0*M_PI) + Ang[i];
                }
            }
        }
        ret = curr;
        ret.angles = Ang;
        curr.angles = Ang;
        printf("New angles are %.2f, %.2f, and %.2f\n", Ang[0]*180/M_PI, Ang[1]*180/M_PI, Ang[2]*180/M_PI);
        max_itr -= 1; 
        if(solved)
        {
            break;
        }
    }
    return ret;
}



int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    HW4 hf;
    // Exercise 1
    {
        int y = 1;
    }

    // Exercise 2
    {
        vector<double> lengths{0.5, 1, 0.5};
        vector<double> angl{M_PI/6, M_PI/3, 7*M_PI/4};
        linkerState link = FK(lengths, angl);
        Eigen::Vector2d t2{0.2,0.1};
        //link.jointPts.back().col(3)
        Eigen::Vector2d test = t2-Eigen::Vector2d{0.2,0.1};
        cout<< link.jointPts.back().col(3)(0) << endl;

        linkerState Final = IK(vector<double>{8,8,9}, Eigen::Vector2d{0.0, 4.0});
        // [86.39082546081538, 85.62652297609263, 153.50224715871929]
        printf("Final angles are %.2f, %.2f, and %.2f\n", Final.angles[0]*180/M_PI, Final.angles[1]*180/M_PI, Final.angles[2]*180/M_PI);
    }

    // Exercise 3
    {

    }

    // Grade method
    // amp::HW4::grade<MyLinkManipulator>(constructor, "nonhuman.biologic@myspace.edu", argc, argv);
    return 0;
}