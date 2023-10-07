// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include "MyLinkManipulator.h"
#include "MyConfigurationSpace2D.h"

using namespace amp;
using namespace std;

vector<Eigen::Vector2d> reorderVerts(vector<Eigen::Vector2d> objSt)
{
    vector<Eigen::Vector2d> obj = objSt;

    for(int i = 0; i < obj.size(); i++)
    {
        obj[i] = -1*obj[i];
    }
    int tmp = 0;
    Eigen::Vector2d tmpPt;
    int i,j;
    double key1;
    double key2;
    for(i = 1; i < obj.size(); i++)
    {
        key1 = obj[i][1];
        key2 = obj[i][0];
        j = i-1;

        while(j >= 0 && (obj[j][1] > key1))
        {
            tmpPt = obj[j+1];
            obj[j+1] = obj[j];
            obj[j] = tmpPt;
            j = j-1;
        }
    }
    printf("Obj Size: %ld\n", obj.size());
    return obj;
}

double calAngle(Eigen::Vector2d pt1, Eigen::Vector2d pt2)
{
    if(abs(pt2[1] - pt1[1]) < 1e-3)
    {
        if(pt2[0] > pt1[0])
        {
            return 0;
        }
        else
        {
            return M_PI;
        }
    }
    else if(abs(pt2[0] - pt1[0]) < 1e-3)
    {
        if(pt2[1] > pt1[1])
        {
            return (2/M_PI);
        }
        else
        {
            return (3*M_PI/2);
        }
    }
    return atan2(pt2[1] - pt1[1], pt2[0] - pt1[0]);
}

vector<Eigen::Vector2d> minSum(Polygon Objs, Polygon Robot)
{
    vector<Eigen::Vector2d> cSpaceObj;
    vector<Eigen::Vector2d> obsVrt = Objs.verticesCCW();
    vector<Eigen::Vector2d> robVrtSt = Robot.verticesCCW();
    int n = robVrtSt.size();
    int m = obsVrt.size();
    int i = 0;
    int j = 0;

    vector<Eigen::Vector2d> robVrt  = reorderVerts(robVrtSt);

 
    while(i < n+1 || j < m+1)
    {
        printf("n = %d, i = %d, j = %d, m = %d\n",n ,i ,j ,m);
        printf(" Obs Vert (%.2f, %.2f)->(%.2f, %.2f) Rob Vert (%.2f, %.2f)->(%.2f, %.2f)\n", obsVrt[j][0], obsVrt[j][1], obsVrt[j+1][0], obsVrt[j+1][1], robVrt[i][0], robVrt[i][1],  robVrt[i+1][0], robVrt[i+1][1]);
        cSpaceObj.push_back(obsVrt[j]+robVrt[i]);
        double robAng = calAngle(robVrt[i],robVrt[i+1]);
        double obsAng = calAngle(obsVrt[j],obsVrt[j+1]);
        printf("robAng = %.2f, obsAng = %.2f\n", robAng, obsAng);
        if(abs(robAng - obsAng) < 1e-3)
        {
            i += 1;
            j += 1;
        }
        else if(robAng < obsAng)
        {
            i += 1;
        }
        else if(robAng > obsAng)
        {
            j += 1;
        }
       
    }
    return cSpaceObj;
}

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    // Exercise 1
    {
        amp::Obstacle2D obs = HW4::getEx1TriangleObstacle();
        vector<Eigen::Vector2d> cObj = minSum(obs, obs);
        Polygon cSpObj(cObj);
        vector<string> cSpName{"C-Space Object"};
        vector<Polygon> polygons{cSpObj};

        Visualizer::makeFigure(polygons, cSpName, true);

    }

    // Exercise 2
    {
        vector<double> linkLens{0.5, 1.0, 0.5};
        Eigen::Vector2d nBase{2.0,2.0};
        MyLinkManipulator a(linkLens);
        ManipulatorState ast = vector<double>{M_PI/6, M_PI/3, 7*M_PI/4};
        Visualizer::makeFigure(a, ast);
        int jtIdx = 3;
        Eigen::Vector2d endEff = a.getJointLocation(ast, jtIdx);
        Eigen::Vector2d aBase = a.getBaseLocation();
        printf("(%.2f, %.2f) at joint number: %d, base at (%.2f, %.2f)\n", endEff[0], endEff[1], jtIdx, aBase[0], aBase[1]);

        printf("Exercise 2a %s\n", HW4::checkFK(endEff, jtIdx, a, ast, true) ? "Passed!" : "Failed :(");

        vector<double> linkLensb{1, 0.5, 1};
        Eigen::Vector2d endTarget{2.0, 0.0};
        MyLinkManipulator b(linkLensb);
        ManipulatorState tarSt = b.getConfigurationFromIK(endTarget);
        Visualizer::makeFigure(b, tarSt);
        
    }

    // Exercise 3
    {

    }
    Visualizer::showFigures();
    // Grade method
    GridCSpace2DConstructor GCon;
    amp::HW4::grade<MyLinkManipulator>(GCon, "cohu8717@colorado.edu", argc, argv);
    return 0;
}