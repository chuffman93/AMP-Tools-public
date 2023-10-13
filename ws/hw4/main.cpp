// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"
#include "MyLinkManipulator.h"
#include "MyGridCSpace2DConstructor.h"
#include "MyGridCSpace2D.h"


using namespace amp;
using namespace std;

vector<Eigen::Vector2d> reorderVerts(vector<Eigen::Vector2d> objSt, vector<Eigen::Vector2d> ang)
{
    vector<Eigen::Vector2d> obj;
    Eigen::Vector2d tmp;
    int itr;
    for(int i = 0; i < objSt.size()+1; i++)
    {
        itr = (int)ang[i][0];
        tmp = objSt[itr];
        obj.push_back(tmp);
    }
    return obj;
}

vector<Eigen::Vector2d> sortVector(vector<Eigen::Vector2d> a)
{
    int tmp = 0;
    Eigen::Vector2d tmpPt;
    vector<Eigen::Vector2d> ret = a;
    int i,j;
    double key1;
    for(i = 1; i < ret.size(); i++)
    {
        key1 = ret[i][1];
        j = i-1;

        while(j >= 0 && (ret[j][1] > key1))
        {
            tmpPt = ret[j+1];
            ret[j+1] = ret[j];
            ret[j] = tmpPt;
            j = j-1;
        }
    }
    return ret;
}

double calAngle(Eigen::Vector2d pt1, Eigen::Vector2d pt2)
{
    return atan2(pt2[1] - pt1[1], pt2[0] - pt1[0]);
}

vector<Eigen::Vector2d> getAng(vector<Eigen::Vector2d> a)
{
    a.push_back(a[0]);
    vector<Eigen::Vector2d> angs;
    double tmpAng;
    for(int i = 0; i < a.size()-1; i++)
    {
        tmpAng = calAngle(a[i], a[i+1]);
        angs.push_back(Eigen::Vector2d{i,tmpAng});
    }
    angs = sortVector(angs);
    angs.push_back(angs[0]);
    return angs;
}

vector<Eigen::Vector2d> minSum(vector<Eigen::Vector2d> obsVrtSt, vector<Eigen::Vector2d> robVrtSt)
{
    vector<Eigen::Vector2d> cSpaceObj;

    vector<Eigen::Vector2d> obsAngs = getAng(obsVrtSt);
    vector<Eigen::Vector2d> robAngs = getAng(robVrtSt);

    vector<Eigen::Vector2d> robVrt = reorderVerts(robVrtSt, robAngs);
    vector<Eigen::Vector2d> obsVrt = reorderVerts(obsVrtSt, obsAngs);

    int n = robVrtSt.size();
    int m = obsVrtSt.size();
    int i = 0;
    int j = 0;
 
    while(i < n && j < m)
    {
        cSpaceObj.push_back(obsVrt[j]+robVrt[i]);
        double robAng = robAngs[i][1];
        double obsAng = obsAngs[j][1];
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
    cSpaceObj.push_back(obsVrt[j]+robVrt[i]);
    return cSpaceObj;
}

vector<Eigen::Vector2d> rotateRob(vector<Eigen::Vector2d> robVert, double rotAng)
{
    vector<Eigen::Vector2d> tmp;
    Eigen::Matrix2d T;
    T << cos(rotAng), -sin(rotAng),
         sin(rotAng), cos(rotAng);
    Eigen::Vector2d tmpVec;
    for (int i = 0; i < robVert.size(); i++)
    {
        tmpVec = T*(robVert[i]);
        tmp.push_back(tmpVec);
    }
    return tmp;
}

int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    amp::RNG::seed(amp::RNG::randiUnbounded());
    // Exercise 1
    {
        // a
        amp::Obstacle2D obs = HW4::getEx1TriangleObstacle();
        vector<Eigen::Vector2d> obsVert = obs.verticesCCW();
        vector<Eigen::Vector2d> robVert = obs.verticesCCW();
        for(int k = 0; k < robVert.size(); k++)
        {
            robVert[k] = -1*robVert[k];
        }
        vector<Eigen::Vector2d> cObj = minSum(obsVert, robVert);
        Polygon cSpObj(cObj);
        vector<string> cSpName{"C-Space Object"};
        vector<Polygon> polygon{cSpObj};

        for(int i = 0; i < cObj.size(); i++)
        {
            printf("Vert %d = (%.2f, %.2f)\n", i, cObj[i][0], cObj[i][1]);
        }

        Visualizer::makeFigure(polygon, cSpName, true);

        // b
        vector<double> rotAng;
        vector<Polygon> polygons;
        double ang = 0;
        double diff = (2*M_PI)/12;
        while(ang < (2*M_PI))
        {
            rotAng.push_back(ang);
            ang += diff;
        }
        vector<Eigen::Vector2d> nVerts;
        vector<Eigen::Vector2d> tmpObj;
        for(int i = 0; i < rotAng.size(); i++)
        {
            nVerts = rotateRob(robVert, rotAng[i]);
            tmpObj = minSum(obsVert, nVerts);
            polygons.push_back(Polygon(tmpObj));
        }
        Visualizer::makeFigure(polygons, rotAng);



    }

    // Exercise 2
    {
        // a
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

        // b
        vector<double> linkLensb{1, 0.5, 1};
        Eigen::Vector2d endTarget{2.0, 0.0};
        MyLinkManipulator b(linkLensb);
        ManipulatorState tarSt = b.getConfigurationFromIK(endTarget);
        for(int i = 0; i < tarSt.size(); i++)
        {
            printf("Angle %d = %.2f rads \n", i, tarSt[i]);
        }
        Visualizer::makeFigure(b, tarSt);
    }

    // Exercise 3
    {
        vector<double> linkLen{1.0, 1.0};
        MyLinkManipulator q3(linkLen);
        ManipulatorState st = vector<double>{0.0, 0.0};
 
        // a
        Environment2D env1 = HW4::getEx3Workspace1();
        Visualizer::makeFigure(env1, q3, st);
        MyGridCSpace2DConstructor GConA;
        auto A = GConA.construct(q3, env1);
        Visualizer::makeFigure(*(A.get()));

        // b
        Environment2D env2 = HW4::getEx3Workspace2();
        Visualizer::makeFigure(env2, q3, st);
        MyGridCSpace2DConstructor GConB;
        auto B = GConB.construct(q3, env2);
        // Visualizer::makeFigure();

        // c
        Environment2D env3 = HW4::getEx3Workspace3();
        Visualizer::makeFigure(env3, q3, st);
        MyGridCSpace2DConstructor GConC;
        auto C = GConC.construct(q3, env3);
        // Visualizer::makeFigure();

        A.release();
        B.release();
        C.release();
    }
    Visualizer::showFigures();
    // Grade method
    MyGridCSpace2DConstructor GCon;
    amp::HW4::grade<MyLinkManipulator>(GCon, "cohu8717@colorado.edu", argc, argv);
    return 0;
}