#include "MyBug1Algorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBug1Algorithm::plan(const amp::Problem2D& problem) 
{
    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    amp::Path2D dirPathToGoal;
    
    int numObs = problem.obstacles.size();
    vector<amp::Path2D> obsPaths = MyBug1Algorithm::obsPaths(problem);

    bool reached = false;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d qG = problem.q_goal;
    Eigen::Vector2d qL = problem.q_init;
    while(!reached)
    {
        // Write to goal function
        if(abs(qL[0] - qG[0]) > MyBug1Algorithm::eps){
            qL[0] += MyBug1Algorithm::step;
        }
        if(abs(qL[1] - qG[1]) > MyBug1Algorithm::eps)
        {
            qL[1] += MyBug1Algorithm::step;
        }

        // Write collision check function 
        for(int i = 0 ; i < numObs; i++)
        {
            vector<Eigen::Vector2d> objwaypoints = obsPaths[i].waypoints;
            for(int i = 0; i < objwaypoints.size(); i++)
            {
                if(((abs(qL[0] - objwaypoints[i][0]) < MyBug1Algorithm::eps) && (abs(qL[1] - objwaypoints[i][1]) < MyBug1Algorithm::eps)))
                {
                    cout << "OBJECT HIT\n";
                }
            }
        }

        if(((abs(qL[0] - qG[0]) < MyBug1Algorithm::eps) && (abs(qL[1] - qG[1]) < MyBug1Algorithm::eps)))
        {
            reached = true;
        }
        path.waypoints.push_back(qL);
    } 

    return path;
}

amp::Path2D MyBug1Algorithm::lines(Eigen::Vector2d ptA, Eigen::Vector2d ptB) 
{

    amp::Path2D path;
    Eigen::Vector2d start = ptA;
    Eigen::Vector2d goal = ptB;
    bool reached = false;
    path.waypoints.push_back(start);
    Eigen::Vector2d qL = start;
    while(!reached)
    {

        bool negXmove = qL[0] > goal[0];
        bool negYmove = qL[1] > goal[1];
        if(abs(qL[0] - goal[0]) > MyBug1Algorithm::eps){
            if(!negXmove)
            {
                qL[0] += MyBug1Algorithm::step;
            }
            else
            {
                qL[0] -= MyBug1Algorithm::step;
            }
        }

        if(abs(qL[1] - goal[1]) > MyBug1Algorithm::eps)
        {
            if(!negYmove)
            {
                qL[1] += MyBug1Algorithm::step;
            }
            else
            {
                qL[1] -= MyBug1Algorithm::step;
            }
        }

        if(((abs(qL[0] - goal[0]) < MyBug1Algorithm::eps) && (abs(qL[1] - goal[1]) < MyBug1Algorithm::eps)))
        {
            reached = true;
        }
        path.waypoints.push_back(qL);
    } 
    return path;
}

vector<amp::Path2D> MyBug1Algorithm::obsPaths(const amp::Problem2D& problem)
{
    int numObs = problem.obstacles.size();
    vector<amp::Path2D> paths;
    vector<Eigen::Vector2d> verts;
    vector<Eigen::Vector2d> tmp;
    for(int i = 0; i < numObs; i++)
    {
        if(MyBug1Algorithm::turn == "left")
        {
            verts = problem.obstacles[i].verticesCCW();
        }
        else
        {
            verts = problem.obstacles[i].verticesCW();
        }
        int numVerts = verts.size();
        amp::Path2D tmpPath;
        for(int i = 0; i < numVerts-1; i++)
        {
            tmp = MyBug1Algorithm::lines(verts[i],verts[i+1]).waypoints;
            for(int i = 0; i < tmp.size(); i++)
            {
                tmpPath.waypoints.push_back(tmp[i]);
            }
        }
        tmp = MyBug1Algorithm::lines(verts[numVerts-1],verts[0]).waypoints;
        for(int i = 0; i < tmp.size(); i++)
        {
            tmpPath.waypoints.push_back(tmp[i]);
        }
        paths.push_back(tmpPath);
    }
    return paths;
}