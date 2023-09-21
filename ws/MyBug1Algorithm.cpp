#include "MyBug1Algorithm.h"

bool operator==(Eigen::Vector2d a, Eigen::Vector2d b)
{
    if(((abs(a[0] - b[0]) < 1e-3) && (abs(a[1] - b[1]) < 1e-3)))
    {
        return true;
    }   
    return false;
}

double round(double a)
{
    double ret;
    char buff[40];

    sprintf(buff,"%.2f",a);

    sscanf(buff,"%lf",&ret);

    return ret;
}

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
    Eigen::Vector2d qL_Next;
    qH qH;
    amp::Path2D searchPath;
    while(!reached)
    {
        qL_Next = nextStep(qL,qG);
        qH = collisionCheck(numObs, obsPaths, qL_Next);
        if(qH.hit)
        {
            searchPath = objTraverse(numObs, obsPaths, qH, qG);
            for(int i = 0; i < searchPath.waypoints.size(); i++)
            {
                path.waypoints.push_back(searchPath.waypoints[i]);
            }
            qL_Next = path.waypoints.back();   
        }
        qL = qL_Next;
        if(qL == qG)
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
    Eigen::Vector2d qL_Next;
    while(!reached)
    {

        qL_Next = nextStep(qL, goal);
        qL = qL_Next;
        

        if(qL == goal)
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
    vector<Eigen::Vector2d> objverts;
    vector<Eigen::Vector2d> verts;
    vector<Eigen::Vector2d> tmp;
    for(int i = 0; i < numObs; i++)
    {
        if(MyBug1Algorithm::turn == "left")
        {
            objverts = problem.obstacles[i].verticesCCW();
        }
        else
        {
            objverts = problem.obstacles[i].verticesCW();
        }
        verts = expandObstacle(objverts);
        int numVerts = verts.size();
        amp::Path2D tmpPath;
        bool debug = false;
        for(int i = 0; i < numVerts-1; i++)
        {
            tmp = MyBug1Algorithm::lines(verts[i],verts[i+1]).waypoints;
            // printf("size of edge %ld\n",tmp.size());
            for(int i = 0; i < tmp.size()-1; i++)
            {
                tmpPath.waypoints.push_back(tmp[i]);
            }
        }
        tmp = MyBug1Algorithm::lines(verts.back(),verts.front()).waypoints;
        for(int i = 0; i < tmp.size()-1; i++)
        {
            tmpPath.waypoints.push_back(tmp[i]);
        }
        paths.push_back(tmpPath);
    }
    return paths;
}

qH MyBug1Algorithm::collisionCheck(int numObs, vector<amp::Path2D> obsPaths, Eigen::Vector2d qL)
{
    qH qH;
    qH.hit = false;

    for(int i = 0 ; i < numObs; i++)
    {
        vector<Eigen::Vector2d> objwaypoints = obsPaths[i].waypoints;
        for(int j = 0; j < objwaypoints.size(); j++)
        {
            if(qL == objwaypoints[j])
            {
                qH.objNum = i;
                qH.hitItr = j;
                qH.hit = true;
                qH.qh = objwaypoints[j];
            }
        }
    }
    return qH;
}

qH MyBug1Algorithm::collisionCheck(int numObs, vector<amp::Path2D> obsPaths, Eigen::Vector2d qL, int currObj)
{
    qH qH;
    qH.hit = false;

    for(int i = 0 ; i < numObs; i++)
    {
        if (i == currObj)
        {
            continue;
        }
        vector<Eigen::Vector2d> objwaypoints = obsPaths[i].waypoints;
        for(int j = 0; j < objwaypoints.size(); j++)
        {
            if(qL == objwaypoints[j])
            {
                qH.objNum = i;
                qH.hitItr = j;
                qH.hit = true;
                qH.qh = objwaypoints[j];
            }
        }
    }
    return qH;
}

Eigen::Vector2d MyBug1Algorithm::nextStep(Eigen::Vector2d ptA, Eigen::Vector2d ptB)
{
        Eigen::Vector2d qL = ptA;
        bool negXmove = qL[0] > ptB[0];
        bool negYmove = qL[1] > ptB[1];
        if(abs(qL[0] - ptB[0]) > MyBug1Algorithm::eps)
        {
            if(!negXmove)
            {
                qL[0] += MyBug1Algorithm::step;
            }
            else
            {
                qL[0] -= MyBug1Algorithm::step;
            }
        }

        if(abs(qL[1] - ptB[1]) > MyBug1Algorithm::eps)
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
        return qL;
}

amp::Path2D MyBug1Algorithm::objTraverse(int numObs, vector<amp::Path2D> obsPaths, qH travStart, Eigen::Vector2d goal)
{
    amp::Path2D pathToShortestPoint;
    Eigen::Vector2d q;
    qH shortPoint;
    qH qTmp;
    double shortDist = 10000;
    
    double tmpDist;
    int obj = travStart.objNum;
    int objPoint = travStart.hitItr;
    int shortObj = obj;
    int shortPt;
    int ptMax = obsPaths[obj].waypoints.size();
    if((objPoint+1) == ptMax)
    {
        shortPt = 0;
    }
    else
    {
        shortPt = objPoint+1;
    }
    Eigen::Vector2d q_Start = travStart.qh;
    bool fullTraversed = false;
    bool shortestPoint = false;
    pathToShortestPoint.waypoints.push_back(q_Start);
    while((!fullTraversed || !shortestPoint))
    {
        if(!fullTraversed)
        {
            if((objPoint+1) == ptMax)
            {
                objPoint = 0;
            }
            else
            {
                objPoint += 1;
            }
            q = obsPaths[obj].waypoints[objPoint];

            qTmp = collisionCheck(numObs, obsPaths, q, obj);
            if(qTmp.hit)
            {
                objPoint = qTmp.hitItr;
                obj = qTmp.objNum;
                ptMax = obsPaths[obj].waypoints.size();
            }
            tmpDist = calcDist(q,goal);
            if(tmpDist < shortDist)
            {
                shortDist = tmpDist;
                shortObj = obj;
                shortPt = objPoint; 
            }
            if(q == q_Start && obj == travStart.objNum && objPoint == travStart.hitItr)
            {
                fullTraversed = true;
            }
            pathToShortestPoint.waypoints.push_back(q);
        }
        else
        {
            if((objPoint+1) == ptMax)
            {
                objPoint = 0;
            }
            else
            {
                objPoint += 1;
            }
            q = obsPaths[obj].waypoints[objPoint];

            qTmp = collisionCheck(numObs, obsPaths, q,  obj);
            if(qTmp.hit)
            {
                objPoint = qTmp.hitItr;
                obj = qTmp.objNum;
                ptMax = obsPaths[obj].waypoints.size();
            }
            if(obj == shortObj && objPoint == shortPt)
            {
                shortestPoint = true;
            }
            pathToShortestPoint.waypoints.push_back(q);
        }
    }
    return pathToShortestPoint;
}

double MyBug1Algorithm::calcDist(Eigen::Vector2d pt, Eigen::Vector2d goal)
{
    return sqrt( pow((goal[0]-pt[0]),2) + pow(goal[1]-pt[1],2));
}

vector<Eigen::Vector2d> MyBug1Algorithm::expandObstacle(vector<Eigen::Vector2d> verts)
{
    vector<Eigen::Vector2d> newVerts;
    double minX = verts[0][0];
    double minY = verts[0][1];
    double maxX = verts[0][0];
    double maxY = verts[0][1];

    for(int i = 0; i < verts.size(); i++)
    {
        if(minX > verts[i][0])
        {
            minX = verts[i][0];
        }

        if(minY > verts[i][1])
        {
            minY = verts[i][1];
        }

        if(maxX < verts[i][0])
        {
            maxX = verts[i][0];
        }

        if(maxY < verts[i][1])
        {
            maxY = verts[i][1];
        }

    }
    // printf("Verticies %ld MinX %3.2f MaxX %3.2f MinY %3.2f MaxY %3.2f\n",verts.size(),minX,maxX,minY,maxY);
    Eigen::Vector2d qL(round(minX+(maxX-minX)/2), round(minY+(maxY-minY)/2));
    // printf("Center (%3.2f,%3.2f)\n", qL[0], qL[1]);
    Eigen::Vector2d tmpVec;
    for(int i = 0; i < verts.size(); i++)
    {    
        bool negXmove = qL[0] > verts[i][0];
        bool negYmove = qL[1] > verts[i][1];
        if(abs(qL[0] - verts[i][0]) > MyBug1Algorithm::eps)
        {
            if(!negXmove)
            {
                tmpVec[0] = round(verts[i][0]+MyBug1Algorithm::step);
            }
            else
            {
                tmpVec[0] = round(verts[i][0]-MyBug1Algorithm::step);
            }
        }

        if(abs(qL[1] - verts[i][1]) > MyBug1Algorithm::eps)
        {
            if(!negYmove)
            {
                tmpVec[1] = round(verts[i][1]+MyBug1Algorithm::step);
            }
            else
            {
                tmpVec[1] = round(verts[i][1]-MyBug1Algorithm::step);
            }
        }
        // printf("Old Vert (%3.2f,%3.2f)\nNew Vert (%3.2f,%3.2f)\n\n",verts[i][0],verts[i][1],tmpVec[0],tmpVec[1]);
        newVerts.push_back(tmpVec);
    }
    return newVerts;
}