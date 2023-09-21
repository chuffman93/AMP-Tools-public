#include "MyBug2Algorithm.h"

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
amp::Path2D MyBug2Algorithm::plan(const amp::Problem2D& problem) 
{
    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    amp::Path2D dirPathToGoal;
    
    int numObs = problem.obstacles.size();
    vector<amp::Path2D> obsPaths = MyBug2Algorithm::obsPaths(problem);
    bool reached = false;
    path.waypoints.push_back(problem.q_init);
    Eigen::Vector2d qG = problem.q_goal;
    Eigen::Vector2d qL = problem.q_init;
    Eigen::Vector2d qL_Next;
    MyBug2Algorithm::qH qH;
    amp::Path2D searchPath;
    double num = (qG[1] - qL[1]);
    double dom = (qG[0] - qL[0]);
    MyBug2Algorithm::m = (qG[1] - qL[1]) / (qG[0] - qL[0]);
    MyBug2Algorithm::b = (qG[1] - (m*qG[0]));
    while(!reached)
    {
        // printf("PLANNING STEP: qL (%.3f,%.3f)::qG (%.3f,%.3f)\n", qL[0], qL[1], qG[0], qG[1]);
        qL_Next = nextStep(qL,qG);
        qH = collisionCheck(numObs, obsPaths, qL_Next);
        if(qH.hit)
        {

            searchPath = objTraverse(numObs, obsPaths, qH, qG);
            for(int i = 0; i < searchPath.waypoints.size(); i++)
            {
                path.waypoints.push_back(searchPath.waypoints[i]);
            }
            qL = path.waypoints.back();
            qL_Next = nextStep(qL,qG);
            if(qL_Next == qH.qh)
            {
                // printf("GOAL CAN'T BE REACHED\n");
                return path;
            }

        }
        qL = qL_Next;
        if(qL == qG)
        {
            reached = true;
        }
        path.waypoints.push_back(qL);
    } 
    // printf("GOAL REACHED!!\n");
    return path;
}

amp::Path2D MyBug2Algorithm::lines(Eigen::Vector2d ptA, Eigen::Vector2d ptB) 
{

    amp::Path2D path;
    Eigen::Vector2d start = ptA;
    Eigen::Vector2d goal = ptB;
    bool reached = false;
    path.waypoints.push_back(start);
    Eigen::Vector2d qL = start;
    Eigen::Vector2d qL_Next;
    double num = (ptB[1] - qL[1]);
    double dom = (ptB[0] - qL[0]);
    MyBug2Algorithm::m = (ptB[1] - qL[1]) / (ptB[0] - qL[0]);
    MyBug2Algorithm::b = (ptB[1] - (m*ptB[0]));
    while(!reached)
    {

        // printf("PRE STEP LINES: qL (%.2f,%.2f)::qG (%.2f,%.2f)\n", qL[0], qL[1], goal[0], goal[1]);
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

vector<amp::Path2D> MyBug2Algorithm::obsPaths(const amp::Problem2D& problem)
{
    int numObs = problem.obstacles.size();
    vector<amp::Path2D> paths;
    vector<Eigen::Vector2d> objverts;
    vector<Eigen::Vector2d> verts;
    vector<Eigen::Vector2d> tmp;
    for(int i = 0; i < numObs; i++)
    {
        if(strcmp(MyBug2Algorithm::turn,"right")==0)
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
        if(numVerts == 5)
        {
            debug = true;
        }
        for(int i = 0; i < numVerts-1; i++)
        {
            tmp = MyBug2Algorithm::lines(verts[i],verts[i+1]).waypoints;
            // printf("size of edge %ld\n",tmp.size());
            for(int i = 0; i < tmp.size()-1; i++)
            {
                tmpPath.waypoints.push_back(tmp[i]);
            }
        }
        tmp = MyBug2Algorithm::lines(verts.back(),verts.front()).waypoints;
        for(int i = 0; i < tmp.size()-1; i++)
        {
            tmpPath.waypoints.push_back(tmp[i]);
        }
        paths.push_back(tmpPath);
    }
    return paths;
}

MyBug2Algorithm::qH MyBug2Algorithm::collisionCheck(int numObs, vector<amp::Path2D> obsPaths, Eigen::Vector2d qL)
{
    MyBug2Algorithm::qH qH;
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

MyBug2Algorithm::qH MyBug2Algorithm::collisionCheck(int numObs, vector<amp::Path2D> obsPaths, Eigen::Vector2d qL, int currObj)
{
    MyBug2Algorithm::qH qH;
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

Eigen::Vector2d MyBug2Algorithm::nextStep(Eigen::Vector2d ptA, Eigen::Vector2d ptB)
{
        Eigen::Vector2d qL = ptA;
        bool negXmove = qL[0] > ptB[0];
        bool negYmove = qL[1] > ptB[1];
        bool yLine = abs(qL[0] - ptB[0]) < MyBug2Algorithm::eps;
        bool xLine = abs(qL[1] - ptB[1]) < MyBug2Algorithm::eps;
        if(!yLine && !xLine)
        {    
            // printf("qL = (%.3f, %.3f) ptB = (%.3f, %.3f)\n", qL[0], qL[1], ptB[0], ptB[1]);
            // printf("%.3fx+%.3f \n",MyBug2Algorithm::m,MyBug2Algorithm::b);
            if(negXmove)
            {
                qL[0] -= MyBug2Algorithm::step;
            }
            else
            {
                qL[0] += MyBug2Algorithm::step;
            }
            qL[1] = round(m*qL[0] + b);
        }
        else if(yLine)
        {
            if(!negYmove)
            {
                qL[1] += MyBug2Algorithm::step;
            }
            else
            {
                qL[1] -= MyBug2Algorithm::step;
            }
        }
        else
        {
            if(!negXmove)
            {
                qL[0] += MyBug2Algorithm::step;
            }
            else
            {
                qL[0] -= MyBug2Algorithm::step;
            }
        }
        return qL;
}

amp::Path2D MyBug2Algorithm::objTraverse(int numObs, vector<amp::Path2D> obsPaths, MyBug2Algorithm::qH travStart, Eigen::Vector2d goal)
{
    amp::Path2D pathToShortestPoint;
    Eigen::Vector2d q;
    MyBug2Algorithm::qH shortPoint;
    MyBug2Algorithm::qH qTmp;
    double shortDist = 10000;
    
    bool atMLine;
    int obj = travStart.objNum;
    int objPoint = travStart.hitItr;
    int ptMax = obsPaths[obj].waypoints.size();
    Eigen::Vector2d q_Start = travStart.qh;
    double startDist = calcDist(q_Start,goal);
    bool jumpOff = false;
    pathToShortestPoint.waypoints.push_back(q_Start);
    while(!jumpOff)
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
        // printf("EXPLORING STEP: qL (%.3f,%.3f)::qG (%.3f,%.3f)\n", q[0], q[1], goal[0], goal[1]);
        atMLine = checkMLine(q);
        if(atMLine)
        {
            // printf("*****MLINE HIT*****\n");
            vector<Eigen::Vector2d> tmpPath =   obsPaths[obj].waypoints;
            double currDis = calcDist(q,goal);
            double tmpDis;
            double closeDis = currDis;
            for(int i = 0; i < ptMax; i++)
            {
                if(checkMLine(tmpPath[i]))
                {
                    tmpDis = calcDist(tmpPath[i],goal);
                    if(tmpDis < closeDis)
                    {
                        closeDis = tmpDis;
                    }
                }
            }
            if(abs(currDis - closeDis) < MyBug2Algorithm::eps && (currDis < startDist))
            {
                jumpOff = true;
            }
            if(q == travStart.qh)
            {
                // printf("******** MADE MY WAY ALL AROUND ***********\n");
                break;
            }
        }
        
        pathToShortestPoint.waypoints.push_back(q);
        
    }
    return pathToShortestPoint;
}

bool MyBug2Algorithm::checkMLine(Eigen::Vector2d pt)
{
    Eigen::Vector2d qTmp = pt;
    qTmp[1] = round(m*pt[0] + b);
    if(pt == qTmp)
    {
        return true;
    }
    return false;
}

vector<Eigen::Vector2d> MyBug2Algorithm::expandObstacle(vector<Eigen::Vector2d> verts)
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
        if(abs(qL[0] - verts[i][0]) > MyBug2Algorithm::eps)
        {
            if(!negXmove)
            {
                tmpVec[0] = round(verts[i][0]+MyBug2Algorithm::buffer);
            }
            else
            {
                tmpVec[0] = round(verts[i][0]-MyBug2Algorithm::buffer);
            }
        }

        if(abs(qL[1] - verts[i][1]) > MyBug2Algorithm::eps)
        {
            if(!negYmove)
            {
                tmpVec[1] = round(verts[i][1]+MyBug2Algorithm::buffer);
            }
            else
            {
                tmpVec[1] = round(verts[i][1]-MyBug2Algorithm::buffer);
            }
        }
        // printf("Old Vert (%3.2f,%3.2f)\nNew Vert (%3.2f,%3.2f)\n\n",verts[i][0],verts[i][1],tmpVec[0],tmpVec[1]);
        newVerts.push_back(tmpVec);
    }
    return newVerts;
}

double MyBug2Algorithm::calcDist(Eigen::Vector2d pt, Eigen::Vector2d goal)
{
    return sqrt( pow((goal[0]-pt[0]),2) + pow(goal[1]-pt[1],2));
}