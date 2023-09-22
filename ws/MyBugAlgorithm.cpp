#include "MyBugAlgorithm.h"

bool operator==(Eigen::Vector2d a, Eigen::Vector2d b)
{
    if(((abs(a[0] - b[0]) < 1e-2) && (abs(a[1] - b[1]) < 1e-2)))
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
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;

    vector<amp::Obstacle2D> obstacles = problem.obstacles;

    bool objDet = false;
    bool objHit = false;
    bool goalDet = false;
    bool found = false;
    
    Eigen::Vector2d qG = problem.q_goal;
    Eigen::Vector2d qL = problem.q_init;
    Eigen::Vector2d qL_Next;

    MyBugAlgorithm::qH qh;

    double num = (qG[1] - qL[1]);
    double dom = (qG[0] - qL[0]);
    m = (qG[1] - qL[1]) / (qG[0] - qL[0]);
    b = (qG[1] - (m*qG[0]));

    path.waypoints.push_back(qL);

    qL_Next = nextStep(qL, qG, step*5);

    while(!found)
    {
        printf("qL = (%.3f,%.3f) and qL_Next = (%.3f,%.3f)\n",qL[0],qL[1],qL_Next[0],qL_Next[1]);
        if((goalOverreach(qL, qL_Next, qG) == 1.0) && !goalDet)
        {
            printf("Goal almost over shot!\n");
            goalDet = true;
            
        }

        if(goalDet)
        {
            qL_Next = nextStep(qL, qG, step);
        }
        else
        {
            qL_Next = nextStep(qL, qG, step*5);
        }
        
        qh = collisionCheck(obstacles, qL, qL_Next);
        if(qh.hit)
        {
            printf("Approaching Object!!\n");
            objDet = true;
        }
        while(objDet)
        {
            qL_Next = nextStep(qL, qG, step);
            qh = collisionCheck(obstacles, qL, qL_Next);
            {
                if(qh.hit)
                {
                    printf("Hit an Object!!\n");
                    objHit = true;
                    qL = qh.qh;
                    path.waypoints.push_back(qL);
                    return path;
                }
            }
            qL = qL_Next;
            printf("Approaching Obj on the way to the goal (%.3f,%.3f), currently at (%.3f,%.3f)\n",qG[0],qG[1],qL[0],qL[1]);
            path.waypoints.push_back(qL);
        }

        qL = qL_Next;
        if(qL == qG)
        {
            found = true;
        }
        // printf("Marching along to the goal (%.3f,%.3f), currently at (%.3f,%.3f)\n",qG[0],qG[1],qL[0],qL[1]);
        path.waypoints.push_back(qL);
    }
    
    return path;
}
bool MyBugAlgorithm::goalOverreach(Eigen::Vector2d ptA1, Eigen::Vector2d ptA2, Eigen::Vector2d goal)
{
    return ( abs((calcDist(ptA1,goal) + calcDist(ptA2,goal)) - calcDist(ptA1,ptA2)) < MyBugAlgorithm::eps );
}

Eigen::Vector2d MyBugAlgorithm::nextStep(Eigen::Vector2d ptA, Eigen::Vector2d ptB, double stepSize)
{
        Eigen::Vector2d qL = ptA;
        bool negXmove = qL[0] > ptB[0];
        bool negYmove = qL[1] > ptB[1];
        bool yLine = abs(qL[0] - ptB[0]) < MyBugAlgorithm::eps;
        bool xLine = abs(qL[1] - ptB[1]) < MyBugAlgorithm::eps;
        if(!yLine && !xLine)
        {    
            // printf("qL = (%.3f, %.3f) ptB = (%.3f, %.3f)\n", qL[0], qL[1], ptB[0], ptB[1]);
            // printf("%.3fx+%.3f \n",MyBugAlgorithm::m,MyBugAlgorithm::b);
            if(negXmove)
            {
                qL[0] -= stepSize;
            }
            else
            {
                qL[0] += stepSize;
            }
            qL[1] = round(m*qL[0] + b);
        }
        else if(yLine)
        {
            if(!negYmove)
            {
                qL[1] += stepSize;
            }
            else
            {
                qL[1] -= stepSize;
            }
        }
        else
        {
            if(!negXmove)
            {
                qL[0] += stepSize;
            }
            else
            {
                qL[0] -= stepSize;
            }
        }
        return qL;
}

double MyBugAlgorithm::lineDirection(Eigen::Vector2d ptA, Eigen::Vector2d ptB, Eigen::Vector2d ptC)
{
    return( (ptB[1]-ptA[1])*(ptC[0]-ptB[0])-(ptB[0]-ptA[0])*(ptC[1]-ptB[1]) );
}
        
bool MyBugAlgorithm::collinearAndOverlapping(Eigen::Vector2d ptA1, Eigen::Vector2d ptB1, Eigen::Vector2d ptA2,  Eigen::Vector2d ptB2)
{
    if(lineDirection(ptA1,ptB1,ptA2) == 0)
    {
        if(ptA2[0] <= max(ptA1[0],ptB1[0]) && ptA2[0] >= min(ptA1[0],ptB1[0])
        && ptA2[1] <= max(ptA1[1],ptB1[1]) && ptA2[1] >= min(ptA1[1],ptB1[1]))
        {
            return true;
        }
    }
    return false;
}

bool MyBugAlgorithm::intersectCheck(Eigen::Vector2d ptA1, Eigen::Vector2d ptB1, Eigen::Vector2d ptA2,  Eigen::Vector2d ptB2)
{
    double d1 = lineDirection(ptA1, ptB1, ptA2);
    double d2 = lineDirection(ptA1, ptB1, ptB2);
    double d3 = lineDirection(ptA2, ptB2, ptA1);
    double d4 = lineDirection(ptA2, ptB2, ptB1);

    // printf("Testing intersection between[(%.3f,%.3f),(%.3f,%.3f)] and [(%.3f,%.3f),(%.3f,%.3f)]\n",ptA1[0],ptA1[1],ptB1[0],ptB1[1],ptA2[0],ptA2[1],ptB2[0],ptB2[1]);

    if( ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) )
    {
        return true;
    }

    if(collinearAndOverlapping(ptA1,ptB1,ptA2,ptB2) || collinearAndOverlapping(ptA2,ptB2,ptA1,ptB1))
    {
        return true;
    }

    return false;
}

MyBugAlgorithm::qH MyBugAlgorithm::collisionCheck(vector<amp::Obstacle2D> obstacles, Eigen::Vector2d qL, Eigen::Vector2d qLNext)
{
    vector<Eigen::Vector2d> objverts;
    int numVerts;
    MyBugAlgorithm::qH ret;
    ret.hit = false;
    for(int i = 0; i < obstacles.size(); i++)
    {
        if(strcmp(MyBugAlgorithm::turn,"right") == 0)
        {
            objverts = obstacles[i].verticesCCW();
        }
        else
        {
            objverts = obstacles[i].verticesCW();
        }
        numVerts = objverts.size();
        for(int j = 0; j < numVerts-1; j++)
        {
            if(intersectCheck(objverts[j],objverts[j+1],qL,qLNext))
            {
                ret.objNum = i;
                ret.vertDir = j+1;
                ret.hit = true;
                ret.qh = qL;
                return ret;
            }
        }
        if(intersectCheck(objverts.back(),objverts.front(),qL,qLNext))
        {
            ret.objNum = i;
            ret.vertDir = 0;
            ret.hit = true;
            ret.qh = qL;
            return ret;
        }
    }
    return ret;
}

double MyBugAlgorithm::calcDist(Eigen::Vector2d pt, Eigen::Vector2d goal)
{
    return sqrt( pow((goal[0]-pt[0]),2) + pow(goal[1]-pt[1],2));
}