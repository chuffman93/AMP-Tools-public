#include "MyBugAlgorithm.h"
// bool operator==(Eigen::Vector2d a, Eigen::Vector2d b)
// {
//     if(((abs(a[0] - b[0]) < 1e-3) && (abs(a[1] - b[1]) < 1e-3)))
//     {
//         return true;
//     }   
//     return false;
// }

double roundD(double a)
{
    return round(a*100.0)/100.0;
}

Eigen::Vector2d roundPt(Eigen::Vector2d pt)
{
    Eigen::Vector2d ret;
    ret[0] = roundD(pt[0]);
    ret[1] = roundD(pt[1]);
    return ret;
}

bool compDoubles(double a, double b, const char * comp)
{
    double a2 = roundD(a);
    double b2 = roundD(b);
    if(strcmp(comp,"lt") == 0)
    {
        return a2 < b2;
    }
    else if(strcmp(comp,"gt") == 0)
    {
        return a2 > b2;
    }
    else if(strcmp(comp,"le") == 0)
    {
        return a2 <= b2;
    }
    else if(strcmp(comp,"ge") == 0)
    {
        return a2 >= b2;
    }
    else 
    {
        return a2 == b2;
    }
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

    amp::Path2D objPath;

    MyBugAlgorithm::qH qh;

    double num = (qG[1] - qL[1]);
    double dom = (qG[0] - qL[0]);
    m = ((qG[1] - qL[1]) / (qG[0] - qL[0]));
    b = (qG[1] - (m*qG[0]));

    path.waypoints.push_back(qL);

    qL_Next = nextStep(qL, qG, step*5);

    while(!found)
    {
        if((goalOverreach(qL, qL_Next, qG) == 1.0) && !goalDet)
        {
            if(debug) printf("Goal almost over shot!\n");
            goalDet = true;
            
        }

        if(goalDet)
        {
            qL_Next = nextStep(qL, qG, step);
        }
        else
        {
            qL_Next = nextStep(qL, qG, step);
        }
        if(debug) printf("qL = (%.6f,%.6f) and qL_Next = (%.6f,%.6f)\n",qL[0],qL[1],qL_Next[0],qL_Next[1]);        
        qh = collisionCheck(obstacles, qL, qL_Next, -1);
        if(qh.hit)
        {
            if(debug) printf("Approaching Object!!\n");
            objDet = true;
        }
        while(objDet)
        {
            qL_Next = nextStep(qL, qG, step);
            qh = collisionCheck(obstacles, qL, qL_Next, -1);
            if(debug) printf("Obj Approach qL = (%.6f,%.6f) and qL_Next = (%.6f,%.6f)\n",qL[0],qL[1],qL_Next[0],qL_Next[1]); 
            if(qh.hit)
            {
                if(debug) printf("OBJECT HECKIN' HIT\n");
                objHit = true;
            }
            if(objHit)
            {
                objPath = objTraverse(obstacles, qh, qG);
                for(int i = 0; i < objPath.waypoints.size(); i++)
                {
                    path.waypoints.push_back(objPath.waypoints[i]);   
                }
                qL_Next = path.waypoints.back();
                objDet = false;
                objHit = false;
                m = (qG[1] - qL_Next[1]) / (qG[0] - qL_Next[0]);
                b = (qG[1] - (m*qG[0]));
                break;
            }
            if (qL == qL_Next)
            {
                printf("PATH NOT FOUND\n");
                path.waypoints.push_back(qL);
                return path;
            }
            qL = qL_Next;
            
            if(debug) printf("Approaching Obj on the way to the goal (%.6f,%.6f), currently at (%.6f,%.6f)\n",qG[0],qG[1],qL[0],qL[1]);
            path.waypoints.push_back(qL);
        }

        qL = qL_Next;
        if(qL == qG)
        {
            if(debug)
            {
                printf("Goal found!!!!!!!!!!!!\n");
                sleep(10);
            }
            found = true;
        }

        // if(debug) printf("Marching along to the goal (%.6f,%.6f), currently at (%.6f,%.6f)\n",qG[0],qG[1],qL[0],qL[1]);
        path.waypoints.push_back(qL);
    }
    
    return path;
}
bool MyBugAlgorithm::goalOverreach(Eigen::Vector2d ptA1, Eigen::Vector2d ptA2, Eigen::Vector2d goal)
{
    return ( (abs((calcDist(ptA1,goal) + calcDist(ptA2,goal)) - calcDist(ptA1,ptA2))) < 1e-6 );
}

Eigen::Vector2d MyBugAlgorithm::nextStep(Eigen::Vector2d ptA, Eigen::Vector2d ptB, double stepSize)
{
        Eigen::Vector2d qL = ptA;
        bool negXmove = compDoubles(qL[0], ptB[0], "gt");
        bool negYmove = compDoubles(qL[1], ptB[1], "gt");
        bool yLine = abs(qL[0] - ptB[0]) < MyBugAlgorithm::eps;
        bool xLine = abs(qL[1] - ptB[1]) < MyBugAlgorithm::eps;
        if(!yLine && !xLine)
        {    
            // if(debug) printf("qL = (%.6f, %.6f) ptB = (%.6f, %.6f)\n", qL[0], qL[1], ptB[0], ptB[1]);
            // if(debug) printf("%.6fx+%.6f \n",MyBugAlgorithm::m,MyBugAlgorithm::b);
            if(negXmove)
            {
                qL[0] -= stepSize;
            }
            else
            {
                qL[0] += stepSize;
            }
            qL[1] = roundD((m*qL[0] + b));
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
        if(compDoubles(ptA2[0], max(ptA1[0],ptB1[0]),"le") && compDoubles(ptA2[0], min(ptA1[0],ptB1[0]),"ge")
        && compDoubles(ptA2[1], max(ptA1[1],ptB1[1]),"le") && compDoubles(ptA2[1], min(ptA1[1],ptB1[1]),"ge"))
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

    // if(debug) printf("Testing intersection between[(%.6f,%.6f),(%.6f,%.6f)] and [(%.6f,%.6f),(%.6f,%.6f)]\n",ptA1[0],ptA1[1],ptB1[0],ptB1[1],ptA2[0],ptA2[1],ptB2[0],ptB2[1]);

    if( ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) )
    {
        if(debug) printf("Intersecting Lines A1(%.6f,%.6f), B1(%.6f,%.6f), A2(%.6f,%.6f), B2(%.6f,%.6f)\n", ptA1[0], ptA1[1], ptB1[0], ptB1[1], ptA2[0], ptA2[1], ptB2[0], ptB2[1]);
        return true;
    }

    if(collinearAndOverlapping(ptA1,ptB1,ptA2,ptB2) || collinearAndOverlapping(ptA2,ptB2,ptA1,ptB1))
    {
        if(debug) printf("Collinear and Overlapping A1(%.6f,%.6f), B1(%.6f,%.6f), A2(%.6f,%.6f), B2(%.6f,%.6f)\n", ptA1[0], ptA1[1], ptB1[0], ptB1[1], ptA2[0], ptA2[1], ptB2[0], ptB2[1]);
        return true;
    }

    return false;
}

MyBugAlgorithm::qH MyBugAlgorithm::collisionCheck(vector<amp::Obstacle2D> obstacles, Eigen::Vector2d qL, Eigen::Vector2d qLNext, int currObj)
{
    vector<Eigen::Vector2d> objverts;
    int numVerts;
    MyBugAlgorithm::qH ret;
    ret.hit = false;
    for(int i = 0; i < obstacles.size(); i++)
    {
        if(i == currObj)
        {
            continue;
        }
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
            if(intersectCheck(roundPt(objverts[j]),roundPt(objverts[j+1]),qL,qLNext) || roundPt(objverts[j]) == qLNext || goalOverreach(roundPt(objverts[j]),roundPt(objverts[j+1]),qLNext))
            {
                if(debug) printf("Found Collision!! target vert (%.6f,%.6f), trail vert (%.6f,%.6f), qL (%.6f,%.6f), qlNext (%.6f,%.6f)\n", roundD(objverts[j+1][0]), roundD(objverts[j+1][1]), roundD(objverts[j][0]), roundD(objverts[j][1]), (qL[0]), (qL[1]), (qLNext[0]), (qLNext[1]));
                if(debug) sleep(5);
                ret.objNum = i;
                ret.vertDir = j+1;
                ret.hit = true;
                ret.qh = qL;
                return ret;
            }
        }
        if(intersectCheck(roundPt(objverts.back()),roundPt(objverts.front()),qL,qLNext) || objverts.back() == qLNext || goalOverreach(roundPt(objverts.back()),roundPt(objverts.front()),qLNext))
        {
            if(debug) printf("Front: Found Collision!! target vert (%.6f,%.6f), trail vert (%.6f,%.6f), qL (%.6f,%.6f), qlNext (%.6f,%.6f)\n", roundD(objverts.front()[0]), roundD(objverts.front()[0]), roundD(objverts.back()[0]), roundD(objverts.back()[1]), qL[0], qL[1], qLNext[0], qLNext[1]);
            if(debug) sleep(5);
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

amp::Path2D MyBugAlgorithm::objTraverse(vector<amp::Obstacle2D> obstacles, qH travStart, Eigen::Vector2d goal)
{
    amp::Path2D ret;
    Eigen::Vector2d q = travStart.qh;
    Eigen::Vector2d travLineSt;
    MyBugAlgorithm::qH qN;
    MyBugAlgorithm::qH closePt;
    MyBugAlgorithm::qH qTmp;

    vector<Eigen::Vector2d> obverts;

    int toVert = travStart.vertDir;
    int prevVert;
    int obj = travStart.objNum;

    qN.objNum = travStart.objNum;
    qN.vertDir = travStart.vertDir;
    qN.qh = q;

    double shortDist = 10000;
    double tmpDist;

    bool fullTraversed = false;
    bool shortPoint = false;

    ret.waypoints.push_back(q);

    int pathLen = 1;
    int lenToShort;
    int numVerts;
    if(strcmp(MyBugAlgorithm::turn,"right") == 0)
    {
        obverts = obstacles[obj].verticesCCW();
    }
    else
    {
        obverts = obstacles[obj].verticesCW();
    }
    numVerts = obverts.size();
    if(toVert == 0)
    {
        prevVert = numVerts-1;
    }
    else
    {
        prevVert = toVert-1;
    }
    travLineSt = calTravLine(obverts[prevVert], obverts[toVert], q);
    m = travLineSt[0];
    b = travLineSt[1];
    if(debug) printf("New M is %.3f and new b %.3f\n",m,b);
    if(debug) printf("---------- Transversing Object ----------\n");

    while(!fullTraversed || !shortPoint)
    {
        
        if(!fullTraversed)
        {
            qN = followObstacle(q, obstacles[obj], toVert, obj);
            qTmp = collisionCheck(obstacles, q, qN.qh, obj);
            if(qTmp.hit)
            {
                
                if(debug) printf("collision!! moving from (%.6f,%.6f) to potenionally (%.6f,%.6f)\n", q[0], q[1], qN.qh[0], qN.qh[1]);
                if(debug) sleep(5);
                toVert = qTmp.vertDir;
                obj = qTmp.objNum;
                qN.vertDir = qTmp.vertDir;
                qN.objNum = qTmp.objNum;
                
            }
            else
            {
                toVert = qN.vertDir;
            }
            if(!qTmp.hit)
            {
                q = qN.qh;
                if(debug) printf("q = (%.6f,%.6f), start = (%.6f,%.6f)\n", q[0], q[1], travStart.qh[0], travStart.qh[1]);
                if((abs(q[0] - travStart.qh[0]) < 1e-2) && (abs(q[1] - travStart.qh[1]) < 1e-2) && !compDoubles(shortDist, 10000, "eq"))
                {
                    if(debug) printf("Object Fully Traversed q = (%.6f,%.6f), start = (%.6f,%.6f)\n", q[0], q[1], travStart.qh[0], travStart.qh[1]);
                    if(debug) sleep(5);
                    fullTraversed = true;
                }
                
                ret.waypoints.push_back(q);
                pathLen+=1;
                tmpDist = calcDist(q,goal);
                if (tmpDist < shortDist)
                {
                    lenToShort = pathLen;
                    shortDist = tmpDist;
                }
            }
        }
        else
        {
            if(debug) printf("Going to Shortest Point @ (%.3f,%.3f) with Distance %.3f\n", ret.waypoints[lenToShort-1][0],ret.waypoints[lenToShort-1][1], shortDist);
            for(int i = 0; i < lenToShort; i++)
            {
                ret.waypoints.push_back(ret.waypoints[i]);   
            }
            shortPoint = true;
        }
        
    }
    if(debug) printf("---------- Done Transversing ----------\n");
    return ret;
}

MyBugAlgorithm::qH MyBugAlgorithm::followObstacle(Eigen::Vector2d q, amp::Obstacle2D obstacle, int verItr, int objNumber)
{
    MyBugAlgorithm::qH nextPt;
    nextPt.qh = q;
    nextPt.objNum = objNumber;
    nextPt.vertDir = verItr;

    vector<Eigen::Vector2d> objverts;

    int prevVert;
    int nxtVert;
    int numVerts;

    if(strcmp(MyBugAlgorithm::turn,"right") == 0)
    {
        objverts = obstacle.verticesCCW();
    }
    else
    {
        objverts = obstacle.verticesCW();
    }
    numVerts = objverts.size();


    if(verItr == 0)
    {
        prevVert = numVerts-1;
    }
    else
    {
        prevVert = verItr-1;
    }

    if(verItr == (numVerts-1))
    {
        nxtVert = 0;
    }
    else
    {
        nxtVert = verItr+1;
    }

    Eigen::Vector2d ptA1 = roundPt(objverts[prevVert]);
    Eigen::Vector2d ptB1 = roundPt(objverts[verItr]);    
    Eigen::Vector2d ptC1 = roundPt(objverts[nxtVert]);   

    Eigen::Vector2d intPt;

    if(debug) printf("Moving from point (%.6f, %.6f) to vert %d [%d] %d (%.3f, %.3f) from (%.3f, %.3f) next vert (%.3f, %.3f) on object %d\n", q[0], q[1], prevVert, verItr, nxtVert, ptB1[0], ptB1[1], ptA1[0], ptA1[1], ptC1[0], ptC1[1], objNumber);

    Eigen::Vector2d mxTrav = calTravLine(ptA1,ptB1,q);
    Eigen::Vector2d mxRef = calLine(ptA1,ptB1);
    Eigen::Vector2d mxNxt = calLine(ptB1,ptC1);

    Eigen::Vector2d mxLn1;
    Eigen::Vector2d mxLn2;
    Eigen::Vector2d mxLnTrav;

    bool negXmove = compDoubles(ptA1[0], ptB1[0], "gt");
    bool negYmove = compDoubles(ptA1[1], ptB1[1], "gt");
    bool curryLine = abs(ptA1[0] - ptB1[0]) < MyBugAlgorithm::eps;
    bool currxLine = abs(ptA1[1] - ptB1[1]) < MyBugAlgorithm::eps;
    bool nxtyLine = abs(ptC1[0] - ptB1[0]) < MyBugAlgorithm::eps;
    bool nxtxLine = abs(ptC1[1] - ptB1[1]) < MyBugAlgorithm::eps;
    bool nextLine = false;

    if(curryLine)
    {
        if(nxtxLine)
        {
            if(negYmove)
            {
                if(compDoubles(ptC1[1], q[1],"gt"))
                {
                    nextLine = true;                    
                }
            }
            else
            {
                if(compDoubles(ptC1[1], q[1],"lt"))
                {
                    nextLine = true;   
                }
            }
            
        }
        else
        {
            mxLn2 = calLine(ptB1,ptC1);
            intPt[0] = q[0];
            intPt[1] = mxLn2[0] * intPt[0] + mxLn2[1];
            nextLine = (negXmove ? compDoubles(intPt[0], q[0],"gt") : compDoubles(intPt[0], q[0],"lt"));

        }

        if(nextLine)
        {
            if(debug) printf("NEXT LINE X AXIS\n");
            negXmove = compDoubles(ptB1[0], ptC1[0], "gt");
            if(!negXmove)
            {
                nextPt.qh[0] += MyBugAlgorithm::step;
            }
            else
            {
                nextPt.qh[0] -= MyBugAlgorithm::step;
            }
            nextPt.vertDir = nxtVert;
        }
        else if(nextLine && !nxtxLine)
        {

        }
        else
        {
            if(debug) printf("SAME LINE Y AXIS\n");
            if(!negYmove)
            {
                nextPt.qh[1] += MyBugAlgorithm::step;
            }
            else
            {
                nextPt.qh[1] -= MyBugAlgorithm::step;
            }
        }
    }
    else if(currxLine)
    {
        if(nxtyLine)
        {
            if(negXmove)
            {
                if(compDoubles(ptC1[0],q[0],"gt"))
                {
                    nextLine = true;
                }
            }
            else
            {
                if(compDoubles(ptC1[0],q[0],"lt"))
                {
                    nextLine = true;
                }
            }
            
        }
        else
        {
            mxLn2 = calLine(ptB1,ptC1);
            intPt[1] = q[1];    
            intPt[0] = (intPt[1]-mxLn2[1])/mxLn2[0];
            nextLine = (negXmove ? compDoubles(intPt[0], q[0],"gt") : compDoubles(intPt[0], q[0],"lt"));
        }

        if(nextLine)
        {
            if(debug) printf("NEXT LINE Y AXIS\n");
            negYmove = compDoubles(ptB1[1], ptC1[1], "gt");

            if(!negYmove)
            {
                nextPt.qh[1] += MyBugAlgorithm::step;
            }
            else
            {
                nextPt.qh[1] -= MyBugAlgorithm::step;
            }
            nextPt.vertDir = nxtVert;
        }
        else if(nextLine && !nxtyLine)
        {

        }
        else
        {
            if(debug) printf("SAME LINE X AXIS\n");
            if(!negXmove)
            {
                nextPt.qh[0] += MyBugAlgorithm::step;
            }
            else
            {
                nextPt.qh[0] -= MyBugAlgorithm::step;
            }

        }
    }
    else
    {
        if(nxtyLine)
        {
            if(negXmove)
            {
                if(compDoubles(ptC1[0],q[0],"gt"))
                {
                    nextLine = true;
                }
            }
            else
            {
                if(compDoubles(ptC1[0],q[0],"lt"))
                {
                    nextLine = true;
                }
            }
            
        }
        else if(nxtxLine)
        {
            if(negYmove)
            {
                if(compDoubles(ptC1[1], q[1],"gt"))
                {
                    nextLine = true;                    
                }
            }
            else
            {
                if(compDoubles(ptC1[1], q[1],"lt"))
                {
                    nextLine = true;   
                }
            }
            
        }
        else
        {
            mxLn2 = calLine(ptC1,ptB1);
            intPt = interCeptPt(mxLn2, mxLnTrav);
            nextLine = (negXmove ? compDoubles(intPt[0], q[0],"gt") : compDoubles(intPt[0], q[0],"lt")) ||
                       (negYmove ? compDoubles(intPt[1], q[1],"gt") : compDoubles(intPt[1], q[1],"lt")) ;
        }

        if(nextLine && nxtyLine)
        {
            if(debug) printf("NEXT LINE Y AXIS\n");
            negYmove = compDoubles(ptB1[1], ptC1[1], "gt");

            if(!negYmove)
            {
                nextPt.qh[1] += MyBugAlgorithm::step;
            }
            else
            {
                nextPt.qh[1] -= MyBugAlgorithm::step;
            }
            nextPt.vertDir = nxtVert;
        }
        else if(nextLine && nxtxLine)
        {
            if(debug) printf("NEXT LINE X AXIS\n");
            negXmove = compDoubles(ptB1[0], ptC1[0], "gt");
            if(!negXmove)
            {
                nextPt.qh[0] += MyBugAlgorithm::step;
            }
            else
            {
                nextPt.qh[0] -= MyBugAlgorithm::step;
            }
            nextPt.vertDir = nxtVert;
        }
        else if(nextLine)
        {
            if(debug) printf("NEXT LINE\n");
            negXmove = compDoubles(ptB1[0], ptC1[0], "gt");
            if(!negXmove)
            {
                nextPt.qh[0] += MyBugAlgorithm::step;
                mxLnTrav = calTravLine(ptB1, ptC1, nextPt.qh);
                m = mxLnTrav[0];
                b = mxLnTrav[1];
                nextPt.qh[1] = m*nextPt.qh[0] + b;
            }
            else
            {
                nextPt.qh[0] -= MyBugAlgorithm::step;
                mxLnTrav = calTravLine(ptB1, ptC1, nextPt.qh);
                m = mxLnTrav[0];
                b = mxLnTrav[1];
                nextPt.qh[1] = m*nextPt.qh[0] + b;
            }
            nextPt.vertDir = nxtVert;
        }
        else
        {
            if(debug) printf("SAME LINE on Line\n");
            if(debug) printf("New M is %.3f and new b %.3f intercept (%.3f,%.3f)\n",m,b,intPt[0],intPt[1]);
            double rad = atan(m);
            double xStep = MyBugAlgorithm::step*cos(rad);
            double yStep = MyBugAlgorithm::step*sin(rad);

            if(debug) printf("Rad %.3f => xStep = %.3f, yStep = %.3f\n",rad,xStep,yStep);
            if(rad > 0)
            {
                if(!negXmove)
                {
                    nextPt.qh[0] += xStep;
                }
                else
                {
                    nextPt.qh[0] -= xStep;
                }
                if(!negYmove)
                {
                    nextPt.qh[1] += yStep;
                }
                else
                {
                    nextPt.qh[1] -= yStep;
                }
            }
            else
            {
                if(!negXmove)
                {
                    nextPt.qh[0] += xStep;
                }
                else
                {
                    nextPt.qh[0] -= xStep;
                }
                if(!negYmove)
                {
                    nextPt.qh[1] += (-1)*yStep;
                }
                else
                {
                    nextPt.qh[1] -= (-1)*yStep;
                }
            }
        }
    }
    // if(debug) printf("Next Point = (%.6f, %.6f)\n", nextPt.qh[0], nextPt.qh[1]);
    qH retPt = nextPt;
    retPt.qh = roundPt(nextPt.qh);
    // if(debug) usleep(1e6);
    return nextPt;
}

Eigen::Vector2d MyBugAlgorithm::interCeptPt(Eigen::Vector2d mx1, Eigen::Vector2d mx2)
{
    Eigen::Vector2d ret;

    if(debug) printf("mx1: %.3fx + %.3f\n", mx1[0] , mx1[1]);
    if(debug) printf("mx2: %.3fx + %.3f\n", m , b);
    ret[0] = (b - mx1[1]) / (mx1[0] - m);
    ret[1] = mx1[1] + ret[0]*mx1[0];

    return ret;
}

Eigen::Vector2d MyBugAlgorithm::calTravLine(Eigen::Vector2d ptA1, Eigen::Vector2d ptB1, Eigen::Vector2d qL)
{
    Eigen::Vector2d mx = calLine(ptA1, ptB1);
    Eigen::Vector2d mxRet;

    double d = 0.01;

    mxRet[0] = mx[0];

    double dir = lineDirection(ptA1, ptB1, qL);

    double posNeg = dir*mx[0];

    double b1 = ((d*(sqrt(1+pow(mx[0],2)))) - mx[1]) * -1;
    double b2 = ((d*(sqrt(1+pow(mx[0],2)))) + mx[1]);

    if(posNeg < 0)
    {
        mxRet[1] = (compDoubles(b1,mx[1],"lt")) ? b1 : b2;
    }
    else
    {
        mxRet[1] = (compDoubles(b1,mx[1],"gt")) ? b1 : b2;
    }


    return mxRet;

}

Eigen::Vector2d MyBugAlgorithm::calLine(Eigen::Vector2d ptA1, Eigen::Vector2d ptB1)
{
    Eigen::Vector2d mx;
    mx[0] = ((ptB1[1] - ptA1[1]) / (ptB1[0] - ptA1[0]));
    mx[1] = (ptA1[1] - (mx[0]*ptA1[0]));
    return mx;

}