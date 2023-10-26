#include <queue>
#include <chrono>

#include "AMPCore.h"
#include "hw/HW2.h"
#include "myUtils.h"

using namespace amp;
using namespace std;
using namespace chrono;

myUtils hw7U;

// /// @brief Derive this class and implement your algorithm in the `plan` method. 
// class PRM : public PointMotionPlanner2D {
//     public:
//         /// @brief Solve a motion planning problem. Create a derived class and override this method
//         //virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;

//         virtual ~PRM() {}
// };

// /// @brief Derive this class and implement your algorithm in the `plan` method. 
// class GoalBiasRRT : public PointMotionPlanner2D {
//     public:
//         /// @brief Solve a motion planning problem. Create a derived class and override this method
//         //virtual amp::Path2D plan(const amp::Problem2D& problem) = 0;

//         virtual ~GoalBiasRRT() {}
// };

struct MapSearchResult {
    /// @brief Set to `true` if path was found, `false` if no path exists
    bool success = false;

    /// @brief Sequence of nodes where `node_path.front()` must contain init node, and `node_path.back()` must contain the goal node
    std::list<pair<double,double>> node_path;

    /// @brief Path cost (must equal sum of edge weights along node_path)
    double path_cost;
};

MapSearchResult dij_search(map< pair<double,double>, map< pair<double,double>, double > > RM,
                           pair<double,double> st, 
                           pair<double,double> g)
{
    MapSearchResult ret;

    pair<double, double> bst;

    vector<pair<double, double>> childs;
    vector<double> weights;
    
    typedef pair<double, pair<double, double>> nW;
    map <pair<double, double>, double> dist;
    map <pair<double, double>, pair<double, double>> bkPtr;
    
    priority_queue<nW, vector<nW>, greater<nW> > O;
    nW tmpTp;
    list<pair<double, double>> Ol;
    list<pair<double, double>> C;
    list<pair<double, double>> q;

    double gn;
    double h;
    double f;

    O.push(make_pair(0, st));
    dist.insert({st, 0});
    Ol.push_back(st);
    q.push_back(st);

    int itrCount = 0;
    while(!O.empty())
    {
        bst = O.top().second;
        // printf("Bst {%.2f, %.2f}\n", bst.first, bst.second);
        O.pop();
        Ol.remove(bst);
        C.push_back(bst);

        if (bst == g)
        {
            q.push_back(g);
            break;
        }
        for(auto chld : RM[bst])
        {
            if(find(C.begin(), C.end(), chld.first) != C.end())
            {
                continue;
            }

            if(find(Ol.begin(), Ol.end(), chld.first) == Ol.end())
            {
                gn = RM[bst][chld.first] + dist.find(bst)->second;
                // printf("GN = %.2f\n",gn);
                O.push( make_pair(gn, chld.first) );
                Ol.push_back(chld.first);
                dist.insert({chld.first, gn});
                bkPtr.insert({chld.first, bst});
            }
            else if((dist.find(bst)->second + RM[bst][chld.first] ) < dist.find(chld.first)->second)
            {
                gn = RM[bst][chld.first] + dist.find(bst)->second;
                // printf("GN = %.2f\n",gn);
                O.push( make_pair(gn, chld.first) );
                bkPtr.find(chld.first)->second = bst;
                dist.find(chld.first)->second = gn;
            }
        }
        itrCount += 1;
    }
    pair<double,double> tmp;
    if (q.back() == g)
    {
        tmp = g;
        
        while(bkPtr.find(tmp)->second != st)
        {
            auto it = q.begin();
            advance(it, 1);
            q.insert(it,bkPtr.find(tmp)->second);
            tmp = bkPtr.find(tmp)->second;
        }
        ret.node_path = q;
        ret.success = true;
        ret.path_cost = dist.find(g)->second;

    }
    else
    {
        ret.node_path = q;
        ret.success = false;
        ret.path_cost = -1;

    }
    printf("Path %s with length of the path { ", ret.success ? "found" : "not found"); 
    for(auto i : q)
    {   
        cout << "("<<i.first << ", " << i.second << ") ";
    }
    printf("} is %.2f found in %d iterations\n",ret.path_cost, itrCount);

    return ret;
}


double euc_dis(pair<double, double> a, pair<double, double> b)
{
    return sqrt(pow(a.first-b.first,2) + pow(a.second-b.second,2));
}

double euc_dis(Eigen::Vector2d a, Eigen::Vector2d b)
{
    return sqrt(pow(a[0]-b[0],2) + pow(a[1]-b[1],2));
}

double round_double(double a, int places)
{
    double multi = pow(10.0, places);
    return(round(a*multi)/multi);
}

bool inRadius(pair<double,double> pt1, pair<double,double>  pt2, double r)
{
    return (euc_dis(pt1, pt2) < r);
}

template<typename T>
T random(T from, T to)
{
    default_random_engine rnd{random_device{}()};
    uniform_real_distribution<T> distr(from, to);
    return distr(rnd);
}


Path2D prm_plan(const Problem2D& problem)
{
    Path2D ret;
    map< pair<double,double>, map< pair<double,double>, double > > RM;
    pair<double, double> st{problem.q_init[0], problem.q_init[1]};
    pair<double, double> g{problem.q_goal[0], problem.q_goal[1]};
    pair<double, double> sampPt;

    RM.insert( make_pair(st, map<pair<double,double>,double>()) );
    RM.insert( make_pair(g, map<pair<double,double>,double>()) );

    vector<Obstacle2D> obs = problem.obstacles;
    vector<Eigen::Vector2d> verts; 

    int n = 200;
    double r = 2.0;
    int dec_per = 2;
    bool smooth = false;

    list<pair<double, double>> samps;
    samps.push_back(st);
    samps.push_back(g);
    double xMin = problem.x_min;
    double xMax = problem.x_max;

    double yMin = problem.y_min;
    double yMax = problem.y_max;

    double xSamp;
    double ySamp;

    bool inObj = false;
    auto stTime = high_resolution_clock::now();
    for(int i = 0; i < n; i++)
    {
        xSamp = round_double(random<double>(xMin, xMax), dec_per);
        ySamp = round_double(random<double>(yMin, yMax), dec_per);
        sampPt = {xSamp, ySamp};

        if(find(samps.begin(), samps.end(), sampPt) == samps.end())
        {
            for(auto obj: obs)
            {
                verts = obj.verticesCCW();
                verts.push_back(verts[0]);
                if(hw7U.checkInObj(Eigen::Vector2d{sampPt.first, sampPt.second}, verts))
                {
                    inObj = true;
                    break;
                }
            }

            if(!inObj)
            {
                RM.insert( make_pair(sampPt, map<pair<double,double>,double>()) );
                samps.push_back(sampPt);
            }
        }
        inObj = false;
    }
    for(auto stPtM : RM)
    {
        for(auto tstPt: samps)
        {
            if(stPtM.first == tstPt)
            {
                continue;
            }
            if(inRadius(stPtM.first,tstPt,r) && !hw7U.checkInObj(stPtM.first, tstPt, obs))
            {
                RM[stPtM.first].insert({tstPt, euc_dis(stPtM.first, tstPt)});
            }
        }
    }
    MapSearchResult dijRet = dij_search(RM, st, g);
    
    if(smooth && dijRet.success)
    {
        
    }
    else if(dijRet.success)
    {

    }
    auto stpTime = high_resolution_clock::now();
    printf("Run Time for Algorithm: %ld ms\n", duration_cast<milliseconds>(stpTime-stTime).count());
    return ret;
}


int main(int argc, char** argv)
{
    
    Problem2D p1w1 = HW2::getWorkspace1();

    Path2D p1w1p = prm_plan(p1w1);

    return 0;
}