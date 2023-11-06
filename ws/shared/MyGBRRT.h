#include <queue>
#include <chrono>

#ifndef  MYUTILS_H
#include "myUtils.h"
#endif
#include "AMPCore.h"
#include "hw/HW7.h"

using namespace amp;
using namespace std;
using namespace chrono;

/// @brief Derive this class and implement your algorithm in the `plan` method. 
class MyGBRRT : public GoalBiasRRT2D {
    public:
        MyGBRRT()
        {
            n = 7000;
            r = 1.0;
            eps = 0.5;
            p_goal = 0.05;
            dec_per = 2;
            smooth = false;
        }

        pair<double, double> grabRandPoint(pair<double, double> g, double xmin, double xmax, double ymin, double ymax)
        {
            bool grabG = (rand() % 100) <= (int)(p_goal*100);
            return grabG ? g : make_pair(GBRRTU.round_double(GBRRTU.random<double>(xmin, xmax), dec_per), GBRRTU.round_double(GBRRTU.random<double>(ymin, ymax), dec_per));
        }

        /// @brief Solve a motion planning problem. Create a derived class and override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override 
        {
            Path2D ret;
            RM.clear();
            pair<double, double> st{problem.q_init[0], problem.q_init[1]};
            pair<double, double> g{problem.q_goal[0], problem.q_goal[1]};
            pair<double, double> sampPt;
            pair<double, double> qNear;
            pair<double, double> qNew;
            double tmp_dist;

            RM.insert( make_pair(st, map<pair<double,double>,double>()) );

            vector<Obstacle2D> obs = problem.obstacles;
            vector<Eigen::Vector2d> verts; 

            bool found = false;

            list<pair<double, double>> samps;
            list<pair<double, double>> smoothPath;
            samps.push_back(st);

            double xSamp;
            double ySamp;
            double itr = 0;
            bool inObj = false;

            double xMin = problem.x_min;
            double xMax = problem.x_max;

            double yMin = problem.y_min;
            double yMax = problem.y_max;


            auto stTime = high_resolution_clock::now();
            while(!found && itr < n)
            {
                sampPt = grabRandPoint(g, xMin, xMax, yMin, yMax);
                for(auto obj: obs)
                {
                    verts = obj.verticesCCW();
                    verts.push_back(verts[0]);
                    if(GBRRTU.checkInObj(Eigen::Vector2d{sampPt.first, sampPt.second}, verts))
                    {
                        inObj = true;
                        break;
                    }
                }
                if(inObj)
                {
                    inObj = false;
                    continue;
                }
                
                qNear = make_pair(0,0);
                tmp_dist = INFINITY;
                for(auto itr : RM)
                {
                    if (itr.first == sampPt)
                        continue;

                    if (tmp_dist > GBRRTU.euc_dis(itr.first, sampPt))
                    {
                        tmp_dist = GBRRTU.euc_dis(itr.first, sampPt);
                        qNear = itr.first;
                    }
                }
                qNew = GBRRTU.newPt(qNear, sampPt, r);
                
                if(!GBRRTU.checkInObj(qNew,obs) && !GBRRTU.checkInObj(qNear, qNew, obs) && GBRRTU.inbounds(qNew, xMin, xMax, yMin, yMax))
                {
                    if(qNew == g || GBRRTU.round_double(GBRRTU.euc_dis(qNew,g),2) <= eps)
                    {
                        found = true;
                        qNew = g;
                        if(RM.find(qNew) == RM.end())
                        {   
                            RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                        }
                        RM[qNear][qNew] = GBRRTU.euc_dis(qNear,qNew);
                    }
                    else if(GBRRTU.round_double(GBRRTU.euc_dis(qNew,sampPt),2) <= eps)
                    {
                        if(!GBRRTU.checkInObj(qNew, sampPt, obs))
                        {
                            qNew = sampPt;
                            if(RM.find(qNew) == RM.end())
                            {   
                                RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                            }
                            RM[qNear][qNew] = GBRRTU.euc_dis(qNear,qNew);
                        }
                    }
                    if(RM.find(qNew) == RM.end())
                    {   
                        RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                    }
                    RM[qNear][qNew] = GBRRTU.euc_dis(qNear,qNew);
                    
                    qNear = qNew;
                    qNew = GBRRTU.newPt(qNear, sampPt, r);
                }
                itr+=1;
            }
            myUtils::MapSearchResult dijRet = GBRRTU.dij_search(RM, st, g);
            
            if(smooth && dijRet.success)
            {
                smoothPath.push_back(dijRet.node_path.front());
                pair<double, double> prev;
                prev = smoothPath.back();
                for(auto pt : dijRet.node_path)
                {
                    if(!GBRRTU.checkInObj(smoothPath.back(), pt, obs))
                    {
                        prev = pt;
                        continue;
                    }
                    else
                    {
                        smoothPath.push_back(prev);
                    }
                }
                smoothPath.push_back(dijRet.node_path.back());
                ret = GBRRTU.pairToEigenVector(smoothPath);
                valid_path = true;
                // printf("Smoothed ");
                // GBRRTU.printPath(ret);
            }
            else if(dijRet.success)
            {
                ret = GBRRTU.pairToEigenVector(dijRet.node_path);
                valid_path = true;
            }
            else
            {
                ret = GBRRTU.pairToEigenVector({st, g});
                valid_path = false;
            }
            auto stpTime = high_resolution_clock::now();
            time = duration_cast<milliseconds>(stpTime-stTime).count();
            // printf("Run Time for Algorithm: %ld ms\n", time);

            return ret;
        }
        void benchMarkAlgo(tuple<int, double, double, double> prams, int numRuns, bool smoothing, list<Problem2D> problems, vector<string> ws)
        {
            Path2D ret;
            vector<double> tmp_lens;
            list<vector<double>> path_lens;
            vector<double> tmp_times;
            list<vector<double>> times;
            double tmp_sols = 0;
            vector<double> valid_sols;

            setSmoothing(smoothing);
            setSampleSize(get<0>(prams));
            setPtRadius(get<1>(prams));
            setPGoal(get<2>(prams));
            setEps(get<3>(prams));

            for(auto prob : problems)
            {
                for(int run = 0; run < numRuns; run++)
                {
                    ret = plan(prob);
                    if(getValid())
                    {
                        tmp_lens.push_back(ret.length());
                        tmp_times.push_back(getTime());
                        tmp_sols += getValid();
                    }
                }
                valid_sols.push_back(tmp_sols);
                path_lens.push_back(tmp_lens);
                times.push_back(tmp_times);
                tmp_sols = 0;
                tmp_lens.clear();
                tmp_times.clear();
            }

            Visualizer::makeBoxPlot(path_lens, ws, "Path Lengths for RRT", "Environments", "Distance [Units]");
            Visualizer::makeBoxPlot(times, ws, "Computation Time for RRT", "Environments", "Time [ms]");
            Visualizer::makeBarGraph(valid_sols, ws, "Valid Solutions for RRT", "Environments", "Valid Solution Count");
            Visualizer::showFigures();
            
        }

        pair<Graph<double>, map<amp::Node, Eigen::Vector2d> > getGraphFromRM()
        {
            Graph<double> ret1;
            map<Node, Eigen::Vector2d> ret2;
            Eigen::Vector2d ikey;
            Eigen::Vector2d okey;

            uint32_t nodeCnt = 0;
            uint32_t fromNode;
            uint32_t toNode;
            uint32_t tmpTo;
            uint32_t tmpFrom;

            for( auto key : RM)
            {   
                okey =  Eigen::Vector2d{key.first.first, key.first.second};
                fromNode = checkInMap(ret2,okey);    
                if(fromNode == -1)
                {
                    fromNode = nodeCnt;
                    ret2.insert({fromNode, okey});
                    nodeCnt++;
                }

                for( auto inKey : key.second)
                {
                    ikey =  Eigen::Vector2d{inKey.first.first, inKey.first.second};
                    toNode = checkInMap(ret2, ikey);
                    if(toNode == -1)
                    {
                        toNode = nodeCnt;
                        ret2.insert({toNode, ikey});
                        nodeCnt++;
                    }

                    ret1.connect(fromNode, toNode, RM[key.first][inKey.first]);
                }
            }
            return make_pair(ret1, ret2);
        }

        Node checkInMap(map<Node, Eigen::Vector2d> a, Eigen::Vector2d b)
        {
            for(auto key : a)
            {
                if(key.second[0] == b[0] && key.second[1] == b[1])
                {
                    return key.first;
                }
            }
            return -1;
        }

        void setSampleSize(int a)
        {
            n = a;
        }
        int getSampleSize()
        {
            return n;
        }

        void setPtRadius(double a)
        {
            r = a;
        }

        double getPtRadius()
        {
            return r;
        }

        void setEps(double a)
        {
            eps = a;
        }

        double getEps()
        {
            return eps;
        }

        void setPGoal(double a)
        {
            p_goal = a;
        }

        double getPGoal()
        {
            return p_goal;
        }

        void setDecPercision(int a)
        {
            dec_per = a;
        }

        void setSmoothing(bool a)
        {
            smooth = a;
        }
        long getTime()
        {
            return time;
        }

        bool getValid()
        {
            return valid_path;
        }


        virtual ~MyGBRRT() {}
    
    private:
        int n;
        double r;
        double eps;
        double p_goal;
        int dec_per;
        bool smooth;
        long time;
        bool valid_path;
        map< pair<double,double>, map< pair<double,double>, double > > RM;
        myUtils GBRRTU;

};