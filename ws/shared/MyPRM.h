#include <queue>
#include <chrono>

#include "AMPCore.h"
#include "hw/HW7.h"
#ifndef  MYUTILS_H
#include "myUtils.h"
#endif

using namespace amp;
using namespace std;
using namespace chrono;

/// @brief Derive this class and implement your algorithm in the `plan` method. 
class MyPRM : public PRM2D {
    public:
        MyPRM()
        {
            n = 1500;
            r = 1.0;
            dec_per = 2;
            smooth = false;
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

        /// @brief Solve a motion planning problem. Create a derived class and override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override
        {
            Path2D ret;
            RM.clear();
            pair<double, double> st{problem.q_init[0], problem.q_init[1]};
            pair<double, double> g{problem.q_goal[0], problem.q_goal[1]};
            pair<double, double> sampPt;

            RM.insert( make_pair(st, map<pair<double,double>,double>()) );
            RM.insert( make_pair(g, map<pair<double,double>,double>()) );

            vector<Obstacle2D> obs = problem.obstacles;
            vector<Eigen::Vector2d> verts; 

            list<pair<double, double>> samps;
            list<pair<double, double>> smoothPath;
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
                xSamp = PRMU.round_double(PRMU.random<double>(xMin, xMax), dec_per);
                ySamp = PRMU.round_double(PRMU.random<double>(yMin, yMax), dec_per);
                sampPt = {xSamp, ySamp};

                if(find(samps.begin(), samps.end(), sampPt) == samps.end())
                {
                    if(PRMU.checkInObj(sampPt, obs))
                    {
                        inObj = true;
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
                    if(PRMU.inRadius(stPtM.first,tstPt,r) && !PRMU.checkInObj(stPtM.first, tstPt, obs))
                    {
                        RM[stPtM.first].insert({tstPt, PRMU.euc_dis(stPtM.first, tstPt)});
                    }
                }
            }

            myUtils::MapSearchResult dijRet = PRMU.dij_search(RM, st, g);
            
            if(smooth && dijRet.success)
            {
                smoothPath.push_back(dijRet.node_path.front());
                pair<double, double> prev;
                prev = smoothPath.back();
                for(auto pt : dijRet.node_path)
                {
                    if(!PRMU.checkInObj(smoothPath.back(), pt, obs))
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
                ret = PRMU.pairToEigenVector(smoothPath);
                valid_path = true;
                // printf("Smoothed ");
                // PRMU.printPath(ret);
            }
            else if(dijRet.success)
            {
                ret = PRMU.pairToEigenVector(dijRet.node_path);
                valid_path = true;
            }
            else
            {
                ret = PRMU.pairToEigenVector({st, g});
                valid_path = false;
            }
            auto stpTime = high_resolution_clock::now();
            time = duration_cast<milliseconds>(stpTime-stTime).count();
            // printf("Run Time for Algorithm: %ld ms\n", time);
            return ret;
        }

        void benchMarkAlgo(list<pair<int, double>> prams, int numRuns, bool smoothing, Problem2D& problem, string ws)
        {
            Path2D ret;
            vector<double> tmp_lens;
            list<vector<double>> path_lens;
            vector<double> tmp_times;
            list<vector<double>> times;
            double tmp_sols = 0;
            vector<double> valid_sols;
            char tmp_label[25];
            vector<string> labels;

            setSmoothing(smoothing);

            for(auto pram : prams)
            {
                sprintf(tmp_label, "n:r {%d:%.2f}", pram.first, pram.second);
                setSampleSize(pram.first);
                setPtRadius(pram.second);
                for(int run = 0; run < numRuns; run++)
                {
                    ret = plan(problem);
                    if(getValid())
                    {
                        tmp_lens.push_back(ret.length());
                        tmp_times.push_back(getTime());
                        tmp_sols += getValid();
                    }
                }
                valid_sols.push_back(tmp_sols);
                labels.push_back(tmp_label);
                path_lens.push_back(tmp_lens);
                times.push_back(tmp_times);
                tmp_sols = 0;
                tmp_label[0] = '\0';
                tmp_lens.clear();
                tmp_times.clear();
            }

            Visualizer::makeBoxPlot(path_lens, labels, "Path Lengths for " + ws, "Parameters", "Distance [Units]");
            Visualizer::makeBoxPlot(times, labels, "Computation Time for " + ws, "Parameters", "Time [ms]");
            Visualizer::makeBarGraph(valid_sols, labels, "Valid Solutions for " + ws, "Parameters", "Valid Solution Count");
            Visualizer::showFigures();
            
        }

        pair<Graph<double>, map<amp::Node, Eigen::Vector2d> > getGraphFromRM()
        {
            Graph<double> ret1;
            map<Node, Eigen::Vector2d> ret2;

            uint32_t nodeCnt = 0;
            uint32_t fromNode;
            uint32_t toNode;

            for( auto key : RM)
            {   
                fromNode = checkInMap(ret2, Eigen::Vector2d{key.first.first, key.first.second});    
                if(fromNode == -1)
                {
                    fromNode = nodeCnt;
                    ret2.insert({fromNode, Eigen::Vector2d{key.first.first, key.first.second}});
                    nodeCnt++;
                }

                for( auto inKey : key.second)
                {
                    toNode = checkInMap(ret2, Eigen::Vector2d{inKey.first.first, inKey.first.second});
                    if(toNode == -1)
                    {
                        toNode = nodeCnt;
                        ret2.insert({toNode, Eigen::Vector2d{inKey.first.first, inKey.first.second}});
                        nodeCnt++;
                    }

                    ret1.connect(fromNode, toNode, inKey.second);
                }
            }
            return make_pair(ret1, ret2);
        }

        Node checkInMap(map<Node, Eigen::Vector2d> a, Eigen::Vector2d b)
        {
            for(auto key : a)
            {
                if(key.second == b)
                {
                    return key.first;
                }
            }
            return -1;
        }

        virtual ~MyPRM() {}

    private:
        int n = 200;
        double r = 2.0;
        int dec_per = 2;
        bool valid_path;
        bool smooth = true;
        long time;
        map< pair<double,double>, map< pair<double,double>, double > > RM;
        myUtils PRMU;
};

