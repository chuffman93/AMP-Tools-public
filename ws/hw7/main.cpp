#include <queue>
#include <chrono>

#include "AMPCore.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "hw/HW7.h"
#include "myUtils.h"

using namespace amp;
using namespace std;
using namespace chrono;

myUtils hw7U;

/// @brief Derive this class and implement your algorithm in the `plan` method. 
class MyGBRRT : public GoalBiasRRT2D {
    public:
        MyGBRRT()
        {
            n = 7000;
            r = 0.5;
            eps = 0.25;
            p_goal = 0.05;
            dec_per = 2;
            smooth = false;
        }

        pair<double, double> grabRandPoint(pair<double, double> g, double xmin, double xmax, double ymin, double ymax)
        {
            bool grabG = (rand() % 100) <= (int)(p_goal*100);
            return grabG ? g : make_pair(hw7U.round_double(hw7U.random<double>(xmin, xmax), dec_per), hw7U.round_double(hw7U.random<double>(ymin, ymax), dec_per));
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
                    if(hw7U.checkInObj(Eigen::Vector2d{sampPt.first, sampPt.second}, verts))
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

                    if (tmp_dist > hw7U.euc_dis(itr.first, sampPt))
                    {
                        tmp_dist = hw7U.euc_dis(itr.first, sampPt);
                        qNear = itr.first;
                    }
                }
                qNew = hw7U.newPt(qNear, sampPt, r);
                while(!hw7U.checkInObj(qNew,obs) && !hw7U.checkInObj(qNear, qNew, obs) && hw7U.inbounds(qNew, xMin, xMax, yMin, yMax))
                {
                    if(qNew == g || hw7U.round_double(hw7U.euc_dis(qNew,g),2) <= eps)
                    {
                        found = true;
                        qNew = g;
                        if(RM.find(qNew) == RM.end())
                        {   
                            RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                        }
                        RM[qNear][qNew] = hw7U.euc_dis(qNear,qNew);
                        break;
                    }
                    else if(hw7U.round_double(hw7U.euc_dis(qNew,sampPt),2) <= eps)
                    {
                        if(!hw7U.checkInObj(qNew, sampPt, obs))
                        {
                            qNew = sampPt;
                            if(RM.find(qNew) == RM.end())
                            {   
                                RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                            }
                            RM[qNear][qNew] = hw7U.euc_dis(qNear,qNew);
                        }
                        break;
                    }
                    RM[qNear][qNew] = hw7U.euc_dis(qNear,qNew);
                    if(RM.find(qNew) == RM.end())
                    {   
                        RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                    }
                    qNear = qNew;
                    qNew = hw7U.newPt(qNear, sampPt, r);
                }
                itr+=1;
            }
            myUtils::MapSearchResult dijRet = hw7U.dij_search(RM, st, g);
            
            if(smooth && dijRet.success)
            {
                smoothPath.push_back(dijRet.node_path.front());
                pair<double, double> prev;
                prev = smoothPath.back();
                for(auto pt : dijRet.node_path)
                {
                    if(!hw7U.checkInObj(smoothPath.back(), pt, obs))
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
                ret = hw7U.pairToEigenVector(smoothPath);
                valid_path = true;
                // printf("Smoothed ");
                // hw7U.printPath(ret);
            }
            else if(dijRet.success)
            {
                ret = hw7U.pairToEigenVector(dijRet.node_path);
                valid_path = true;
            }
            else
            {
                ret = hw7U.pairToEigenVector({st, g});
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

        // pair<Graph<double>, map<amp::Node, Eigen::Vector2d> > getGraphFromRM()
        // {
            
        // }

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
};

/// @brief Derive this class and implement your algorithm in the `plan` method. 
class MyPRM : public PRM2D {
    public:
        MyPRM()
        {
            n = 2000;
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
                xSamp = hw7U.round_double(hw7U.random<double>(xMin, xMax), dec_per);
                ySamp = hw7U.round_double(hw7U.random<double>(yMin, yMax), dec_per);
                sampPt = {xSamp, ySamp};

                if(find(samps.begin(), samps.end(), sampPt) == samps.end())
                {
                    if(hw7U.checkInObj(sampPt, obs))
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
                    if(hw7U.inRadius(stPtM.first,tstPt,r) && !hw7U.checkInObj(stPtM.first, tstPt, obs))
                    {
                        RM[stPtM.first].insert({tstPt, hw7U.euc_dis(stPtM.first, tstPt)});
                    }
                }
            }
            myUtils::MapSearchResult dijRet = hw7U.dij_search(RM, st, g);
            
            if(smooth && dijRet.success)
            {
                smoothPath.push_back(dijRet.node_path.front());
                pair<double, double> prev;
                prev = smoothPath.back();
                for(auto pt : dijRet.node_path)
                {
                    if(!hw7U.checkInObj(smoothPath.back(), pt, obs))
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
                ret = hw7U.pairToEigenVector(smoothPath);
                valid_path = true;
                // printf("Smoothed ");
                // hw7U.printPath(ret);
            }
            else if(dijRet.success)
            {
                ret = hw7U.pairToEigenVector(dijRet.node_path);
                valid_path = true;
            }
            else
            {
                ret = hw7U.pairToEigenVector({st, g});
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

        virtual ~MyPRM() {}

    private:
        int n = 200;
        double r = 2.0;
        int dec_per = 2;
        bool valid_path;
        bool smooth = true;
        long time;
        map< pair<double,double>, map< pair<double,double>, double > > RM;
};



int main(int argc, char** argv)
{

    Problem2D wa  = HW5::getWorkspace1();
    Problem2D wb1 = HW2::getWorkspace1();
    Problem2D wb2 = HW2::getWorkspace2();

    int numRuns = 100;

    list<pair<int, double>> p1a{{200, 0.5}, {200, 1.0}, {200, 1.5}, {200, 2.0}, {500, 0.5}, {500, 1.0}, {500, 1.5}, {500,2.0}};
    list<pair<int, double>> p1b{{200, 1.0}, {200, 2.0}, {500, 1.0}, {500,2.0}, {1000, 1.0}, {1000,2.0}};
    list<pair<int, double>> p2{{200, 1.0}, {200, 2.0}, {500, 1.0}, {500,2.0}, {1000, 1.0}, {1000,2.0}};

    MyPRM prm;
    MyGBRRT rrt;
    if(false)
    {
        prm.benchMarkAlgo(p1a, numRuns, false, wa, "Ex 2 HW 5");
        prm.benchMarkAlgo(p1a, numRuns, true, wa, "Ex 2 HW 5 - Smoothed");

        prm.benchMarkAlgo(p1b, numRuns, false, wb1, "Ex 2 HW 2 WS 1");
        prm.benchMarkAlgo(p1b, numRuns, true, wb1, "Ex 2 HW 2 WS 1 - Smoothed");

        prm.benchMarkAlgo(p1b, numRuns, false, wb2, "Ex 2 HW 2 WS 2");
        prm.benchMarkAlgo(p1b, numRuns, true, wb2, "Ex 2 HW 2 WS 2 - Smoothed");
    }
    prm.setSmoothing(false);

    prm.setPtRadius(1);
    prm.setSampleSize(200);
    Path2D p1wa = prm.plan(wa);
    prm.setPtRadius(2);
    Path2D p1wb2 = prm.plan(wb1);
    Path2D p1wb3 = prm.plan(wb2);



    tuple<int, double, double, double> prams{5000, 0.5, 0.05, 0.25};
    list<Problem2D> probs{wa, wb1, wb2};
    vector<string> ws{"Ex 2 HW 5", "Ex 2 HW 2 WS 1", "Ex 2 HW 2 WS 2"};

    if(false)
    {
        rrt.benchMarkAlgo(prams, numRuns, false, probs, ws);
    }

    Path2D p2wa = rrt.plan(wa);
    Path2D p2wb2 = rrt.plan(wb1);
    Path2D p2wb3 = rrt.plan(wb2);

    MyPRM prm_grade;
    MyGBRRT rrt_grade;

    // HW7::grade(prm_grade, rrt_grade, "corey.huffman@colorado.edu", argc, argv);

    return 0;
}