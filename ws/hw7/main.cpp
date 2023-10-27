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
            n = 5000;
            r = 0.5;
            eps = 0.5;
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
            map< pair<double,double>, map< pair<double,double>, double > > RM;
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

                    if (tmp_dis > hw7U.euc_dis(itr.first, sampPt))
                    {
                        tmp_dis = hw7U.euc_dis(itr.first, sampPt);
                        qNear = itr.first;
                    }
                }
                qNew = hw7U.newPt(qNear, sampPt, r);
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
                printf("Smoothed ");
                hw7U.printPath(ret);
            }
            else if(dijRet.success)
            {
                ret = hw7U.pairToEigenVector(dijRet.node_path);
            }
            auto stpTime = high_resolution_clock::now();
            printf("Run Time for Algorithm: %ld ms\n", duration_cast<milliseconds>(stpTime-stTime).count());

            return ret;
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


        virtual ~MyGBRRT() {}
    
    private:
        int n;
        double r;
        double eps;
        double p_goal;
        int dec_per;
        bool smooth;
};

/// @brief Derive this class and implement your algorithm in the `plan` method. 
class MyPRM : public PRM2D {
    public:
        MyPRM()
        {
            n = 500;
            r = 2.0;
            dec_per = 2;
            smooth = true;
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

        /// @brief Solve a motion planning problem. Create a derived class and override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override
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
                printf("Smoothed ");
                hw7U.printPath(ret);
            }
            else if(dijRet.success)
            {
                ret = hw7U.pairToEigenVector(dijRet.node_path);
            }
            auto stpTime = high_resolution_clock::now();
            printf("Run Time for Algorithm: %ld ms\n", duration_cast<milliseconds>(stpTime-stTime).count());
            return ret;
        }

        virtual ~MyPRM() {}

    private:
        int n = 200;
        double r = 2.0;
        int dec_per = 2;
        bool smooth = true;
};



int main(int argc, char** argv)
{

    Problem2D wa  = HW5::getWorkspace1();
    Problem2D wb1 = HW2::getWorkspace1();
    Problem2D wb2 = HW2::getWorkspace2();

    MyPRM prm;
    MyGBRRT rrt;

    Path2D p1wa = prm.plan(wa);
    Path2D p1wb2 = prm.plan(wb1);
    Path2D p1wb3 = prm.plan(wb2);

    return 0;
}