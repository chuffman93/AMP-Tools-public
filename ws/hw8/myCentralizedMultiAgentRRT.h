#include "AMPCore.h"
#include "hw/HW8.h"

#ifndef MYUTILS_H
#include "myUtils.h"
#endif

using namespace amp;
using namespace std;
using namespace chrono;


class myCentralizedMultiAgentRRT : public CentralizedMultiAgentRRT{
    public:
        myCentralizedMultiAgentRRT()
        {
            n = 7500;
            r = 0.5;
            p_goal = 0.05;
            eps = 0.25;
            dec_per = 2;
        }

        vector<pair<double,double>> grabRandPoint(vector<pair<double,double>> g, size_t numAgents, double xmin, double xmax, double ymin, double ymax)
        {
            bool grabG = (rand() % 100) <= (p_goal*100);
            if(grabG)
            {
                return g;
            }
            else
            {
                vector<pair<double,double>> ret;
                for(int i = 0; i < numAgents; i++)
                {
                    ret.push_back(make_pair(myCMA.round_double(myCMA.random<double>(xmin, xmax), dec_per), myCMA.round_double(myCMA.random<double>(ymin, ymax), dec_per)));
                }
                return ret;
            }
        }

        bool isSystemInGoal(vector<pair<double, double>> q, vector<pair<double, double>> g, size_t numAgents)
        {
            for(int i = 0; i < numAgents; i++)
            {
                if(myCMA.euc_dis(q[i], g[i]) > eps)
                {
                    return false;
                }
            }
            return true;
        }

        vector<bool> goalsReached(vector<pair<double, double>> q, vector<pair<double, double>> g, size_t numAgents)
        {
            vector<bool> ret;
            for(int i = 0; i < numAgents; i++)
            {
                if(myCMA.euc_dis(q[i], g[i]) > eps)
                {
                    ret.push_back(false);
                }
                else
                {
                    ret.push_back(true);
                }
            }
            return ret;
        }

        bool validState(vector<pair<double, double>> q, vector<double> radii, size_t numAgents, vector<Obstacle2D> obs)
        {
            for(int i = 0; i < numAgents; i++)
            {
                if(rToOCollision(q[i], radii[i], obs))
                {
                    return false;
                }
                for(int j = i+1; j < numAgents; j++)
                {
                    if(rToRCollision(q[i], q[j], radii[i], radii[j]))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        bool rToRCollision(pair<double, double> a, pair<double, double> b, double ra, double rb)
        {
            return (myCMA.round_double(myCMA.euc_dis(a, b),dec_per) < ra+rb);
        }

        bool rToOCollision(pair<double, double> a, double ra, vector<Obstacle2D> obs)
        {
            return myCMA.checkInObj(a, ra, obs, dec_per) || myCMA.checkInObj(a,obs);
        }

        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        virtual MultiAgentPath2D plan(const MultiAgentProblem2D& problem) override
        {
            MultiAgentPath2D ret;
            RM.clear();

            size_t numAgents = problem.numAgents();
            vector<CircularAgentProperties> agents = problem.agent_properties;
            vector<Obstacle2D> obs = problem.obstacles;
            vector<Eigen::Vector2d> verts; 

            vector<double> radii;
            vector<pair<double,double>> st;
            vector<pair<double,double>> g;

            for(auto agent : agents)
            {
                radii.push_back(agent.radius);
                st.push_back({agent.q_init[0],agent.q_init[1]});
                g.push_back({agent.q_goal[0],agent.q_goal[1]});

            }
            vector<pair<double,double>> smpPt;
            vector<pair<double,double>> qNear;
            vector<pair<double,double>> qNew;

            RM.insert(make_pair(st, map<vector<pair<double,double>>,double>()));

            bool found = false;
            bool inObj = false;

            double itr = 0;

            double xMin = problem.x_min;
            double xMax = problem.x_max;

            double yMin = problem.y_min;
            double yMax = problem.y_max;

            double tmp_dist;
            
            auto stTime = high_resolution_clock::now();

            while(!found && itr < n)
            {
                smpPt = grabRandPoint(g, numAgents, xMin, xMax, yMin, yMax);
                tmp_dist = INFINITY;

                for(auto itrRm : RM)
                {
                    if(itrRm.first == smpPt)
                    {
                        continue;
                    }
                    if(tmp_dist > myCMA.euc_dis(itrRm.first, smpPt))
                    {
                        tmp_dist = myCMA.euc_dis(itrRm.first, smpPt);
                        qNear = itrRm.first;
                    }
                }
                qNew = myCMA.newPt(qNear, smpPt, numAgents, r);


                if(validState(qNew, radii, numAgents, obs))
                {
                    if(isSystemInGoal(qNew, g, numAgents))
                    {
                        found = true;
                        qNew = g;
                        if(RM.find(qNew) == RM.end())
                        {
                            RM.insert(make_pair(qNew, map<vector<pair<double,double>>,double>()));
                        }
                        RM[qNear][qNew] = myCMA.euc_dis(qNear, qNew);
                    }
                    else
                    {
                        if(RM.find(qNew) == RM.end())
                        {
                            RM.insert(make_pair(qNew, map<vector<pair<double,double>>,double>()));
                        }
                        RM[qNear][qNew] = myCMA.euc_dis(qNear, qNew); 
                    }
                    itr += 1; 
                }
                
            }
            if(!found)
            {
                printf("PATH NOT FOUND\n");
            }
            myUtils::MapSearchResultMultiAgent dijRet = myCMA.dij_search(RM, st, g);
            if(dijRet.success)
            {
                ret = myCMA.pairToEigenVector(dijRet.node_path, numAgents);
                valid_path = true;
                ret.valid = valid_path;
            }
            else
            {
                ret = myCMA.pairToEigenVector({st,g}, numAgents);
                valid_path = false;
                ret.valid = valid_path;
            }
            auto stpTime = high_resolution_clock::now();
            time = duration_cast<milliseconds>(stpTime - stTime).count();

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

        long getTime()
        {
            return time;
        }

        bool getValid()
        {
            return valid_path;
        }

        virtual ~myCentralizedMultiAgentRRT() {}

    private:
        int n;
        int dec_per;
        double r;
        double eps;
        double p_goal;
        double treeSize;
        long time;
        bool valid_path;
        map< vector<pair<double,double>>, map< vector<pair<double,double>>, double> > RM;
        myUtils myCMA;
};