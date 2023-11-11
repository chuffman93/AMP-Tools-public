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

        vector<double> grabRandPoint(vector<double> g, size_t numAgents, double xmin, double xmax, double ymin, double ymax)
        {
            bool grabG = (rand() % 100) <= (p_goal*100);
            if(grabG)
            {
                return g;
            }
            else
            {
                vector<double> ret;
                for(int i = 0; i < numAgents; i++)
                {
                    ret.push_back(myCMA.round_double(myCMA.random<double>(xmin, xmax), dec_per));
                    ret.push_back(myCMA.round_double(myCMA.random<double>(ymin, ymax), dec_per));
                }
                return ret;
            }
        }

        bool isSystemInPoint(vector<double> q, vector<double> g, size_t numAgents)
        {
            for(int i = 0; i < numAgents*2; i+=2)
            {
                if(myCMA.euc_dis(make_pair(q[i],q[i+1]), make_pair(g[i],g[i+1])) > eps)
                {
                    return false;
                }
            }
            return true;
        }

        bool validState(vector<double> q, vector<double> radii, size_t numAgents, vector<Obstacle2D> obs, bool printDis = false)
        {
            if(printDis) printf("Num of agents = %ld \n", numAgents);
            for(int i = 0; i < numAgents*2; i+=2)
            {
                if(rToOCollision({q[i], q[i+1]}, radii[(int)(i/2)], obs))
                {
                    if(printDis) printf("Object Collision for Robot %d\n", (int)(i/2));
                    return false;
                }
                for(int j = i+2; j < numAgents*2; j+=2)
                {
                    if(rToRCollision({q[i], q[i+1]}, {q[j], q[j+1]}, radii[(int)(i/2)], radii[(int)(j/2)]))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        bool rToRCollision(pair<double, double> a, pair<double, double> b, double ra, double rb)
        {
            return (myCMA.euc_dis(a, b) < ra+rb+0.1);
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
            vector<double> st;
            vector<double> g;

            for(auto agent : agents)
            {
                radii.push_back(agent.radius);
                st.push_back(agent.q_init[0]);
                st.push_back(agent.q_init[1]);
                g.push_back(agent.q_goal[0]);
                g.push_back(agent.q_goal[1]);

            }
            vector<double> smpPt;
            vector<double> qNear;
            vector<double> qNew;

            RM.insert(make_pair(st, map<vector<double>,double>()));

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
                if(!validState(smpPt, radii, numAgents, obs))
                {
                    // if(smpPt == g)
                    // {
                    //     printf("Resampling from goal \n");
                    //     sleep(2);
                    // }
                    continue;
                }
                tmp_dist = INFINITY;

                for(auto itrRm : RM)
                {
                    if(itrRm.first == smpPt)
                    {
                        continue;
                    }
                    if(tmp_dist > myCMA.mink_dis(itrRm.first, smpPt, 1.0))
                    {
                        tmp_dist = myCMA.mink_dis(itrRm.first, smpPt, 1.0);
                        qNear = itrRm.first;
                    }
                }
                qNew = myCMA.newPt(qNear, smpPt, numAgents, r, dec_per);

                if(validState(qNew, radii, numAgents, obs) &&  myCMA.inbounds(qNew, xMin, xMax, yMin, yMax))
                {
                    // if(smpPt == g)
                    // {
                    //     printf("qNew = {%.3f %.3f %.3f %.3f} qNear = {%.3f %.3f %.3f %.3f} g = {%.3f %.3f %.3f %.3f}\n",
                    //     qNew[0], qNew[1], qNew[2], qNew[3],
                    //     qNear[0], qNear[1], qNear[2], qNear[3],
                    //     g[0], g[1], g[2], g[3]);
                    // }
                    
                    if(isSystemInPoint(qNew, g, numAgents))
                    {
                        found = true;
                        qNew = g;
                        if(RM.find(qNew) == RM.end())
                        {
                            RM.insert(make_pair(qNew, map<vector<double>,double>()));
                        }
                        RM[qNear][qNew] = myCMA.mink_dis(qNear, qNew, 1.0);
                    }
                    else if(isSystemInPoint(qNew, smpPt, numAgents))
                    {
                        qNew = smpPt;
                        if(RM.find(qNew) == RM.end())
                        {
                            RM.insert(make_pair(qNew, map<vector<double>,double>()));
                        }
                        RM[qNear][qNew] = myCMA.mink_dis(qNear, qNew, 1.0); 
                    }
                    else 
                    {
                        if(RM.find(qNew) == RM.end())
                        {
                            RM.insert(make_pair(qNew, map<vector<double>,double>()));
                        }
                        RM[qNear][qNew] = myCMA.mink_dis(qNear, qNew, 1.0); 
                    }
                    qNear = qNew;
                    qNew = myCMA.newPt(qNear, smpPt, numAgents, r, dec_per);
                    itr+=1;
                }
            }
            // if(!found)
            // {
            //     printf("PATH NOT FOUND\n");
            // }
            myUtils::MapSearchResultMultiAgent dijRet = myCMA.dij_search(RM, st, g);
            if(dijRet.success)
            {
                ret = myCMA.pairToEigenVector(dijRet.node_path, numAgents);
                valid_path = true;
                ret.valid = valid_path;
                treeSize = RM.size();
            }
            else
            {
                ret = myCMA.pairToEigenVector({st,g}, numAgents);
                valid_path = false;
                ret.valid = valid_path;
                // if(found) printf("path not found with dij\n");
            }
            auto stpTime = high_resolution_clock::now();
            time = duration_cast<milliseconds>(stpTime - stTime).count();

            return ret;
        }

        struct benchRes {
            double at;
            double as;
        };

        benchRes benchMarkAlgo(int numRuns, MultiAgentProblem2D prob)
        {
            int validRun = 0;
            MultiAgentPath2D ret;
            benchRes Ret;
            vector<double> tmp_lens;
            list<vector<double>> path_lens;
            vector<double> tmp_times;
            list<vector<double>> times;

            char trSt[50];
            char ctSt[50];
            char numSt[50];

            sprintf(trSt, "CMA Tree Size for %ld agents", prob.numAgents());
            sprintf(ctSt, "CMA Computation Time for %ld agents", prob.numAgents());


            auto stTime = high_resolution_clock::now();
            while(validRun < numRuns && duration_cast<seconds>(high_resolution_clock::now() - stTime).count() < 600)
            {
                ret = plan(prob);
                if(getValid())
                {
                    tmp_lens.push_back(getTreeSize());
                    tmp_times.push_back(getTime());
                    validRun++;
                }
            }

            sprintf(numSt, "Results for %d runs and %ld agents", validRun, prob.numAgents());

            path_lens.push_back(tmp_lens);
            times.push_back(tmp_times);
            Visualizer::makeBoxPlot(path_lens, {numSt}, trSt, "", "Size [Nodes]");
            Visualizer::makeBoxPlot(times, {numSt}, ctSt, "", "Time [ms]");
            Ret.at = accumulate(tmp_times.begin(), tmp_times.end(), 0.0) / tmp_times.size();
            Ret.as = accumulate(tmp_lens.begin(), tmp_lens.end(), 0.0) / tmp_lens.size();
            return Ret;
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

        double getTreeSize()
        {
            return treeSize;
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
        map< vector<double>, map< vector<double>, double> > RM;
        myUtils myCMA;
};