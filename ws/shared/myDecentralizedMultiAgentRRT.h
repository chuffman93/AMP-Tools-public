#include "AMPCore.h"
#include "hw/HW8.h"

#ifndef MYUTILS_H
#include "myUtils.h"
#endif

using namespace amp;
using namespace std;
using namespace chrono;

class myDecentralizedMultiAgentRRT : public DecentralizedMultiAgentRRT {
    public:
        myDecentralizedMultiAgentRRT()
        {
            n = 4000;
            r = 0.5;
            R = 1.0;
            p_goal = 0.05;
            eps = 0.25;
            dec_per = 2;
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
            }
            return true;
        }

        bool rToOCollision(pair<double, double> a, double ra, vector<Obstacle2D> obs)
        {
            return myDMA.checkInObj(a, ra, obs, dec_per) || myDMA.checkInObj(a,obs);
        }

        bool isSystemInPoint(vector<double> q, vector<double> g, size_t numAgents)
        {
            for(int i = 0; i < numAgents*2; i+=2)
            {
                if(myDMA.euc_dis(make_pair(q[i],q[i+1]), make_pair(g[i],g[i+1])) > eps)
                {
                    return false;
                }
            }
            return true;
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
                    ret.push_back(myDMA.round_double(myDMA.random<double>(xmin, xmax), dec_per));
                    ret.push_back(myDMA.round_double(myDMA.random<double>(ymin, ymax), dec_per));
                }
                return ret;
            }
        }

        vector<Path2D> timingFunction(vector<Path2D> init)
        {
            vector<Path2D> ret;
            Path2D tmp;
            if(init.size() == 1)
            {
                return init;
            }

            for(int i = 0; i < init.size(); i++)
            {
                if(i == 0)
                {
                    ret.push_back(init[i]);
                    continue;
                }
                tmp = init[i];
                for(int j = 0; j < ret[i-1].waypoints.size(); j++)
                {
                    tmp.waypoints.insert(tmp.waypoints.begin(), tmp.waypoints.front());
                }
                ret.push_back(tmp);

            }
            return ret;
        }

        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override
        {
            MultiAgentPath2D ret;
            RM.clear();

            size_t numAgents = 1;
            size_t nA = problem.numAgents();
            vector<CircularAgentProperties> agents = problem.agent_properties;
            vector<Obstacle2D> obs = problem.obstacles;
            vector<Eigen::Vector2d> verts; 

            vector<double> radii;
            vector<double> st;
            vector<double> g;

            vector<double> smpPt;
            vector<double> qNear;
            vector<double> qNew;

            bool found = false;
            bool inObj = false;

            double itr = 0;

            double xMin = problem.x_min;
            double xMax = problem.x_max;

            double yMin = problem.y_min;
            double yMax = problem.y_max;

            double tmp_dist;
            totDis = 0;

            auto stTime = high_resolution_clock::now();
            for(auto agent : agents)
            {
                radii.push_back(agent.radius);
                st.push_back(agent.q_init[0]);
                st.push_back(agent.q_init[1]);
                g.push_back(agent.q_goal[0]);
                g.push_back(agent.q_goal[1]);

                RM.insert(make_pair(st, map<vector<double>,double>()));

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
                        if(tmp_dist > myDMA.mink_dis(itrRm.first, smpPt, 1.0))
                        {
                            tmp_dist = myDMA.mink_dis(itrRm.first, smpPt, 1.0);
                            qNear = itrRm.first;
                        }
                    }
                    qNew = myDMA.newPt(qNear, smpPt, numAgents, r, dec_per);

                    if(validState(qNew, radii, numAgents, obs) &&  myDMA.inbounds(qNew, xMin, xMax, yMin, yMax))
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
                            RM[qNear][qNew] = myDMA.mink_dis(qNear, qNew, 1.0);
                        }
                        else if(isSystemInPoint(qNew, smpPt, numAgents))
                        {
                            qNew = smpPt;
                            if(RM.find(qNew) == RM.end())
                            {
                                RM.insert(make_pair(qNew, map<vector<double>,double>()));
                            }
                            RM[qNear][qNew] = myDMA.mink_dis(qNear, qNew, 1.0); 
                        }
                        else 
                        {
                            if(RM.find(qNew) == RM.end())
                            {
                                RM.insert(make_pair(qNew, map<vector<double>,double>()));
                            }
                            RM[qNear][qNew] = myDMA.mink_dis(qNear, qNew, 1.0); 
                        }
                        qNear = qNew;
                        qNew = myDMA.newPt(qNear, smpPt, numAgents, r, dec_per);
                        itr+=1;
                    }
                }
                // if(!found)
                // {
                //     printf("PATH NOT FOUND\n");
                // }
                // else
                // {
                //     printf("Path found\n");
                // }
                myUtils::MapSearchResultMultiAgent dijRet = myDMA.dij_search(RM, st, g);
                if(dijRet.success)
                {
                    ret.agent_paths.push_back(myDMA.pairToEigenVector(dijRet.node_path));
                    valid_path = true;
                    ret.valid = valid_path;
                    totDis+= dijRet.path_cost;
                }
                else
                {
                    ret.agent_paths.push_back(myDMA.pairToEigenVector({st,g}));
                    valid_path = false;
                    ret.valid = valid_path;
                    // if(found) printf("path not found with dij\n");   
                }   

                st.clear();
                g.clear();
                radii.clear();
                RM.clear();
                itr = 0;
                found = false;

            }

            vector<Path2D> tmpRet = timingFunction(ret.agent_paths);
            ret.agent_paths = tmpRet;
            
            auto stpTime = high_resolution_clock::now();
            time = duration_cast<milliseconds>(stpTime - stTime).count();

            return ret;
        }

        pair<vector<double>, list<vector<double>>> findNeighbors(map<vector<double>, double> Cost, vector<double> q, size_t numA, vector<double> radii, vector<Polygon> obs)
        {
            list<vector<double>> retList;
            vector<double> retBest;

            double minDis = INFINITY;
            double tmpDis;
            double tmpCost;

            for(auto itr : RM)
            {
                tmpDis = myDMA.mink_dis(itr.first, q, 2.0);
                if((tmpDis < R))
                {
                    tmpCost = Cost[itr.first] + tmpDis;
                    if(minDis > tmpCost)
                    {
                        minDis = tmpCost;
                        retBest = itr.first;
                    }
                    retList.push_back(itr.first);
                }
            }
            return make_pair(retBest, retList);
        }

        amp::MultiAgentPath2D planStar(const amp::MultiAgentProblem2D& problem)
        {
            MultiAgentPath2D ret;
            RM.clear();

            size_t numAgents = 1;
            size_t nA = problem.numAgents();
            vector<CircularAgentProperties> agents = problem.agent_properties;
            map<vector<double>, double> costMap;
            vector<Obstacle2D> obs = problem.obstacles;
            vector<Eigen::Vector2d> verts; 

            vector<double> radii;
            vector<double> st;
            vector<double> g;

            vector<double> smpPt;
            vector<double> qNear;
            vector<double> qNew;

            bool found = false;
            bool inObj = false;

            double itr = 0;

            double xMin = problem.x_min;
            double xMax = problem.x_max;

            double yMin = problem.y_min;
            double yMax = problem.y_max;

            double tmp_dist;

            totDis = 0;

            pair<vector<double>, list<vector<double>>> bestNeighbors;

            auto stTime = high_resolution_clock::now();
            for(auto agent : agents)
            {
                radii.push_back(agent.radius);
                st.push_back(agent.q_init[0]);
                st.push_back(agent.q_init[1]);
                g.push_back(agent.q_goal[0]);
                g.push_back(agent.q_goal[1]);

                RM.insert(make_pair(st, map<vector<double>,double>()));
                costMap.insert(make_pair(st, 0.0));

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
                        if(tmp_dist > myDMA.mink_dis(itrRm.first, smpPt, 2.0))
                        {
                            tmp_dist = myDMA.mink_dis(itrRm.first, smpPt, 2.0);
                            qNear = itrRm.first;
                        }
                    }
                    qNew = myDMA.newPt(qNear, smpPt, numAgents, r, dec_per);

                    if(validState(qNew, radii, numAgents, obs) &&  myDMA.inbounds(qNew, xMin, xMax, yMin, yMax))
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
                            qNew = g;
                        }
                        bestNeighbors = findNeighbors(costMap, qNew, numAgents, radii ,obs);
                        RM[bestNeighbors.first][qNew] = myDMA.mink_dis(bestNeighbors.first,qNew,2.0);
                        if(RM.find(qNew) == RM.end())
                        {   
                            RM.insert( make_pair(qNew, map<vector<double>,double>()) );
                            costMap.insert(make_pair(qNew, costMap[bestNeighbors.first]+RM[bestNeighbors.first][qNew]));
                        }
                        for(auto x : bestNeighbors.second)
                        {
                            if(costMap[x] > costMap[qNew] + myDMA.mink_dis(qNew,x,2.0))
                            {
                                for(auto itr: RM)
                                {
                                    RM[itr.first].erase(x);
                                }
                                RM[qNew][x] = myDMA.mink_dis(qNew,x,2.0);
                                costMap[x] = costMap[qNew]+RM[qNew][x];
                            }
                        }
                    }
                    else
                    {
                        continue;
                    }
                    itr+=1;
                }
                // if(!found)
                // {
                //     printf("PATH NOT FOUND\n");
                // }
                // else
                // {
                //     printf("Path found\n");
                // }
                myUtils::MapSearchResultMultiAgent dijRet = myDMA.dij_search(RM, st, g);
                if(dijRet.success)
                {
                    ret.agent_paths.push_back(myDMA.pairToEigenVector(dijRet.node_path));
                    valid_path = true;
                    ret.valid = valid_path;
                    totDis+= dijRet.path_cost;
                }
                else
                {
                    ret.agent_paths.push_back(myDMA.pairToEigenVector({st,g}));
                    valid_path = false;
                    ret.valid = valid_path;
                    // if(found) printf("path not found with dij\n");   
                }   

                st.clear();
                g.clear();
                radii.clear();
                RM.clear();
                costMap.clear();
                itr = 0;
                found = false;

            }

            vector<Path2D> tmpRet = timingFunction(ret.agent_paths);
            ret.agent_paths = tmpRet;
            
            auto stpTime = high_resolution_clock::now();
            time = duration_cast<milliseconds>(stpTime - stTime).count();

            return ret;
        }

        tuple<double, vector<double>, vector<double>> benchMarkAlgo(int numRuns, MultiAgentProblem2D prob)
        {
            int validRun = 0;

            MultiAgentPath2D ret;
            vector<double> tmp_times;
            vector<double> tmp_dis;
            list<vector<double>> times;
            list<vector<double>> dis;

            char ctSt[50];
            char numSt[50];

            sprintf(ctSt, "DMA Computation Time for w/ %ld agents", prob.numAgents());


            auto stTime = high_resolution_clock::now();
            while(validRun < numRuns && duration_cast<seconds>(high_resolution_clock::now() - stTime).count() < 300)
            {
                ret = plan(prob);
                if(getValid())
                {
                    tmp_times.push_back(getTime());
                    tmp_dis.push_back(getTotDis());
                    validRun++;
                }
            }

            // sprintf(numSt, "Results for %d runs and %ld agents", validRun, prob.numAgents());

            times.push_back(myDMA.removeOutliers(tmp_times));
            dis.push_back(tmp_dis);
            // Visualizer::makeBoxPlot(times, {numSt}, ctSt, "", "Time [ms]");

            return make_tuple(accumulate(tmp_times.begin(), tmp_times.end(), 0.0) / tmp_times.size(), tmp_times, tmp_dis);
        }

        tuple<double, vector<double>, vector<double>> benchMarkAlgoSt(int numRuns, MultiAgentProblem2D prob)
        {
            int validRun = 0;

            MultiAgentPath2D ret;
            vector<double> tmp_times;
            vector<double> tmp_dis;
            list<vector<double>> times;
            list<vector<double>> dis;

            char ctSt[50];
            char numSt[50];

            sprintf(ctSt, "DMA Computation Time for w/ %ld agents", prob.numAgents());


            auto stTime = high_resolution_clock::now();
            while(validRun < numRuns && duration_cast<seconds>(high_resolution_clock::now() - stTime).count() < 300)
            {
                ret = planStar(prob);
                if(getValid())
                {
                    tmp_times.push_back(getTime());
                    tmp_dis.push_back(getTotDis());
                    validRun++;
                }
            }

            // sprintf(numSt, "Results for %d runs and %ld agents", validRun, prob.numAgents());

            times.push_back(myDMA.removeOutliers(tmp_times));
            dis.push_back(tmp_dis);
            // Visualizer::makeBoxPlot(times, {numSt}, ctSt, "", "Time [ms]");

            return make_tuple(accumulate(tmp_times.begin(), tmp_times.end(), 0.0) / tmp_times.size(), tmp_times, tmp_dis);
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

        double getTotDis()
        {
            return totDis;
        }

        virtual ~myDecentralizedMultiAgentRRT() {}
    
    private:
        int n;
        int dec_per;
        double r;
        double R;
        double eps;
        double p_goal;
        double treeSize;
        long time;
        double totDis;
        bool valid_path;
        map< vector<double>, map< vector<double>, double> > RM;
        myUtils myDMA;
};