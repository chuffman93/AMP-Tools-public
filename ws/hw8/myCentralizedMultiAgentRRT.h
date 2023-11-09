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

        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override
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

            RM.inset(make_pair(st, map<vector<pair<double,double>>,double>()));

            bool found = false;
            bool inObj = false;

            double itr = 0;
            
            double xMin = problem.x_min;
            double xMax = problem.x_max;

            double yMin = problem.y_min;
            double yMax = problem.y_max;
            
            auto stTime = high_resolution_clock::now();

            while(!found && itr < n)
            {
                itr += 1;
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
        double r;
        double eps;
        double p_goal;
        int dec_per;
        long time;
        bool valid_path;
        map< vector<pair<double,double>>, map< vector<pair<double,double>>, double> > RM;
        myUtils myCMA;
};