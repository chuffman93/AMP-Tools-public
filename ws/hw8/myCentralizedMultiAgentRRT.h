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
        /// @brief Solve a motion planning problem. Derive class and override this method
        /// @param problem Multi-agent motion planning problem
        /// @return Array of paths that are ordered corresponding to the `agent_properties` field in `problem`.
        virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override
        {
            MultiAgentPath2D ret;


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
        myUtils myCMA;
};