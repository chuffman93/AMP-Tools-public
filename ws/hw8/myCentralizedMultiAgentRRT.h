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

        virtual ~myCentralizedMultiAgentRRT() {}

    private:
        myUtils myCMA;
};