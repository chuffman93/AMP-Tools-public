#include "MyBug2Algorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBug2Algorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    path.waypoints.push_back(problem.q_init);
    path.waypoints.push_back(problem.q_goal);

    return path;
}
