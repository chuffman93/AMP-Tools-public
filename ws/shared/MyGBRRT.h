#include <queue>
#include <chrono>

#ifndef  MYUTILS_H
#include "myUtils.h"
#endif
#include "AMPCore.h"
#include "hw/HW7.h"

using namespace amp;
using namespace std;
using namespace chrono;

myUtils gbrrtUtilstils;

/// @brief Derive this class and implement your algorithm in the `plan` method. 
class MyGBRRT : public GoalBiasRRT2D {
    public:
        MyGBRRT()
        {
            n = 7500;
            r = 0.5;
            eps = 0.25;
            p_goal = 0.05;
            dec_per = 2;
            smooth = false;
            dis = 0.1;
        }

        pair<double, double> grabRandPoint(pair<double, double> g, double xmin, double xmax, double ymin, double ymax)
        {
            bool grabG = (rand() % 100) <= (int)(p_goal*100);
            return grabG ? g : make_pair(gbrrtUtils.round_double(gbrrtUtils.random<double>(xmin, xmax), dec_per), gbrrtUtils.round_double(gbrrtUtils.random<double>(ymin, ymax), dec_per));
        }

        ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) 
        {
            ASSERT(link_manipulator_agent.nLinks() == 2, "Manipulator must have two links");

            ManipulatorState init_state = link_manipulator_agent.getConfigurationFromIK(problem.q_init);

            ManipulatorState goal_state = link_manipulator_agent.getConfigurationFromIK(problem.q_goal);

            unique_ptr<GridCSpace2D> grid_cspace = construct(link_manipulator_agent, problem);

            return planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
        }

        std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env)
        { 
            double x0min = 0.0;
            double x0max = 2*M_PI;
            double x1min = 0.0;
            double x1max = 2*M_PI;


            pair<size_t, size_t> tmp;

            vector<Obstacle2D> obs = env.obstacles;
            int n = manipulator.nLinks();
            std::size_t x0cells = ((abs(x0min) + abs(x0max)) / this->dis);
            std::size_t x1cells = ((abs(x1min) + abs(x1max)) / this->dis);
            unique_ptr<MyGridCSpace> ret = std::make_unique<MyGridCSpace>(this->dis, x0cells, x1cells, x0min, x0max, x1min, x1max);
            vector<myUtils::line> links; 

            try 
            {
                x0cells = ((abs(x0min) + abs(x0max)) / this->dis);
                x1cells = ((abs(x1min) + abs(x1max)) / this->dis);
                ret = std::make_unique<MyGridCSpace>(this->dis, x0cells, x1cells, x0min, x0max, x1min, x1max);
                links.clear(); 

                for(double i = x0min; i < x0max; i+=(this->dis/3) )
                {
                    for(double j = x1min; j < x1max; j+=(this->dis/3) )
                    {
                    
                        tmp = ret->getCellFromPoint(i,j);
                        if(ret->operator()(tmp.first, tmp.second))
                        {
                            continue;
                        }          
                        for(int k = 0; k < n; k++)
                        {
                            links.push_back(myUtils::line{manipulator.getJointLocation(Eigen::Vector2d{i,j},k),manipulator.getJointLocation(Eigen::Vector2d{i,j},k+1)});
                        }
                        for(int k = 0; k < obs.size(); k++)
                        {
                            vector<Eigen::Vector2d> verts = obs[k].verticesCCW();
                            verts.push_back(verts[0]); 
                            if(gbrrtUtilstils.checkInObj(links, verts))
                            {
                                ret->operator()(tmp.first, tmp.second) = true;
                                break;
                            }
                        }

                        links.clear();
                    }
                }
            }
            catch(...)
            {
                x0cells = ceil((abs(x0min) + abs(x0max)) / this->dis);
                x1cells = ceil((abs(x1min) + abs(x1max)) / this->dis);
                ret = std::make_unique<MyGridCSpace>(this->dis, x0cells, x1cells, x0min, x0max, x1min, x1max);
                links.clear(); 

                for(double i = x0min; i < x0max; i+=(this->dis/3) )
                {
                    for(double j = x1min; j < x1max; j+=(this->dis/3) )
                    {
                    
                        tmp = ret->getCellFromPoint(i,j);
                        if(ret->operator()(tmp.first, tmp.second))
                        {
                            continue;
                        }          
                        for(int k = 0; k < n; k++)
                        {
                            links.push_back(myUtils::line{manipulator.getJointLocation(Eigen::Vector2d{i,j},k),manipulator.getJointLocation(Eigen::Vector2d{i,j},k+1)});
                        }
                        for(int k = 0; k < obs.size(); k++)
                        {
                            vector<Eigen::Vector2d> verts = obs[k].verticesCCW();
                            verts.push_back(verts[0]); 
                            if(gbrrtUtilstils.checkInObj(links, verts))
                            {
                                ret->operator()(tmp.first, tmp.second) = true;
                                break;
                            }
                        }

                        links.clear();
                    }
                }
            } 

            return ret;
        }

        pair<double, double> getPointFromCell(double dis, std::pair<std::size_t, std::size_t> cell, double xMin, double yMin) {
            return make_pair(((xMin+(this->dis*cell.first)) + (xMin+(this->dis*(cell.first+1))))/2,
                             ((yMin+(this->dis*cell.second)) + (yMin+(this->dis*(cell.second+1))))/2);
        }
   

        Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) 
        {
            Path2D ret;
            RM.clear();
            double tmp_dist;

            // printf("------- RECEIVED -------\n");
            // printf("Goal Point is at angle: (%.3f & %.3f)\n",goal[0]*180/M_PI, goal[1]*180/M_PI);
            // printf("Start Point is at angle: (%.3f & %.3f)\n",init[0]*180/M_PI, init[1]*180/M_PI);

            // sleep(5);

            Eigen::Vector2d goal = q_goal;
            Eigen::Vector2d init = q_init;

            if(init[0] < 0)
            {
                init[0] = init[0] + 2*M_PI;
            }

            if(init[1] < 0)
            {
                init[1] = init[1] + 2*M_PI;
            }

            if(goal[0] < 0)
            {
                goal[0] = goal[0] + 2*M_PI;
            }

            if(goal[1] < 0)
            {
                goal[1] = goal[1] + 2*M_PI;
            }

            pair<size_t, size_t> stCell = grid_cspace.getCellFromPoint(init[0], init[1]);
            pair<size_t, size_t> gCell  = grid_cspace.getCellFromPoint(goal[0], goal[1]);
            pair<size_t, size_t> tmpCell;

            // printf("Start: {%.2f, %.2f} Stop: {%.2f, %.2f}\n", init[0], init[1], goal[0], goal[1]);

            if(grid_cspace.operator()(gCell.first, gCell.second))
            {
                printf("Correcting goal!!\n");
                goal = correctPosition(goal);
                gCell  = grid_cspace.getCellFromPoint(goal[0], goal[1]);
            }

            if(grid_cspace.operator()(stCell.first, stCell.second))
            {
                printf("Correcting start!!\n");
                init = correctPosition(init);
                stCell = grid_cspace.getCellFromPoint(init[0], init[1]);
            }

            if(grid_cspace.operator()(stCell.first, stCell.second) || grid_cspace.operator()(gCell.first, gCell.second))
            {
                printf("No proper start and/or finish... \n");
                ret.waypoints.push_back(init);
                ret.waypoints.push_back(goal);
                return ret;
            } 

            pair<double, double> st{init[0], init[1]};
            pair<double, double> g{goal[0], goal[1]};
            pair<double, double> sampPt;
            pair<double, double> qNear;
            pair<double, double> qNew;

            pair<double,double> x0 = grid_cspace.x0Bounds();
            pair<double,double> x1 = grid_cspace.x1Bounds();

            RM.insert( make_pair(st, map<pair<double,double>,double>()) );

            bool found = false;

            list<pair<double, double>> samps;
            list<pair<double, double>> smoothPath;
            samps.push_back(st);

            double xSamp;
            double ySamp;
            double itr = 0;

            double xMin = x0.first;
            double xMax = x0.second;

            double yMin = x1.first;
            double yMax = x1.second;  
            Eigen::Vector2d tmpVec;

            bool first = true;
            int attempts = 1;

            auto stTime = high_resolution_clock::now(); 
            while(!(attempts > 2) && !found)
            {    
                while(!found && itr < n)
                {
                    sampPt = grabRandPoint(g, xMin, xMax, yMin, yMax);
                    tmpCell = grid_cspace.getCellFromPoint(sampPt.first, sampPt.second);
                    if(grid_cspace.operator()(tmpCell.first, tmpCell.second))
                    {
                        continue;
                    }
                    sampPt = getPointFromCell(dis,tmpCell,xMin,yMin);
                    qNear = make_pair(0,0);
                    tmp_dist = INFINITY;
                    for(auto itr : RM)
                    {
                        if (itr.first == sampPt)
                            continue;

                        if (tmp_dist > gbrrtUtils.euc_dis(itr.first, sampPt))
                        {
                            tmp_dist = gbrrtUtils.euc_dis(itr.first, sampPt);
                            qNear = itr.first;
                        }
                    }
                    qNew = gbrrtUtils.newPt(qNear, sampPt, dis*2, first);
                    tmpVec = correctPosition(qNew);
                    qNew = make_pair(tmpVec[0], tmpVec[1]);
                    tmpCell = grid_cspace.getCellFromPoint(qNew.first, qNew.second);
                    qNew = getPointFromCell(dis,tmpCell,xMin,yMin);
                    
                    if(!grid_cspace.operator()(tmpCell.first, tmpCell.second))
                    {
                        if(qNew == g || (tmpCell.first == gCell.first && tmpCell.second == gCell.second))
                        {
                            found = true;
                            qNew = g;
                            if(RM.find(qNew) == RM.end())
                            {   
                                RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                            }
                            RM[qNear][qNew] = gbrrtUtils.euc_dis(qNear,qNew);
                        }
                        else 
                        {
                            if(RM.find(qNew) == RM.end())
                            {   
                                RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                            }
                            RM[qNear][qNew] = gbrrtUtils.euc_dis(qNear,qNew);
                        }
                        
                        qNear = qNew;
                        qNew = gbrrtUtils.newPt(qNear, sampPt, dis, first);
                        tmpVec = correctPosition(qNew);
                        qNew = make_pair(tmpVec[0], tmpVec[1]);
                        tmpCell = grid_cspace.getCellFromPoint(qNew.first, qNew.second);
                        qNew = getPointFromCell(dis,tmpCell,xMin,yMin);
                    }
                    itr+=1;
                }
                myUtils::MapSearchResult dijRet = gbrrtUtils.dij_search(RM, st, g);
                
                if(smooth && dijRet.success)
                {
                    smoothPath.push_back(dijRet.node_path.front());
                    pair<double, double> prev;
                    prev = smoothPath.back();
                    for(auto pt : dijRet.node_path)
                    {
                        tmpVec = correctPosition(Eigen::Vector2d{pt.first, pt.second});
                        if(!grid_cspace.operator()(tmpCell.first, tmpCell.second))
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
                    ret = gbrrtUtils.pairToEigenVector(smoothPath);
                    valid_path = true;
                    // printf("Smoothed ");
                    // gbrrtUtils.printPath(ret);
                }
                else if(dijRet.success)
                {
                    ret = gbrrtUtils.pairToEigenVector(dijRet.node_path);
                    valid_path = true;
                }
                else
                {
                    printf("No Valid Path Found\n");
                    ret = gbrrtUtils.pairToEigenVector({st, g});
                    valid_path = false;
                    itr = 0;
                    attempts++;
                    first = !first;
                }
            }
            finalItr = itr;
            auto stpTime = high_resolution_clock::now();
            time = duration_cast<milliseconds>(stpTime-stTime).count();
            // printf("Run Time for Algorithm: %ld ms\n", time);

            return ret;      
            
        }

        Eigen::Vector2d correctPosition(Eigen::Vector2d Ang)
        {
            Eigen::Vector2d ret;
            double atanxy;
            atanxy = Ang[0] + atan2( sin(Ang[1]),(1+cos(Ang[1])) );
            ret[1] = -Ang[1] + 2*M_PI;
            ret[0] = atanxy + atan2( sin(Ang[1]-2*M_PI),(1+cos(Ang[1]-2*M_PI)) );

            for(int i = 0; i < ret.size(); i++)
            {
                if(ret[i] > 2.0*M_PI || abs(ret[i] - 2.0*M_PI) < 1e-4)
                {
                    ret[i] = ret[i] - (2.0*M_PI);
                }
                else if(ret[i] < 0.0)
                {
                    ret[i] = (2.0*M_PI) + ret[i];
                }
            }
            return ret;
        }

        Eigen::Vector2d correctPosition(pair<double, double> Ang)
        {
            Eigen::Vector2d ret;
            double atanxy;
            atanxy = Ang.first + atan2( sin(Ang.second),(1+cos(Ang.second)) );
            // ret[1] = -Ang.second + 2*M_PI;
            // ret[0] = atanxy + atan2( sin(Ang.second-2*M_PI),(1+cos(Ang.second-2*M_PI)) );

            ret[1] = Ang.second;
            ret[0] = Ang.first;

            for(int i = 0; i < ret.size(); i++)
            {
                if(ret[i] > 2.0*M_PI || abs(ret[i] - 2.0*M_PI) < 1e-4)
                {
                    ret[i] = ret[i] - (2.0*M_PI);
                }
                else if(ret[i] < 0.0)
                {
                    ret[i] = (2.0*M_PI) + ret[i];
                }
            }
            return ret;
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
                    if(gbrrtUtils.checkInObj(Eigen::Vector2d{sampPt.first, sampPt.second}, verts))
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

                    if (tmp_dist > gbrrtUtils.euc_dis(itr.first, sampPt))
                    {
                        tmp_dist = gbrrtUtils.euc_dis(itr.first, sampPt);
                        qNear = itr.first;
                    }
                }
                qNew = gbrrtUtils.newPt(qNear, sampPt, r, true);
                
                if(!gbrrtUtils.checkInObj(qNew,obs) && !gbrrtUtils.checkInObj(qNear, qNew, obs) && gbrrtUtils.inbounds(qNew, xMin, xMax, yMin, yMax))
                {
                    if(qNew == g || gbrrtUtils.round_double(gbrrtUtils.euc_dis(qNew,g),2) <= eps)
                    {
                        found = true;
                        qNew = g;
                        if(RM.find(qNew) == RM.end())
                        {   
                            RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                        }
                        RM[qNear][qNew] = gbrrtUtils.euc_dis(qNear,qNew);
                    }
                    else if(gbrrtUtils.round_double(gbrrtUtils.euc_dis(qNew,sampPt),2) <= eps)
                    {
                        if(!gbrrtUtils.checkInObj(qNew, sampPt, obs))
                        {
                            qNew = sampPt;
                            if(RM.find(qNew) == RM.end())
                            {   
                                RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                            }
                            RM[qNear][qNew] = gbrrtUtils.euc_dis(qNear,qNew);
                        }
                    }
                    else 
                    {
                        if(RM.find(qNew) == RM.end())
                        {   
                            RM.insert( make_pair(qNew, map<pair<double,double>,double>()) );
                        }
                        RM[qNear][qNew] = gbrrtUtils.euc_dis(qNear,qNew);
                    }
                    
                    qNear = qNew;
                    qNew = gbrrtUtils.newPt(qNear, sampPt, r, true);
                }
                itr+=1;
            }
            myUtils::MapSearchResult dijRet = gbrrtUtils.dij_search(RM, st, g);
            
            if(smooth && dijRet.success)
            {
                smoothPath.push_back(dijRet.node_path.front());
                pair<double, double> prev;
                prev = smoothPath.back();
                for(auto pt : dijRet.node_path)
                {
                    if(!gbrrtUtils.checkInObj(smoothPath.back(), pt, obs))
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
                ret = gbrrtUtils.pairToEigenVector(smoothPath);
                valid_path = true;
                // printf("Smoothed ");
                // gbrrtUtils.printPath(ret);
            }
            else if(dijRet.success)
            {
                ret = gbrrtUtils.pairToEigenVector(dijRet.node_path);
                valid_path = true;
            }
            else
            {
                ret = gbrrtUtils.pairToEigenVector({st, g});
                valid_path = false;
            }
            finalItr = itr;
            auto stpTime = high_resolution_clock::now();
            time = duration_cast<milliseconds>(stpTime-stTime).count();
            // printf("Run Time for Algorithm: %ld ms\n", time);

            return ret;
        }

        void benchMarkAlgo(int numRuns, bool smoothing, list<Problem2D> problems, vector<string> ws)
        {
            Path2D ret;
            vector<double> tmp_lens;
            list<vector<double>> path_lens;
            vector<double> tmp_times;
            list<vector<double>> times;
            double tmp_sols = 0;
            vector<double> valid_sols;
            list<vector<double>> iters;
            vector<double> tmp_itrs;

            for(auto prob : problems)
            {
                for(int run = 0; run < numRuns; run++)
                {
                    ret = plan(prob);
                    if(getValid())
                    {
                        tmp_lens.push_back(ret.length());
                        tmp_times.push_back(getTime());
                        tmp_itrs.push_back(getItr());
                        tmp_sols += getValid();
                    }
                }
                valid_sols.push_back(tmp_sols);
                path_lens.push_back(tmp_lens);
                times.push_back(tmp_times);
                iters.push_back(tmp_itrs);
                tmp_sols = 0;
                tmp_lens.clear();
                tmp_times.clear();
                tmp_itrs.clear();
            }

            Visualizer::makeBoxPlot(path_lens, ws, "Path Lengths for Goal Biased RRT", "Environments", "Distance [Units]");
            Visualizer::makeBoxPlot(times, ws, "Computation Time for Goal Biased RRT", "Environments", "Time [ms]");
            Visualizer::makeBoxPlot(iters, ws, "Number of Iterations for Goal Biased RRT", "Environments", "# of Iterations");
            Visualizer::makeBarGraph(valid_sols, ws, "Valid Solutions for RRT", "Environments", "Valid Solution Count");
            Visualizer::showFigures();
            
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

            Visualizer::makeBoxPlot(path_lens, ws, "Path Lengths for Goal Biased RRT", "Environments", "Distance [Units]");
            Visualizer::makeBoxPlot(times, ws, "Computation Time for Goal Biased RRT", "Environments", "Time [ms]");
            Visualizer::makeBarGraph(valid_sols, ws, "Valid Solutions for RRT", "Environments", "Valid Solution Count");
            Visualizer::showFigures();
            
        }

        void benchMarkAlgo(int numRuns, bool smoothing, list<Problem2D> problems, LinkManipulator2D& man, vector<string> ws)
        {
            Path2D ret;
            vector<double> tmp_lens;
            list<vector<double>> path_lens;
            vector<double> tmp_times;
            list<vector<double>> times;
            double tmp_sols = 0;
            vector<double> valid_sols;
            list<vector<double>> iters;
            vector<double> tmp_itrs;

            for(auto prob : problems)
            {
                for(int run = 0; run < numRuns; run++)
                {
                    ret = plan(man, prob);
                    if(getValid())
                    {
                        tmp_lens.push_back(ret.length());
                        tmp_times.push_back(getTime());
                        tmp_itrs.push_back(getItr());
                        tmp_sols += getValid();
                    }
                }
                valid_sols.push_back(tmp_sols);
                path_lens.push_back(tmp_lens);
                times.push_back(tmp_times);
                iters.push_back(tmp_itrs);
                tmp_sols = 0;
                tmp_lens.clear();
                tmp_times.clear();
                tmp_itrs.clear();
            }

            Visualizer::makeBoxPlot(path_lens, ws, "Path Lengths for Goal Biased RRT", "Environments", "Distance [Rads]");
            Visualizer::makeBoxPlot(times, ws, "Computation Time for Goal Biased RRT", "Environments", "Time [ms]");
            Visualizer::makeBoxPlot(iters, ws, "Number of Iterations for Goal Biased RRT", "Environments", "# of Iterations");
            Visualizer::makeBarGraph(valid_sols, ws, "Valid Solutions for RRT", "Environments", "Valid Solution Count");
            Visualizer::showFigures();
            
        }

        pair<Graph<double>, map<amp::Node, Eigen::Vector2d> > getGraphFromRM()
        {
            Graph<double> ret1;
            map<Node, Eigen::Vector2d> ret2;
            Eigen::Vector2d ikey;
            Eigen::Vector2d okey;

            uint32_t nodeCnt = 0;
            uint32_t fromNode;
            uint32_t toNode;
            uint32_t tmpTo;
            uint32_t tmpFrom;

            for( auto key : RM)
            {   
                okey =  Eigen::Vector2d{key.first.first, key.first.second};
                fromNode = checkInMap(ret2,okey);    
                if(fromNode == -1)
                {
                    fromNode = nodeCnt;
                    ret2.insert({fromNode, okey});
                    nodeCnt++;
                }

                for( auto inKey : key.second)
                {
                    ikey =  Eigen::Vector2d{inKey.first.first, inKey.first.second};
                    toNode = checkInMap(ret2, ikey);
                    if(toNode == -1)
                    {
                        toNode = nodeCnt;
                        ret2.insert({toNode, ikey});
                        nodeCnt++;
                    }

                    ret1.connect(fromNode, toNode, RM[key.first][inKey.first]);
                }
            }
            return make_pair(ret1, ret2);
        }

        Node checkInMap(map<Node, Eigen::Vector2d> a, Eigen::Vector2d b)
        {
            for(auto key : a)
            {
                if(key.second[0] == b[0] && key.second[1] == b[1])
                {
                    return key.first;
                }
            }
            return -1;
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

        int getItr()
        {
            return finalItr;
        }


        virtual ~MyGBRRT() {}
    
    private:
        int n;
        double r;
        double eps;
        double p_goal;
        double dis;
        int dec_per;
        bool smooth;
        long time;
        bool valid_path;
        int finalItr;
        map< pair<double,double>, map< pair<double,double>, double > > RM;
        myUtils gbrrtUtils;

};