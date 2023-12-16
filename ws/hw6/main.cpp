#include <queue>
#include <map>

#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"

#include "MyLinkManipulator.h"
#include "myUtils.h"

using namespace amp;
using namespace std;

myUtils hw6Utils;

double global_dis = 0.1;

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        MyPointWFAlgo()
        {
            dis = 0.25;
        }

        MyPointWFAlgo(double a)
        {
            dis = a;
        }

        double dis;

        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            double x0min = environment.x_min;
            double x0max = environment.x_max;
            double x1min = environment.y_min;
            double x1max = environment.y_max;


            pair<size_t, size_t> tmp;

            std::size_t x0cells = (abs(x0min) + abs(x0max)) / this->dis;
            std::size_t x1cells = (abs(x1min) + abs(x1max)) / this->dis;
            unique_ptr<MyGridCSpace> ret = std::make_unique<MyGridCSpace>(this->dis, x0cells, x1cells, x0min, x0max, x1min, x1max);

            vector<Obstacle2D> obs = environment.obstacles;

            Eigen::Vector2d tstPt;

            for(double i = x0min; i < x0max; i+=(this->dis/3) )
            {
                for(double j = x1min; j < x1max; j+=(this->dis/3) )
                {                   
                    tmp = ret->getCellFromPoint(i,j);
                    if(ret->operator()(tmp.first, tmp.second))
                    {
                        continue;
                    }                   
                    for(int k = 0; k < obs.size(); k++)
                    {
                        vector<Eigen::Vector2d> verts = obs[k].verticesCCW();
                        verts.push_back(verts[0]); 
                        if(hw6Utils.checkInObj(Eigen::Vector2d{i,j}, verts))
                        {
                            ret->operator()(tmp.first, tmp.second) = true;
                            break;
                        }
                    }
                }
            }            
            return ret;
        }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            Path2D ret;
            
            map< pair<size_t, size_t> , int > wave;

            queue< pair<size_t,size_t> > tiles;

            pair<size_t, size_t> tmp;

            pair<size_t, size_t> stCell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
            pair<size_t, size_t> gCell  = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

            pair<size_t, size_t> up; 
            pair<size_t, size_t> down; 
            pair<size_t, size_t> left; 
            pair<size_t, size_t> right; 

            pair<double,double> x0 = grid_cspace.x0Bounds();
            pair<double,double> x1 = grid_cspace.x1Bounds();

            pair<size_t, size_t> max_cells = grid_cspace.size();

            int xMax = max_cells.first;
            int yMax = max_cells.second;

            bool found = false;

            tiles.push(gCell);
            wave.insert({gCell, 2});
            ret.waypoints.push_back(q_init);
            
            while(!tiles.empty())
            {

                tmp = tiles.front();
                tiles.pop();
                if(tmp.second == stCell.second && tmp.first == stCell.first)
                {   
                    found = true;
                    break;
                }

                if(wave.find(tmp)->second == 1)
                {
                    continue;
                }

                up = make_pair(tmp.first, tmp.second+1);
                down = make_pair(tmp.first, tmp.second-1);
                left = make_pair(tmp.first-1, tmp.second);
                right = make_pair(tmp.first+1, tmp.second);

                if( (up.first >= 0 && up.first < xMax) 
                &&  (up.second >= 0 && up.second < yMax) 
                &&  wave.find(up) == wave.end())
                {
                    tiles.push(up);
                    if(grid_cspace.operator()(up.first, up.second))
                    {
                        wave.insert({up, 1});
                    }
                    else if((wave.find(tmp)->second) > 1)
                    {
                        wave.insert({up, (wave.find(tmp)->second)+1});
                    }
                }

                if( (down.first >= 0 && down.first < xMax) 
                &&  (down.second >= 0 && down.second < yMax) 
                &&  wave.find(down) == wave.end())
                {
                    tiles.push(down);
                    if(grid_cspace.operator()(down.first, down.second))
                    {
                        wave.insert({down, 1});
                    }
                    else
                    {
                        wave.insert({down, (wave.find(tmp)->second)+1});
                    }
                }

                if( (left.first >= 0 && left.first < xMax) 
                &&  (left.second >= 0 && left.second < yMax) 
                &&  wave.find(left) == wave.end())
                {
                    if(grid_cspace.operator()(left.first, left.second))
                    {
                        wave.insert({left, 1});
                    }
                    else
                    {
                        wave.insert({left, (wave.find(tmp)->second)+1});
                    }
                    tiles.push(left);
                }

                if( (right.first >= 0 && right.first < xMax) 
                &&  (right.second >= 0 && right.second < yMax) 
                &&  wave.find(right) == wave.end())
                {
                    if(grid_cspace.operator()(right.first, right.second))
                    {
                        wave.insert({right, 1});
                    }
                    else
                    {
                        wave.insert({right, (wave.find(tmp)->second)+1});
                    }
                    tiles.push(right);
                }
            }

            int key = wave.find(stCell)->second;
            tmp = stCell;

            if(!found)
            {
                ret.waypoints.push_back(q_goal);
                return ret;
            }

            while(true)
            {
                up = make_pair(tmp.first, tmp.second+1);
                down = make_pair(tmp.first, tmp.second-1);
                left = make_pair(tmp.first-1, tmp.second);
                right = make_pair(tmp.first+1, tmp.second);

                if(tmp.first == gCell.first && tmp.second == gCell.second)
                {
                    break;
                }

                if( (up.first >= 0 && up.first < xMax) 
                &&  (up.second >= 0 && up.second < yMax)
                && wave.find(up) != wave.end() && wave.find(up)->second == key-1)
                {
                    key = key-1;
                    tmp = up;
                    ret.waypoints.push_back(Eigen::Vector2d{ 
                        ((x0.first+(this->dis*up.first)) + (x0.first+(this->dis*(up.first+1))))/2,
                        ((x1.first+(this->dis*up.second)) + (x1.first+(this->dis*(up.second+1))))/2});
                }
                else if( (down.first >= 0 && down.first < xMax) 
                &&  (down.second >= 0 && down.second < yMax)
                && wave.find(down) != wave.end() && wave.find(down)->second == key-1)
                {
                    key = key-1;
                    tmp = down;
                    ret.waypoints.push_back(Eigen::Vector2d{ 
                        ((x0.first+(this->dis*down.first)) + (x0.first+(this->dis*(down.first+1))))/2,
                        ((x1.first+(this->dis*down.second)) + (x1.first+(this->dis*(down.second+1))))/2});
                }
                else if( (left.first >= 0 && left.first < xMax) 
                &&  (left.second >= 0 && left.second < yMax)
                && wave.find(left) != wave.end() && wave.find(left)->second == key-1)
                {
                    key = key-1;
                    tmp = left;
                    ret.waypoints.push_back(Eigen::Vector2d{ 
                        ((x0.first+(this->dis*left.first)) + (x0.first+(this->dis*(left.first+1))))/2,
                        ((x1.first+(this->dis*left.second)) + (x1.first+(this->dis*(left.second+1))))/2});
                }
                else if( (right.first >= 0 && right.first < xMax) 
                &&  (right.second >= 0 && right.second < yMax)
                && wave.find(right) != wave.end() && wave.find(right)->second == key-1)
                {
                    key = key-1;
                    tmp = right;
                    ret.waypoints.push_back(Eigen::Vector2d{ 
                        ((x0.first+(this->dis*right.first)) + (x0.first+(this->dis*(right.first+1))))/2,
                        ((x1.first+(this->dis*right.second)) + (x1.first+(this->dis*(right.second+1))))/2});
                }
            }
            ret.waypoints.push_back(q_goal);
            return ret;
        }

};

class MyCSpaceCtor : public amp::GridCSpace2DConstructor {
    public:
        MyCSpaceCtor(){
            dis = global_dis;
        }

        MyCSpaceCtor(double res){
            dis = res;
        }

        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override { 
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
                            if(hw6Utils.checkInObj(links, verts))
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
                            if(hw6Utils.checkInObj(links, verts))
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

        double dis;
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) 
            {
                myLinkAngent = MyLinkManipulator(vector<double>{1.0, 1.0});
            }

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(MyLinkManipulator Link_Agent) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) 
            {
                myLinkAngent = Link_Agent;
            }
        
        double dis = global_dis;

        MyLinkManipulator myLinkAngent;

        virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
            ASSERT(myLinkAngent.nLinks() == 2, "Manipulator must have two links");

            // Get the initial state from IK
            amp::ManipulatorState init_state = myLinkAngent.getConfigurationFromIK(problem.q_init);

            // Get the goal state from IK
            amp::ManipulatorState goal_state = myLinkAngent.getConfigurationFromIK(problem.q_goal);

            // Construct the grid cspace
            std::unique_ptr<amp::GridCSpace2D> grid_cspace = m_c_space_constructor->construct(myLinkAngent, problem);

            // Now that we have everything, we can call method to plan in C-space using the WaveFront algorithm
            // Note, we can use the `convert` overloads to easily go between ManipulatorState and ManipulatorState2Link
            return planInCSpace(convert(init_state), convert(goal_state), *grid_cspace);
        }


        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            Path2D ret;
            
            map< pair<size_t, size_t> , int > wave;

            queue< pair<size_t,size_t> > tiles;

            pair<size_t, size_t> tmp;

            Eigen::Vector2d goal = q_goal;
            Eigen::Vector2d init = q_init;

            // printf("------- RECEIVED -------\n");
            // printf("Goal Point is at angle: (%.3f & %.3f)\n",goal[0]*180/M_PI, goal[1]*180/M_PI);
            // printf("Start Point is at angle: (%.3f & %.3f)\n",init[0]*180/M_PI, init[1]*180/M_PI);

            // sleep(5);

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


            if(grid_cspace.operator()(gCell.first, gCell.second))
            {
                // printf("Correcting goal!!\n");
                goal = correctPosition(goal);
                gCell  = grid_cspace.getCellFromPoint(goal[0], goal[1]);
            }

            if(grid_cspace.operator()(stCell.first, stCell.second))
            {
                // printf("Correcting start!!\n");
                init = correctPosition(init);
                stCell = grid_cspace.getCellFromPoint(init[0], init[1]);
            }

            // printf("------- ADJUSTED -------\n");
            // printf("Goal Point is at angle: (%.3f & %.3f)\n",goal[0]*180/M_PI, goal[1]*180/M_PI);
            // printf("Start Point is at angle: (%.3f & %.3f)\n",init[0]*180/M_PI, init[1]*180/M_PI);

            if(grid_cspace.operator()(stCell.first, stCell.second) || grid_cspace.operator()(gCell.first, gCell.second))
            {
                printf("No proper start and/or finish... \n");
                ret.waypoints.push_back(init);
                ret.waypoints.push_back(goal);
                return ret;
            }   

            pair<size_t, size_t> up; 
            pair<size_t, size_t> down; 
            pair<size_t, size_t> left; 
            pair<size_t, size_t> right; 

            pair<double,double> x0 = grid_cspace.x0Bounds();
            pair<double,double> x1 = grid_cspace.x1Bounds();

            pair<size_t, size_t> max_cells = grid_cspace.size();

            int xMax = max_cells.first;
            int yMax = max_cells.second;

            bool found = false;

            tiles.push(gCell);
            wave.insert({gCell, 2});
            ret.waypoints.push_back(init);
            // printf("Searching for Start Cell {%ld, %ld} from Goal Cell {%ld, %ld} xMax = %d, yMax %d\n", stCell.first, stCell.second, gCell.first, gCell.second, xMax, yMax);
            while(!tiles.empty())
            {
                
                tmp = tiles.front();
                tiles.pop();

                if(tmp.second == stCell.second && tmp.first == stCell.first)
                {   
                    found = true;
                    break;
                }

                if(wave.find(tmp)->second == 1)
                {
                    // if(tmp.first == 36 && tmp.second == 16)
                    // {
                    //     printf("----Building from Cell {%ld, %ld}----\n", tmp.first, tmp.second);
                    //     printf("-Up Cell {%ld, %ld}-\n", up.first, up.second);
                    //     printf("-Down Cell {%ld, %ld}-\n", down.first, down.second);
                    //     printf("-Left Cell {%ld, %ld}-\n", left.first, left.second);
                    //     printf("-Right Cell {%ld, %ld}-\n", right.first, right.second);
                    // }
                    continue;
                }

                up = make_pair(tmp.first, (tmp.second+1)%(yMax));
                down = make_pair(tmp.first, (tmp.second-1+yMax)%(yMax));
                left = make_pair((tmp.first-1+xMax)%(xMax), tmp.second);
                right = make_pair((tmp.first+1)%(xMax), tmp.second);
                // if(tmp.first == 36 && tmp.second == 16)
                // {
                //     printf("----Building from Cell {%ld, %ld}----\n", tmp.first, tmp.second);
                //     printf("-Up Cell {%ld, %ld}-\n", up.first, up.second);
                //     printf("-Down Cell {%ld, %ld}-\n", down.first, down.second);
                //     printf("-Left Cell {%ld, %ld}-\n", left.first, left.second);
                //     printf("-Right Cell {%ld, %ld}-\n", right.first, right.second);
                // }
                if( wave.find(up) == wave.end())
                {
                    tiles.push(up);
                    if(grid_cspace.operator()(up.first, up.second))
                    {
                        wave.insert({up, 1});
                    }
                    else
                    {
                        wave.insert({up, (wave.find(tmp)->second)+1});
                    }
                }

                if( wave.find(down) == wave.end())
                {
                    tiles.push(down);
                    if(grid_cspace.operator()(down.first, down.second))
                    {
                        wave.insert({down, 1});
                    }
                    else
                    {
                        wave.insert({down, (wave.find(tmp)->second)+1});
                    }
                }

                if( wave.find(left) == wave.end())
                {
                    tiles.push(left);
                    if(grid_cspace.operator()(left.first, left.second))
                    {
                        wave.insert({left, 1});
                    }
                    else
                    {
                        wave.insert({left, (wave.find(tmp)->second)+1});
                    }
                }

                if(  wave.find(right) == wave.end())
                {
                    tiles.push(right);
                    if(grid_cspace.operator()(right.first, right.second))
                    {
                        wave.insert({right, 1});
                    }
                    else
                    {
                        wave.insert({right, (wave.find(tmp)->second)+1});
                    }
                }
            }

            int key = wave.find(stCell)->second;
            tmp = stCell;

            if(!found)
            {
                printf("Goal not found\n");
                ret.waypoints.push_back(goal);
                return ret;
            }

            int move = 0;
            bool stepped = false;
            int ops = 0;
            // printf("Working way back to goal {%ld, %ld} from start {%ld, %ld}\n",gCell.first, gCell.second, stCell.first, stCell.second);
            while(true)
            {
                
                up = make_pair(tmp.first, (tmp.second+1)%(yMax));
                down = make_pair(tmp.first, (tmp.second-1+yMax)%yMax);
                left = make_pair((tmp.first-1+xMax)%xMax, tmp.second);
                right = make_pair((tmp.first+1)%(xMax), tmp.second);
                // if(key < 4 || key > wave.find(stCell)->second-3)
                // {
                //     printf("----Checking Cell {%ld, %ld} with value: %d----\n", tmp.first, tmp.second, wave.find(tmp)->second);
                //     printf("up Cell {%ld, %ld} with value: %d\n", up.first, up.second, wave.find(up)->second);
                //     printf("down Cell {%ld, %ld} with value: %d\n", down.first, down.second, wave.find(down)->second);
                //     printf("left Cell {%ld, %ld} with value: %d\n", left.first, left.second, wave.find(left)->second);
                //     printf("right Cell {%ld, %ld} with value: %d\n", right.first, right.second, wave.find(right)->second);
                // }
            
                if( wave.find(up) != wave.end() && wave.find(up)->second == key-1 )
                {
                    key = key-1;
                    tmp = up;
                    if(wave.find(tmp)->second == 2)
                    {
                        ret.waypoints.push_back(goal);
                        break;
                    }
                    ret.waypoints.push_back(Eigen::Vector2d{ 
                        ((x0.first+(this->dis*up.first)) + (x0.first+(this->dis*(up.first+1))))/2,
                        ((x1.first+(this->dis*up.second)) + (x1.first+(this->dis*(up.second+1))))/2});
                    stepped = true;
                }
                else if( wave.find(down) != wave.end() && wave.find(down)->second == key-1 )
                {
                    key = key-1;
                    tmp = down;
                    if(wave.find(tmp)->second == 2)
                    {
                        ret.waypoints.push_back(goal);
                        break;
                    }
                    ret.waypoints.push_back(Eigen::Vector2d{ 
                        ((x0.first+(this->dis*down.first)) + (x0.first+(this->dis*(down.first+1))))/2,
                        ((x1.first+(this->dis*down.second)) + (x1.first+(this->dis*(down.second+1))))/2});
                    stepped = true;
                }
                else if( wave.find(left) != wave.end() && wave.find(left)->second == key-1 )
                {
                    key = key-1;
                    tmp = left;
                    if(wave.find(tmp)->second == 2)
                    {
                        ret.waypoints.push_back(goal);
                        break;
                    }
                    ret.waypoints.push_back(Eigen::Vector2d{ 
                        ((x0.first+(this->dis*left.first)) + (x0.first+(this->dis*(left.first+1))))/2,
                        ((x1.first+(this->dis*left.second)) + (x1.first+(this->dis*(left.second+1))))/2});
                    stepped = true;
                }
                else if( wave.find(right) != wave.end() && wave.find(right)->second == key-1 )
                {
                    key = key-1;
                    tmp = right;
                    if(wave.find(tmp)->second == 2)
                    {
                        ret.waypoints.push_back(goal);
                        break;
                    }
                    ret.waypoints.push_back(Eigen::Vector2d{ 
                        ((x0.first+(this->dis*right.first)) + (x0.first+(this->dis*(right.first+1))))/2,
                        ((x1.first+(this->dis*right.second)) + (x1.first+(this->dis*(right.second+1))))/2});
                    stepped = true;
                    
                }
                else
                {
                    stepped = false;
                }
                if(wave.find(tmp)->second == 1 || !stepped)
                {
                    ret.waypoints.push_back(goal);
                    break;
                }
            }
            
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
};

class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override 
        {
            GraphSearchResult ret;

            Node st = problem.init_node;
            Node g = problem.goal_node;
            Node bst;

            vector<Node> childs;
            vector<double> weights;
           
            typedef tuple<double, double, Node> nW;
            map <Node, double> dist;
            map <Node, Node> bkPtr;
            
            priority_queue<nW, vector<nW>, greater<nW> > O;
            priority_queue<nW, vector<nW>, greater<nW> > tmpQ;
            nW tmpTp;
            list<Node> Ol;
            list<Node> C;
            list<Node> q;

            double gn;
            double h;
            double f;

            O.push(make_tuple(0, 0, st));
            dist.insert({st, 0});
            Ol.push_back(st);
            q.push_back(st);

            int itrCount = 0;

            while(!O.empty())
            {
                tmpQ = O;

                bst = get<2>(O.top());
                O.pop();
                Ol.remove(bst);
                C.push_back(bst);

                if (bst == g)
                {
                    q.push_back(g);
                    break;
                }

                childs = problem.graph->children(bst);
                weights = problem.graph->outgoingEdges(bst);

                for(int i = 0; i < childs.size(); i++)
                {
                    
                    if(find(C.begin(), C.end(), childs[i]) != C.end())
                    {
                        continue;
                    }

                    if(find(Ol.begin(), Ol.end(), childs[i]) == Ol.end())
                    {
                        h = heuristic.operator()(childs[i]);
                        gn = weights[i] + dist.find(bst)->second;
                        O.push( make_tuple(h + gn, gn, childs[i]) );
                        Ol.push_back(childs[i]);
                        dist.insert({childs[i], gn});
                        bkPtr.insert({childs[i], bst});
                    }
                    else if((dist.find(bst)->second + weights[i] ) < dist.find(childs[i])->second)
                    {
                        h = heuristic.operator()(childs[i]);
                        gn = weights[i] + dist.find(bst)->second;
                        O.push( make_tuple(h + gn, gn, childs[i]) );
                        bkPtr.find(childs[i])->second = bst;
                        dist.find(childs[i])->second = gn;
                    }
                }
                itrCount += 1;
                tmpQ = O;
            }
            Node tmp;
            if (q.back() == g)
            {
                tmp = g;
                
                while(bkPtr.find(tmp)->second != st)
                {
                    auto it = q.begin();
                    advance(it, 1);
                    q.insert(it,bkPtr.find(tmp)->second);
                    tmp = bkPtr.find(tmp)->second;
                }
                ret.node_path = q;
                ret.success = true;
                ret.path_cost = dist.find(g)->second;

            }
            else
            {
                ret.node_path = q;
                ret.success = false;
                ret.path_cost = -1;

            }
            printf("Path %s with length of the path { ", ret.success ? "found" : "not found"); 
            for(auto i : q)
            {   
                cout << i << " ";
            }
            printf("} is %.2f found in %d iterations\n",ret.path_cost, itrCount);

            return ret;
        }

        GraphSearchResult dij_search(const amp::ShortestPathProblem& problem)
        {
            GraphSearchResult ret;

            Node st = problem.init_node;
            Node g = problem.goal_node;
            Node bst;

            vector<Node> childs;
            vector<double> weights;
           
            typedef pair<double, Node> nW;
            map <Node, double> dist;
            map <Node, Node> bkPtr;
            
            priority_queue<nW, vector<nW>, greater<nW> > O;
            priority_queue<nW, vector<nW>, greater<nW> > tmpQ;
            nW tmpTp;
            list<Node> Ol;
            list<Node> C;
            list<Node> q;

            double gn;
            double h;
            double f;

            O.push(make_pair(0, st));
            dist.insert({st, 0});
            Ol.push_back(st);
            q.push_back(st);

            int itrCount = 0;

            while(!O.empty())
            {
                tmpQ = O;

                bst = O.top().second;
                O.pop();
                Ol.remove(bst);
                C.push_back(bst);

                if (bst == g)
                {
                    q.push_back(g);
                    break;
                }

                childs = problem.graph->children(bst);
                weights = problem.graph->outgoingEdges(bst);

                for(int i = 0; i < childs.size(); i++)
                {
                    
                    if(find(C.begin(), C.end(), childs[i]) != C.end())
                    {
                        continue;
                    }

                    if(find(Ol.begin(), Ol.end(), childs[i]) == Ol.end())
                    {
                        gn = weights[i] + dist.find(bst)->second;
                        O.push( make_pair(gn, childs[i]) );
                        Ol.push_back(childs[i]);
                        dist.insert({childs[i], gn});
                        bkPtr.insert({childs[i], bst});
                    }
                    else if((dist.find(bst)->second + weights[i] ) < dist.find(childs[i])->second)
                    {
                        gn = weights[i] + dist.find(bst)->second;
                        O.push( make_pair(gn, childs[i]) );
                        bkPtr.find(childs[i])->second = bst;
                        dist.find(childs[i])->second = gn;
                    }
                }
                itrCount += 1;
                tmpQ = O;
            }
            Node tmp;
            if (q.back() == g)
            {
                tmp = g;
                
                while(bkPtr.find(tmp)->second != st)
                {
                    auto it = q.begin();
                    advance(it, 1);
                    q.insert(it,bkPtr.find(tmp)->second);
                    tmp = bkPtr.find(tmp)->second;
                }
                ret.node_path = q;
                ret.success = true;
                ret.path_cost = dist.find(g)->second;

            }
            else
            {
                ret.node_path = q;
                ret.success = false;
                ret.path_cost = -1;

            }
            printf("Path %s with length of the path { ", ret.success ? "found" : "not found"); 
            for(auto i : q)
            {   
                cout << i << " ";
            }
            printf("} is %.2f found in %d iterations\n",ret.path_cost, itrCount);

            return ret;
        }
};

int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    // Exercise 1
    if(false)
    {
        MyPointWFAlgo ex1;

        amp::Problem2D ex1a = HW2::getWorkspace1();
        amp::GridCSpace2D * ex1g = ex1.constructDiscretizedWorkspace(ex1a).release();
        // Visualizer::makeFigure(*ex1g);
        Path2D ex1p = ex1.planInCSpace(ex1a.q_init, ex1a.q_goal, *ex1g);
        Visualizer::makeFigure(ex1a,ex1p);
        printf("Length of Path for EX1 World 1: %.2f\n", ex1p.length());
        bool ex1aPass = HW6::checkPointAgentPlan(ex1p,ex1a,true);

        Problem2D ex1b = HW2::getWorkspace2();
        amp::GridCSpace2D * ex1gb = ex1.constructDiscretizedWorkspace(ex1b).release();
        // Visualizer::makeFigure(*ex1gb);
        Path2D ex1pb = ex1.planInCSpace(ex1b.q_init, ex1b.q_goal, *ex1gb);
        Visualizer::makeFigure(ex1b,ex1pb);
        printf("Length of Path for EX1 World 2: %.2f\n", ex1pb.length());
        bool ex1bPass = HW6::checkPointAgentPlan(ex1pb,ex1b,true);

        // const Random2DEnvironmentSpecification spec;

        // MyPointWFAlgo ex2;
        // Problem2D r1 =  EnvironmentTools::generateRandomPointAgentProblem(spec,0u);
        // amp::GridCSpace2D * r1g = ex2.constructDiscretizedWorkspace(r1).release();
        // // Visualizer::makeFigure(*r1g);
        // Path2D r1p = ex2.planInCSpace(r1.q_init, r1.q_goal, *r1g);
        // Visualizer::makeFigure(r1,r1p);
        // bool r1pass = HW6::checkPointAgentPlan(r1p,r1,true);


        // bool ex1CPass = HW6::generateAndCheck(ex2, true, 0U);

        Visualizer::showFigures();
    }

    // Exercise 2
    if(false)
    {
        MyManipWFAlgo e2;
        MyLinkManipulator man(vector<double>{1.0, 1.0});
        MyCSpaceCtor csp;

        Problem2D p1 = HW6::getHW4Problem1();
        GridCSpace2D * p1g = csp.construct(man,p1).release();
        Path2D p1p = e2.plan(man,p1);
        bool p1pass = HW6::checkLinkManipulatorPlan(p1p, man, p1, true);
        // Visualizer::makeFigure(p1,man,p1p);
        Visualizer::makeFigure(*p1g, p1p);

        Problem2D p2 = HW6::getHW4Problem2();
        GridCSpace2D * p2g = csp.construct(man,p2).release();
        Path2D p2p = e2.plan(man,p2);
        bool p2pass = HW6::checkLinkManipulatorPlan(p2p, man, p2, true);
        // Visualizer::makeFigure(p2, man, p2p);
        Visualizer::makeFigure(*p2g,p2p);

        Problem2D p3 = HW6::getHW4Problem3();
        GridCSpace2D * p3g = csp.construct(man,p3).release();
        Path2D p3p = e2.plan(man,p3);
        bool p3pass = HW6::checkLinkManipulatorPlan(p3p, man, p3, true);
        // Visualizer::makeFigure(p3, man, p3p);
        Visualizer::makeFigure(*p3g, p3p);

        const Random2DManipulatorEnvironmentSpecification Spec;
        Problem2D pt = EnvironmentTools::generateRandomManipulatorProblem(Spec, man);
        GridCSpace2D * ptg = csp.construct(man,pt).release();
        Path2D ptp = e2.plan(man,pt);
        bool ptpass = HW6::checkLinkManipulatorPlan(ptp, man, pt, true);
        Visualizer::makeFigure(*ptg, ptp);
        // Visualizer::makeFigure(pt,man,ptp);

        pt = EnvironmentTools::generateRandomManipulatorProblem(Spec, man);
        ptg = csp.construct(man,pt).release();
        ptp = e2.plan(man,pt);
        ptpass = HW6::checkLinkManipulatorPlan(ptp, man, pt, true);
        Visualizer::makeFigure(*ptg, ptp);
        // Visualizer::makeFigure(pt,man,ptp);

        pt = EnvironmentTools::generateRandomManipulatorProblem(Spec, man);
        ptg = csp.construct(man,pt).release();
        ptp = e2.plan(man,pt);
        ptpass = HW6::checkLinkManipulatorPlan(ptp, man, pt, true);
        Visualizer::makeFigure(*ptg, ptp);
        // Visualizer::makeFigure(pt,man,ptp);

        pt = EnvironmentTools::generateRandomManipulatorProblem(Spec, man);
        ptg = csp.construct(man,pt).release();
        ptp = e2.plan(man,pt);
        ptpass = HW6::checkLinkManipulatorPlan(ptp, man, pt, true);
        Visualizer::makeFigure(*ptg, ptp);
        // Visualizer::makeFigure(pt,man,ptp);

        pt = EnvironmentTools::generateRandomManipulatorProblem(Spec, man);
        ptg = csp.construct(man,pt).release();
        ptp = e2.plan(man,pt);
        ptpass = HW6::checkLinkManipulatorPlan(ptp, man, pt, true);
        Visualizer::makeFigure(*ptg, ptp);
        // Visualizer::makeFigure(pt,man,ptp);


        Visualizer::showFigures();
    }

    // Exercise 3
    if(false)
    {
        MyAStarAlgo ex3;
        amp::ShortestPathProblem ex3p = HW6::getEx3SPP();
        amp::LookupSearchHeuristic ex3h = HW6::getEx3Heuristic();
        amp::AStar::GraphSearchResult res = ex3.search(ex3p, ex3h);
        amp::AStar::GraphSearchResult resD = ex3.dij_search(ex3p);

        bool ex3Pass = HW6::checkGraphSearchResult(res, ex3p, ex3h, true);
        bool ex3PassD = HW6::checkGraphSearchResult(resD, ex3p, ex3h, true);
    }

    amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("corey.huffman@colorado.edu", argc, argv, std::make_tuple(0.25), std::make_tuple(), std::make_tuple());
    return 0;
}