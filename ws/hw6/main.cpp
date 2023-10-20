#include <queue>
#include <map>

#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"

using namespace amp;
using namespace std;


struct line {
    Eigen::Vector2d p1, p2;
};

int linDir(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c)
{
    int val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1]);
    if(val == 0)
    {
        return 0;
    }
    else if(val < 0)
    {
        return 2;
    }
    return 1;
}

bool onL(line a, Eigen::Vector2d b)
{
    if(b[0] <= max(a.p1[0], a.p2[0]) && b[0] >= min(a.p1[0], a.p2[0])
    && b[1] <= max(a.p1[1], a.p2[1]) && b[1] >= min(a.p1[1], a.p2[1]))
    {
        return true;
    }
    return false;
}

bool isInt(line a, line b)
{
    int d1 = linDir(a.p1, a.p2, b.p1);
    int d2 = linDir(a.p1, a.p2, b.p2);
    int d3 = linDir(b.p1, b.p2, a.p1);
    int d4 = linDir(b.p1, b.p2, a.p2);

    if(d1 != d2 && d3 != d4)
    {
        return true;
    }

    if(d1 == 0 && onL(a,b.p1))
    {
        return true;
    }

    if(d2 == 0 && onL(a,b.p2))
    {
        return true;
    }

    if(d3 == 0 && onL(b,a.p1))
    {
        return true;
    }

    if(d4 == 0 && onL(b,a.p2))
    {
        return true;
    }

    return false;

}

bool checkInObj(line p, vector<Eigen::Vector2d> verts)
{
    int count = 0;
    for(int wI = 0; wI < verts.size()-1; wI++)
    {
        line v{verts[wI], verts[wI+1]};
        if(isInt(v,p))
        {
            if(linDir(v.p1, p.p1, v.p2) == 0)
            {
                return onL(v, p.p1);
            } 
            count++;
        }
    }
    return count & 1;
}

class MyGridCSpace : public amp::GridCSpace2D {
    public:
        //MyGridCSpace()
        //    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {
                xMin = x0_min;
                yMin = x1_min;
            }

        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {
            return make_pair(floor(x0/0.25)-((this->xMin)/0.25), floor(x1/0.25)-((this->yMin)/0.25));
        }

        double xMin;
        double yMin;
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            double x0min = environment.x_min;
            double x0max = environment.x_max;
            double x1min = environment.y_min;
            double x1max = environment.y_max;

            double dis = 0.25;

            pair<size_t, size_t> tmp;

            std::size_t x0cells = (abs(x0min) + abs(x0max)) / dis;
            std::size_t x1cells = (abs(x1min) + abs(x1max)) / dis;
            unique_ptr<MyGridCSpace> ret = std::make_unique<MyGridCSpace>(x0cells, x1cells, x0min, x0max, x1min, x1max);

            vector<Obstacle2D> obs = environment.obstacles;

            for(double i = x0min; i < x0max; i+=0.1)
            {
                for(double j = x1min; j < x1max; j+=0.1)
                {
                    // printf("x0: %.2f x1: %.2f\n", i,j);
                    tmp = ret->getCellFromPoint(i, j);
                    // printf("Cell {%ld, %ld}\n", tmp.first, tmp.second);
                    // sleep(0.8);
                    line p{Eigen::Vector2d{i,j}, Eigen::Vector2d{9999,j}};

                    for(int k = 0; k < obs.size(); k++)
                    {
                        // printf("Obj # %d\n", k);
                        vector<Eigen::Vector2d> verts = obs[k].verticesCCW();
                        verts.push_back(verts[0]);

                        ret->operator()(tmp.first, tmp.second) = checkInObj(p, verts);
                    }
                }
            }            
            return ret;
        }

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {
            return amp::Path2D();
        }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            
            
            return amp::Path2D();
        }

};

class MyCSpaceCtor : public amp::GridCSpace2DConstructor {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override {            
            return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
        }
};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo()
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {}

        // You can have custom ctor params for all of these classes
        MyManipWFAlgo(const std::string& beep) 
            : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyCSpaceCtor>()) {LOG("construcing... " << beep);}

        // This is just to get grade to work, you DO NOT need to override this method
        virtual amp::ManipulatorTrajectory2Link plan(const LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem) override {
            return amp::ManipulatorTrajectory2Link();
        }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override {
            return amp::Path2D();
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
    if(true)
    {
        amp::Problem2D ex1a = HW2::getWorkspace1();
        printf("Max x = %.2f\n", ex1a.x_max);
        MyPointWFAlgo ex1;
        amp::GridCSpace2D * ex1g = ex1.constructDiscretizedWorkspace(ex1a).release();
        pair<size_t, size_t> tst= ex1g->getCellFromPoint(1.8,2.0);

        bool p1 = ex1g->inCollision(0, 0);
        bool p2 = ex1g->inCollision(1.4, 2.1);

        Visualizer::makeFigure(*ex1g);
        printf("Grid (0,0) = %d Grid (%ld, %ld) = %d\n",p1, tst.first, tst.second, p2);

        Problem2D ex1b = HW2::getWorkspace2();
        Visualizer::showFigures(); 
    }

    // Exercise 2
    {

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

    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}