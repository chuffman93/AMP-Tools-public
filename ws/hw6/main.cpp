#include <queue>
#include <map>

#include "AMPCore.h"
#include "hw/HW6.h"

using namespace amp;
using namespace std;

class MyGridCSpace : public amp::GridCSpace2D {
    public:
        //MyGridCSpace()
        //    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {
            return {0, 0};
        }
};

class MyCSpaceCtor : public amp::GridCSpace2DConstructor {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override {
            return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
        }
};

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override {
            return std::make_unique<MyGridCSpace>(1, 1, 0.0, 1.0, 0.0, 1.0);
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
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override {
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

            O.push(make_pair(0,st));
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

                printf("Best Node: %d\n", bst);

                for(int i = 0; i < childs.size(); i++)
                {
                    printf("Child Node: %d\n", childs[i]);
                    
                    if(find(C.begin(), C.end(), childs[i]) != C.end())
                    {
                        printf("Child %d found!!\n", childs[i]);
                        continue;
                    }

                    if(find(Ol.begin(), Ol.end(), childs[i]) == Ol.end() || Ol.empty())
                    {
                        h = heuristic.operator()(childs[i]);
                        gn = weights[i] + dist.find(bst)->second;
                        printf("Heur Return: %.2f, Length: %.2f for Node: %d\n",h, gn, childs[i]);
                        O.push( make_pair(h + gn, childs[i]) );
                        Ol.push_back(childs[i]);
                        dist.insert({childs[i], gn});
                        bkPtr.insert({childs[i], bst});
                    }
                    else if((dist.find(bst)->second + weights[i] ) < dist.find(childs[i])->second)
                    {
                        bkPtr.find(childs[i])->second = bst;
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
                ret.path_cost = round(dist.find(g)->second*1000.0)/1000.0;

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
    // Exercise 3
    {
        MyAStarAlgo ex3;
        amp::ShortestPathProblem ex3p = HW6::getEx3SPP();
        amp::LookupSearchHeuristic ex3h = HW6::getEx3Heuristic();
        amp::AStar::GraphSearchResult res = ex3.search(ex3p, ex3h);

        bool ex3Pass = HW6::checkGraphSearchResult(res, ex3p, ex3h, true);
    }

    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    return 0;
}