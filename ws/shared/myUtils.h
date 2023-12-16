#ifndef MYUTILS_H
#define MYUTILS_H
#include <queue>
#include <cmath>

#include "AMPCore.h"

using namespace std;
using namespace amp;


class MyGridCSpace : public amp::GridCSpace2D {
    public:
        //MyGridCSpace()
        //    : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0) {}
        MyGridCSpace(double a, std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {
                xMin = x0_min;
                yMin = x1_min;
                dis = a;
            }

        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {
                xMin = x0_min;
                yMin = x1_min;
                dis = 0.25;
            }

        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const {
            return make_pair(floor( (x0 - this->xMin)/this->dis ), floor( (x1 - this->yMin)/this->dis ));
        }

        double dis;
        double xMin;
        double yMin;
};

class myUtils {
    public:
        void hereIsAMethod();

        struct line {
            Eigen::Vector2d p1, p2;
        };

        int linDir(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c)
        {
            double val = ((b[1] - a[1]) * (c[0] - b[0])) 
                       - ((b[0] - a[0]) * (c[1] - b[1]));
            if(abs(val) < 1e-6)
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
            if((b[0] < max(a.p1[0], a.p2[0]) || abs(b[0]-max(a.p1[0], a.p2[0])) < 1e-6) 
            && (b[0] > min(a.p1[0], a.p2[0]) || abs(b[0]-min(a.p1[0], a.p2[0])) < 1e-6) 
            && (b[1] < max(a.p1[1], a.p2[1]) || abs(b[1]-max(a.p1[1], a.p2[1])) < 1e-6)
            && (b[1] > min(a.p1[1], a.p2[1]) || abs(b[1]-min(a.p1[1], a.p2[1])) < 1e-6))
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

        bool checkInObj(Eigen::Vector2d p, vector<Eigen::Vector2d> verts)
        {
            
            int count = 0;
            line pLine{p, Eigen::Vector2d{999.0, p[1]}};
            line v;
            for(int wI = 0; wI < verts.size()-1; wI++)
            {
                v.p1 = verts[wI];
                v.p2 = verts[wI+1];
                if(isInt(v,pLine))
                {
                    if(linDir(v.p1, p, v.p2) == 0)
                    {
                        return onL(v, p);
                    } 
                    count++;
                }
            }
            return count & 1;
        }

        bool checkInObj(vector<line> arm, vector<Eigen::Vector2d> verts)
        {
            
            int count = 0;
            line v;
            for(auto al : arm)
            {
                for(int wI = 0; wI < verts.size()-1; wI++)
                {
                    v.p1 = verts[wI];
                    v.p2 = verts[wI+1];
                    if(isInt(v,al) || checkInObj(al.p2, verts) || checkInObj(al.p1, verts)) 
                    {
                        return true;
                        // if(linDir(v.p1, p, v.p2) == 0)
                        // {
                        //     return onL(v, p);
                        // } 
                        // count++;
                    }
                }
            }
            return false;
        }

        bool checkInObj(pair<double, double> a, pair<double, double> b, vector<Eigen::Vector2d> verts)
        {
            
            int count = 0;
            line pLine{Eigen::Vector2d{a.first, a.second}, Eigen::Vector2d{b.first, b.second}};
            line v;
            for(int wI = 0; wI < verts.size()-1; wI++)
            {
                v.p1 = verts[wI];
                v.p2 = verts[wI+1];
                if(isInt(v,pLine)) 
                {
                    return true;
                    // if(linDir(v.p1, p, v.p2) == 0)
                    // {
                    //     return onL(v, p);
                    // } 
                    // count++;
                }
            }
            return false;
        }


        bool checkInObj(pair<double, double> a, pair<double, double> b, vector<Obstacle2D> obs)
        {
            
            int count = 0;
            line pLine{Eigen::Vector2d{a.first, a.second}, Eigen::Vector2d{b.first, b.second}};
            line v;
            vector<Eigen::Vector2d> verts;
            for(int i = 0; i < obs.size(); i++)
            {
                verts = obs[i].verticesCCW();
                for(int wI = 0; wI < verts.size()-1; wI++)
                {
                    v.p1 = verts[wI];
                    v.p2 = verts[wI+1];
                    if(isInt(v,pLine)) 
                    {
                        return true;
                        // if(linDir(v.p1, p, v.p2) == 0)
                        // {
                        //     return onL(v, p);
                        // } 
                        // count++;
                    }
                }
            }
            return false;
        }

        bool checkInObj(pair<double, double> a, vector<Obstacle2D> obs)
        {
            
            int count = 0;
            line pLine{Eigen::Vector2d{a.first, a.second}, Eigen::Vector2d{999.0, a.second}};
            line v;
            vector<Eigen::Vector2d> verts;
            for(int i = 0; i < obs.size(); i++)
            {
                verts = obs[i].verticesCCW();
                verts.push_back(verts[0]);
                for(int wI = 0; wI < verts.size()-1; wI++)
                {
                    v.p1 = verts[wI];
                    v.p2 = verts[wI+1];
                    if(isInt(v,pLine)) 
                    {
                        if(linDir(v.p1, Eigen::Vector2d{a.first, a.second}, v.p2) == 0)
                        {
                            return onL(v,  Eigen::Vector2d{a.first, a.second});
                        } 
                        count++;
                    }
                }
                if(count % 2 != 0)
                {
                    break;
                }
            }
            return count & 1;
        }

        bool checkInObj(pair<double, double> a, double radius, vector<Obstacle2D> obs, int dec_per)
        {
            
            int count = 0;
            line pLine;
            pLine.p1 = Eigen::Vector2d{a.first,a.second};
            Eigen::Vector2d cPt;
            line v;
            double m;
            double rad;
            vector<Eigen::Vector2d> verts;
            for(int i = 0; i < obs.size(); i++)
            {
                verts = obs[i].verticesCCW();
                verts.push_back(verts[0]);
                for(int wI = 0; wI < verts.size()-1; wI++)
                {
                    v.p1 = verts[wI];
                    v.p2 = verts[wI+1];
                    cPt = getClosestPoint(v.p1, v.p2, pLine.p1);
                    rad = atan2((cPt[1] - pLine.p1[1]), (cPt[0] - pLine.p1[0]));
                    pLine.p2 = Eigen::Vector2d{a.first+((radius+0.1)*cos(rad)), a.second+((radius+0.1)*sin(rad))};
                    if(isInt(v,pLine) || (euc_dis(pLine.p1, cPt) < radius+0.1)) 
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        Eigen::Vector2d getClosestPoint(Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d P)
        {
            Eigen::Vector2d AB = B - A;
            Eigen::Vector2d AP = P - A;
            double sq =  (AB[0]*AB[0]) + (AB[1]*AB[1]);
            double t  = ((AP[0]*AB[0]) + (AP[1]*AB[1]))/sq;
            if(t < 0)
            {
                t = 0;
            }
            if(t > 1)
            {
                t = 1;
            }


            return A + (t*AB);
        }

        struct MapSearchResult {
            /// @brief Set to `true` if path was found, `false` if no path exists
            bool success = false;

            /// @brief Sequence of nodes where `node_path.front()` must contain init node, and `node_path.back()` must contain the goal node
            std::list<pair<double,double>> node_path;

            /// @brief Path cost (must equal sum of edge weights along node_path)
            double path_cost;
        };

        struct MapSearchResultMultiAgent {
            /// @brief Set to `true` if path was found, `false` if no path exists
            bool success = false;

            /// @brief Sequence of nodes where `node_path.front()` must contain init node, and `node_path.back()` must contain the goal node
            std::list<vector<double>> node_path;

            /// @brief Path cost (must equal sum of edge weights along node_path)
            double path_cost;
        };

        MapSearchResult dij_search(map< pair<double,double>, map< pair<double,double>, double > > RM,
                                pair<double,double> st, 
                                pair<double,double> g)
        {
            MapSearchResult ret;

            pair<double, double> bst;

            vector<pair<double, double>> childs;
            vector<double> weights;
            
            typedef pair<double, pair<double, double>> nW;
            map <pair<double, double>, double> dist;
            map <pair<double, double>, pair<double, double>> bkPtr;
            
            priority_queue<nW, vector<nW>, greater<nW> > O;
            nW tmpTp;
            list<pair<double, double>> Ol;
            list<pair<double, double>> C;
            list<pair<double, double>> q;

            double gn;
            double h;
            double f;

            O.push(make_pair(0, st));
            dist.insert({st, 0});
            Ol.push_back(st);
            q.push_back(st);

            int itrCount = 0;
            if(st == g)
            {
                ret.success = true;
                ret.path_cost = 0;
                return ret;
            }
            while(!O.empty())
            {
                bst = O.top().second;
                // printf("Bst {%.2f, %.2f}\n", bst.first, bst.second);
                O.pop();
                Ol.remove(bst);
                C.push_back(bst);

                if (bst == g)
                {
                    q.push_back(g);
                    break;
                }
                for(auto chld : RM[bst])
                {
                    if(find(C.begin(), C.end(), chld.first) != C.end())
                    {
                        continue;
                    }

                    if(find(Ol.begin(), Ol.end(), chld.first) == Ol.end())
                    {
                        gn = RM[bst][chld.first] + dist.find(bst)->second;
                        // printf("GN = %.2f\n",gn);
                        O.push( make_pair(gn, chld.first) );
                        Ol.push_back(chld.first);
                        dist.insert({chld.first, gn});
                        bkPtr.insert({chld.first, bst});
                    }
                    else if((dist.find(bst)->second + RM[bst][chld.first] ) < dist.find(chld.first)->second)
                    {
                        gn = RM[bst][chld.first] + dist.find(bst)->second;
                        // printf("GN = %.2f\n",gn);
                        O.push( make_pair(gn, chld.first) );
                        bkPtr.find(chld.first)->second = bst;
                        dist.find(chld.first)->second = gn;
                    }
                }
                itrCount += 1;
            }
            pair<double,double> tmp;
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
            // printPath(ret);
            return ret;
        }

        MapSearchResultMultiAgent dij_search(map< vector<double>, map< vector<double>, double> > RM,
                                vector<double> st, 
                                vector<double> g)
        {
            MapSearchResultMultiAgent ret;

            vector<double> bst;

            vector<double> weights;
            
            typedef pair<double, vector<double>> nW;
            map< vector<double>, double>dist;
            map <vector<double>, vector<double>> bkPtr;
            
            priority_queue<nW, vector<nW>, greater<nW> > O;
            nW tmpTp;
            list<vector<double>> Ol;
            list<vector<double>> C;
            list<vector<double>> q;

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
                bst = O.top().second;
                // printf("Bst {%.2f, %.2f}\n", bst.first, bst.second);
                O.pop();
                Ol.remove(bst);
                C.push_back(bst);

                if (bst == g)
                {
                    q.push_back(g);
                    break;
                }
                for(auto chld : RM[bst])
                {
                    if(find(C.begin(), C.end(), chld.first) != C.end())
                    {
                        continue;
                    }

                    if(find(Ol.begin(), Ol.end(), chld.first) == Ol.end())
                    {
                        gn = RM[bst][chld.first] + dist.find(bst)->second;
                        // printf("GN = %.2f\n",gn);
                        O.push( make_pair(gn, chld.first) );
                        Ol.push_back(chld.first);
                        dist.insert({chld.first, gn});
                        bkPtr.insert({chld.first, bst});
                    }
                    else if((dist.find(bst)->second + RM[bst][chld.first] ) < dist.find(chld.first)->second)
                    {
                        gn = RM[bst][chld.first] + dist.find(bst)->second;
                        // printf("GN = %.2f\n",gn);
                        O.push( make_pair(gn, chld.first) );
                        bkPtr.find(chld.first)->second = bst;
                        dist.find(chld.first)->second = gn;
                    }
                }
                itrCount += 1;
            }
            vector<double> tmp;
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
            // printPath(ret);
            return ret;
        }

        bool inbounds(pair<double, double> a, double xmin, double xmax, double ymin, double ymax)
        {
            return (a.first > xmin && a.first < xmax && a.second > ymin && a.second < ymax);
        }

        bool inbounds(vector<double> a, double xmin, double xmax, double ymin, double ymax)
        {
            for(int i = 0; i < a.size(); i+=2)
            {
                if(!(a[i] > xmin && a[i] < xmax && a[i+1] > ymin && a[i+1] < ymax))
                {
                    return false;
                }
            }

            return true;
        }

        void printPath(MapSearchResult a)
        {
            printf("Path %s with length of the path { ", a.success ? "found" : "not found"); 
            for(auto i : a.node_path)
            {   
                cout << "("<<i.first << ", " << i.second << ") ";
            }
            printf("} is %.2f\n",a.path_cost);
        }

        void printPath(Path2D a)
        {
            printf("Length of the path { "); 
            for(auto i : a.waypoints)
            {   
                cout << "("<<i[0] << ", " << i[1] << ") ";
            }
            printf("} is %.2f\n",a.length());
        }

        vector<double> removeOutliers(vector<double> a)
        {
            vector<double> ret;
            auto const q1 = a.size()/4;
            auto const q2 = a.size()/2;
            auto const q3 = q1+q2;
            sort(a.begin(),a.end());
            for(int i = q1; i < q3; i++)
            {
                ret.push_back(a[i]);
            }
            return ret;
        }

        pair<double, double> newPt(pair<double,double> a, pair<double,double> b, double step_size, bool first)
        {
            double rad;
            double m = (b.second - a.second) / (b.first - a.first);
            if(first)
            {
                rad = atan2((b.second - a.second), (b.first - a.first));
            }
            else
            {
                rad = atan(m);
            }
            return make_pair(round_double(a.first+step_size*cos(rad),2), round_double(a.second+step_size*sin(rad),2));
        }

        pair<double, double> newPtWrapped(pair<double,double> a, pair<double,double> b, double step_size, bool first, double xmin, double xmax, double ymin, double ymax)
        {
            double rad;
            double dx = abs(a.first-b.first);
            double dy = abs(a.second-b.second);

            double yW = ymax - ymin;
            double xW = xmax - xmin;

            if(dx > (0.5*xW))
            {
                dx = xW-dx;
            }
            if(dy > (0.5*yW))
            {
                dy = yW-dy;
            }
            double m = (dy) / (dx);
            if(first)
            {
                rad = atan2((dy), (dx));
            }
            else
            {
                rad = atan(m);
            }
            return make_pair(round_double(a.first+step_size*cos(rad),2), round_double(a.second+step_size*sin(rad),2));
        }

        vector<double> newPt(vector<double> a, vector<double> b, size_t numAgents, double step_size, int dec_per)
        {
            double rad;
            vector<double> ret;
            double step;
            for(int i = 0; i < numAgents*2; i+=2)
            {
                rad = atan2((b[i+1] - a[i+1]), (b[i] - a[i]));
                if(euc_dis(make_pair(a[i],a[i+1]), make_pair(b[i],b[i+1])) < step_size)
                {
                    ret.push_back( b[i] );
                    ret.push_back( b[i+1] );
                }
                else
                {
                    ret.push_back( round_double(a[i]   +(step_size*cos(rad)),dec_per) );
                    ret.push_back( round_double(a[i+1] +(step_size*sin(rad)),dec_per) );
                }
                
            }
            return ret;
        }

        double euc_dis(pair<double, double> a, pair<double, double> b)
        {
            return sqrt(pow(a.first-b.first,2) + pow(a.second-b.second,2));
        }

        double euc_disWrapped(pair<double, double> a, pair<double, double> b, double xmin, double xmax, double ymin, double ymax)
        {
            double dx = abs(a.first-b.first);
            double dy = abs(a.second-b.second);

            double yW = ymax - ymin;
            double xW = xmax - xmin;

            if(dx > (0.5*xW))
            {
                dx = xW-dx;
            }
            if(dy > (0.5*yW))
            {
                dy = yW-dy;
            }

            return sqrt(pow(dx,2) + pow(dy,2));
        }

         double mink_dis(vector<double> a, vector<double> b, double p)
        {
            double sum = 0;
            for(int i = 0; i < a.size(); i++)
            {
                sum += pow(abs(a[i]-b[i]),p);
            }
            return pow(sum,1/p);
        }

        double euc_dis(Eigen::Vector2d a, Eigen::Vector2d b)
        {
            return sqrt(pow(a[0]-b[0],2) + pow(a[1]-b[1],2));
        }

        double round_double(double a, int places)
        {
            double multi = pow(10.0, places);
            return(round(a*multi)/multi);
        }

        bool inRadius(pair<double,double> pt1, pair<double,double>  pt2, double r)
        {
            return (round_double(euc_dis(pt1, pt2),2) <= r);
        }

        template<typename T>
        T random(T from, T to)
        {
            default_random_engine rnd{random_device{}()};
            uniform_real_distribution<T> distr(from, to);
            return distr(rnd);
        }

        Path2D pairToEigenVector(list<pair<double, double>> a)
        {
            Path2D ret;
            for(auto itr : a)
            {
                ret.waypoints.push_back(Eigen::Vector2d{itr.first, itr.second});
            }
            return ret;
        }

        Path2D pairToEigenVector(list<vector<double>> a)
        {
            Path2D ret;
            for(auto itr : a)
            {
                ret.waypoints.push_back(Eigen::Vector2d{itr[0], itr[1]});
            }
            return ret;
        }

        MultiAgentPath2D pairToEigenVector(list<vector<double>> a, size_t numAgents)
        {
            MultiAgentPath2D ret(numAgents);

            for(auto itr : a)
            {
                for(int i = 0; i < numAgents*2; i+=2)
                {
                    ret.agent_paths[(int)(i/2)].waypoints.push_back(Eigen::Vector2d{itr[i], itr[i+1]});
                }
            }
            return ret;
        }

};
#endif