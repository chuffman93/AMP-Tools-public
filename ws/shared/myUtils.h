#include <queue>
#include <cmath>

#include "AMPCore.h"

using namespace std;
using namespace amp;

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
            if(abs(val) < 1e-3)
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
            if((b[0] < max(a.p1[0], a.p2[0]) || abs(b[0]-max(a.p1[0], a.p2[0])) < 1e-3) 
            && (b[0] > min(a.p1[0], a.p2[0]) || abs(b[0]-min(a.p1[0], a.p2[0])) < 1e-3) 
            && (b[1] < max(a.p1[1], a.p2[1]) || abs(b[1]-max(a.p1[1], a.p2[1])) < 1e-3)
            && (b[1] > min(a.p1[1], a.p2[1]) || abs(b[1]-min(a.p1[1], a.p2[1])) < 1e-3))
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

        struct MapSearchResult {
            /// @brief Set to `true` if path was found, `false` if no path exists
            bool success = false;

            /// @brief Sequence of nodes where `node_path.front()` must contain init node, and `node_path.back()` must contain the goal node
            std::list<pair<double,double>> node_path;

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
            printPath(ret);
            return ret;
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

        pair<double, double> newPt(pair<double,double> a, pair<double,double> b, double step_size)
        {
            double m = (b.second - a.second) / (b.first - a.first);
            double rad = atan(m);
            return make_pair(round<double>(a.first+step_size*cos(rad),2), round<double>(a.second+step_size*sin(rad),2));
        }

        double euc_dis(pair<double, double> a, pair<double, double> b)
        {
            return sqrt(pow(a.first-b.first,2) + pow(a.second-b.second,2));
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
            return (euc_dis(pt1, pt2) < r);
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

};