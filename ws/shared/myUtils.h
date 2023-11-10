#ifndef MYUTILS_H
#define MYUTILS_H
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
                    break;;
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
                    m = (cPt[1] - pLine.p1[1]) / (cPt[0] - pLine.p1[0]);
                    rad = atan(m);
                    pLine.p2 = Eigen::Vector2d{a.first+((radius+0.01)*cos(rad)), a.second+((radius+0.01)*sin(rad))};
                    if(isInt(v,pLine)) 
                    {
                        return true;
                    }
                }
            }
            return false;
        }

        Eigen::Vector2d getClosestPoint(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d pt)
        {
            Eigen::Vector2d v1v2 = v2 - v1;
            Eigen::Vector2d v1pt = pt - v1;
            double sq = (v1v2[0]*v1v2[0]) + (v1v2[1]*v1v2[1]);
            double t = ((v1pt[0]*v1v2[0]) + (v1pt[1]*v1v2[1]))/sq;
            return v1 + t*v1v2;
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
            std::list<vector<pair<double,double>>> node_path;

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
            // printPath(ret);
            return ret;
        }

        MapSearchResultMultiAgent dij_search(map< vector<pair<double,double>>, map< vector<pair<double,double>>, double> > RM,
                                vector<pair<double,double>> st, 
                                vector<pair<double,double>> g)
        {
            MapSearchResultMultiAgent ret;

            vector<pair<double, double>> bst;

            vector<double> weights;
            
            typedef pair<double, vector<pair<double, double>>> nW;
            map< vector<pair<double,double>>, double>dist;
            map <vector<pair<double, double>>, vector<pair<double, double>>> bkPtr;
            
            priority_queue<nW, vector<nW>, greater<nW> > O;
            nW tmpTp;
            list<vector<pair<double, double>>> Ol;
            list<vector<pair<double, double>>> C;
            list<vector<pair<double, double>>> q;

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
            vector<pair<double,double>> tmp;
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

        bool inbounds(vector<pair<double, double> > a, double xmin, double xmax, double ymin, double ymax)
        {
            for(auto itr : a)
            {
                if(!(itr.first > xmin && itr.first < xmax && itr.second > ymin && itr.second < ymax))
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

        pair<double, double> newPt(pair<double,double> a, pair<double,double> b, double step_size)
        {
            double m = (b.second - a.second) / (b.first - a.first);
            double rad = atan(m);
            return make_pair(round_double(a.first+step_size*cos(rad),2), round_double(a.second+step_size*sin(rad),2));
        }

        vector<pair<double, double>> newPt(vector<pair<double,double>> a, vector<pair<double,double>> b, size_t numAgents, double step_size)
        {
            double m;
            double rad;
            vector<pair<double,double>> ret;
            for(int i = 0; i < numAgents; i++)
            {
                m = (b[i].second - a[i].second) / (b[i].first - a[i].first);
                rad = atan(m);
                ret.push_back(make_pair(round_double(a[i].first+step_size*cos(rad),2), round_double(a[i].second+step_size*sin(rad),2)));
            }
            return ret;
        }

        double euc_dis(pair<double, double> a, pair<double, double> b)
        {
            return sqrt(pow(a.first-b.first,2) + pow(a.second-b.second,2));
        }

         double euc_dis(vector<pair<double, double>> a, vector<pair<double, double>> b)
        {
            double sum = 0;
            for(int i = 0; i < a.size(); i++)
            {
                sum += pow(a[i].first-b[i].first,2) + pow(a[i].second-b[i].second,2);
            }
            return sqrt(sum);
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

        MultiAgentPath2D pairToEigenVector(list<vector<pair<double, double>>> a, size_t numAgents)
        {
            MultiAgentPath2D ret(numAgents);

            for(auto itr : a)
            {
                for(int i = 0; i < numAgents; i++)
                {
                    ret.agent_paths[i].waypoints.push_back(Eigen::Vector2d{itr[i].first, itr[i].second});
                }
            }
            return ret;
        }

};
#endif