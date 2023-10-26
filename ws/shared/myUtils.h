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

};