#include "MyGridCSpace2DConstructor.h"

std::unique_ptr<amp::GridCSpace2D> MyGridCSpace2DConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env)
{ 
    ASSERT(manipulator.nLinks() == 2, "Manipulator must have two links");
    double x0min = 0.0;
    double x0max = 2*M_PI;
    double x1min = 0.0;
    double x1max = 2*M_PI;


    pair<size_t, size_t> tmp;

    vector<Obstacle2D> obs = env.obstacles;
    int n = manipulator.nLinks();
    std::size_t x0cells;
    std::size_t x1cells;
    unique_ptr<MyGridCSpace> ret;
    vector<myUtils::line> links; 
    // printf("trying to run cspace stuff\n");
    try 
    {
        // printf("Initial Try\n");
        x0cells = ((abs(x0min) + abs(x0max)) / this->dis);
        x1cells = ((abs(x1min) + abs(x1max)) / this->dis);
        ret = std::make_unique<MyGridCSpace>(this->dis, x0cells, x1cells, x0min, x0max, x1min, x1max);
        links.clear(); 

        for(double i = x0min; i < x0max; i+=(this->dis/3) )
        {
            for(double j = x1min; j < x1max; j+=(this->dis/3) )
            {
                // printf("{%f,%f}\n",i,j);
            
                tmp = ret->getCellFromPoint(i,j);
                // printf("Checking cell {%ld, %ld} with xmax = %ld and ymax = %ld\n",tmp.first, tmp.second, x0cells, x1cells);
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
                    if(this->hw4Utils.checkInObj(links, verts))
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
        // printf("caught!!\n");
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
                    if(hw4Utils.checkInObj(links, verts))
                    {
                        ret->operator()(tmp.first, tmp.second) = true;
                        break;
                    }
                }

                links.clear();
            }
        }
    } 
    printf("Returning!\n");
    return ret;
}