#include "MyGridCSpace2DConstructor.h"

unique_ptr<GridCSpace2D> MyGridCSpace2DConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env)
{
    unique_ptr<MyGridCSpace2D> ret;
    int m = 100;
    int n = 100;
    double x0_max = 2*M_PI;
    double x1_max = x0_max;
    double x0_min = 0.0;
    double x1_min = x0_min;
    double x0Diff = (x0_max - x0_min)/m;
    double x1Diff = (x1_max - x1_min)/n;
    
    
    double x0 = x0_min;
    double x1 = x1_min;

    printf("cSpace size(%ld, %ld)\n",  cSpace.size().first,     cSpace.size().second);
    printf("x0 bounds (%.2f, %.2f)\n", cSpace.x0Bounds().first, cSpace.x0Bounds().second);
    printf("x1 bounds (%.2f, %.2f)\n", cSpace.x1Bounds().first, cSpace.x1Bounds().second);

    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < n; j++)
        {
            cSpace(i,j) = cSpace.inCollision(x0+(x0Diff*i), x1+(x1Diff*j));
        }
    }
    cout << cSpace.data()[0] << endl;
    ret.reset(&cSpace);
    return ret;
}