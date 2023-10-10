#include "MyGridCSpace2DConstructor.h"

unique_ptr<GridCSpace2D> MyGridCSpace2DConstructor::construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env)
{
    
    int m = 100;
    int n = 100;
    double x0_max = 2*M_PI;
    double x1_max = x0_max;
    double x0_min = 0.0;
    double x1_min = x0_min;
    double x0Diff = (x0_max - x0_min)/m;
    double x1Diff = (x1_max - x1_min)/n;
    MyGridCSpace2D cSpace = MyGridCSpace2D(m, n, x0_min, x0_max, x1_min, x1_max);
    unique_ptr<MyGridCSpace2D> cSPtr;
    cSPtr.reset(&cSpace);

    double x0 = x0_min;
    double x1 = x1_min;

    // printf("cSpace size(%ld, %ld)\n",  cSPtr->size().first,     cSPtr->size().second);
    // printf("x0 bounds (%.2f, %.2f)\n", cSPtr->x0Bounds().first, cSPtr->x0Bounds().second);
    // printf("x1 bounds (%.2f, %.2f)\n", cSPtr->x1Bounds().first, cSPtr->x1Bounds().second);

    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < n; j++)
        {
            cSPtr->operator()(i,j) = cSPtr->inCollision(x0+(x0Diff*i), x1+(x1Diff*j));
        }
    }
    return cSPtr;
}