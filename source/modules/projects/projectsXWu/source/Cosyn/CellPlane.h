#ifndef CellPlane_H
#define CellPlane_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Util\SurfaceGrid.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

struct CellPlane
{
public:
    CellPlane(void);
    ~CellPlane(void);
    bool empty(void) { return points.empty(); }
    bool empty(const float& num) { return points.size() < num; }
    size_t size(void) { return points.size(); }
    void push_back(const unsigned& px) { points.push_back(px); }
    PointSet* GetCellPS(UICPC* data = nullptr);

    void EstimatePlane(PointSet* PS);
    void EstimateNormal(PointSet* PS);
    void DrawPS(UICPC* data, const Vector3f &color);
    void DrawPlane(const Vector3f &color, const float& scale = 1.f, const float& linewidth = 1.f);
public:
    std::deque<unsigned> points;
    Vector3f normal;
    Vector3f dir0, dir1;
    Vector3f point;
    PointSetANNQueryPtr KNN;
    SurfaceGrid<CellPlane>::Ptr sub;
    bool isBon;
private:
    PointSet* patch;
};

#endif
