#ifndef CellNN_H
#define CellNN_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Util\SurfaceGrid.h"
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

struct CellNN
{
public:
    CellNN(void);
    ~CellNN(void);
    bool empty(void) { return points.empty(); }
    size_t size(void) { return points.size(); }
    void push_back(const unsigned& px) { points.push_back(px); }
    PointSet* GetCellPS(UICPC* data = nullptr);

    void DrawPS(UICPC* data, const Vector3f &color);
public:
    std::deque<unsigned> points;
    PointSetANNQueryPtr KNN;
    bool isBon;
private:
    PointSet* patch;
};

#endif
