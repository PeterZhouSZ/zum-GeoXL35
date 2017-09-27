#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Util/TrimeshUnorderedGrid.h"
#include "Util/NoUse.h"
#include <cmath>
//---------------------------------------------------------------------------
#include "VertexArray.h"
#include "TriangleBBIntersection.h"
//---------------------------------------------------------------------------

UnorderedGrid::UnorderedGrid(void)
{
    Clear();
}

UnorderedGrid::UnorderedGrid(const float& cellSize)
{
    Clear();
    cell_size_ = cellSize;
}

UnorderedGrid::~UnorderedGrid(void)
{
    Clear();
}

void UnorderedGrid::Clear(void)
{
    cell_grid_.clear();
    cell_size_ = -.1f;
    num_points_ = 0;
}

int UnorderedGrid::AddPoint(const Vector3f& point, Vector3i* p_idx)
{
    Vector3i& idx = *p_idx;
    idx = GetCellIndexFromPoint(point);
    if (cell_grid_.end() == cell_grid_.find(idx)) {
        GCell pgc;
        pgc.indice.push_back(num_points_++);
        pgc.posVec.push_back(point);
        cell_grid_[idx] = pgc;
        return 1;
    } else {
        GCell& pgc = cell_grid_[idx];
        pgc.indice.push_back(num_points_++);
        pgc.posVec.push_back(point);
        return 0;
    }
}

int UnorderedGrid::AddPoint(const Vector3f& point, Vector3i* p_idx, Vector3f* p_posUpdated)
{
    Vector3i& idx = *p_idx;
    idx = GetCellIndexFromPoint(point);
    if (cell_grid_.end() == cell_grid_.find(idx)) {
        GCell pgc;
        pgc.indice.push_back(num_points_++);
        pgc.posVec.push_back(point);
        cell_grid_[idx] = pgc;
        *p_posUpdated = pgc.GetCenter();
        return 1;
    } else {
        GCell& pgc = cell_grid_[idx];
        pgc.indice.push_back(num_points_++);
        pgc.posVec.push_back(point);
        *p_posUpdated = cell_grid_[idx].GetCenter();
        return 0;
    }
}

int UnorderedGridUniq::AddPoint(const Vector3f& point, Vector3i* p_idx)
{
    Vector3i& idx = *p_idx;
    idx = GetCellIndexFromPoint(point);
    if (cell_grid_.end() == cell_grid_.find(idx)) {
        GCell pgc;
        pgc.indice.push_back(num_points_++);
        pgc.posVec.push_back(point);
        cell_grid_[idx] = pgc;
        return 1;
    } else {
        GCell& pgc = cell_grid_[idx];
        pgc.posVec.push_back(point);
        return 0;
    }
}

int UnorderedGridUniq::AddPoint(const Vector3f& point, Vector3i* p_idx, Vector3f* p_posUpdated)
{
    Vector3i& idx = *p_idx;
    idx = GetCellIndexFromPoint(point);
    if (cell_grid_.end() == cell_grid_.find(idx)) {
        GCell pgc;
        pgc.indice.push_back(num_points_++);
        pgc.posVec.push_back(point);
        cell_grid_[idx] = pgc;
        *p_posUpdated = pgc.GetCenter();
        return 1;
    } else {
        GCell& pgc = cell_grid_[idx];
        pgc.posVec.push_back(point);
        *p_posUpdated = pgc.GetCenter();
        return 0;
    }
}

unsigned UnorderedGridUniq::GetOrdx(const Vector3i& idx)
{
    return cell_grid_[idx].indice.front();
}

unsigned UnorderedGridUniq::GetOrdx(const Vector3f& point)
{
    Vector3i idx;
    AddPoint(point, &idx);
    return GetOrdx(idx);
}

void UnorderedGrid::DrawWithDR(const Vector3f& color)
{
    for (grid_map_type::iterator it = cell_grid_.begin();
        it != cell_grid_.end(); ++it) {
            const BoundingBox3f& bb = GetCellBBoxFromIndex(it->first);
            debugRenderer->addBoundingBox(bb, color, 1);
    }
}

bool UnorderedGrid::CheckDrawAdjacency(void)
{
    bool ret = false;
    for (iterator it = cell_grid_.begin(); it != cell_grid_.end(); ++it) {
        const Vector3i index = it->first;
        unsigned cnt = 0;
        std::deque<Vector3f> npos;
        for (int x = -1; x < 2; ++x) {
            for (int y = -1; y < 2; ++y) {
                for (int z = -1; z < 2; ++z) {
                    const Vector3i neighbor = index + makeVector3i(x, y, z);
                    if (cell_grid_.end() == cell_grid_.find(neighbor)) continue;
                    ++cnt;
                    npos.push_back(GetCellCenterFromIndex(neighbor));
                }
            }
        }
        if (1 == cnt) continue;
        ret = true;

        const Vector3f ccen = GetCellCenterFromIndex(index);
        BoundingBox3f bb;
        bb.lowerCorner = bb.upperCorner = ccen;
        bb.addBorder(cell_size_ * 1.5f);
        debugRenderer->addBoundingBox(bb, makeVector3f(1, 1, 0), 1);
        for (const Vector3f pos : npos) {
            debugRenderer->addLine(ccen, pos, makeVector3f(1, 1, 0), makeVector3f(1, 1, 0), 1);
        }
    }
    if (ret) DrawWithDR();
    return ret;
}

Vector3i UnorderedGrid::GetCellIndexFromPoint(const Vector3f& pos) const
{
    // rounding behevior, so centered at grid point exactly
    Vector3f cen = pos / cell_size_;
    cen[0] = (cen[0] > 0.0) ? floor(cen[0] + 0.5) : ceil(cen[0] - 0.5);
    cen[1] = (cen[1] > 0.0) ? floor(cen[1] + 0.5) : ceil(cen[1] - 0.5);
    cen[2] = (cen[2] > 0.0) ? floor(cen[2] + 0.5) : ceil(cen[2] - 0.5);

    return makeVector3i(
        static_cast<int32>(cen[0]), static_cast<int32>(cen[1]), static_cast<int32>(cen[2])
        );
}

Vector3f UnorderedGrid::GetCellCenterFromIndex(const Vector3i& idx) const
{
    return makeVector3f(idx[0], idx[1], idx[2]) * cell_size_;
}

Vector3f UnorderedGrid::GetCellCenterFromPoint(const Vector3f& pos) const
{
    Vector3i idx = GetCellIndexFromPoint(pos);
    return GetCellCenterFromIndex(idx);
}

BoundingBox3f UnorderedGrid::GetCellBBoxFromIndex(const Vector3i& idx) const
{
    Vector3f ccen = GetCellCenterFromIndex(idx);
    BoundingBox3f bb;
    bb.lowerCorner = bb.upperCorner = ccen;
    bb.addBorder(cell_size_ / 2.0f);
    return bb;
}

BoundingBox3f UnorderedGrid::GetCellBBoxFromPoint(const Vector3f& pos) const
{
    Vector3i idx = GetCellIndexFromPoint(pos);
    return GetCellBBoxFromIndex(idx);
}

void TrimeshUnorderedGrid::Setup(UnstructuredInCoreTriangleMesh* mesh, const float& cellSize)
{
    Clear();
    bounding_box_ = getPCBBox(mesh);
    bounding_box_.addBorder(cellSize);
    cell_size_ = cellSize;

    VertexArray va_ver(mesh->getPointSet());
    VertexArray va_tri(mesh->getTriangles());

    for (unsigned ti = 0; ti < va_tri.getNumElements(); ++ti) {
        Vector3i tri = va_tri.getIndex3i(ti);
        array3vec3f verts;
        verts[0] = va_ver.getPosition3f(tri[0]);
        verts[1] = va_ver.getPosition3f(tri[1]);
        verts[2] = va_ver.getPosition3f(tri[2]);
        Vector3f normal;
        if (!NU::CompTriNormal(verts, normal))
            continue;

        BoundingBox3f bbGrid;
        bbGrid.lowerCorner = bbGrid.upperCorner =
            GetCellCenterFromPoint(verts[0]);
        bbGrid.addPoint(GetCellCenterFromPoint(verts[1]));
        bbGrid.addPoint(GetCellCenterFromPoint(verts[2]));
        bbGrid.addBorder(cellSize * 2); // for better coverage at curved part

        for (float x = bbGrid.lowerCorner[0]; x < bbGrid.upperCorner[0]; x += cellSize) {
            for (float y = bbGrid.lowerCorner[1]; y < bbGrid.upperCorner[1]; y += cellSize) {
                for (float z = bbGrid.lowerCorner[2]; z < bbGrid.upperCorner[2]; z += cellSize) {
                    BoundingBox3f bb = GetCellBBoxFromPoint(makeVector3f(x,y,z));
                    bb.addBorder(cell_size_ * 1e-1);
                    if (!triangleIntersectsBB(verts[0], verts[1], verts[2], bb))
                        continue;

                    const Vector3f posGrid = makeVector3f(x,y,z);
                    const Vector3i idx = GetCellIndexFromPoint(posGrid);
                    if (cell_grid_.end() == cell_grid_.find(idx)) {
                        GCell pgc;
                        pgc.indice.push_back(ti);
                        pgc.posVec.push_back(posGrid);
                        cell_grid_[idx] = pgc;
                    } else {
                        GCell& pgc = cell_grid_[idx];
                        pgc.indice.push_back(ti);
                        pgc.posVec.push_back(posGrid);
                    }
                }
            }
        }
    }
}

void TrimeshUnorderedGridStatic::Setup(TrimeshStatic::Ptr smesh, const float& cellSize)
{
    smesh_ = smesh;
    TrimeshUnorderedGrid::Setup(&*smesh->GetMesh(), cellSize);
}
