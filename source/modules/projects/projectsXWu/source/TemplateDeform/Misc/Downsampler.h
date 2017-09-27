#ifndef Downsampler_h_
#define Downsampler_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
//#include "PTypes.h"
//----------------------------------------------------------------------
#include "SparseGrid.h"
//----------------------------------------------------------------------
#include <vector>

namespace X4
{

//======================================================================
// class Downsampler
//----------------------------------------------------------------------
class Downsampler
{
public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  Downsampler(float32 const gridSpacing);

  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  Vector3i Cell(Vector3f const& point) const;

  void AddPoint(Vector3f const& point);

  void AddPoints(
    PointSet const& points, std::vector<mpcard> const& selection);

  void AddPoints(std::vector<Vector3f> const& points);

  std::vector<Vector3f> Points() const;

private:
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  SparseGrid<3, int32, Object> grid_;
  
  std::vector<std::vector<Vector3f> > entries_;

  float32 gridSpacing_;
};

//======================================================================
// class DownsamplerWithTargetAndNormal
//----------------------------------------------------------------------
class DownsamplerWithTargetAndNormal
{
public:
  //====================================================================
  // Constructors
  //--------------------------------------------------------------------
  DownsamplerWithTargetAndNormal(float32 const gridSpacing);

  //====================================================================
  // Member functions
  //--------------------------------------------------------------------
  Vector3i Cell(Vector3f const& point) const;

  void AddPoint(Vector3f const& anchor,
                Vector3f const& target,
                Vector3f const& normal,
                Vector3f const& source = NULL_VECTOR3F);

  std::vector<Vector3f> Anchors() const;
  std::vector<Vector3f> Targets() const;
  std::vector<Vector3f> Normals() const;
  std::vector<Vector3f> Sources() const;

private:
  //====================================================================
  // Member variables
  //--------------------------------------------------------------------
  SparseGrid<3, int32, Object> grid_;
  
  std::vector<std::vector<Vector3f> > anchors_;
  std::vector<std::vector<Vector3f> > targets_;
  std::vector<std::vector<Vector3f> > normals_;
  std::vector<std::vector<Vector3f> > sources_;

  float32 gridSpacing_;
};

} //namespace X4

#endif //Downsampler_h_
