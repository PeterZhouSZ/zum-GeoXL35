#ifndef Downsampler_inline_h_
#define Downsampler_inline_h_

//----------------------------------------------------------------------
//#include "StdAfx.h"
//----------------------------------------------------------------------
#include "Downsampler.h"
//----------------------------------------------------------------------

namespace X4
{

//======================================================================
// Constructors
//----------------------------------------------------------------------
inline Downsampler::Downsampler(float32 const gridSpacing)
  : gridSpacing_(gridSpacing)
{}

//======================================================================
// Member functions
//----------------------------------------------------------------------
inline Vector3i Downsampler::Cell(Vector3f const& point) const
{
  return makeVector3i(floor(point[0] / gridSpacing_),
                      floor(point[1] / gridSpacing_),
                      floor(point[2] / gridSpacing_));
}

//----------------------------------------------------------------------
inline void Downsampler::AddPoint(Vector3f const& point)
{
  Vector3i const cell = Cell(point);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 id = 0;

  bool present = grid_.getObject(cell, id);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (!present)
  {
    id = int32(entries_.size());

    grid_.setObject(cell, id);

    entries_.push_back(std::vector<Vector3f>());
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  entries_.at(id).push_back(point);
}

//----------------------------------------------------------------------
inline void Downsampler::AddPoints(
  PointSet const& points, std::vector<mpcard> const& selection)
{
  AAT const positionAAT = points.getAAT("position");

  if (positionAAT == NULL_AAT)
  {
    throw PException("Downsampler::AddPoints() - Attribute missing.");
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (std::vector<mpcard>::const_iterator it = selection.begin();
    it != selection.end(); ++it)
  {
    AddPoint(points.get3f(*it, positionAAT));
  }
}

//----------------------------------------------------------------------
inline void Downsampler::AddPoints(std::vector<Vector3f> const& points)
{
  for (std::vector<Vector3f>::const_iterator it = points.begin();
    it != points.end(); ++it)
  {
    AddPoint(*it);
  }
}

//----------------------------------------------------------------------
inline std::vector<Vector3f> Downsampler::Points() const
{
  std::vector<Vector3f> result;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (std::vector<std::vector<Vector3f> >::const_iterator it =
    entries_.begin(); it != entries_.end(); ++it)
  {
    Vector3f mean = NULL_VECTOR3F;

    for (std::vector<Vector3f>::const_iterator pIt = it->begin();
      pIt != it->end(); ++pIt)
    {
      mean += *pIt;
    }

    mean /= it->size();

    result.push_back(mean);
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//======================================================================
// Constructors
//----------------------------------------------------------------------
inline DownsamplerWithTargetAndNormal::DownsamplerWithTargetAndNormal(
  float32 const gridSpacing)
  : gridSpacing_(gridSpacing)
{}

//======================================================================
// Member functions
//----------------------------------------------------------------------
inline Vector3i DownsamplerWithTargetAndNormal::Cell(
  Vector3f const& point) const
{
  return makeVector3i(floor(point[0] / gridSpacing_),
                      floor(point[1] / gridSpacing_),
                      floor(point[2] / gridSpacing_));
}

//----------------------------------------------------------------------
inline void DownsamplerWithTargetAndNormal::AddPoint(
  Vector3f const& anchor,
  Vector3f const& target,
  Vector3f const& normal,
  Vector3f const& source)
{
  Vector3i const cell = Cell(anchor);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  int32 id = 0;

  bool present = grid_.getObject(cell, id);

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (!present)
  {
    id = int32(anchors_.size());

    grid_.setObject(cell, id);

    anchors_.push_back(std::vector<Vector3f>());
    targets_.push_back(std::vector<Vector3f>());
    normals_.push_back(std::vector<Vector3f>());
    sources_.push_back(std::vector<Vector3f>());
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  anchors_.at(id).push_back(anchor);
  targets_.at(id).push_back(target);
  normals_.at(id).push_back(normal);
  sources_.at(id).push_back(source);
}

//----------------------------------------------------------------------
inline std::vector<Vector3f> DownsamplerWithTargetAndNormal::Anchors() const
{
  std::vector<Vector3f> result;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (std::vector<std::vector<Vector3f> >::const_iterator it =
    anchors_.begin(); it != anchors_.end(); ++it)
  {
    Vector3f mean = NULL_VECTOR3F;

    for (std::vector<Vector3f>::const_iterator pIt = it->begin();
      pIt != it->end(); ++pIt)
    {
      mean += *pIt;
    }

    mean /= it->size();

    result.push_back(mean);
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
inline std::vector<Vector3f> DownsamplerWithTargetAndNormal::Targets() const
{
  std::vector<Vector3f> result;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (std::vector<std::vector<Vector3f> >::const_iterator it =
    targets_.begin(); it != targets_.end(); ++it)
  {
    Vector3f mean = NULL_VECTOR3F;

    for (std::vector<Vector3f>::const_iterator pIt = it->begin();
      pIt != it->end(); ++pIt)
    {
      mean += *pIt;
    }

    mean /= it->size();

    result.push_back(mean);
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
inline std::vector<Vector3f> DownsamplerWithTargetAndNormal::Normals() const
{
  std::vector<Vector3f> result;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (std::vector<std::vector<Vector3f> >::const_iterator it =
    normals_.begin(); it != normals_.end(); ++it)
  {
    Vector3f mean = NULL_VECTOR3F;

    for (std::vector<Vector3f>::const_iterator pIt = it->begin();
      pIt != it->end(); ++pIt)
    {
      mean += *pIt;
    }

    mean /= it->size();

    result.push_back(mean);
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

//----------------------------------------------------------------------
inline std::vector<Vector3f> DownsamplerWithTargetAndNormal::Sources() const
{
  std::vector<Vector3f> result;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  for (std::vector<std::vector<Vector3f> >::const_iterator it =
    sources_.begin(); it != sources_.end(); ++it)
  {
    Vector3f mean = NULL_VECTOR3F;

    for (std::vector<Vector3f>::const_iterator pIt = it->begin();
      pIt != it->end(); ++pIt)
    {
      mean += *pIt;
    }

    mean /= it->size();

    result.push_back(mean);
  }
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return result;
}

} //namespace X4

#endif
