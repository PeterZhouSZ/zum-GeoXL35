#ifndef CommonHdrXWu_H
#define CommonHdrXWu_H

#define NAMESPACE_VERSION X4
using namespace NAMESPACE_VERSION;
#define DEFINE_CLASS X4_CLASS
#define DEFINE_ABSTRACT_CLASS X4_ABSTRACT_CLASS
#define IMPLEMENT_CLASS IMPLEMENT_X4_CLASS
#define IMPLEMENT_ABSTRACT_CLASS IMPLEMENT_X4_ABSTRACT_CLASS
#define COPY_CONTEXT X4CopyContext
#define pAssert x4Assert
#define PException X4Exception

//---------------------------------------------------------------------------
// Boost headers
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/array.hpp>
#include <boost/heap/priority_queue.hpp>
//#include <array>
//#include <unordered_map>
//#include <unordered_set>
#include <boost/unordered_set.hpp>
#include <boost/unordered_map.hpp>
#include <boost/bimap.hpp>
#include <boost/cstdint.hpp>
#include <boost/utility.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/format.hpp>
//using namespace boost;

//---------------------------------------------------------------------------
// NAMESPACE_VERSION headers
#include "Camera.h"
#include "FastSphereQuerry.h"
#include "HierarchicalKNNIterator.h"
#include "Modifiers.h"
#include "NumericalClassProperty.h"
#include "PointSetANNQuery.h"
#include "PointCloud.h"
#include "PointCloudTools.h"
#include "PCCommand.h"
#include "Random.h"
#include "UnstructuredInCoreTriangleMesh.h"
#include "UnstructuredInCorePointCloud.h"
#include "Vertexarray.h"
#include "VertexDescriptor.h"
#include "ViewFrustum.h"

//---------------------------------------------------------------------------
// boost::shared_ptr wrapper for fundamental classes
#define SHARED_PTR_WRAPPER( cls_name ) \
    typedef boost::shared_ptr< cls_name > cls_name##Ptr; \
    typedef boost::shared_ptr< const cls_name > cls_name##ConstPtr;

SHARED_PTR_WRAPPER( PointCloud );
SHARED_PTR_WRAPPER( UnstructuredInCoreTriangleMesh );
typedef UnstructuredInCoreTriangleMesh UICTM;
typedef UnstructuredInCoreTriangleMeshPtr UICTMPtr;
SHARED_PTR_WRAPPER( UnstructuredInCorePointCloud );
typedef UnstructuredInCorePointCloud UICPC;
typedef UnstructuredInCorePointCloudPtr UICPCPtr;
SHARED_PTR_WRAPPER( VertexArray );
SHARED_PTR_WRAPPER( VertexDescriptor );
SHARED_PTR_WRAPPER( HierarchicalKNNIterator );
SHARED_PTR_WRAPPER( PointSetANNQuery );
SHARED_PTR_WRAPPER( FastSphereQuerry );

//---------------------------------------------------------------------------
// useful headers
#include "Util/TAssert.h"
#include "Util/PrefixOutput.h"

//---------------------------------------------------------------------------
// useful things
typedef boost::array<Vector3f, 2> array2vec3f;
typedef boost::array<Vector3f, 3> array3vec3f;

//---------------------------------------------------------------------------
// Eigen
//#include "Eigen/Eigen"

//---------------------------------------------------------------------------
#include "projectsXWu.h"

#endif
