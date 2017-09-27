#ifndef MeshStructure_h_
#define MeshStructure_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "UnstructuredInCoreTriangleMesh.h"
//----------------------------------------------------------------------

namespace X4
{
//======================================================================
// Forward declaration
//----------------------------------------------------------------------
class VertexArray;

//======================================================================
// class MeshIndexTablesHack
//----------------------------------------------------------------------
class PROJECTSXWU_API MeshIndexTablesHack
{
public:
	MeshIndexTablesHack(UnstructuredInCoreTriangleMesh * mesh);
	~MeshIndexTablesHack(void);

	// ------------------------------------------
	// vertex interface
	// ------------------------------------------
	mpcard getNumVertices() const {return m_Mesh->getNumPoints();}
	Vector3f getVertexPosition(mpcard i ) const;
	Vector3f getVertexNormal(mpcard i) const;
	/// returns number of edges sharing vertex vi
	mpcard getVertexNumEdges(mpcard vi) const;	/// return i-th edge sharing vertex vi
	mpcard getVertexEdge(mpcard vi, mpcard i) const;
	/// get all edges with vertex vi
	vector<mpcard> getVertexEdges(mpcard vi) const;
	/// get all faces sharing vertex vi
	vector<mpcard> getVertexFaces(mpcard vi) const;
	/// get adjacent vertex index connected to vi via edge ei
	mpcard getVertexAdjacentVertex(mpcard vi, mpcard ei) const;

	// ------------------------------------------
	// edge interface
	// ------------------------------------------
	mpcard getNumEdges() const;
	/// returns true if edge is a border (belongs to only one face)
	bool isBorder( mpcard ei ) const;
	/// returns average normal between adjacent faces
	Vector3f getEdgeNormal(mpcard ei) const;
	/// returns first vertex of edge ei (smaller vertex id)
	mpcard GetEdgeVertex0( mpcard ei) const;	
	/// returns second vertex of edge ei (larger vertex id)
	mpcard getEdgeVertex1( mpcard ei) const;	
	/// returns (vertex0 + vertex1)/2
	Vector3f getEdgeCenter(mpcard ei) const;
	/// returns cosinus of angle between adjacent face normals (1 in case of border edge)
	float getEdgeCosAngle(mpcard ei) const;
	/// returns number of faces (bad meshes usually have those cases with more than 2 faces)
	mpcard getEdgeNumFaces(mpcard ei) const;
	/// return i-th face index of edge ei
	mpcard getEdgeFace( mpcard ei, mpcard i ) const;

	// ------------------------------------------
	// face interface
	// ------------------------------------------
	mpcard getNumFaces() const;
	Vector3f getFaceNormal(mpcard fi) const;
	/// compute normal weighted by triangle area
	Vector3f computeAreaNormal(mpcard fi) const;
	Vector3f getFaceCentroid(mpcard fi) const;
	/// return vertex index of i-th vertex in triangle fi, 0<=i<3
	mpcard getFaceVertex(mpcard fi, mpcard i) const;
	/// return triangle as Vector3i
	Vector3i getFaceVertices(mpcard fi) const;
	/// return edge index of i-th edge in triangle fi, 0<=i<3
	mpcard getFaceEdge(mpcard fi, mpcard i) const;
	/// return directly adjacent faces (meaning sharing an edge)
	vector<mpcard> getFaceAdjacentFaces(mpcard fi) const;
	/// returns corresponding polygon id (will be identical to fi if no id is present)
    mpcard getFacePolygonID(mpcard fi) const;

	// ------------------------------------------------
	// polygon interface (might be equivalent to faces)
	// ------------------------------------------------
	mpcard getNumPolygons() const;
	mpcard getPolygonNumFaces(mpcard pi) const;
	mpcard getPolygonFace(mpcard pi, mpcard i) const;
	Vector3f getPolygonNormal(mpcard pi) const;


private:

	UnstructuredInCoreTriangleMesh	*	m_Mesh;
	VertexArray						*	m_VertexArray;
	VertexArray						*	m_TriangleArray;
	AAT									m_PolyIndexAAT;
	bool								m_ProvidesPolyIndex;

	struct TEdgeInfo
	{
		mpcard v0,v1;	// vertex indices, v0 < v1
		vector<mpcard> faces;
		Vector3f	normal;
		float cos_normal_angle;	// cosinus of angle between face normals (1 for borders)
	};
	vector<TEdgeInfo>	m_Edge;

	struct TVertexInfo
	{
		vector<mpcard> edges;
		vector<mpcard> faces;
	};
	vector<TVertexInfo>	m_Vertex;

	struct TFaceInfo
	{
		vector<mpcard> edges;
		Vector3f	normal;
	};
	vector<TFaceInfo>	m_Face;

	struct TPolygonInfo
	{
		vector<mpcard> faces;
		Vector3f normal;
	};
	vector<TPolygonInfo> m_Polygons;
};

} //namespace X4

#endif MeshStructure_h_
