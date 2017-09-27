#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "MeshStructure.h"
//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "VertexArray.h"
//---------------------------------------------------------------------------

namespace X4
{

MeshIndexTablesHack::MeshIndexTablesHack(UnstructuredInCoreTriangleMesh * mesh)
{
	m_Mesh = mesh;
	m_VertexArray = new VertexArray(mesh);
	m_TriangleArray = new VertexArray(mesh->getTriangles());

	// precompute tables
	m_Face = vector<TFaceInfo>(m_TriangleArray->getNumElements());
	m_Edge.reserve(m_Face.size());
	m_Vertex = vector<TVertexInfo>(mesh->getNumPoints());

	for( mpcard fi=0;fi<m_Face.size();fi++ )
	{
		TFaceInfo & face = m_Face[fi];
		Vector3i tri = m_TriangleArray->getIndex3i(fi);
		Vector3f faceNormal = normalize( (m_VertexArray->getPosition3f( tri[1] )-m_VertexArray->getPosition3f( tri[0] )).crossProduct(
			m_VertexArray->getPosition3f( tri[2] )-m_VertexArray->getPosition3f( tri[0] )));
		face.normal = faceNormal;
		for( mpcard j=0;j<3;j++ )
		{
			TVertexInfo & v = m_Vertex[tri[j]];
			v.faces.push_back(fi);
			
			mpcard edge_v0 = min(tri[j],tri[(j+1)%3]);
			mpcard edge_v1 = max(tri[j],tri[(j+1)%3]);

			if( edge_v0 == edge_v1 )
				throw PException("MeshIndexTablesHack() - invalid triangle edge found! (equal vertex indices)");
			
			// search for edge
			bool foundEdge = false;
			for( mpcard k=0;k<v.edges.size();k++ )
			{
				TEdgeInfo & e = m_Edge[v.edges[k]];
				if( e.v0 == edge_v0 && 
					e.v1 == edge_v1 )
				{
					e.faces.push_back(fi);
					e.normal = normalize(e.normal + faceNormal);
					face.edges.push_back(v.edges[k]);
					foundEdge = true;
					break;
				}
			}

			// create new edge if necessary
			if( !foundEdge )
			{
				TEdgeInfo e;
				e.v0 = edge_v0;
				e.v1 = edge_v1;
				e.faces.push_back(fi);
				e.normal = faceNormal;
				m_Vertex[e.v0].edges.push_back(m_Edge.size());
				m_Vertex[e.v1].edges.push_back(m_Edge.size());
				face.edges.push_back(m_Edge.size());
				m_Edge.push_back(e);
			}
		}

	}

	m_ProvidesPolyIndex = m_Mesh->getTriangles()->providesAttribute("polyIndex");
	if( m_ProvidesPolyIndex )
	{
		m_PolyIndexAAT = m_Mesh->getTriangles()->getAAT("polyIndex");
		mpcard numPolygons = 0;
		for( mpcard i=0;i<m_TriangleArray->getNumElements();i++ )
		{
			numPolygons = max(numPolygons,getFacePolygonID(i)+1);
		}
		m_Polygons = vector<TPolygonInfo>(m_TriangleArray->getNumElements());
		for( mpcard i=0;i<m_TriangleArray->getNumElements();i++ )
		{
			TPolygonInfo & p = m_Polygons[getFacePolygonID(i)];
			p.faces.push_back(i);
			Vector3f weightedFaceNormal = computeAreaNormal(i);
			if( p.faces.size() == 1)
				p.normal = weightedFaceNormal;
			else
				p.normal += weightedFaceNormal;
		}
		for( mpcard i=0;i<m_Polygons.size();i++ )
		{
			TPolygonInfo & p = m_Polygons[getFacePolygonID(i)];
			if(p.faces.empty())
				throw PException("inconsistent polygon ids!");
			p.normal = normalize(p.normal);
		}
	}
	else
	{
		m_Polygons = vector<TPolygonInfo>(m_TriangleArray->getNumElements());
		for( mpcard i=0;i<m_TriangleArray->getNumElements();i++ )
		{
			TPolygonInfo & p = m_Polygons[i];
			p.faces.push_back(i);
			p.normal = getFaceNormal(i);
		}
	}
}

MeshIndexTablesHack::~MeshIndexTablesHack(void)
{
	delete m_VertexArray;
	delete m_TriangleArray;
}


Vector3f X4::MeshIndexTablesHack::getVertexPosition(mpcard i ) const 
{
	return m_VertexArray->getPosition3f(i);
}

Vector3f X4::MeshIndexTablesHack::getVertexNormal(mpcard i) const 
{
	return m_VertexArray->getNormal3f(i);
}

mpcard MeshIndexTablesHack::getVertexNumEdges(mpcard vi) const 
{
	return m_Vertex[vi].edges.size();
}

mpcard MeshIndexTablesHack::getVertexEdge(mpcard vi, mpcard i) const 
{
	return m_Vertex[vi].edges[i];
}

vector<mpcard> MeshIndexTablesHack::getVertexFaces(mpcard vi) const 
{
	return m_Vertex[vi].faces;
}


vector<mpcard> X4::MeshIndexTablesHack::getVertexEdges(mpcard vi) const 
{
	return m_Vertex[vi].edges;
}

mpcard X4::MeshIndexTablesHack::getNumEdges() const 
{
	return m_Edge.size();
}

bool X4::MeshIndexTablesHack::isBorder( mpcard ei ) const 
{
	return m_Edge[ei].faces.size() == 1;
}

Vector3f X4::MeshIndexTablesHack::getEdgeNormal(mpcard ei) const 
{
	return m_Edge[ei].normal;
}

mpcard X4::MeshIndexTablesHack::GetEdgeVertex0( mpcard ei) const 
{
	return m_Edge[ei].v0;
}

mpcard X4::MeshIndexTablesHack::getEdgeVertex1( mpcard ei) const 
{
	return m_Edge[ei].v1;
}

mpcard X4::MeshIndexTablesHack::getEdgeNumFaces(mpcard ei) const 
{
	return m_Edge[ei].faces.size();
}

mpcard X4::MeshIndexTablesHack::getEdgeFace( mpcard ei, mpcard i ) const 
{
	return m_Edge[ei].faces[i];
}

mpcard X4::MeshIndexTablesHack::getNumFaces() const 
{
	return m_Face.size();
}

Vector3f X4::MeshIndexTablesHack::getFaceNormal(mpcard fi) const 
{
	return m_Face[fi].normal;
}


Vector3f MeshIndexTablesHack::getFaceCentroid(mpcard fi) const 
{
	const Vector3i tri = getFaceVertices(fi);
	return (getVertexPosition(tri[0])+getVertexPosition(tri[1])+getVertexPosition(tri[2]))/3.0f;
}


mpcard X4::MeshIndexTablesHack::getFaceVertex(mpcard fi, mpcard i) const 
{
	return m_TriangleArray->getIndex3i(fi)[i];
}

mpcard X4::MeshIndexTablesHack::getFaceEdge(mpcard fi, mpcard i) const 
{
	return m_Face[fi].edges[i];
}

Vector3i MeshIndexTablesHack::getFaceVertices(mpcard fi) const 
{
	return m_TriangleArray->getIndex3i(fi);
}

mpcard MeshIndexTablesHack::getVertexAdjacentVertex(mpcard vi, mpcard ei) const 
{
	const TEdgeInfo &e = m_Edge[ei];
	return ( vi == e.v0 )?e.v1:e.v0;
}

vector<mpcard> MeshIndexTablesHack::getFaceAdjacentFaces(mpcard fi) const 
{
	vector<mpcard> neighbors;
	
	for( mpcard i=0;i<3;i++ )
	{
		mpcard ei = getFaceEdge(fi,i);
		const mpcard numFaces = getEdgeNumFaces(ei);
		for( mpcard j=0;j<numFaces;j++ )
		{
			mpcard nfi = getEdgeFace(ei,j);
			if( nfi != fi )
				neighbors.push_back(nfi);
		}
	}
	return neighbors;
}


X4::mpcard X4::MeshIndexTablesHack::getFacePolygonID(mpcard fi) const
{
	if( m_ProvidesPolyIndex )
		return m_TriangleArray->get<int32,1>(fi,m_PolyIndexAAT)[0];
	else
		return fi;
}


X4::mpcard X4::MeshIndexTablesHack::getNumPolygons() const
{
	return m_Polygons.size();
}

X4::mpcard X4::MeshIndexTablesHack::getPolygonNumFaces( mpcard pi ) const
{
	return m_Polygons[pi].faces.size();
}

X4::mpcard X4::MeshIndexTablesHack::getPolygonFace( mpcard pi, mpcard i ) const
{
	return m_Polygons[pi].faces[i];
}

X4::Vector3f X4::MeshIndexTablesHack::getPolygonNormal( mpcard pi ) const
{
	return m_Polygons[pi].normal;
}

X4::Vector3f X4::MeshIndexTablesHack::computeAreaNormal( mpcard fi ) const
{
	Vector3i tri = getFaceVertices(fi);
	return (m_VertexArray->getPosition3f( tri[1] )-m_VertexArray->getPosition3f( tri[0] )).crossProduct(
		m_VertexArray->getPosition3f( tri[2] )-m_VertexArray->getPosition3f( tri[0] ));
}

} //namespace X4
