#ifndef TrimeshStatic_H
#define TrimeshStatic_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "SceneObject.h"
//---------------------------------------------------------------------------

class PROJECTSXWU_API TrimeshStatic //: public NAMESPACE_VERSION::SceneObject
{
public:
    typedef boost::shared_ptr< TrimeshStatic > Ptr;
    typedef boost::shared_ptr< const TrimeshStatic > ConstPtr;

public:
    struct TEdge;
    struct THalf
    {
        mpcard indx; // 0 or 1
        mpcard vsrc, vtgt;
        mpcard face;
        THalf* next;
        THalf* prev;
        TEdge* uplk;
    };

    struct TEdge
    {
        TEdge(size_t x) { indx = x; }
        ~TEdge();
        std::vector<THalf*> halfs;
        mpcard indx;
    };

    struct TVert
    {
        std::vector<THalf*> halfs;
        std::vector<THalf*> halfs_in;
        std::vector<mpcard> verts;
        std::vector<mpcard> edges;
        std::vector<mpcard> faces;
    };

    struct TFace
    {
        std::vector<THalf*> halfs;
        std::vector<mpcard> neighbors;
        Vector3f normal;
    };

    typedef std::set<int> VKey_;
    struct hash_value : std::unary_function< VKey_, boost::uint64_t > {
        boost::uint64_t operator() (VKey_ const& v) const {
            return boost::hash_range(v.begin(), v.end());
        }
    };
    typedef boost::unordered_map< VKey_, TEdge*, hash_value > EdgeTab;
    typedef boost::unordered_map< VKey_, mpcard, hash_value > FaceTab;

public:
	// ------------------------------------------
	// vertex interface
	// ------------------------------------------
	mpcard GetNumVerts(void) const;

    Vector3f GetVertPosition(mpcard vi) const;

    Vector3f GetVertNormal(mpcard vi) const;

    mpcard GetVertNumEdges(mpcard vi) const;

    std::vector<mpcard> GetVert1RingVerts(mpcard vi) const;
    std::vector<mpcard> GetVert1RingEdges(mpcard vi) const;
    std::vector<mpcard> GetVert1RingFaces(mpcard vi) const;

    std::vector<THalf*> GetVertHalfs(mpcard vi) const;

    THalf* GetVertHalf(mpcard vi, mpcard ei) const;

    std::vector<mpcard> GetVertFaces(mpcard vi) const;

	// ------------------------------------------
	// edge interface
	// ------------------------------------------
    mpcard GetNumEdges(void) const;

    bool IsEdgeBorder(mpcard ei) const;

    bool IsEdgeRegular(mpcard ei) const;

    bool IsEdgeMultiple(mpcard ei) const;

    bool IsEdgeCrease(mpcard ei) const;

    mpcard GetEdgeVert0(mpcard ei) const;

    mpcard GetEdgeVert1(mpcard ei) const;

    mpcard GetEdge2ndVert(mpcard ei, mpcard vi) const;

    mpcard GetEdgeNumFaces(mpcard ei) const;

    mpcard GetEdgeFace(mpcard ei, mpcard fi) const;

    mpcard GetEdgeNextFace(mpcard ei, mpcard fi) const;

	// ------------------------------------------
	// face interface
	// ------------------------------------------
    mpcard GetNumFaces(void) const;

    Vector3f GetFaceNormal(mpcard fi) const;

    Vector3f GetFaceCentroid(mpcard fi) const;

    mpcard GetFaceVert(mpcard fi, mpcard vi) const;

    mpcard GetFace3rdVert(mpcard fi, mpcard v0, mpcard v1) const;

    THalf* GetFaceHalf(mpcard fi, mpcard ei) const;

    Vector3i GetFaceVertices(mpcard fi) const;

    std::vector<mpcard> GetFaceAdjacentFaces(mpcard fi) const;

public:
    TrimeshStatic(const UnstructuredInCoreTriangleMesh* mesh,
                  bool const process = true);
    ~TrimeshStatic(void);

    UnstructuredInCoreTriangleMeshPtr GetMesh(void) const { return mMesh; }
    VertexArrayPtr GetVertArray(void) const { return mVertArray; }
    VertexArrayPtr GetFaceArray(void) const { return mFaceArray; }
    const std::vector<TEdge*>& GetEdges(void) const { return mEdges; }
    const std::vector<TVert>& GetVerts(void) const { return mVerts; }
    const std::vector<TFace>& GetFaces(void) const { return mFaces; }

private:
    void cleanAndSetupMesh(const UnstructuredInCoreTriangleMesh* mesh,
                           bool const process = true);
    void sortVertNeighbors(void);
    void retrieveFaceNeighbors(void);

    UnstructuredInCoreTriangleMeshPtr mMesh;
    VertexArrayPtr mVertArray;
    VertexArrayPtr mFaceArray;
    std::vector<TEdge*> mEdges;
    std::vector<TVert> mVerts;
    std::vector<TFace> mFaces;
    EdgeTab mVP2E;
};

typedef TrimeshStatic::THalf TSHalf;
typedef std::vector<TSHalf*> TSHalfV;
typedef TrimeshStatic::TEdge TSEdge;
typedef std::vector<TSEdge*> TSEdgeV;
typedef TrimeshStatic::TVert TSVert;
typedef std::vector<TSVert> TSVertV;
typedef TrimeshStatic::TFace TSFace;
typedef std::vector<TSFace> TSFaceV;

#endif
