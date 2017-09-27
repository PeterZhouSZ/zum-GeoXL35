#ifndef DampingICLH
#define DampingICLH
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "LineFeature.h"
//---------------------------------------------------------------------------

/**
refine alignment based on iterative energy minimization of point-to-line constraints
*/
class DampingICL
{
public:
    DampingICL(void);
    ~DampingICL(void);

    void begin(LineFeatureSet * lfs, unsigned * lfs_flags = nullptr);
    void end();

    /// aligns a subset of feature lines to a feature set
    /// returns true if converged
    bool align( const std::vector<LineFeature*>& subset, Matrix4f & relativeTransformation, float & score );

    /// search corresponding line for l (index returned)
    bool findCorrespondingLine( LineFeature * l, const Matrix4f &relativeTransformation, unsigned * index=nullptr );
    bool findCorrespondingLineBIO( LineFeature * l, const Matrix4f &relativeTransformation, unsigned * index=nullptr );

public:
    typedef void (*RedrawFunctionPtr)();
    void setRedrawFunction( RedrawFunctionPtr rf ) {m_RedrawFunction = rf;}

    // member interface
    bool visualization() const {return m_Visualize;}
    void setVisualization(bool v) {m_Visualize = v;}
    void setMaxNumIterations(card32 n) {m_MaxNumIterations = n;}
    void setVisualization_MinScore(float32 v) {m_Visualization_MinScore=v;}
    card32 getMaxNumIterations() const {return m_MaxNumIterations;}
    bool getVisualizeInWorldSpace() const {return m_VisualizeInWorldSpace;}
    void setLineDistanceSigma(float s) {m_LineDistanceSigma = s;}

private:
    float32	m_AngleSigma;			// angle difference distribution of two corresponding line features
    float32	m_LineDistanceSigma; 
    float32	m_CurvatureSigma;
    float32	m_NormalAngleSigma;
    bool		m_Visualize;
    bool		m_VisualizeInWorldSpace;

    card32	m_MaxNumIterations;
    float32	m_Visualization_MinScore;



    // temporal members	
    HierarchicalKNNIterator * m_hIt;
    LineFeatureSet * m_LineFeatureSet;
    unsigned * m_LineFeatureSet_Flags;

private:
	RedrawFunctionPtr m_RedrawFunction;
	typedef float OptFloatType;
	typedef StaticVector<OptFloatType,3> OptVec3;
	typedef StaticVector<OptFloatType,6> OptVec6;
	typedef StaticMatrix<OptFloatType,3,3> OptMat3;
	typedef StaticMatrix<OptFloatType,6,6> OptMat6;
};

#endif
