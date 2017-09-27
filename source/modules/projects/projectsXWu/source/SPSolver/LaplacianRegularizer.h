//----------------------------------------------------------------------
#ifndef LaplacianRegularizer_h_
#define LaplacianRegularizer_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Util/TrimeshStatic.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

class LaplacianRegularizer
{
public:
    typedef boost::shared_ptr< LaplacianRegularizer > Ptr;
    typedef boost::shared_ptr< const LaplacianRegularizer > ConstPtr;

public:
    LaplacianRegularizer(const float& weight, const bool& inertia, const unsigned& D);
    virtual ~LaplacianRegularizer(void) {}
    const SparseMatrixD& getQT(void) const { return QT; }
    const DVectorD& getLT(void) const { return LT; }
    const size_t getDim(void) const { return QT.getNumRows(); }
    const std::deque< std::deque< unsigned> >& getTpy(void) const { return samTpy; }

    virtual void Update(
        const std::vector<Vector3f>& latestPositions,
        const std::vector<Vector3f>& currentPositions
        ) = 0;

protected:
    float regularizerWeight_;
    bool bUseInertia_;
    const unsigned Dim;

    SparseMatrixD QT;
    DVectorD LT;
    std::vector<Matrix3f> per_point_rotation_;
    //std::deque< std::vector< float > > cotan_weight_;
    std::deque< std::deque< unsigned> > samTpy;
};

class LaplacianRegularizerSS : public LaplacianRegularizer
{
public:
    typedef boost::shared_ptr< LaplacianRegularizerSS > Ptr;
    typedef boost::shared_ptr< const LaplacianRegularizerSS > ConstPtr;

public:
    LaplacianRegularizerSS(
        const UICPC* samplePC, const UICTM* inputTM,
        const SparseMatrixD& liftWeight,
        const float& weight = 1.0f,
        const bool& inertia = false,
        const unsigned& D = 3
        );

    const SparseMatrixD& getQT(void) const { return QT; }
    const DVectorD& getLT(void) const { return LT; }
    const size_t getDim(void) const { return QT.getNumRows(); }

    void Update(
        const std::vector<Vector3f>& latestPositions,
        const std::vector<Vector3f>& currentPositions
        );

private:
    void BuildRegularizerDelta(
        const unsigned& num_points, const TrimeshStatic& tsMesh,
        const SparseMatrixD& liftWeight)
        ;

    void ComputePerPointRotations(
        const std::vector<Vector3f>& latestPositions,
        const std::vector<Vector3f>& currentPositions
        );

    void UpdateRotation(
        const std::vector<Vector3f>& currentPositions
        );
};

class LaplacianRegularizerMM : public LaplacianRegularizer
{
public:
    typedef boost::shared_ptr< LaplacianRegularizerMM > Ptr;
    typedef boost::shared_ptr< const LaplacianRegularizerMM > ConstPtr;

public:
    LaplacianRegularizerMM(
        const UICPC* samplePC, const UICTM* inputTM,
        const SparseMatrixD& liftWeight, const SparseMatrixD& liftWeight3,
        const float& weight = 1.0f,
        const bool& inertia = false,
        const unsigned& D = 3
        );

    void Update(
        const std::vector<Vector3f>& latestPositions,
        const std::vector<Vector3f>& currentPositions
        );

private:
    void BuildRegularizerDelta(
        const unsigned& num_points, const TrimeshStatic& tsMesh,
        const SparseMatrixD& liftWeight, const SparseMatrixD& liftWeight3
        );

    void ComputePerPointRotations(
        const std::vector<Vector3f>& latestPositions,
        const std::vector<Vector3f>& currentPositions
        );

    void UpdateRotation(
        const std::vector<Vector3f>& currentPositions
        );
};

#endif
