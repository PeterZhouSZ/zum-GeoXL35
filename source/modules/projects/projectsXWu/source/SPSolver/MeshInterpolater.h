//----------------------------------------------------------------------
#ifndef MeshInterpolater_h_
#define MeshInterpolater_h_
//----------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//----------------------------------------------------------------------
#include "Util\numerical\CudaAdaptor.h"
//----------------------------------------------------------------------

class MeshInterpolater
{
public:
    typedef boost::shared_ptr< MeshInterpolater > Ptr;
    typedef boost::shared_ptr< const MeshInterpolater > ConstPtr;

public:
    virtual ~MeshInterpolater(void) {}
    virtual void Update(const DVectorD& delta) = 0;
    virtual void Interpolate(UICTM* resultTM) = 0;
};

class MeshInterpolaterN2M : public MeshInterpolater
{
public:
    typedef boost::shared_ptr< MeshInterpolaterN2M > Ptr;
    typedef boost::shared_ptr< const MeshInterpolaterN2M > ConstPtr;

public:
    explicit MeshInterpolaterN2M(const UICPC* samplePC, const UICTM* inputTM,
        const SparseMatrixD& nullSpace, const SparseMatrixD& liftWeight
        );
    virtual void Update(const DVectorD& delta);
    virtual void Interpolate(UICTM* resultTM);

private:
    //const Vector3f evaluate(const unsigned& vi, const bool& debug_output = false) const;

private:
    DVectorD nsDelta;
    DVectorD meshDelta;

    SparseMatrixD liftWeightProj;
#ifdef USE_CUDA
    CudaSparseCSR<double> csrInterWeight;
#endif
};

class MeshInterpolaterS2M : public MeshInterpolater
{
public:
    typedef boost::shared_ptr< MeshInterpolaterS2M > Ptr;
    typedef boost::shared_ptr< const MeshInterpolaterS2M > ConstPtr;

public:
    explicit MeshInterpolaterS2M(UICPC* samplePC, const UICTM* inputTM,
        const SparseMatrixD& liftWeight
        );
    virtual void Update(const DVectorD& delta);
    virtual void Interpolate(UICTM* resultTM);

private:
    DVectorD sampleDelta;
    DVectorD meshDelta;

    SparseMatrixD liftWeightProj;
#ifdef USE_CUDA
    CudaSparseCSR<double> csrInterWeight;
#endif
};

#endif
