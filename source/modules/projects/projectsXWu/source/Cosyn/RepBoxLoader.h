#ifndef RepBoxLoader_H
#define RepBoxLoader_H
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
#include "Util\SurfaceGrid.h"
//---------------------------------------------------------------------------
#include "Eigen/Eigen"
//---------------------------------------------------------------------------

struct RepBox
{
public:
    typedef boost::shared_ptr< RepBox > Ptr;
    typedef boost::shared_ptr< const RepBox > ConstPtr;
public:
    Matrix3f basis;
    Vector3f coeffs;
    Vector3f centroid;
    Vector3f cenref;
    float32 radiusref;
    int32 label;
    RepBox() {
        basis[0] = makeVector3f(1, 0, 0);
        basis[1] = makeVector3f(0, 1, 0);
        basis[2] = makeVector3f(0, 0, 1);
        coeffs = makeVector3f(0, 0, 0);
        centroid = makeVector3f(0, 0, 0);
        cenref = makeVector3f(0, 0, 0);
        radiusref = 0;
        label = -1;
    }
    RepBox(const RepBox& rhs) {
        basis = rhs.basis;
        coeffs = rhs.coeffs;
        centroid = rhs.centroid;
        cenref = rhs.cenref;
        radiusref = rhs.radiusref;
        label = rhs.label;
    }
};

class RepBoxLoader
{
public:
    typedef boost::shared_ptr< RepBoxLoader > Ptr;
    typedef boost::shared_ptr< const RepBoxLoader > ConstPtr;

public:
    RepBoxLoader(void);
    ~RepBoxLoader(void) {}

    void SetPC(UICPC* pc);

    void LoadGT(void);
    void SaveGT(void);

    void LoadSymmetries(void);

    static void glDrawBox(RepBox::Ptr box, const Vector3f& color, const float32& linewidth);
    static void DrawBox(RepBox::Ptr box, const Vector3f& color, const float32& linewidth);
    static void DrawBoxes(
        const std::deque<RepBox::Ptr>& boxList,
        const Vector3f& color, const float32& linewidth = 4
        );
    static void DrawBoxes(const std::deque< std::deque<RepBox::Ptr> >& boxList,
        const float32& linewidth = 4);
    void DrawBoxes(const float32& linewidth = 4);
    Vector3f MapGroupColor(const unsigned& groupID);

    std::deque<RepBox::Ptr> GetClassList(const unsigned& num);
    void GetTemplateList(std::deque<RepBox::Ptr>* pTempList);
    void GetRepBoxList(std::deque<RepBox::Ptr>* pBoxList);

public:
    UICPC* data_;
    std::deque< std::deque<RepBox::Ptr> > boxRecord;
    std::deque< Eigen::Matrix4f > symmetries_;
};

#endif
