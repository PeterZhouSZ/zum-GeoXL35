#ifndef PCCSDTestCasesH
#define PCCSDTestCasesH
//---------------------------------------------------------------------------
#include "projectsXWu.h"
#include "CommonHdrXWu.h"
//---------------------------------------------------------------------------
#include "PCCommand.h"
//---------------------------------------------------------------------------

namespace NAMESPACE_VERSION
{
/**
	creates test cases for deformation project
*/
class PROJECTSXWU_API PCCSDTestCases : public PCCommand
{
	DEFINE_CLASS( PCCSDTestCases )

private:

	Vector3i	m_NumRepetitions;
	card32		m_Tesselation_NumBaseQuads;
	card32		m_Tesselation_NumBorderQuads;
	float		m_RotationAngle;


	// element of which we build the pattern (make sure element is within [0,1]?
	UnstructuredInCoreTriangleMesh * m_ElementMesh;

	UnstructuredInCoreTriangleMesh * getElementMesh() {return m_ElementMesh;}

	void constructCubeElementMesh();
	UnstructuredInCoreTriangleMesh * construct1DPattern(const Matrix4f &T);
	UnstructuredInCoreTriangleMesh * construct2DPattern(const Matrix4f &T0,const Matrix4f &T1);

public:
	PCCSDTestCases(void);
	~PCCSDTestCases(void);

   virtual void assign(const Object* obj, COPY_CONTEXT *context = nullptr);

   void createTranslation();
   void createRotation();
   void createHelix();
   void createTranslation2D();
   void createTransRot2D();
   void create3x3();
   void create5x5();

   virtual void execute(Scene *scene) {}

};

}

#endif