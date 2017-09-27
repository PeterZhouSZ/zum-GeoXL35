#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "PCC/PCCSDTestCases.h"
//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "ClassMethods.h"
#include "VertexArray.h"
//---------------------------------------------------------------------------
IMPLEMENT_CLASS( PCCSDTestCases ,1)
{
	BEGIN_CLASS_INIT( PCCSDTestCases );
	ADD_VECTOR3I_PROP(m_NumRepetitions,0);
	ADD_FLOAT32_PROP(m_RotationAngle,0);
	ADD_CARD32_PROP_UPDATE(m_Tesselation_NumBaseQuads,1,&constructCubeElementMesh);
	ADD_CARD32_PROP_UPDATE(m_Tesselation_NumBorderQuads,1,&constructCubeElementMesh);

	ADD_NOARGS_METHOD(createTranslation);
	ADD_NOARGS_METHOD(createRotation);
	ADD_NOARGS_METHOD(createHelix);
	ADD_NOARGS_METHOD(createTranslation2D);
	ADD_NOARGS_METHOD(createTransRot2D);
    ADD_NOARGS_METHOD(create3x3);
    ADD_NOARGS_METHOD(create5x5);
}
//---------------------------------------------------------------------------

PCCSDTestCases::PCCSDTestCases(void)
{
	m_NumRepetitions = makeVector3i(10,4,4);
	m_RotationAngle = 90.0f;
	m_ElementMesh = nullptr;
	m_Tesselation_NumBaseQuads = 3;
	m_Tesselation_NumBorderQuads = 1;
	constructCubeElementMesh();
}

PCCSDTestCases::~PCCSDTestCases(void)
{
	delete m_ElementMesh;
}

void PCCSDTestCases::assign(const Object* obj, COPY_CONTEXT *context)
{
	PCCommand::assign(obj,context);
	const PCCommand * o = dynamic_cast<const PCCommand*>(obj);
	pAssert( o != nullptr );

	copyObjectProperties(obj,this);
}

void PCCSDTestCases::createTranslation()
{
	Matrix4f genT = makeTranslation4f(1,0,0);	
	addPointCloud(getScene(),construct1DPattern(genT),"testTranslation");
}

void PCCSDTestCases::createRotation()
{
	Matrix4f genT = makeTranslation4f(1,0,0)*makeRotY4f(m_RotationAngle/180.0f*M_PI/(m_NumRepetitions[0]));	
	addPointCloud(getScene(),construct1DPattern(genT),"testRotation");
}

void PCCSDTestCases::createTranslation2D()
{
	Matrix4f genT0 = makeTranslation4f(1,0,0);	
	Matrix4f genT1 = makeTranslation4f(0,1,0);	
	addPointCloud(getScene(),construct2DPattern(genT0,genT1),"testTranslation2D");
}

void PCCSDTestCases::constructCubeElementMesh()
{
	if( m_Tesselation_NumBaseQuads > 40)
		return;

	mpcard numBaseQuads = m_Tesselation_NumBaseQuads;
	mpcard numBorderQuads = m_Tesselation_NumBorderQuads;
	mpcard numCubeQuads = numBaseQuads-2*numBorderQuads;
	std::vector<Vector3f>	points;
	std::vector<Vector3f>	normals;
	std::vector<Vector3i>	indices;
	for( mpcard x=0;x<numBaseQuads;x++ )
	{
		for( mpcard y=0;y<numBaseQuads;y++ )
		{
			if( x<numBorderQuads || x>numBaseQuads-1-numBorderQuads || 
				y<numBorderQuads || y>numBaseQuads-1-numBorderQuads)
			{
				int index0 = (int)points.size();
				points.push_back(makeVector3f(x,y,0)/(numBaseQuads));
				points.push_back(makeVector3f(x+1,y,0)/(numBaseQuads));
				points.push_back(makeVector3f(x ,y+1,0)/(numBaseQuads));
				points.push_back(makeVector3f(x+1,y+1,0)/(numBaseQuads));
				normals.push_back(makeVector3f(0,0,1));
				normals.push_back(makeVector3f(0,0,1));
				normals.push_back(makeVector3f(0,0,1));
				normals.push_back(makeVector3f(0,0,1));
				indices.push_back(makeVector3i(index0,index0+1,index0+2));
				indices.push_back(makeVector3i(index0+2,index0+1,index0+3));
			}
		}
	}

	Vector3f corner_x_front = makeVector3f(numBorderQuads,numBorderQuads,0) / numBaseQuads;
	Vector3f corner_y_front = makeVector3f(numBorderQuads,numBorderQuads,0) / numBaseQuads;
	Vector3f corner_x_back = makeVector3f(numBaseQuads-numBorderQuads,numBorderQuads,0) / numBaseQuads;
	Vector3f corner_y_back = makeVector3f(numBorderQuads,numBaseQuads-numBorderQuads,0) / numBaseQuads;
	Vector3f corner_top = makeVector3f(numBorderQuads,numBorderQuads,numCubeQuads) / numBaseQuads;
	for( mpcard x=0;x<numCubeQuads;x++ )
	{
		for( mpcard y=0;y<numCubeQuads;y++ )
		{
			// x front
			int index0 = (int)points.size();
			points.push_back(corner_x_front + makeVector3f(0,x,y)/(numBaseQuads));	
			points.push_back(corner_x_front + makeVector3f(0,x+1,y)/(numBaseQuads));	
			points.push_back(corner_x_front + makeVector3f(0,x,y+1)/(numBaseQuads));	
			points.push_back(corner_x_front + makeVector3f(0,x+1,y+1)/(numBaseQuads));	
			normals.push_back(makeVector3f(-1,0,0));
			normals.push_back(makeVector3f(-1,0,0));
			normals.push_back(makeVector3f(-1,0,0));
			normals.push_back(makeVector3f(-1,0,0));
			indices.push_back(makeVector3i(index0+2,index0+1,index0));
			indices.push_back(makeVector3i(index0+3,index0+1,index0+2));

			// x back
			index0 = (int)points.size();
			points.push_back(corner_x_back + makeVector3f(0,x,y)/(numBaseQuads));	
			points.push_back(corner_x_back + makeVector3f(0,x+1,y)/(numBaseQuads));	
			points.push_back(corner_x_back + makeVector3f(0,x,y+1)/(numBaseQuads));	
			points.push_back(corner_x_back + makeVector3f(0,x+1,y+1)/(numBaseQuads));	
			normals.push_back(makeVector3f(1,0,0));
			normals.push_back(makeVector3f(1,0,0));
			normals.push_back(makeVector3f(1,0,0));
			normals.push_back(makeVector3f(1,0,0));
			indices.push_back(makeVector3i(index0+0,index0+1,index0+2));
			indices.push_back(makeVector3i(index0+2,index0+1,index0+3));

			// y front
			index0 = (int)points.size();
			points.push_back(corner_y_front + makeVector3f(x,0,y)/(numBaseQuads));	
			points.push_back(corner_y_front + makeVector3f(x+1,0,y)/(numBaseQuads));	
			points.push_back(corner_y_front + makeVector3f(x,0,y+1)/(numBaseQuads));	
			points.push_back(corner_y_front + makeVector3f(x+1,0,y+1)/(numBaseQuads));	
			normals.push_back(makeVector3f(0,-1,0));
			normals.push_back(makeVector3f(0,-1,0));
			normals.push_back(makeVector3f(0,-1,0));
			normals.push_back(makeVector3f(0,-1,0));
			indices.push_back(makeVector3i(index0+0,index0+1,index0+2));
			indices.push_back(makeVector3i(index0+2,index0+1,index0+3));

			// y back
			index0 = (int)points.size();
			points.push_back(corner_y_back + makeVector3f(x,0,y)/(numBaseQuads));	
			points.push_back(corner_y_back + makeVector3f(x+1,0,y)/(numBaseQuads));	
			points.push_back(corner_y_back + makeVector3f(x,0,y+1)/(numBaseQuads));	
			points.push_back(corner_y_back + makeVector3f(x+1,0,y+1)/(numBaseQuads));	
			normals.push_back(makeVector3f(0,1,0));
			normals.push_back(makeVector3f(0,1,0));
			normals.push_back(makeVector3f(0,1,0));
			normals.push_back(makeVector3f(0,1,0));
			indices.push_back(makeVector3i(index0+2,index0+1,index0));
			indices.push_back(makeVector3i(index0+3,index0+1,index0+2));

			// top
			index0 = (int)points.size();
			points.push_back(corner_top + makeVector3f(x,y,0)/(numBaseQuads));	
			points.push_back(corner_top + makeVector3f(x+1,y,0)/(numBaseQuads));	
			points.push_back(corner_top + makeVector3f(x,y+1,0)/(numBaseQuads));	
			points.push_back(corner_top + makeVector3f(x+1,y+1,0)/(numBaseQuads));	
			normals.push_back(makeVector3f(0,0,1));
			normals.push_back(makeVector3f(0,0,1));
			normals.push_back(makeVector3f(0,0,1));
			normals.push_back(makeVector3f(0,0,1));
			indices.push_back(makeVector3i(index0+0,index0+1,index0+2));
			indices.push_back(makeVector3i(index0+2,index0+1,index0+3));

		}
	}

	// copy into mesh
	delete m_ElementMesh;
	m_ElementMesh = new UnstructuredInCoreTriangleMesh;
	m_ElementMesh->clearAndSetupDefault(points.size(),indices.size());
	VertexArray va_points(m_ElementMesh);
	VertexArray va_mesh(m_ElementMesh->getTriangles());
	for( mpcard i=0;i<points.size();i++ )
	{
		va_points.setPosition3f(i,points[i]);
		va_points.setNormal3f(i,normals[i]);
	}

	for( mpcard i=0;i<indices.size();i++)
	{
		va_mesh.setIndex3i(i,indices[i]);
	}

	m_ElementMesh->closeMesh(true);
	m_ElementMesh->clearGarbage();
}

UnstructuredInCoreTriangleMesh * PCCSDTestCases::construct1DPattern( const Matrix4f &T )
{
	UnstructuredInCoreTriangleMesh * elMesho = getElementMesh();
	UnstructuredInCoreTriangleMesh * elResulto = new UnstructuredInCoreTriangleMesh;
	elResulto->clearAndSetup(elMesho->getDescr(),0,elMesho->getTriangleDescr(),0);
	elResulto->setMaterialIndex(3);

	// ------------------------------
	// generate mesh
	Matrix4f currentT = IDENTITY4F;
	std::vector<Matrix4f> frames;
	for( mpcard i=0;i<m_NumRepetitions[0];i++ )
	{
		elResulto->joinOtherMesh(elMesho,currentT);
		frames.push_back(currentT);
		currentT = T * currentT;
	}

	return elResulto;
}

UnstructuredInCoreTriangleMesh * PCCSDTestCases::construct2DPattern( const Matrix4f &T0, const Matrix4f &T1 )
{
	UnstructuredInCoreTriangleMesh * elMesho = getElementMesh();
	UnstructuredInCoreTriangleMesh * elResulto = new UnstructuredInCoreTriangleMesh;
	elResulto->clearAndSetup(elMesho->getDescr(),0,elMesho->getTriangleDescr(),0);
	elResulto->setMaterialIndex(3);

	// ------------------------------
	// generate mesh
	Matrix4f currentT0 = IDENTITY4F;
	for( mpcard i=0;i<m_NumRepetitions[0];i++ )
	{
		Matrix4f currentT = currentT0;
		for( mpcard j=0;j<m_NumRepetitions[1];j++ )
		{
			elResulto->joinOtherMesh(elMesho,currentT);
			currentT = T1 * currentT;
		}
		currentT0 = T0 * currentT0;
	}

	return elResulto;
}

void PCCSDTestCases::createTransRot2D()
{
	Matrix4f genT1 = makeTranslation4f(1,0,0);	
	Matrix4f genT0 = makeTranslation4f(0,1,0)*makeRotX4f(m_RotationAngle/180.0f*M_PI/(m_NumRepetitions[0]));	
	addPointCloud(getScene(),construct2DPattern(genT0,genT1),"testTransRot2D");
}

void PCCSDTestCases::createHelix() 
{
	Matrix4f genT = makeTranslation4f(1,0.5f,0)*makeRotY4f(m_RotationAngle/180.0f*M_PI/(m_NumRepetitions[0]));	
	addPointCloud(getScene(),construct1DPattern(genT),"testHelix");
}

void PCCSDTestCases::create3x3()
{
    std::vector<Vector3f>	points;
    std::vector<Vector3f>	normals;
    std::vector<Vector3i>	indices;

    // y
    // | 6-7-8
    // | |\|\|
    // | 3-4-5
    // | |\|\|
    // | 0-1-2
    // -------- x
    for (mpcard yi = 0; yi < 3; ++yi) {
        for (mpcard xi = 0; xi < 3; ++xi) {
            points.push_back(makeVector3f(xi, yi, 0));
        }
    }

    for (mpcard ii = 0; ii < 9; ++ii) {
        normals.push_back(makeVector3f(0, 0, 1));
    }
    normals[5] *= -1.f; // flip edge test

    indices.push_back(makeVector3i(0, 1, 3));
    indices.push_back(makeVector3i(3, 1, 4));
    indices.push_back(makeVector3i(1, 2, 4));
    indices.push_back(makeVector3i(5, 2, 4)); // flip edge test
    //indices.push_back(makeVector3i(4, 2, 5));
    indices.push_back(makeVector3i(3, 4, 6));
    indices.push_back(makeVector3i(6, 4, 7));
    indices.push_back(makeVector3i(4, 5, 7));
    indices.push_back(makeVector3i(7, 5, 8));

    // copy into mesh
    //UnstructuredInCoreTriangleMeshPtr mesh (new UnstructuredInCoreTriangleMesh);
    UnstructuredInCoreTriangleMesh* mesh = new UnstructuredInCoreTriangleMesh;
    mesh->clearAndSetupDefault(points.size(), indices.size());
    VertexArray va_vert(mesh->getPointSet());
    VertexArray va_face(mesh->getTriangles());
    for (mpcard ii = 0; ii < points.size(); ++ii) {
        va_vert.setPosition3f(ii, points[ii]);
        va_vert.setNormal3f(ii, normals[ii]);
    }
    for (mpcard ii = 0; ii < indices.size(); ++ii) {
        va_face.setIndex3i(ii, indices[ii]);
    }
    addPointCloud(getScene(), mesh, "3x3Mesh");
}

void PCCSDTestCases::create5x5()
{
    std::vector<Vector3f>	points;
    std::vector<Vector3f>	normals;
    std::vector<Vector3i>	indices;
    for (mpcard yi = 0; yi < 4; ++yi) {
        for (mpcard xi = 0; xi < 4; ++xi) {
            int index0 = (int)points.size();
            points.push_back(makeVector3f(xi, yi, 0));
            points.push_back(makeVector3f(xi+1, yi, 0));
            points.push_back(makeVector3f(xi, yi+1, 0));
            points.push_back(makeVector3f(xi+1, yi+1, 0));
            normals.push_back(makeVector3f(0,0,1));
            normals.push_back(makeVector3f(0,0,1));
            normals.push_back(makeVector3f(0,0,1));
            normals.push_back(makeVector3f(0,0,1));
            indices.push_back(makeVector3i(index0, index0+1, index0+2));
            indices.push_back(makeVector3i(index0+2, index0+1, index0+3));
        }
    }

    UnstructuredInCoreTriangleMesh* mesh = new UnstructuredInCoreTriangleMesh;
    mesh->clearAndSetupDefault(points.size(), indices.size());
    VertexArray va_vert(mesh->getPointSet());
    VertexArray va_face(mesh->getTriangles());
    for (mpcard ii = 0; ii < points.size(); ++ii) {
        va_vert.setPosition3f(ii, points[ii]);
        va_vert.setNormal3f(ii, normals[ii]);
    }
    for (mpcard ii = 0; ii < indices.size(); ++ii) {
        va_face.setIndex3i(ii, indices[ii]);
    }
    addPointCloud(getScene(), mesh, "5x5Mesh");
}
