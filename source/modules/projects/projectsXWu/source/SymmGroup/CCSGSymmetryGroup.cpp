#include "StdAfx.h"

#include "CCSGSymmetryGroup.h"

#include <CopyObjectProperties.h>

#include <SGObjectNode.h>
#include <SGListNode.h>
#include <UnstructuredInCorePointCloudManifold.h>
#include <UnstructuredInCoreTriangleMeshManifold.h>
#include <Ransac.h>
#include <ObjectClassProperty.h>
#include "GLMaterial.h"
#include "GLImage.h"
#include "GLRenderTextureCache.h"
#include "GLHelper.h"
#include "GLContext.h"
#include "ProgressWindow.h"
#include <HighQualityRenderer.h>
#include <SlippageFeatureDetector.h>
#include <ManifoldCorrespondences.h>
#include <LandmarkCoordinate.h>
#include "RegularInCoreVolume.h"
#include "RegularVolume.h"
#ifdef XGRT_3_0
#include "RegularVolume_templCode.h"
#elif GEOXL_3_5
#include "RegularVolume.inline.h"
#endif

#include <PCCSmoothing.h>
#include "Random.h"
#include "MeshExporter.h"
#include "PointCloudExporter.h"

#include "ColorTools.h"
#include "PickFromPointSet.h"
#include "SGRelativeTimeAnimationNode.h"
#include <GeometricTools.h>
#include "TopologyAttachment.h"
#include "PCCResampler.h"
#include "SeparatorClassProperty.h"
#include "FastSphereQuerry.h"
#include "VertexArray.h"
#include "PropertyTableProperty.h"

#include "AttachedFloatSignatures.h"
#include <PCCResampleByCurvature.h>
#include <SpinImageDescriptor.h>
#include <SimpleAttachment.h>
#include <PCAGeneral.h>
#include <FSIcp.h>

#include <FileDialogs.h>
#include <InterfaceGlobals.h>
#include <detector/FeatureLine.h>
#include <group/Lattice.h>
#include <group/Rotation.h>
#include <group/Reflection.h>
#include <group/Dihedral.h>
#include <detector/CreaseLineFeatureDetector.h>
#include <DVectorObject.h>
#include <RigidMotion.h>
#include <PCCComputeTopology.h>

#include <ImportanceSampling.h>
#include <GeometricTools.h>
#include <GLShaderMaterial.h>
#include <FileDialogs.h>

#include <TopologicalKNNIterator.h>
#include <PCCTriangleMeshSampler.h>
#include <InverseDynamicKMedianClustering.h>

#include <ctime>
#include <omp.h>
#include <fstream>
#include <qthread.h>

//--------------------------------------------------------------------------------------
IMPLEMENT_CLASS(CCSGSymmetryGroup, 5)
{
	BEGIN_CLASS_INIT(CCSGSymmetryGroup);
	INIT_PROPERTY_TABLE();

	ADD_STRING_PROP(key, 1);
	ADD_CARD32_PROP(votes, 1);
	ADD_OBJECT_PROP(group, 1, sym::SymmetryGroup::getClass(), true);
	ADD_OBJECT_PROP(featureSet, 2, FeatureSet::getClass(), false);

	ADD_CARD32_PROP(id, 3);

	ADD_SEPARATOR("Indices");
	ADD_CARD32_VAR_ARRAY_PROP(generatorGIDs, 0);
	ADD_CARD32_VAR_ARRAY_PROP(lines, 1);
	ADD_CARD32_VAR_ARRAY_PROP(symmetryPoints, 3);
	ADD_CARD32_VAR_ARRAY_PROP(allLinesCovered, 4);

	ADD_BOOLEAN_PROP(visible, 5);
}

//--------------------------------------------------------------------------------------
CCSGSymmetryGroup::~CCSGSymmetryGroup()
{
	//if (featureSet) delete featureSet;
	if (group) delete group;
}

//--------------------------------------------------------------------------------------
bool CCSGSymmetryGroup::operator<(CCSGSymmetryGroup& g) const
{ 
	size_t numlines = lines.size();
	size_t gnumlines= g.lines.size();

	if (numlines == gnumlines) 
		return allLinesCovered < g.allLinesCovered; 
	else 
		return numlines < gnumlines;
}

//--------------------------------------------------------------------------------------
void CCSGSymmetryGroup::read(InputObjectStream *s)
{
	AttachedData::read(s);
	update();
}

//--------------------------------------------------------------------------------------
void CCSGSymmetryGroup::assign(const Object* obj, COPY_CONTEXT *context)
{
	const CCSGSymmetryGroup* gr = dynamic_cast<const CCSGSymmetryGroup*>(obj);
	if (!obj) return;

	generatorGIDs = gr->generatorGIDs;
	lines = gr->lines;
	symmetryPoints = gr->symmetryPoints;
	allLinesCovered = gr->allLinesCovered;

	id = gr->id;
	bbox = gr->bbox;

	featureSet = gr->featureSet;

	key = gr->key;
	votes = gr->votes;
	group = (sym::SymmetryGroup*)gr->group->copy();
}

//--------------------------------------------------------------------------------------
CCSGSymmetryGroup* CCSGSymmetryGroup::buildFromSymmetryGroupAttachment(sym::SymmetryGroupAttachment* gr)
{
	CCSGSymmetryGroup* result = new CCSGSymmetryGroup();

	result->generatorGIDs = gr->generatorGIDs;
	result->lines = gr->lines;
	result->symmetryPoints = gr->symmetryPoints;
	result->allLinesCovered = gr->allLinesCovered;

	result->id = gr->id;
	result->bbox = gr->bbox;

	result->featureSet = gr->featureSet;

	result->key = gr->key;
	result->votes = 0;
	result->group = (sym::SymmetryGroup*)gr->group->copy();

	return result;
}

//--------------------------------------------------------------------------------------
sym::SymmetryGroupAttachment* CCSGSymmetryGroup::buildSymmetryGroupAttachment(CCSGSymmetryGroup* gr)
{
	sym::SymmetryGroupAttachment* result = new sym::SymmetryGroupAttachment();

	result->generatorGIDs = gr->generatorGIDs;
	result->lines = gr->lines;
	result->symmetryPoints = gr->symmetryPoints;
	result->allLinesCovered = gr->allLinesCovered;

	result->id = gr->id;
	result->bbox = gr->bbox;

	result->featureSet = gr->featureSet;

	result->key = gr->key;
	result->group = (sym::SymmetryGroup*)gr->group->copy();

	return result;
}

//--------------------------------------------------------------------------------------
void CCSGSymmetryGroup::update()
{
	key = group->getName() + ":";
	std::sort(lines.begin(), lines.end());
	lines.resize(std::unique(lines.begin(), lines.end()) - lines.begin());
	for (unsigned i=0; i < lines.size(); i++)
		key += intToStr(lines[i])+",";

	bbox = BoundingBox3f();
	if (featureSet)
	{
		for (unsigned i=0; i < allLinesCovered.size(); i++)
		{
			sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(featureSet->m_Features[allLinesCovered[i]]);
			if (!line) continue;
			bbox.addPoint(line->getPosition());
			bbox.addPoint(line->getEndPosition());
		}
	}

}

#if 0
//--------------------------------------------------------------------------------------
bool CCSGSymmetryGroup::clearGeometrically(UnstructuredInCorePointCloud* baseShape, card32 numpoints, UnstructuredInCorePointCloud** resultShape, bool doSymmetricCheck, bool doTopologyCheck)
{
	if (symmetryPoints.size() <= numpoints) return true;

	UnstructuredInCorePointCloud* pc = new UnstructuredInCorePointCloud;
	pc->setPointSet(baseShape->getPointSet()->subset(symmetryPoints));

	if (doSymmetricCheck)
	{
		symmetryPoints = group->detectSymmetricSubset(symmetryPoints, baseShape, pc);
		delete pc;

		if (symmetryPoints.size() <= numpoints) return true;

		pc = new UnstructuredInCorePointCloud;
		pc->setPointSet(baseShape->getPointSet()->subset(symmetryPoints));
	}

	if (doTopologyCheck)
	{
		PCCComputeTopology cmd;
		cmd.setup(PCCComputeTopology::TOPTYPE_EPS, group->getSpatialTolerance());
		InCorePCTopologyGraph* _tpg = cmd.computeEpsKNNTopology(pc->getPointSet(), pc);
		pc->clearAttachments();
		std::vector<mpcard> pts = removeSmallPatches(pc, &_tpg, numpoints);
		delete _tpg;

		for (unsigned i=0; i < pts.size(); i++)
			symmetryPoints[pts[i]] = (mpcard)-1;
		for (std::vector<mpcard>::iterator it = symmetryPoints.begin(); it != symmetryPoints.end(); )
		{
			if (*it == (mpcard)-1) it = symmetryPoints.erase(it);
			else it++;
		}
	}

	if (symmetryPoints.size() > numpoints && resultShape)
		*resultShape = pc;
	else 
		delete pc;

	return symmetryPoints.size() <= numpoints;
}

//--------------------------------------------------------------------------------------
void CCSGSymmetryGroup::merge(CCSGSymmetryGroup* group)
{
	mergeLinesFromGroup(group);

	symmetryPoints.insert(symmetryPoints.end(), group->symmetryPoints.begin(), group->symmetryPoints.end());
	sort(symmetryPoints.begin(), symmetryPoints.end());
	symmetryPoints.resize(unique(symmetryPoints.begin(), symmetryPoints.end()) - symmetryPoints.begin());

	update();
}

//--------------------------------------------------------------------------------------
void CCSGSymmetryGroup::mergeLinesFromGroup(CCSGSymmetryGroup* group)
{
	if (!group) return;

	if (group->lines.size())
	{
		sort(lines.begin(), lines.end());
		sort(group->lines.begin(), group->lines.end());

		std::vector<card32> uni(lines.size() + group->lines.size());
		uni.erase(std::set_union(lines.begin(), lines.end(), group->lines.begin(), group->lines.end(), uni.begin()), uni.end());
		lines.swap(uni);
	}

	if (group->allLinesCovered.size())
	{
		sort(allLinesCovered.begin(), allLinesCovered.end());
		sort(group->allLinesCovered.begin(), group->allLinesCovered.end());

		std::vector<card32> uni2(allLinesCovered.size() + group->allLinesCovered.size());
		uni2.erase(std::set_union(allLinesCovered.begin(), allLinesCovered.end(), group->allLinesCovered.begin(), group->allLinesCovered.end(), uni2.begin()), uni2.end());
		allLinesCovered.swap(uni2);
	}
}
#endif 

//--------------------------------------------------------------------------------------
void CCSGSymmetryGroup::renderGL(SceneObject *obj)
{
	if (!visible) return;

	using namespace sym;

	SceneObject* selcloud = obj;
	if (selcloud == nullptr) return;

	BoundingBox3f _bbox = dynamic_cast<UnstructuredInCorePointCloud*>(obj)->getPointSet()->getBoundingBox();

	// draw lines
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glEnable(GL_DEPTH_TEST);

	//glDepthMask(GL_FALSE);
	glLineWidth(3.0f);
	glBegin(GL_LINES);
	glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
	for (unsigned i=0; i < lines.size() && featureSet; i++)
	{
		sym::FeatureLine* line = dynamic_cast<FeatureLine*>(featureSet->m_Features[lines[i]]);
		if (!line) continue;
		Vector3f pos0 = line->getPosition();
		Vector3f pos1 = line->getEndPosition();

		glColor3f(1,0,0);
		glVertex3f(pos0[0], pos0[1], pos0[2]);
		glColor3f(1,0,0);
		glVertex3f(pos1[0], pos1[1], pos1[2]);
	}
	glEnd();
	//glPolygonOffset(0, 1e-3f);
	//glDepthMask(GL_TRUE);

	Rotation* rot = dynamic_cast<Rotation*>(group);
	Reflection* ref = dynamic_cast<Reflection*>(group);
	Lattice* lat = dynamic_cast<Lattice*>(group);
	Dihedral* dih = dynamic_cast<Dihedral*>(group);

	if (dih)
	{
		rot = dih->getBaseRotation();
		if (dih->getInstanceClass() == DihedralH::getClass())
			ref = dynamic_cast<DihedralH*>(dih)->getReflection();
	}

	if (lat)
	{
		Lattice1DGenerator* lat1dgen = dynamic_cast<Lattice1DGenerator*>(lat->getGenerator());
		if (lat1dgen)
		{
			float size = 0.01f;

			Matrix4f modelView;
			glGetFloatv(GL_MODELVIEW_MATRIX, modelView.data());

			Vector3f right = shrink4To3(modelView[0]);
			Vector3f up = shrink4To3(modelView[1]);

			//if (normalize(right) * normalize(lat1dgen->u()) < 0) right *= -1;
			//right = normalize(lat1dgen->u());
			//up = right.crossProduct(normalize(up));
			right *= size;
			up *= size;
			Vector3f pos = lat1dgen->pos();
			Vector3f dir = lat1dgen->u();

			glColor3f(1,0,0);
			glBegin(GL_LINES);
				glVertex3fv(pos.data());
				glVertex3fv((pos + dir).data());
			glEnd();

			glBegin(GL_LINES);
				glVertex3fv((pos + dir).data());
				glVertex3fv((pos + dir - right - up).data());
			glEnd();

			glBegin(GL_LINES);
				glVertex3fv((pos + dir).data());
				glVertex3fv((pos + dir - right + up).data());
			glEnd();

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glColor4f(0,1,1, 0.75f);
			for (int i=lat->getMinParameterCoordinate()[0]; i <= lat->getMaxParameterCoordinate()[0]; i++)
			{
				Vector3f ppos = pos + (float)i * dir;

				glBegin(GL_QUADS);
					glVertex3fv((ppos + right + up).data());
					glVertex3fv((ppos - right + up).data());
					glVertex3fv((ppos - right - up).data());
					glVertex3fv((ppos + right - up).data());
				glEnd();
			}
			glDisable(GL_BLEND);

		}

	
	}

	if (ref)
	{
		ReflectionGenerator* gen = ref->getGenerator();

		Matrix3f f = shrink4To3(gen->frame) * _bbox.getMaxSideLength() * 0.5f;

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(1,1,0, 0.35f);
		glNormal3fv(gen->plane.getNormal().data());
		glBegin(GL_QUADS);
			glVertex3fv((gen->plane.getPoint() + f * makeVector3f(1,1,0)).data());
			glVertex3fv((gen->plane.getPoint() + f * makeVector3f(-1,1,0)).data());
			glVertex3fv((gen->plane.getPoint() + f * makeVector3f(-1,-1,0)).data());
			glVertex3fv((gen->plane.getPoint() + f * makeVector3f(1,-1,0)).data());
		glEnd();
		glDisable(GL_BLEND);
	}
	
	if (rot)
	{
		RotationGenerator* gen = rot->getGenerator();
		Matrix3f f = calcTangentSystem(normalize(gen->getRotationAxis())) * _bbox.getMaxSideLength() * 0.5f;

		glColor3f(1,0,0);
		glBegin(GL_LINES);
		glVertex3fv((gen->getRotationAxisCenter() - gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65).data());
		glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65).data());
		glEnd();

		// draw rotation arrow
		float angle = 2.0f * M_PI / (float)rot->getNumberRotations();
		float anglestep = angle / 40.f;

		glBegin(GL_LINE_STRIP);
		for (float a=0; a < angle; a+=anglestep)
		{
			glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(a), sin(a), 0)*0.3f).data()); 
		}
			glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f).data()); 
		glEnd();
		glBegin(GL_LINES);
			glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f).data()); 
			glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle-0.15f), sin(angle-0.15f), 0)*0.325f).data()); 

			glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f).data()); 
			glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle-0.15f), sin(angle-0.15f), 0)*0.275f).data()); 
		glEnd();

		RotationV* rotv = dynamic_cast<RotationV*>(rot);
		RotationH* roth = dynamic_cast<RotationH*>(rot);

		if (rotv)
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			for (unsigned i=0; i < rotv->getNumReflections(); i++)
			{
				Reflection* ref = (Reflection*)rotv->getReflection(i);
				ReflectionGenerator* gen = ref->getGenerator();

				Matrix3f f = shrink4To3(gen->frame) * _bbox.getMaxSideLength() * 0.5f;

				glNormal3fv(gen->plane.getNormal().data());
				if (ref->isVirtual())
					glColor4f(1,0,1, 0.35f);
				else
					glColor4f(1,1,0, 0.35f);

				glBegin(GL_QUADS);
					glVertex3fv((gen->plane.getPoint() + f * makeVector3f(1,1,0)).data());
					glVertex3fv((gen->plane.getPoint() + f * makeVector3f(-1,1,0)).data());
					glVertex3fv((gen->plane.getPoint() + f * makeVector3f(-1,-1,0)).data());
					glVertex3fv((gen->plane.getPoint() + f * makeVector3f(1,-1,0)).data());
				glEnd();
			}
			glDisable(GL_BLEND);
		}else if (roth)
		{
			Reflection* ref = (Reflection*)roth->getReflection();
			ReflectionGenerator* gen = ref->getGenerator();

			Matrix3f f = shrink4To3(gen->frame) * _bbox.getMaxSideLength() * 0.5f;

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glColor4f(1,1,0, 0.5f);
			glNormal3fv(gen->plane.getNormal().data());
			glBegin(GL_QUADS);
				glVertex3fv((gen->plane.getPoint() + f * makeVector3f(1,1,0)).data());
				glVertex3fv((gen->plane.getPoint() + f * makeVector3f(-1,1,0)).data());
				glVertex3fv((gen->plane.getPoint() + f * makeVector3f(-1,-1,0)).data());
				glVertex3fv((gen->plane.getPoint() + f * makeVector3f(1,-1,0)).data());
			glEnd();
			glDisable(GL_BLEND);
		}
	}

	if (dih)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		for (unsigned i=0; i < dih->getNumRotations(); i++)
		{
			Rotation* ref = (Rotation*)dih->getRotation(i);
			RotationGenerator* gen = ref->getGenerator();
				
			Matrix3f f = /*calcTangentSystem(normalize(gen->getRotationAxis()))*/ gen->getFrame() * _bbox.getMaxSideLength() * 0.5f;

			if (ref->isVirtual())
				glColor4f(1,0,1, 0.5f);
			else
				glColor4f(1,1,0, 0.5f);

			// draw rotation arrow
			float angle = 2.0f * M_PI / (float)ref->getNumberRotations();
			float anglestep = angle / 40.f;

			glBegin(GL_LINES);
			glVertex3fv((gen->getRotationAxisCenter() - gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65).data());
			glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65).data());
			glEnd();

			for (unsigned x=0; x < 2; x++)
			{
				glBegin(GL_LINE_STRIP);
				for (float a=0; a < angle; a+=anglestep)
					glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis() * ((x%2) ? -1  : 1) * _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(a), sin(a), 0)*0.3f).data()); 
				glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f).data()); 
				glEnd();

				glBegin(GL_LINES);
					glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f).data()); 
					glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle-0.15f), sin(angle-0.15f), 0)*0.325f).data()); 

					glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f).data()); 
					glVertex3fv((gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65 + f * makeVector3f(cos(angle-0.15f), sin(angle-0.15f), 0)*0.275f).data()); 
				glEnd();
			}
		}
		glDisable(GL_BLEND);
	}

	glPopAttrib();

}


