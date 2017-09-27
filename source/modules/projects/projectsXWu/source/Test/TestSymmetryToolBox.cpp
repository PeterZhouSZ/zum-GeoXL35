#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "Test/PCITestCentr.h"
//---------------------------------------------------------------------------
#include "Timer.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

#include "SGRelativeTimeAnimationNode.h"
#include "TopologyAttachment.h"
#include "SymmetryGroupAttachment.h"
#include "group/Reflection.h"
#include "group/Rotation.h"
#include "group/Dihedral.h"
#include "group/Lattice.h"

#include "SToolBox/SymmSampler.h"
#include "PCCTriangleMeshSampler.h"
void TestSymmSampler(
    Scene* scene,
    float sampleth = 0.1,
    std::string name = "pc1",
    std::string shapeName = "__sym_sample_shape_"
    )
{
    //SymmSampler sampler;
    //sampler.DoSample(scene, 0.05);

    //Matrix4f T;
    //{
    //    T[0][0] = 1; T[1][0] = 2; T[2][0] = 3; T[3][0] = 4;
    //    T[0][1] = 5; T[1][1] = 6; T[2][1] = 7; T[3][1] = 8;
    //    T[0][2] = 9; T[1][2] = 0; T[2][2] = 1; T[3][2] = 2;
    //    T[0][3] = 3; T[1][3] = 4; T[2][3] = 5; T[3][3] = 6;
    //}
    //Vector3f a = makeVector3f(1, 2, 3);
    //Vector3f b1 = transformVector3f(T, a); b1 = transformVector3f(T, b1);
    //debugOutput << b1 << "\n";
    //Matrix4f T2 = T*T;
    //debugOutput << T2 << "\n";
    //Vector3f b2 = transformVector3f(T2, a);
    //debugOutput << b2 << "\n";

    //std::string symsamname = "root/" + SymmSampler::SSSNameBase + name;
    //UICPC* symsampc = dynamic_cast<UICPC*>(getPointCloud(scene, symsamname));
    //PointSet const& symsamps = *symsampc->getPointSet();
    //AAT const symsamPosAAT = symsamps.getAAT("position");

    //DefieldSymmVec* defsymvec = dynamic_cast<DefieldSymmVec*>(
    //    symsampc->getAttachments()->getData("DefieldSymmetry"));
    //DefieldSymmDihedralH* symm = dynamic_cast<DefieldSymmDihedralH*>(defsymvec->symmvec_[0]);
    //symm->ExtractAttData();
    //DefieldSymmRotation* rot = dynamic_cast<DefieldSymmRotation*>(symm);
    //rot->UpdatedTransformation(symsamps, symsamPosAAT);
    //Matrix4f Trot = rot->trans_;
    //debugOutput << Trot << "\n";
    //debugOutput << Trot*Trot << "\n";
    ////debugOutput << Tf*(Tf*expand3To4(a)) << "\n";
    ////debugOutput << (Tf*Tf)*expand3To4(a) << "\n";
    //std::set<Vector3f> pv0, pv1;
    //std::set<card32> const& in0 = rot->overt_[0];
    //for (unsigned ii = 0; ii < in0.size(); ++ii) {
    //    pv0.push_back(symsamps.get3f(in0[ii], symsamPosAAT));
    //}
    //pv1.resize(pv0.size());
    //std::copy(pv0.begin(), pv0.end(), pv1.begin());
    //for (unsigned ii = 0; ii < in0.size(); ++ii) {
    //    Matrix4f T = Trot;
    //    debugRenderer->beginRenderJob_OneFrame("draw_rotation_", DR_FRAME++);
    //    for (unsigned ti = 0; ti < rot->numtrans_; ++ti) {
    //        Vector3f p1 = transformVector3f(T, pv0[ii]);
    //        debugRenderer->addLine(
    //            pv1[ii], p1,
    //            makeVector3f(1, 0, 0),
    //            makeVector3f(0, 0, 1),
    //            3);
    //        pv1[ii] = p1;
    //        T *= Trot;
    //    }
    //    debugRenderer->endRenderJob();
    //}
}

void TestDrawGroup(
    sym::SymmetryGroup* group,
    UnstructuredInCorePointCloud* pc
    )
{
    using namespace NAMESPACE_VERSION::sym;

    BoundingBox3f _bbox = pc->getPointSet()->getBoundingBox();

	Rotation* rot = dynamic_cast<Rotation*>(group);
	Reflection* ref = dynamic_cast<Reflection*>(group);
	Dihedral* dih = dynamic_cast<Dihedral*>(group);

	if (dih)
	{
		rot = dih->getBaseRotation();
		if (dih->getInstanceClass() == DihedralH::getClass())
			ref = dynamic_cast<DihedralH*>(dih)->getReflection();
	}

	if (rot)
    {
        {
            unsigned numtrans = rot->getNumberRotations();
            Matrix4f trans = rot->getGenerator()->getWorldTransformation(1);
            Vector3f pos = makeVector3f(1, 0, 0);
            for (unsigned ii = 0; ii < numtrans; ++ii)
            {
                debugRenderer->beginRenderJob_OneFrame("draw_rotation_", DR_FRAME++);
                Vector3f pos_t = transformVector3f(trans, pos);
                pos = pos_t;
                debugRenderer->addPoint(pos_t,
                    makeVector3f(1, 0, 0));
                debugRenderer->endRenderJob();
            }
        }

        {
            unsigned numtrans = rot->getNumTransformations();
            Vector3f pos = makeVector3f(1, 0, 0);
            for (unsigned ii = 0; ii < numtrans; ++ii)
            {
                Matrix4f trans = rot->getGenerator()->getWorldTransformation(ii);
                Vector3f pos_t = transformVector3f(trans, pos);
                debugRenderer->beginRenderJob_OneFrame("draw_rotation_", DR_FRAME++);
                debugRenderer->addPoint(pos_t,
                    makeVector3f(1, 0, 0));
                debugRenderer->endRenderJob();
            }
        }

        {
            unsigned numtrans = rot->getNumTransformations();
            Vector3f pos = makeVector3f(1, 0, 0);
            debugRenderer->beginRenderJob_OneFrame("draw_rotation_", DR_FRAME++);
            for (unsigned ii = 0; ii < numtrans; ++ii)
            {
                Matrix4f trans = rot->getGenerator()->getWorldTransformation(ii);
                Vector3f pos_t = transformVector3f(trans, pos);
                debugRenderer->addPoint(pos_t,
                    makeVector3f(1, 0, 0));
            }
            debugRenderer->endRenderJob();
        }

        debugRenderer->beginRenderJob_OneFrame("draw_group_", DR_FRAME++);

		RotationGenerator* gen = rot->getGenerator();
		Matrix3f f = calcTangentSystem(normalize(gen->getRotationAxis())) * _bbox.getMaxSideLength() * 0.5f;

		// draw rotation arrow
		float angle = 2.0f * M_PI / (float)rot->getNumberRotations();
		float anglestep = angle / 40.f;

        debugRenderer->addLine(
            gen->getRotationAxisCenter() - gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f,
            gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f,
            makeVector3f(1, 0, 0),
            makeVector3f(1, 0, 0),
            3);

        std::vector<Vector3f> rarr;
		for (float a=0; a < angle; a+=anglestep)
		{
            rarr.push_back(gen->getRotationAxisCenter() + 
                gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f + 
                f * makeVector3f(cos(a), sin(a), 0)*0.3f);
		}
        rarr.push_back(gen->getRotationAxisCenter() + 
                gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f + 
                f * makeVector3f(cos(angle), sin(angle), 0)*0.3f);
        debugRenderer->addLineStrip(rarr, makeVector3f(1, 0, 0), 3);
        debugRenderer->addLine(
            gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f,
            gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle-0.15f), sin(angle-0.15f), 0)*0.325f,
            makeVector3f(1, 0, 0),
            makeVector3f(1, 0, 0),
            3);
        debugRenderer->addLine(
            gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f,
            gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle-0.15f), sin(angle-0.15f), 0)*0.275f,
            makeVector3f(1, 0, 0),
            makeVector3f(1, 0, 0),
            3);

		RotationV* rotv = dynamic_cast<RotationV*>(rot);
		RotationH* roth = dynamic_cast<RotationH*>(rot);

		if (rotv)
		{
            for (unsigned i=0; i < rotv->getNumReflections(); i++)
            {
                Reflection* ref = (Reflection*)rotv->getReflection(i);
                ReflectionGenerator* gen = ref->getGenerator();

                Matrix3f f = shrink4To3(gen->frame) * _bbox.getMaxSideLength() * 0.5f;

                debugRenderer->addQuad(
                    gen->plane.getPoint() + f * makeVector3f(1,1,0),
                    gen->plane.getPoint() + f * makeVector3f(-1,1,0),
                    gen->plane.getPoint() + f * makeVector3f(-1,-1,0),
                    gen->plane.getPoint() + f * makeVector3f(1,-1,0),
                    makeVector3f(1,1,0));
            }
        } else if (roth)
        {
            Reflection* ref = (Reflection*)roth->getReflection();
			ReflectionGenerator* gen = ref->getGenerator();

            Matrix3f f = shrink4To3(gen->frame) * _bbox.getMaxSideLength() * 0.5f;

            debugRenderer->addQuad(
                gen->plane.getPoint() + f * makeVector3f(1,1,0),
                gen->plane.getPoint() + f * makeVector3f(-1,1,0),
                gen->plane.getPoint() + f * makeVector3f(-1,-1,0),
                gen->plane.getPoint() + f * makeVector3f(1,-1,0),
                makeVector3f(1,1,0));
		}

        debugRenderer->endRenderJob();
	}

	if (ref)
	{
        debugRenderer->beginRenderJob_OneFrame("draw_group_", DR_FRAME++);

		ReflectionGenerator* gen = ref->getGenerator();

		Matrix3f f = shrink4To3(gen->frame) * _bbox.getMaxSideLength() * 0.5f;

        debugRenderer->addQuad(
            gen->plane.getPoint() + f * makeVector3f(1,1,0),
            gen->plane.getPoint() + f * makeVector3f(-1,1,0),
            gen->plane.getPoint() + f * makeVector3f(-1,-1,0),
            gen->plane.getPoint() + f * makeVector3f(1,-1,0),
            makeVector3f(1,1,0));

        debugRenderer->endRenderJob();
	}

	if (dih)
	{
        debugRenderer->beginRenderJob_OneFrame("draw_group_", DR_FRAME++);

		for (unsigned i=0; i < dih->getNumRotations(); i++)
		{
			Rotation* ref = (Rotation*)dih->getRotation(i);
			RotationGenerator* gen = ref->getGenerator();

			Matrix3f f = /*calcTangentSystem(normalize(gen->getRotationAxis()))*/ gen->getFrame() * _bbox.getMaxSideLength() * 0.5f;

            // draw rotation arrow
            float angle = 2.0f * M_PI / (float)ref->getNumberRotations();
            float anglestep = angle / 40.f;

            debugRenderer->addLine(
                gen->getRotationAxisCenter() - gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f,
                gen->getRotationAxisCenter() + gen->getRotationAxis() * _bbox.getMaxSideLength() * 0.65f,
                makeVector3f(1, 0, 0),
                makeVector3f(1, 0, 0),
                3);

            for (unsigned x=0; x < 2; x++)
            {
                std::vector<Vector3f> rarr;
                for (float a=0; a < angle; a+=anglestep)
                {
                    rarr.push_back(gen->getRotationAxisCenter() + gen->getRotationAxis() * ((x%2) ? -1  : 1) * _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(a), sin(a), 0)*0.3f);
                }
                rarr.push_back(gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f);
                debugRenderer->addLineStrip(rarr, makeVector3f(1, 0, 0), 3);
                debugRenderer->addLine(
                    gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f,
                    gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle-0.15f), sin(angle-0.15f), 0)*0.325f,
                    makeVector3f(1, 0, 0),
                    makeVector3f(1, 0, 0),
                    3);
                debugRenderer->addLine(
                    gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle), sin(angle), 0)*0.3f,
                    gen->getRotationAxisCenter() + gen->getRotationAxis()  * ((x%2) ? -1  : 1)* _bbox.getMaxSideLength() * 0.65f + f * makeVector3f(cos(angle-0.15f), sin(angle-0.15f), 0)*0.275f,
                    makeVector3f(1, 0, 0),
                    makeVector3f(1, 0, 0),
                    3);
            }
        }
        glDisable(GL_BLEND);

        debugRenderer->endRenderJob();
    }
}

void PCITestCentr::TestSymmetryToolBox(void)
{
    std::string name = "pc1";
    //UICPC* mesh = dynamic_cast<UICPC*>(getPointCloud(getScene(), name));
    SGListNode* root = dynamic_cast<SGListNode*>(getScene()->getRootNode());

    SGObjectNode* meshnode = dynamic_cast<SGObjectNode*>(
        root->getChildNode(0, root->getChildIndex(name)));
    if (!meshnode)
    {
        error("\"" + name + "\" not found.");
        return;
    }
    UnstructuredInCorePointCloud* mesh = dynamic_cast<UnstructuredInCorePointCloud*>(
        meshnode->getSceneObject());
    if (!mesh)
    {
        error("Mesh not found.");
        return;
    }
    PointSet const& meshps = *mesh->getPointSet();
    AAT const meshPositionAAT = meshps.getAAT("position");

    SGRelativeTimeAnimationNode* listnode = dynamic_cast<SGRelativeTimeAnimationNode*>(
        root->getChildNode(0, root->getChildIndex("root_" + name + "_groups")));
    if (!listnode)
    {
        error("Node \"root_" + name + "_groups\" not found.");
        return;
    }

    SGObjectNode* pointnode = dynamic_cast<SGObjectNode*>(
        root->getChildNode(0, root->getChildIndex("__sym_work_shape_" + name)));
    if (!pointnode)
    {
        error("\"__sym_work_shape_" + name + "\" not found.");
        return;
    }

    UnstructuredInCorePointCloud* points = dynamic_cast<UnstructuredInCorePointCloud*>(
        pointnode->getSceneObject());
    if (!points)
    {
        error("Points not found.");
        return;
    }
    PointSet const& pointps = *points->getPointSet();
    AAT const pointPositionAAT = pointps.getAAT("position");

    InCorePCTopologyGraph* tpg = dynamic_cast<TopologyAttachment*>(
        points->getAttachments()->getData(TopologyAttachment::getDefaultName()))->getTopology();
    PointSet const& eset = *tpg->getEdges();
    AAT eset_vindex = eset.getAAT("vindex");

    //for (mpcard i = 0; i < sublist->getNumChildNodes(0); ++i)
    //{
        SGObjectNode* sym = dynamic_cast<SGObjectNode*>(
            listnode->getChildNode(0, 0));
        if (!sym)
        {
            error("Sym not found");
            return;
        }

        UnstructuredInCorePointCloud* pc = dynamic_cast<UnstructuredInCorePointCloud*>(
            sym->getSceneObject());
        if (!pc)
        {
            error("Pc not found");
            return;
        }

        sym::SymmetryGroupAttachment* groupatt = dynamic_cast<sym::SymmetryGroupAttachment*>(
            pc->getAttachments()->getData("SymmetryGroup"));
        if (!groupatt)
        {
            error("Group not found");
            return;
        }
        sym::SymmetryGroup* group = groupatt->group;
    //}

    //std::vector<mpcard> invalidPoints;
    //const pair<std::vector<mpcard>, std::vector<mpcard>>& result = group->detectBaseElements_Improved(
    //    points, tpg, groupatt->featureSet, groupatt->lines, false, &invalidPoints);
    //debugRenderer->beginRenderJob_OneFrame("group_base_", DR_FRAME++);
    //for (std::vector<mpcard>::const_iterator it = result.first.begin();
    //    it != result.first.end(); ++it)
    //{
    //    Vector3f const& position = pointps.get3f(*it, pointPositionAAT);
    //    debugRenderer->addPoint(position,
    //        makeVector3f(1, 0, 0));
    //}
    //for (std::vector<mpcard>::const_iterator it = result.second.begin();
    //    it != result.second.end(); ++it)
    //{
    //    Vector3f const& position = pointps.get3f(*it, pointPositionAAT);
    //    debugRenderer->addPoint(position,
    //        makeVector3f(0, 0, 1));
    //}
    //debugRenderer->endRenderJob();
    //debugOutput << "base number: " << result.first.size() << " ["
    //    << result.second.size() << ", "
    //    << pointps.getNumEntries() << ", "
    //    << invalidPoints.size()
    //    << "]" << "\n";

    //debugRenderer->beginRenderJob_OneFrame("feature_line_", DR_FRAME++);
    //std::vector<GeometricFeature*> flines = groupatt->featureSet->m_Features;
    //std::vector<card32> lines = groupatt->lines;
    //for (unsigned i=0; i < lines.size(); i++)
    //{
    //    sym::FeatureLine* line = dynamic_cast<sym::FeatureLine*>(flines[lines[i]]);
    //    if (!line) continue;
    //    Vector3f pos0 = line->getPosition();
    //    Vector3f pos1 = line->getEndPosition();
    //    debugRenderer->addLine(
    //        pos0, pos1,
    //        makeVector3f(0, 0, 1),
    //        makeVector3f(0, 0, 1),
    //        3);
    //}

    //debugRenderer->beginRenderJob_OneFrame("topology_", DR_FRAME++);
    //for (unsigned i=0; i < tpg->getNumEdges(); i++)
    //{
    //    Vector2i vi = eset.get2i(i, eset_vindex);
    //    Vector3f const& p0 = pset.get3f(vi[0], pointPositionAAT);
    //    Vector3f const& p1 = pset.get3f(vi[1], pointPositionAAT);
    //    debugRenderer->addLine(
    //        p0, p1,
    //        makeVector3f(1, 0, 0),
    //        makeVector3f(0, 0, 1),
    //        3);
    //}
    //debugRenderer->endRenderJob();

    unsigned numtrans = group->getNumTransformations();
    //debugOutput << "trans number: " << numtrans << "\n";
    //for (unsigned ii = 0; ii < numtrans; ++ii)
    //{
    //    Matrix4f trans = group->getTransformation(ii);
    //    debugRenderer->beginRenderJob_OneFrame("group_transformation_", DR_FRAME++);
    //    for (unsigned k = 0; k < points->getNumPoints(); ++k)
    //    {
    //        Vector3f p = transformVector3f(trans, pointps.get3f(k, pointPositionAAT));
    //        debugRenderer->addPoint(p,
    //            makeVector3f(1, 0, 0));
    //    }
    //    debugRenderer->endRenderJob();
    //}
    //for (unsigned ii = 0; ii < numtrans; ++ii)
    //{
    //    //Matrix4f trans = group->getTransformation(ii);
    //    Matrix4f trans = group->getGenerator()->getWorldTransformation(ii);
    //    debugRenderer->beginRenderJob_OneFrame("group_transformation_", DR_FRAME++);
    //    for (unsigned k = 0; k < mesh->getNumPoints(); ++k)
    //    {
    //        Vector3f p = transformVector3f(trans, meshps.get3f(k, meshPositionAAT));
    //        debugRenderer->addPoint(p,
    //            makeVector3f(1, 0, 0));
    //    }
    //    debugRenderer->endRenderJob();
    //}
    //for (unsigned k = 0; k < mesh->getNumPoints(); ++k)
    //{
    //    Vector3f p0 = meshps.get3f(k, meshPositionAAT);
    //    debugRenderer->beginRenderJob_OneFrame("group_transformation_", DR_FRAME++);
    //    for (unsigned ii = 0; ii < numtrans; ++ii)
    //    {
    //        //Matrix4f trans = group->getTransformation(ii);
    //        Matrix4f trans = group->getGenerator()->getWorldTransformation(ii);
    //        Vector3f p = transformVector3f(trans, p0);
    //        debugRenderer->addPoint(p,
    //            makeVector3f(1, 0, 0));
    //    }
    //    debugRenderer->endRenderJob();
    //}

    //TestDrawGroup(group, pc);
    TestSymmSampler(getScene(), 0.2);
}
