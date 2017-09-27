#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SToolBox/DummySampler.h"
#include "SToolBox/SymmSampler.h"
//---------------------------------------------------------------------------
#include "SGRelativeTimeAnimationNode.h"
#include "PCCTriangleMeshSampler.h"
#include "SymmGroup/CCSGSymmetryGroup.h"
#include "SymmetryGroupAttachment.h"
#include "detector/FeatureLine.h"

#include "UnstructuredInCoreTriangleMesh.h"
#include "Timer.h"
#include "ProgressWindow.h"

#include <group/Lattice.h>
#include <group/Rotation.h>
#include <group/Reflection.h>
#include <group/Dihedral.h>
#include "detector/CreaseLineFeatureDetector.h"
#include "detector/SymmetryGroupDetector.h"
#include "GeometricFeature.h"
#include "DVectorObject.h"

#include "PCCTriangleMeshSampler.h"
#include "GeometricTools.h"

#include "SGListNode.h"
#include "SGRelativeTimeAnimationNode.h"

#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

void DummySampler::DoSample(
    Scene* scene,
    const std::string& symmWorkName,
    const std::string& inputSymmName,
    UnorderedGridUniq* pPointgrid,
    const float& sampleth
    )
{
    //UnorderedGridUniq& pointgrid = *pPointgrid;
    //const bool debug_output = false;

    //UICPC* workPoints = dynamic_cast<UICPC*>(
    //    getPointCloud(scene, symmWorkName));
    //if (!workPoints)
    //{
    //    error("Symmetry work points not found.");
    //    return;
    //}
    //UICPC* inPoisson = workPoints;
    //const PointSet& inPoissonPS = *inPoisson->getPointSet();
    //const AAT& inPoissonPosAAT = inPoissonPS.getAAT("position");
    //PointSet& inPoissonPSmod = *inPoisson->getPointSet();
    //const AAT& symGroupAAT = inPoissonPSmod.getAAT("group");
    //const AAT& colorAAT = inPoissonPSmod.getAAT("color");
    //UICPC* outPoisson = dynamic_cast<UICPC*>(
    //    getPointCloud(scene, inputSymmName));
    //if (!outPoisson)
    //{
    //    error("Symmetry sampling points not found.");
    //    return;
    //}
    //PointSet& outPoissonPS = *outPoisson->getPointSet();
    //const AAT& outPoissonAAT = outPoissonPS.getAAT("position");

    //const unsigned& num_poisson = inPoisson->getNumPoints();
    //std::deque<int> markPC(num_poisson, SymmSampler::UNTOUCHED);
    //for (unsigned pi = 0; pi < num_poisson; ++pi) {
    //    markPC[pi] = SymmSampler::GroupCode(inPoissonPS.get2i(pi, symGroupAAT));
    //}

    //FastSphereQuerryPtr querySam( new FastSphereQuerry(inPoisson));
    //for (unsigned pi = 0; pi < num_poisson; ++pi)
    //{
    //    if (SymmSampler::UNTOUCHED != markPC[pi]) continue;

    //    const Vector3f& pos_pi = inPoissonPS.get3f(pi, inPoissonPosAAT);

    //    // add new point
    //    const size_t& numEntries = outPoissonPS.getNumEntries();
    //    outPoissonPS.changeHeight(numEntries + 1);
    //    outPoissonPS.set3f(numEntries, outPoissonAAT, pos_pi);
    //    outPoissonPS.set2i(numEntries, symGroupAAT, SymmSampler::FILL_IN);
    //    outPoissonPS.set3f(numEntries, colorAAT, SymmSampler::MapGroupColor(SymmSampler::FILL_IN));

    //    // mark processed points, within sampling radius
    //    mpcard numAllPoints; mpcard *allIndices;
    //    querySam->querry(pos_pi, sampleth, &allIndices, numAllPoints);
    //    for (size_t qi = 0; qi < numAllPoints; ++qi) {
    //        if (SymmSampler::UNTOUCHED == markPC[allIndices[qi]])
    //            markPC[allIndices[qi]] = SymmSampler::FILL_IN;
    //    }
    //}

    //// update work shape, just for visualization
    //for (unsigned pi = 0; pi < num_poisson; ++pi) {
    //    inPoissonPSmod.set2i(pi, symGroupAAT, markPC[pi]);
    //    inPoissonPSmod.set3f(pi, colorAAT, SymmSampler::MapGroupColor(markPC[pi]));
    //}
}
