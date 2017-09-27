//---------------------------------------------------------------------------
#ifndef PROJECTS_XW_H
#define PROJECTS_XW_H
//---------------------------------------------------------------------------
#include "GlobalDefinitions.h"
#include "projectsXWu.h"
//---------------------------------------------------------------------------

namespace xgrt {

    extern void PROJECTSXWU_API initProjects_xw();
    extern void PROJECTSXWU_API shutdownProjects_xw();

}

//---------------------------------------------------------------------------
#ifdef USE_CUDA
#include "GPU/common.h"
#include "GPU/PCCNormalPCA.h"
#endif
#include "PCC/PCCSDTestCases.h"
#include "PCI/PCISymmDet.h"
#include "SymmGroup/PCIConsistentCrossShapeGrammar.h"
#include "SymmGroup/PCISymmGroup.h"
#include "Test/PCITestCentr.h"
#include "PCI/PCISymmDefield.h"
#include "SToolBox/DefieldSymm.h"
#include "SToolBox/DefieldSymmRotation.h"
#include "SToolBox/DefieldSymmDihedral.h"
#include "Cosyn/PCIMSFeatureAlign.h"
#include "Cosyn/PCICosyn.h"
#include "Cosyn\C13DescriptorAttachment.h"
#include "Util\MultiScalePCCreator.h"
#include "Cosyn\C13MultiScaleFeatureDetector.h"
#include "Cosyn\C13MultiScaleFeatureDescriptor.h"
#include "Cosyn\GridLabelAttachment.h"
#include "Util/PlaneCluster.h"
#include "SToolBox/SamplerSymmBlock.h"
#include "SToolBox/SymmSampleGraph.h"
//---------------------------------------------------------------------------
using namespace xgrt;
//---------------------------------------------------------------------------

#define XWU_InitClass(className,baseClass) \
    className##::init( baseClass##::getClass());
#define XWU_ShutClass(className,baseClass) \
    className##::shutdown();

void PROJECTSXWU_API xgrt::initProjects_xw()
{        
    //xgrt::InitCUDA();
#ifdef USE_CUDA
    XWU_InitClass( PCCNormalPCA, PCCommand );
#endif
    XWU_InitClass( PCCSDTestCases, PCCommand );
    XWU_InitClass( PCISymmDet, PCInteractionTool );
    XWU_InitClass( PCITestCentr, PCInteractionTool );

    XWU_InitClass( PCIConsistentCrossShapeGrammar, PCInteractionTool );
    XWU_InitClass( ConsistentCrossShapeGrammarSettings, AttachedData );
    XWU_InitClass( CCSGClassification, AttachedData );
    XWU_InitClass( CCSGSymmetryGroup, AttachedData );
    XWU_InitClass( SymmDetContCCSG, AttachedData );

    XWU_InitClass( PCISymmGroup, PCInteractionTool );
    XWU_InitClass( SymmGroupSettings, AttachedData );
    XWU_InitClass( SymmGroupClass, AttachedData );

    XWU_InitClass( PCISymmDefield, PCInteractionTool );
    XWU_InitClass( DefieldSymm, AttachedData );
    XWU_InitClass( DefieldSymmVec, AttachedData );
    XWU_InitClass( DefieldSymmReflection, DefieldSymm );
    XWU_InitClass( DefieldSymmRotation, DefieldSymm );
    XWU_InitClass( DefieldSymmRotationH, DefieldSymmRotation );
    XWU_InitClass( DefieldSymmRotationV, DefieldSymmRotation );
    XWU_InitClass( DefieldSymmDihedral, DefieldSymm );
    XWU_InitClass( DefieldSymmDihedralH, DefieldSymmDihedral );
    XWU_InitClass( PointSymmBlock, AttachedData );
    XWU_InitClass( PointSymmBlockVec, AttachedData );
    XWU_InitClass( LineSymmBlock, AttachedData );
    XWU_InitClass( LineSymmBlockVec, AttachedData );
    XWU_InitClass( SymmSampleGraph, AttachedData );

    XWU_InitClass( PCIMSFeatureAlignSettings, AttachedData );
    XWU_InitClass( PCIMSFeatureAlign, PCInteractionTool );
    XWU_InitClass( PCICosyn, PCInteractionTool );
    XWU_InitClass( C13DescriptorAttachment, AttachedData );
    XWU_InitClass( MultiScalePCCreator, Persistent );
    XWU_InitClass( C13MultiScaleFeatureDetector, Persistent );
    XWU_InitClass( C13MSFeatDetectCrossPoint, C13MultiScaleFeatureDetector );
    XWU_InitClass( C13MultiScaleFeatureDescriptor, Persistent );
    XWU_InitClass( C13MSFeatDescrHOG, C13MultiScaleFeatureDescriptor );
    XWU_InitClass( PlaneCluster, Persistent );
    XWU_InitClass( PlaneClusterExtractor, AttachedData );
    XWU_InitClass( GridLabelAttachment, AttachedData );
}

//---------------------------------------------------------------------------

void PROJECTSXWU_API xgrt::shutdownProjects_xw()
{
#ifdef USE_CUDA
    XWU_ShutClass( PCCNormalPCA, PCCommand );
#endif
    XWU_ShutClass( PCCSDTestCases, PCCommand );
    XWU_ShutClass( PCISymmDet, PCInteractionTool );
    XWU_ShutClass( PCITestCentr, PCInteractionTool );

    XWU_ShutClass( PCIConsistentCrossShapeGrammar, PCInteractionTool );
    XWU_ShutClass( ConsistentCrossShapeGrammarSettings, AttachedData );
    XWU_ShutClass( CCSGClassification, AttachedData );
    XWU_ShutClass( CCSGSymmetryGroup, AttachedData );
    XWU_ShutClass( SymmDetContCCSG, AttachedData );

    XWU_ShutClass( PCISymmGroup, PCInteractionTool );
    XWU_ShutClass( SymmGroupSettings, AttachedData );
    XWU_ShutClass( SymmGroupClass, AttachedData );

    XWU_ShutClass( PCISymmDefield, PCInteractionTool );
    XWU_ShutClass( DefieldSymm, AttachedData );
    XWU_ShutClass( DefieldSymmVec, AttachedData );
    XWU_ShutClass( DefieldSymmReflection, DefieldSymm );
    XWU_ShutClass( DefieldSymmRotation, DefieldSymm );
    XWU_ShutClass( DefieldSymmRotationH, DefieldSymmRotation );
    XWU_ShutClass( DefieldSymmRotationV, DefieldSymmRotation );
    XWU_ShutClass( DefieldSymmDihedral, DefieldSymm );
    XWU_ShutClass( DefieldSymmDihedralH, DefieldSymmDihedral );
    XWU_ShutClass( PointSymmBlock, AttachedData );
    XWU_ShutClass( PointSymmBlockVec, AttachedData );
    XWU_ShutClass( LineSymmBlock, AttachedData );
    XWU_ShutClass( LineSymmBlockVec, AttachedData );
    XWU_ShutClass( SymmSampleGraph, AttachedData );

    XWU_ShutClass( PCIMSFeatureAlignSettings, AttachedData );
    XWU_ShutClass( PCIMSFeatureAlign, PCInteractionTool );
    XWU_ShutClass( PCICosyn, PCInteractionTool );
    XWU_ShutClass( C13DescriptorAttachment, AttachedData );
    XWU_ShutClass( MultiScalePCCreator, Persistent );
    XWU_ShutClass( C13MultiScaleFeatureDetector, Persistent );
    XWU_ShutClass( C13MSFeatDetectCrossPoint, C13MultiScaleFeatureDetector );
    XWU_ShutClass( C13MultiScaleFeatureDescriptor, Persistent );
    XWU_ShutClass( C13MSFeatDescrHOG, C13MultiScaleFeatureDescriptor );
    XWU_ShutClass( PlaneCluster, Persistent );
    XWU_ShutClass( PlaneClusterExtractor, AttachedData );
    XWU_ShutClass( GridLabelAttachment, AttachedData );
}

#endif
