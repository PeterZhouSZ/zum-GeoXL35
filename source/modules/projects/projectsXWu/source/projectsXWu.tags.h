SET_MODULE_TYPE_APPLICATION
ADD_DEPENDENCY("x4basics")
ADD_DEPENDENCY("x4modules")

ADD_AUTHOR("Xiaokun Wu")

ADD_INSTITUITION("Max-Planck-Institut Informatik")

ADD_LICENSE("LGPL")

ADD_ICON(":/images_projectsXWu/images/projectsXWu.png")

// Add classes here: format is "class", "baseClass". class.h is included
// ADD_CLASS("SomeNewClass", "Persistent")

// Alternative: specify file to include for this declaration
// ADD_CLASS_INC("SomeNewClass", "Persistent", "AHeaderWithManyClasses.h")

// general settings
ADD_CLASS("ProjectSettings", "Persistent")

// unit test
ADD_CLASS("PCITestCentr", "PCInteractionTool")

// ccsg symmetry
ADD_CLASS("PCISymmDet", "PCInteractionTool")
ADD_CLASS("PCIConsistentCrossShapeGrammar", "PCInteractionTool")
ADD_CLASS_INC("ConsistentCrossShapeGrammarSettings", "AttachedData", "PCIConsistentCrossShapeGrammar.h")
ADD_CLASS_INC("CCSGClassification", "AttachedData", "PCIConsistentCrossShapeGrammar.h")
ADD_CLASS("CCSGSymmetryGroup", "AttachedData")
ADD_CLASS_INC("SymmDetContCCSG", "AttachedData", "SymmDetCont.h")
ADD_CLASS("PCISymmGroup", "PCInteractionTool")
ADD_CLASS_INC("SymmGroupSettings", "AttachedData", "PCISymmGroup.h")
ADD_CLASS_INC("SymmGroupClass", "AttachedData", "PCISymmGroup.h")

// symmetry deformation field
ADD_CLASS("PCISymmDefield", "PCInteractionTool")
ADD_CLASS("DefieldSymm", "AttachedData")
    ADD_CLASS_INC("DefieldSymmVec", "AttachedData", "DefieldSymm.h")
    ADD_CLASS_INC("DefieldSymmReflection", "DefieldSymm", "DefieldSymm.h")
ADD_CLASS("DefieldSymmRotation", "DefieldSymm")
    ADD_CLASS_INC("DefieldSymmRotationH", "DefieldSymmRotation", "DefieldSymmRotation.h")
    ADD_CLASS_INC("DefieldSymmRotationV", "DefieldSymmRotation", "DefieldSymmRotation.h")
ADD_CLASS("DefieldSymmDihedral", "DefieldSymm")
    ADD_CLASS_INC("DefieldSymmDihedralH", "DefieldSymmDihedral", "DefieldSymmDihedral.h")
ADD_CLASS_INC("PointSymmBlock", "AttachedData", "SamplerSymmBlock.h")
ADD_CLASS_INC("PointSymmBlockVec", "AttachedData", "SamplerSymmBlock.h")
ADD_CLASS_INC("LineSymmBlock", "AttachedData", "SamplerSymmBlock.h")
ADD_CLASS_INC("LineSymmBlockVec", "AttachedData", "SamplerSymmBlock.h")
ADD_CLASS("SymmSampleGraph", "AttachedData")

// procedural graph-cut
ADD_CLASS("PCIMSFeatureAlign", "PCInteractionTool")
ADD_CLASS_INC("PCIMSFeatureAlignSettings", "AttachedData", "PCIMSFeatureAlign.h")
ADD_CLASS("PCICosyn", "PCInteractionTool")
ADD_CLASS("C13DescriptorAttachment", "AttachedData")
ADD_CLASS_INC("SimpleFeaturePointCloudSmoother", "Persistent", "MultiScalePCCreator.h")
ADD_CLASS("MultiScalePCCreator", "Persistent")
ADD_CLASS("C13MultiScaleFeatureDetector", "Persistent")
ADD_CLASS_INC("C13MSFeatDetectCrossPoint", "C13MultiScaleFeatureDetector", "C13MultiScaleFeatureDetector.h")
ADD_CLASS("C13MultiScaleFeatureDescriptor", "Persistent")
ADD_CLASS_INC("C13MSFeatDescrHOG", "C13MultiScaleFeatureDescriptor", "C13MultiScaleFeatureDescriptor.h")
ADD_CLASS("PlaneCluster", "Persistent")
ADD_CLASS_INC("PlaneClusterExtractor", "AttachedData", "PlaneCluster.h")
ADD_CLASS("GridLabelAttachment", "AttachedData")

// CVT
//ADD_CLASS("PCILpCVT", "PCInteractionTool")

// statistical procedural
ADD_CLASS("PCIStatiCosyn", "PCInteractionTool")

// co-occurrence repetition detection
ADD_CLASS("PCISupres", "PCInteractionTool")
ADD_CLASS("PCISelDelMark", "PCInteractionTool")
ADD_CLASS("PCIEditSyn", "PCInteractionTool")

// Symmetry-aware Template Deformation and Fitting
ADD_CLASS("PCIConstrainedICP", "PCInteractionTool")
ADD_CLASS("CICPHandle", "Handle3D")
ADD_CLASS("CICPPersistentHandles", "Persistent")

// CL 3DRepetitionGroundTruth2015
ADD_CLASS("PCICLPointCloudProcessing", "PCInteractionTool")
ADD_CLASS("PCICL3DRepetitionGroundTruth2015", "PCInteractionTool")
ADD_CLASS_INC("PCICL3DRepetitionGroundTruth2015Editor", "ClassEditor", "PCICL3DRepetitionGroundTruth2015EditorWidget.h")
	