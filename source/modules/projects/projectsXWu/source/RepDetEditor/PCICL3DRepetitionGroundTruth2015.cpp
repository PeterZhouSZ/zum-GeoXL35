//---------------------------------------------------------------------------
#include"StdAfx.h"
//---------------------------------------------------------------------------
#include "PCICL3DRepetitionGroundTruth2015.h"
#include "BinaryObjectStreams.h"
#include "PCCComputeTopology.h"
#include "InCorePCTopologyGraph.h"
#include "PointCloudExporter.h"
#include "PointCloudImporterReaderCustomASCII_standard.h"
#include "PointCloudImporterAdder.h"
#include "CmdObjSelection.h"
#include "SelectionTools.h"
#include "ExaminerCameraController.h"
#include "GeneralKey.h"
#include "PCICameraController.h"
#include "GeometricTools.h"
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include "PCICLPointCloudProcessing.h"
#include "SGListNode.h"
#include "PCCResampler.h"
#include "SceneGraphState.h"
#include "PointSetKNNQuery.h"
#include "FileDialogs.h"
#include "InterfaceGlobals.h"
#include "CommandHistory.h"

//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

//static float roundf(float x)
//{
//	return x >= 0.0f ? floorf(x + 0.5f) : ceilf(x - 0.5f);
//}

IMPLEMENT_X4_CLASS(PCICL3DRepetitionGroundTruth2015, 0)
{
    BEGIN_CLASS_INIT(PCICL3DRepetitionGroundTruth2015);
	
    //ADD_NOARGS_METHOD(loadGT);
    //ADD_NOARGS_METHOD(saveGT);
}

PCICL3DRepetitionGroundTruth2015::PCICL3DRepetitionGroundTruth2015(void)
{
	// name
	pcName = "";
	dumpName = "C:/Chuan/3DRepetitionBenchmark2015/dump/";

	m_unit_absDiagonal = 100;
	m_unit_relRadiusFullres = 0.00005f;
	
	m_preprocess_NormalPCACamera_numNN = 50;
	m_preprocess_NormalPCA_numNN = 100;
	m_preprocess_split_v = makeVector3i(3, 6, 1);
	m_sw_numBasis = 18;
	m_sw_sigma = 0.5f;
	m_localsnapping_numCandidate = 16;

	// UI
	start_x = 0;
	start_y = 0;
	ST_brushsize = 12; /// size of the brush in pixels
	ST_tolerance = 12; /// tolerance value for color attrib selection
	ST_mode = SELMODE_BRUSHQUAD; /// selection mode which is currently activated. 0=Cloud, 1=Point, 2=CircleBrush, 3=RectangleArea, 4=QuadBrush, 5=CircleArea
	ST_ctrlPressed = false; /// true while ctrl key is pressed. Used for deselection mode.
	ST_SurfaceMode = true; /// true if surface mode is activated. See corresponding X4 help file for details of surface mode.
	ST_SelColor = makeVector3f(0.0f, 0.0f, 0.0f); /// color for selection by color attrib
	w = 100;
	h = 100;
	WidthToHeight = 1.0f;
	leftMBdown = false;
	rightMBdown = false;
	leftMBdownX = 0;
	leftMBdownY = 0;
	leftMBmoveY = 0;
	leftMBmoveX = 0;
	rightMBdownX = 0;
	rightMBdownY = 0;
	rightMBmoveY = 0;
	rightMBmoveX = 0;

	// UI mode
	flag_createtemplate = false;
	flag_editbox = false;
	flag_manualdetect = false;
	flag_manualcreateobject = false;

	pcFullres = new UnstructuredInCorePointCloud;
	VertexDescriptor * m_vd = new VertexDescriptor;
	m_vd->pushAttrib(mVAD("position", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("normal", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("color", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("flags", 1, VAD::DATA_FORMAT_INT32));
	pcFullres->clearAndSetup(m_vd, 0);

	pcFeature = new UnstructuredInCorePointCloud;
	pcFeature->clearAndSetup(m_vd, 0);

	boxTemplate = new Box;
	boxCurrent = new Box;
	boxRecord.resize(0);
	i_classCurrent = -1;
	i_boxCurrent = -1;
	generateColor(colorList, 100);

	translateEdit = makeVector3f(0, 0, 0);
	rotateEdit = makeVector3f(0, 0, 0);

	initializeParameters();
}

PCICL3DRepetitionGroundTruth2015::~PCICL3DRepetitionGroundTruth2015(void)
{

}

// preprocess IGN
void PCICL3DRepetitionGroundTruth2015::PreprocessIGN()
{
	debugOutput << "PCICL3DRepetitionGroundTruth2015::PreprocessIGN() called.\n";
	SGListNode* rootList = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(scene, "root"));
	UnstructuredInCorePointCloud *pcFullres;
	SGObjectNode * pcFullresNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pc1"));
	if (!pcFullresNode)
	{
		warning("PCICLGroundTruth2015::PreprocessIGN() - no fullres point cloud.");
		return;
	}
	pcFullres = dynamic_cast<UnstructuredInCorePointCloud*>(pcFullresNode->getSceneObject());

	// compute normal using camera origin
	PCICLPointCloudProcessing *myPCProcessor = new PCICLPointCloudProcessing;
	myPCProcessor->computeNormalPCACamera(pcFullres, m_preprocess_NormalPCACamera_numNN);
	delete myPCProcessor;

	// remove origin channel
	VertexDescriptor * m_vd = new VertexDescriptor;
	m_vd->pushAttrib(mVAD("position", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("normal", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("color", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("flags", 1, VAD::DATA_FORMAT_INT32));
	UnstructuredInCorePointCloud *pcFullres_copy = new UnstructuredInCorePointCloud;
	pcFullres_copy->clearAndSetup(m_vd, pcFullres->getNumPoints());
	AAT POSITION = pcFullres_copy->getAAT("position");
	AAT COLOR = pcFullres_copy->getAAT("color");
	AAT NORMAL = pcFullres_copy->getAAT("normal");
	AAT FLAGS = pcFullres_copy->getAAT("flags");
	for (mpcard i = 0; i < pcFullres_copy->getNumPoints(); i++)
	{
		pcFullres_copy->getPointSet()->set3f(i, POSITION, pcFullres->getPointSet()->get3f(i, POSITION));
		pcFullres_copy->getPointSet()->set3f(i, COLOR, pcFullres->getPointSet()->get3f(i, COLOR));
		pcFullres_copy->getPointSet()->set3f(i, NORMAL, pcFullres->getPointSet()->get3f(i, NORMAL));
		pcFullres_copy->getPointSet()->set1i(i, FLAGS, pcFullres->getPointSet()->get1i(i, FLAGS));
	}
	deletePointCloud(scene, "root/pc1");
	addPointCloud(scene, pcFullres_copy, "root/pc1");
}

void PCICL3DRepetitionGroundTruth2015::removeoutlier()
{
	debugOutput << "PCICL3DRepetitionGroundTruth2015::removeoutlier() called.\n";
	SGListNode* rootList = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(scene, "root"));
	SGObjectNode * pcFullresNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pc1"));
	if (!pcFullresNode)
	{
		warning("PCICLGroundTruth2015::detect_rigid() - no fullres point cloud.");
		return;
	}
	pcFullres = dynamic_cast<UnstructuredInCorePointCloud*>(pcFullresNode->getSceneObject());

	// clean the point cloud 

	// remove outliers
	PCICLPointCloudProcessing *myPCProcessor = new PCICLPointCloudProcessing;
	myPCProcessor->removeOutlier_dist2KNN(pcFullres, 32, 0.5f);
	delete myPCProcessor;
}

void PCICL3DRepetitionGroundTruth2015::splitdata()
{
	debugOutput << "PCICL3DRepetitionGroundTruth2015::splitdata() called.\n";
	SGListNode* rootList = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(scene, "root"));
	UnstructuredInCorePointCloud *pcFullres;
	SGObjectNode * pcFullresNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pc1"));
	if (!pcFullresNode)
	{
		warning("PCICLGroundTruth2015::detect_rigid() - no fullres point cloud.");
		return;
	}
	pcFullres = dynamic_cast<UnstructuredInCorePointCloud*>(pcFullresNode->getSceneObject());

	// compute the volume of the point cloud
	BoundingBox3f * bb;
	bool posset = false;
	if (!pcFullres->providesAttribute("position")) return;
	AAT POSITION = pcFullres->getAAT("position");
	SceneObjectIterator *newIt = pcFullres->createIterator(SOIT(SOIT::CAP_BASIC_PC));
	if (newIt == NULL) return;
	BasicPointCloudIterator *basicIt = dynamic_cast<BasicPointCloudIterator*>(newIt);
	while (!basicIt->atEnd()) {
		Vector3f pos = basicIt->get3f(POSITION);
		if (posset) {
			bb->addPoint(pos);
		}
		else {
			bb = new BoundingBox3f(pos);
			posset = true;
		}
		basicIt->next();
	};
	delete newIt;
	Vector3f cen = bb->getCenter();
	Vector3f dim = bb->getSideLength();

	// compute all the cells
	Vector3f model_corner_lower = cen - dim * 0.5f;
	Vector3f model_corner_upper = cen + dim * 0.5f;

	float32 min_x = model_corner_lower[0];
	float32 min_y = model_corner_lower[1];
	float32 min_z = model_corner_lower[2];
	float32 max_x = model_corner_upper[0];
	float32 max_y = model_corner_upper[1];
	float32 max_z = model_corner_upper[2];

	//min_x = ceil(min_x / (m_unit_absDiagonal * m_unit_relSizeVoxel)) * (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//min_y = ceil(min_y / (m_unit_absDiagonal * m_unit_relSizeVoxel)) * (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//min_z = ceil(min_z / (m_unit_absDiagonal * m_unit_relSizeVoxel)) * (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//max_x = ceil(max_x / (m_unit_absDiagonal * m_unit_relSizeVoxel)) * (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//max_y = ceil(max_y / (m_unit_absDiagonal * m_unit_relSizeVoxel)) * (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//max_z = ceil(max_z / (m_unit_absDiagonal * m_unit_relSizeVoxel)) * (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//model_corner_lower = makeVector3f(min_x, min_y, min_z);
	//model_corner_upper = makeVector3f(max_x, max_y, max_z);
	//int nVoxelsX_input = (max_x - min_x) / (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//int nVoxelsY_input = (max_y - min_y) / (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//int nVoxelsZ_input = (max_z - min_z) / (m_unit_absDiagonal * m_unit_relSizeVoxel);
	//int nVoxelsXYZ_input = nVoxelsX_input * nVoxelsY_input * nVoxelsZ_input;
	//Vector3f step_split = makeVector3f(ceil(nVoxelsX_input / float32(m_preprocess_split_v[0])), ceil(nVoxelsY_input / float32(m_preprocess_split_v[1])), ceil(nVoxelsZ_input / float32(m_preprocess_split_v[2])));
	
	Vector3f step_split = makeVector3f((max_x - min_x) / float32(m_preprocess_split_v[0]), (max_y - min_y) / float32(m_preprocess_split_v[1]), (max_z - min_z) / float32(m_preprocess_split_v[2]));

	// TO do: parallel this 
	// split 
	int32 total_split = m_preprocess_split_v[0] * m_preprocess_split_v[1] * m_preprocess_split_v[2];

#ifdef USEOPENMP
	omp_set_dynamic(0);     // Explicitly disable dynamic teams
	omp_set_num_threads(NUMCORE); // Use customized threads for all consecutive parallel regions
#pragma omp parallel for schedule(static)
#endif
	for (int i_split = 0; i_split < total_split; i_split++)
	{
		int32 z_split = i_split % m_preprocess_split_v[2];
		int32 y_split = (i_split - z_split) % (m_preprocess_split_v[1] * m_preprocess_split_v[2]);
		int32 x_split = (i_split - z_split - y_split * m_preprocess_split_v[2]) / (m_preprocess_split_v[1] * m_preprocess_split_v[2]);

		UnstructuredInCorePointCloud * pcSplit = new UnstructuredInCorePointCloud;
		float32 min_x_ = model_corner_lower[0] + float32(x_split) * step_split[0];
		float32 min_y_ = model_corner_lower[1] + float32(y_split) * step_split[1];
		float32 min_z_ = model_corner_lower[2] + float32(z_split) * step_split[2];
		float32 max_x_ = model_corner_lower[0] + float32(x_split + 1) * step_split[0];
		float32 max_y_ = model_corner_lower[1] + float32(y_split + 1) * step_split[1];
		float32 max_z_ = model_corner_lower[2] + float32(z_split + 1) * step_split[2];

		pcSplit = dynamic_cast<UnstructuredInCorePointCloud *>(pcFullres->copy());
		AAT FLAGS = pcSplit->getAAT("flags");
		AAT POSITION = pcSplit->getAAT("position");
		SceneObjectIterator* it = pcSplit->createIterator(SOIT(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC | SOIT::CAP_DYNAMIC_PC));
		BasicPointCloudIterator* basicIt = dynamic_cast<BasicPointCloudIterator*>(it);
		DynamicPointCloudIterator* dynIt = dynamic_cast<DynamicPointCloudIterator*>(it);
		ModifyablePointCloudIterator* modIt = dynamic_cast<ModifyablePointCloudIterator*>(it);

		basicIt->reset();
		while (!basicIt->atEnd()) {
			modIt->set1i(FLAGS, 0);
			basicIt->next();
		}
		basicIt->reset();
		while (!basicIt->atEnd()) {
			const Vector3f& p = basicIt->get3f(POSITION);
			if (p[0] < min_x_ || p[0] > max_x_ || p[1] < min_y_ || p[1] > max_y_ || p[2] < min_z_ || p[2] > max_z_)
				modIt->set1i(FLAGS, 1);
			basicIt->next();
		}
		basicIt->reset();
		while (!basicIt->atEnd()) {
			if (basicIt->get1i(FLAGS) == 1)
				dynIt->deleteCurrent();
			else
				basicIt->next();
		}
		basicIt->unlock();
		delete it;

		std::stringstream ss;
		ss << i_split;
		std::string filename_out = dumpName + pcName + "_" + ss.str() + ".points";
		std::ofstream f(filename_out.c_str());
		PointCloudExporter exporter(pcSplit, &f);
		exporter.exportPointCloud();
		f << endl;
		delete pcSplit;
	}
}

void PCICL3DRepetitionGroundTruth2015::points2x4obj()
{
    QDirIterator dirIt(QString::fromStdString(dumpName));

	int num_model = 0;
	while (dirIt.hasNext()) {
		dirIt.next();
		if (!QFileInfo(dirIt.filePath()).isFile()) continue;
		if (QFileInfo(dirIt.filePath()).suffix() != "points") continue;
		const QString& objF = dirIt.filePath();
		std::string cur_pcName = QFileInfo(dirIt.fileName()).baseName().toUtf8().constData();

		UnstructuredInCorePointCloud* pcFullres = new UnstructuredInCorePointCloud;

		PointCloudImporterReaderCustomASCII_standard * importer = new PointCloudImporterReaderCustomASCII_standard;
		if (importer->beginImport(objF.toUtf8().constData(), true))
		{
			VertexDescriptor * m_vd = new VertexDescriptor;
			m_vd->pushAttrib(mVAD("position", 3, VAD::DATA_FORMAT_FLOAT32));
			m_vd->pushAttrib(mVAD("normal", 3, VAD::DATA_FORMAT_FLOAT32));
			m_vd->pushAttrib(mVAD("color", 3, VAD::DATA_FORMAT_FLOAT32));
			m_vd->pushAttrib(mVAD("flags", 1, VAD::DATA_FORMAT_INT32));
			pcFullres->clearAndSetup(m_vd, 0);
			PointCloudImporterAdderDynamic * adder = new PointCloudImporterAdderDynamic;
			adder->doImport(importer, pcFullres);
			delete adder;
		}
		// import file
		if (pcFullres == nullptr) {
			warning(std::string("Cannot read file: ") + objF.toUtf8().constData());
			return;
		}
		else {
			addPointCloud(scene, pcFullres, "root/pcFullres");
			pcFullres->setMaterialIndex(1);
			getScene()->rebuildRenderObjects();
			
		}
		delete importer;

		// resample ------------------------------------------
		{
			PCCResampler::resamplePC(pcFullres, m_unit_absDiagonal * m_unit_relRadiusFullres, false);
		}

		// save scene ------------------------------------------
		{
			getScene()->rebuildRenderObjects();
			std::string dirname = dumpName + cur_pcName;
			mkdir(dirname.c_str());
			std::string filename = dumpName + cur_pcName + '/' + cur_pcName + ".x4obj";
			BinaryOutputObjectStream out(filename.c_str());
			out.writeObject((Persistent*)scene);
		}

		// clear scene ****************************************************
		{
			SGListNode* rootList = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(scene, "root"));
			rootList->clear();
			getScene()->rebuildRenderObjects();
			updateSceneView();
		}
	}
}

// preprocess General
void PCICL3DRepetitionGroundTruth2015::PreprocessGeneral()
{
	debugOutput << "PCICL3DRepetitionGroundTruth2015::PreprocessGeneral() called.\n";
	SGListNode* rootList = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(scene, "root"));
	UnstructuredInCorePointCloud *pcFullres;
	SGObjectNode * pcFullresNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pc1"));
	if (!pcFullresNode)
	{
		warning("PCICLGroundTruth2015::PreprocessGeneral() - no fullres point cloud.");
		return;
	}
	pcFullres = dynamic_cast<UnstructuredInCorePointCloud*>(pcFullresNode->getSceneObject());

	PCCResampler::resamplePC(pcFullres, m_unit_absDiagonal * m_unit_relRadiusFullres, false);

	// rename
	VertexDescriptor * m_vd = new VertexDescriptor;
	m_vd->pushAttrib(mVAD("position", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("normal", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("color", 3, VAD::DATA_FORMAT_FLOAT32));
	m_vd->pushAttrib(mVAD("flags", 1, VAD::DATA_FORMAT_INT32));
	UnstructuredInCorePointCloud *pcFullres_copy = new UnstructuredInCorePointCloud;
	pcFullres_copy->clearAndSetup(m_vd, pcFullres->getNumPoints());
	AAT POSITION = pcFullres_copy->getAAT("position");
	AAT COLOR = pcFullres_copy->getAAT("color");
	AAT NORMAL = pcFullres_copy->getAAT("normal");
	AAT FLAGS = pcFullres_copy->getAAT("flags");

	AAT POSITION_old = pcFullres->getAAT("position");
	AAT COLOR_old = pcFullres->getAAT("color");
	AAT NORMAL_old = pcFullres->getAAT("normal");
	AAT FLAGS_old = pcFullres->getAAT("flags");
	for (mpcard i = 0; i < pcFullres_copy->getNumPoints(); i++)
	{
		pcFullres_copy->getPointSet()->set3f(i, POSITION, pcFullres->getPointSet()->get3f(i, POSITION_old));
		pcFullres_copy->getPointSet()->set3f(i, COLOR, pcFullres->getPointSet()->get3f(i, COLOR_old));
		pcFullres_copy->getPointSet()->set3f(i, NORMAL, pcFullres->getPointSet()->get3f(i, NORMAL_old));
		pcFullres_copy->getPointSet()->set1i(i, FLAGS, pcFullres->getPointSet()->get1i(i, FLAGS_old));
	}
	deletePointCloud(scene, "root/pc1");
	addPointCloud(scene, pcFullres_copy, "root/pcFullres");
}

void PCICL3DRepetitionGroundTruth2015::flipyz()
{
	std::vector<std::string> selection = scene->getRootState()->staticState->getSelectedNodes();
	if (selection.size() < 1)
	{
		warning("PCICLDetectionBenchmark::flipyz() - no point cloud selected.");
		return;
	}

	if (dynamic_cast<UnstructuredInCoreTriangleMesh*>(getPointCloud(scene, selection[0]))) {
		// trianbglemesh
		UnstructuredInCoreTriangleMesh* utm = dynamic_cast<UnstructuredInCoreTriangleMesh*>(getPointCloud(scene, selection[0]));
		AAT upcposAAT = utm->getAAT("position");
		AAT upcnormalAAT = utm->getAAT("normal");
		for (card32 ii = 0; ii < utm->getNumPoints(); ii++)
		{
			Vector3f tmp_position = utm->getPointSet()->get3f(ii, upcposAAT);
			Vector3f tmp_normal = utm->getPointSet()->get3f(ii, upcnormalAAT);
			utm->getPointSet()->set3f(ii, upcposAAT, makeVector3f(tmp_position[0], -tmp_position[2], tmp_position[1]));
			utm->getPointSet()->set3f(ii, upcnormalAAT, makeVector3f(tmp_normal[0], -tmp_normal[2], tmp_normal[1]));
		}
	}
	else{
		// point cloud
		UnstructuredInCorePointCloud* upc = dynamic_cast<UnstructuredInCorePointCloud*>(getPointCloud(scene, selection[0]));
		AAT upcposAAT = upc->getAAT("position");
		AAT upcnormalAAT = upc->getAAT("normal");
		for (card32 ii = 0; ii < upc->getNumPoints(); ii++)
		{
			Vector3f tmp_position = upc->getPointSet()->get3f(ii, upcposAAT);
			Vector3f tmp_normal = upc->getPointSet()->get3f(ii, upcnormalAAT);
			upc->getPointSet()->set3f(ii, upcposAAT, makeVector3f(tmp_position[0], -tmp_position[2], tmp_position[1]));
			upc->getPointSet()->set3f(ii, upcnormalAAT, makeVector3f(tmp_normal[0], -tmp_normal[2], tmp_normal[1]));
		}
	}
}

void PCICL3DRepetitionGroundTruth2015::computeDiagonal()
{
	debugOutput << "PCICL3DRepetitionGroundTruth2015::computeDiagonal() called.\n";
	SGListNode* rootList = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(scene, "root"));
	UnstructuredInCorePointCloud *pcFullres;
	SGObjectNode * pcFullresNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pcFullres"));
	if (!pcFullresNode)
	{
		warning("PCICLGroundTruth2015::computeDiagonal() - no fullres point cloud.");
		return;
	}
	pcFullres = dynamic_cast<UnstructuredInCorePointCloud*>(pcFullresNode->getSceneObject());
	BoundingBox3f bb = getPCBBox(pcFullres);
	debugOutput << "diagonal length: " << bb.getDiagonalLength() << "\n";
}


// Create GT
void PCICL3DRepetitionGroundTruth2015::initialdatastructure()
{
	initializeParameters();

	SGListNode* rootList = dynamic_cast<SGListNode*>(getSceneGraphNodeByName(getScene(), "root"));
	SGObjectNode * pcFullresNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pcFullres"));
	if (!pcFullresNode)
	{
		warning("PCICLGroundTruth2015::PCICL3DRepetitionGroundTruth2015() - no fullres point cloud.");
		pcFullres = NULL;
		return;
	}
	else
	{
		pcFullres = new UnstructuredInCorePointCloud;
		pcFullres = dynamic_cast<UnstructuredInCorePointCloud*>(pcFullresNode->getSceneObject());
	}

	pcFeature = new UnstructuredInCorePointCloud;
	pcFeature = dynamic_cast<UnstructuredInCorePointCloud*>(pcFullres->copy());
	PCCResampler::resamplePC(pcFeature, m_unit_absDiagonal * m_unit_relRadiusFeature, false);
	
	knnOctreeFeature = PointSetKNNQuery::createOctree(pcFeature->getPointSet());
	spherequeryFeature = new FastSphereQuerry(knnOctreeFeature, pcFeature->getPointSet());
	knnItFeature = new HierarchicalKNNIterator(pcFeature, 16, NULL);

	addPointCloud(scene, pcFeature, "root/pcFeature");
	pcFeature->setMaterialIndex(1);
	SGObjectNode * pcFeatureNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pcFeature"));
	pcFeatureNode->setVisible(true);
	pcFullresNode->setVisible(true);
	getScene()->rebuildRenderObjects();

	// maybe automatically load result txt file here

	// render the template and the detection in different styles
	
}

void PCICL3DRepetitionGroundTruth2015::saveGT()
{
	std::string filename = FileDialogs::getSaveFileName(InterfaceGlobals::getMainWindow(), "save GT", "*.box");
	if (filename != "")
	{
		std::ofstream fs;
		std::string line;
		fs.open(filename.c_str(), std::ios::binary);
		fs << "# .gt v0.0 - 3D Repetition Benchmark box format\n";
		fs << "# basis[0][0] basis[0][1] basis[0][2] basis[1][0] basis[1][1] basis[1][2] basis[2][0] basis[2][1] basis[2][2] coeffs[0] coeffs[1] coeffs[2] centroid[0] centroid[1] centroid[2] cenref[0] cenref[1] cenref[2] radiusref label" << "\n";
		fs << boxRecord.size() << "\n";
		for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++) {
			fs << boxRecord[i_class].size() << "\n";
		}
		for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
		{
			for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
			{
				fs << boxRecord[i_class][i_box]->basis[0][0] << " " << boxRecord[i_class][i_box]->basis[0][1] << " " << boxRecord[i_class][i_box]->basis[0][2] << " ";
				fs << boxRecord[i_class][i_box]->basis[1][0] << " " << boxRecord[i_class][i_box]->basis[1][1] << " " << boxRecord[i_class][i_box]->basis[1][2] << " ";
				fs << boxRecord[i_class][i_box]->basis[2][0] << " " << boxRecord[i_class][i_box]->basis[2][1] << " " << boxRecord[i_class][i_box]->basis[2][2] << " ";
				fs << boxRecord[i_class][i_box]->coeffs[0] << " " << boxRecord[i_class][i_box]->coeffs[1] << " " << boxRecord[i_class][i_box]->coeffs[2] << " ";
				fs << boxRecord[i_class][i_box]->centroid[0] << " " << boxRecord[i_class][i_box]->centroid[1] << " " << boxRecord[i_class][i_box]->centroid[2] << " ";
				fs << boxRecord[i_class][i_box]->cenref[0] << " " << boxRecord[i_class][i_box]->cenref[1] << " " << boxRecord[i_class][i_box]->cenref[2] << " ";
				fs << boxRecord[i_class][i_box]->radiusref << " ";
				fs << boxRecord[i_class][i_box]->label;
				fs << "\n";
			}
		}
		fs.close();
	}
}

void PCICL3DRepetitionGroundTruth2015::loadGT()
{
	std::string filename = FileDialogs::getOpenFileName(InterfaceGlobals::getMainWindow(), "load GT",
		"*.box");
	std::ifstream fs;
	std::string line;
	fs.open(filename.c_str(), std::ios::binary);
	if (!fs.is_open() || fs.fail()) {
		error("Failed to open file...");
		return;
	}

	std::vector<std::string> st;
	// skip the first two lines
	getline(fs, line);
	getline(fs, line);

	// read data
	getline(fs, line);
	boost::trim(line);
	boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
	float32 value;
	value = atof(st.at(0).c_str());
	boxRecord.resize((int32)value);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		getline(fs, line);
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
		value = atof(st.at(0).c_str());
		boxRecord[i_class].resize((int32)value);
	}
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			getline(fs, line);
			boost::trim(line);
			boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);
			boxRecord[i_class][i_box] = new Box;
			boxRecord[i_class][i_box]->basis[0][0] = atof(st.at(0).c_str());
			boxRecord[i_class][i_box]->basis[0][1] = atof(st.at(1).c_str());
			boxRecord[i_class][i_box]->basis[0][2] = atof(st.at(2).c_str());
			boxRecord[i_class][i_box]->basis[1][0] = atof(st.at(3).c_str());
			boxRecord[i_class][i_box]->basis[1][1] = atof(st.at(4).c_str());
			boxRecord[i_class][i_box]->basis[1][2] = atof(st.at(5).c_str());
			boxRecord[i_class][i_box]->basis[2][0] = atof(st.at(6).c_str());
			boxRecord[i_class][i_box]->basis[2][1] = atof(st.at(7).c_str());
			boxRecord[i_class][i_box]->basis[2][2] = atof(st.at(8).c_str());
			boxRecord[i_class][i_box]->coeffs[0] = atof(st.at(9).c_str());
			boxRecord[i_class][i_box]->coeffs[1] = atof(st.at(10).c_str());
			boxRecord[i_class][i_box]->coeffs[2] = atof(st.at(11).c_str());
			boxRecord[i_class][i_box]->centroid[0] = atof(st.at(12).c_str());
			boxRecord[i_class][i_box]->centroid[1] = atof(st.at(13).c_str());
			boxRecord[i_class][i_box]->centroid[2] = atof(st.at(14).c_str());
			boxRecord[i_class][i_box]->cenref[0] = atof(st.at(15).c_str());
			boxRecord[i_class][i_box]->cenref[1] = atof(st.at(16).c_str());
			boxRecord[i_class][i_box]->cenref[2] = atof(st.at(17).c_str());
			boxRecord[i_class][i_box]->radiusref = atof(st.at(18).c_str());
			boxRecord[i_class][i_box]->label = atof(st.at(19).c_str());
		}
	}
	fs.close();

	// initialdatastructure
	initialdatastructure();

	// render the record
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
		}
	}
	debugRenderer->endRenderJob();
}

// Create GT (Semi Auto)
void PCICL3DRepetitionGroundTruth2015::createtempalte()
{
	translateEdit = makeVector3f(0, 0, 0);
	rotateEdit = makeVector3f(0, 0, 0);
	flag_createtemplate = true;
	flag_manualdetect = false;
	flag_manualcreateclass = false;
	flag_manualcreateobject = false;

	debugOutput << "createtempalte activated\n";

	SGObjectNode * pcFullresNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pcFullres"));
	if (!pcFullresNode)
	{
		warning("PCICL3DRepetitionGroundTruth2015::createtempalte() - no pcFullres.");
		pcFullres = NULL;
		return;
	}
	else
	{
		pcFullresNode->setVisible(true);
	}
	SGObjectNode * pcFeatureNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pcFeature"));
	if (!pcFeatureNode)
	{
		warning("PCICL3DRepetitionGroundTruth2015::createtempalte() - no pcFeature.");
		pcFeature = NULL;
		return;
	}
	else
	{
		pcFeatureNode->setVisible(true);
	}

	// check if the latest class is empty and if there is no class
	if (boxRecord.size() > 0 && boxRecord[boxRecord.size() - 1].size() == 0)
	{
		i_classCurrent = (int32)boxRecord.size() - 1;
		boxTemplate = new Box;
		boxCurrent = new Box;
		debugOutput << "use the last class, current number of classes: " << boxRecord.size() << "\n";
	}
	else{
		boxRecord.resize(boxRecord.size() + 1);
		boxRecord[boxRecord.size() - 1].resize(0);
		i_classCurrent = (int32)boxRecord.size() - 1;
		boxTemplate = new Box;
		boxCurrent = new Box;
		debugOutput << "new class created, current number of classes: " << boxRecord.size() << "\n";
	}
}

void PCICL3DRepetitionGroundTruth2015::manualdetect()
{
	translateEdit = makeVector3f(0, 0, 0);
	rotateEdit = makeVector3f(0, 0, 0);
	flag_createtemplate = false;
	flag_manualdetect = true;
	flag_manualcreateclass = false;
	flag_manualcreateobject = false;

	debugOutput << "manualdetect activated\n";

	SGObjectNode * pcFeatureNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pcFeature"));
	if (!pcFeatureNode)
	{
		warning("PCICLGroundTruth2015::PCICL3DRepetitionGroundTruth2015() - no pcFeature.");
		pcFeature = NULL;
		return;
	}
	else
	{
		pcFeatureNode->setVisible(false);
	}

	// pcTemplate -----------------------------------------
	float32 radius = boxTemplate->coeffs.getNorm();
	AAT POSITION = pcFeature->getAAT("position");
	AAT NORMAL = pcFeature->getAAT("normal");
	AAT COLOR = pcFeature->getAAT("color");
	AAT FLAGS = pcFeature->getAAT("flags");
	mpcard numAllPointsBox;
	mpcard *allIndicesBox;
	spherequeryFeature->querry(boxTemplate->centroid, radius, &allIndicesBox, numAllPointsBox);
	pcTemplate = new UnstructuredInCorePointCloud;
	pcTemplate->clearAndSetup(pcFeature->getDescr(), 0);
	Matrix3f world2loc = invertMatrix(boxTemplate->basis);
	Vector3f boundary = makeVector3f(abs(boxTemplate->coeffs[0]), abs(boxTemplate->coeffs[1]), abs(boxTemplate->coeffs[2]));
	std::vector<char> vbuffer(pcFeature->getDescr()->getSize());
	for (mpcard i = 0; i < numAllPointsBox; i++) {
		Vector3f pos = pcFeature->getPointSet()->get3f(allIndicesBox[i], POSITION);
		Vector3f pos2center = pos - boxTemplate->centroid;
		if (checkpointinbox(pos2center, world2loc, boundary))
		{
			pcFeature->getPointSet()->getVertex(allIndicesBox[i], &vbuffer[0]);
			pcTemplate->getPointSet()->addPoint(&vbuffer[0]);
			pcTemplate->getPointSet()->set3f(pcTemplate->getNumPoints() - 1, COLOR, makeVector3f(1, 1, 1));
		}
	}
	for (mpcard i = 0; i < pcTemplate->getNumPoints(); i++)
	{
		pcTemplate->getPointSet()->set1i(i, FLAGS, 0);
	}
	pcTemplate->clearAttachments();
}

// Create GT (Manual)
void PCICL3DRepetitionGroundTruth2015::manualcreatenewclass()
{
	translateEdit = makeVector3f(0, 0, 0);
	rotateEdit = makeVector3f(0, 0, 0);
	flag_createtemplate = false;
	flag_manualdetect = false;
	flag_manualcreateclass = true;
	flag_manualcreateobject = false;

	debugOutput << "manualcreatenewclass activated\n";

	SGObjectNode * pcFullresNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pcFullres"));
	if (!pcFullresNode)
	{
		warning("PCICL3DRepetitionGroundTruth2015::createtempalte() - no pcFullres.");
		pcFullres = NULL;
		return;
	}
	else
	{
		pcFullresNode->setVisible(true);
	}
	SGObjectNode * pcFeatureNode = dynamic_cast<SGObjectNode*>(getSceneGraphNodeByName(getScene(), "root/pcFeature"));
	if (!pcFeatureNode)
	{
		warning("PCICL3DRepetitionGroundTruth2015::createtempalte() - no pcFeature.");
		pcFeature = NULL;
		return;
	}
	else
	{
		pcFeatureNode->setVisible(true);
	}

	// check if the latest class is empty and if there is no class
	if (boxRecord.size() > 0 && boxRecord[boxRecord.size() - 1].size() == 0)
	{
		i_classCurrent = (int32)boxRecord.size() - 1;
		boxTemplate = new Box;
		boxCurrent = new Box;
		debugOutput << "use the last class, current number of classes: " << boxRecord.size() << "\n";
	}
	else{
		boxRecord.resize(boxRecord.size() + 1);
		boxRecord[boxRecord.size() - 1].resize(0);
		i_classCurrent = (int32)boxRecord.size() - 1;
		boxTemplate = new Box;
		boxCurrent = new Box;
		debugOutput << "new class created, current number of classes: " << boxRecord.size() << "\n";
	}
}

void PCICL3DRepetitionGroundTruth2015::manualcreatenewobject()
{
	translateEdit = makeVector3f(0, 0, 0);
	rotateEdit = makeVector3f(0, 0, 0);
	flag_createtemplate = false;
	flag_manualdetect = false;
	flag_manualcreateclass = false;
	flag_manualcreateobject = true;
	debugOutput << "manualcreatenewobject activated\n";

}

// edit
void PCICL3DRepetitionGroundTruth2015::translateX(int32 step)
{
	translateEdit[0] = float32(step) * 0.01f;
	Matrix3f M_rotationZ;
	M_rotationZ[0] = makeVector3f(cos(rotateEdit[2]), sin(rotateEdit[2]), 0);
	M_rotationZ[1] = makeVector3f(-sin(rotateEdit[2]), cos(rotateEdit[2]), 0);
	M_rotationZ[2] = makeVector3f(0, 0, 1);

	if (i_classCurrent < 0)
		return;

	Box * boxEdit = new Box;
	*boxEdit = *boxRecord[i_classCurrent][i_boxCurrent];
	Vector3f offset = boxEdit->cenref - boxEdit->centroid;
	boxEdit->centroid = boxEdit->centroid + boxEdit->basis[0] * translateEdit[0] + boxEdit->basis[1] * translateEdit[1] + boxEdit->basis[2] * translateEdit[2];
	boxEdit->cenref = boxEdit->centroid + M_rotationZ * offset;
	boxEdit->basis = M_rotationZ * boxEdit->basis;

	// render the record
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			if (i_class == i_classCurrent && i_box == i_boxCurrent)
				continue;
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
		}
	}
	drawBox(boxEdit, colorList[i_classCurrent], 6);
	debugRenderer->endRenderJob();
	updateSceneView();
}

void PCICL3DRepetitionGroundTruth2015::translateY(int32 step)
{
	translateEdit[1] = float32(step) * 0.01f;

	Matrix3f M_rotationZ;
	M_rotationZ[0] = makeVector3f(cos(rotateEdit[2]), sin(rotateEdit[2]), 0);
	M_rotationZ[1] = makeVector3f(-sin(rotateEdit[2]), cos(rotateEdit[2]), 0);
	M_rotationZ[2] = makeVector3f(0, 0, 1);

	if (i_classCurrent < 0)
		return;

	Box * boxEdit = new Box;
	*boxEdit = *boxRecord[i_classCurrent][i_boxCurrent];
	Vector3f offset = boxEdit->cenref - boxEdit->centroid;
	boxEdit->centroid = boxEdit->centroid + boxEdit->basis[0] * translateEdit[0] + boxEdit->basis[1] * translateEdit[1] + boxEdit->basis[2] * translateEdit[2];
	boxEdit->cenref = boxEdit->centroid + M_rotationZ * offset;
	boxEdit->basis = M_rotationZ * boxEdit->basis;

	// render the record
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			if (i_class == i_classCurrent && i_box == i_boxCurrent)
				continue;
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
		}
	}
	drawBox(boxEdit, colorList[i_classCurrent], 6);
	debugRenderer->endRenderJob();
	updateSceneView();
}

void PCICL3DRepetitionGroundTruth2015::translateZ(int32 step)
{
	translateEdit[2] = float32(step) * 0.01f;

	Matrix3f M_rotationZ;
	M_rotationZ[0] = makeVector3f(cos(rotateEdit[2]), sin(rotateEdit[2]), 0);
	M_rotationZ[1] = makeVector3f(-sin(rotateEdit[2]), cos(rotateEdit[2]), 0);
	M_rotationZ[2] = makeVector3f(0, 0, 1);

	if (i_classCurrent < 0)
		return;

	Box * boxEdit = new Box;
	*boxEdit = *boxRecord[i_classCurrent][i_boxCurrent];
	Vector3f offset = boxEdit->cenref - boxEdit->centroid;
	boxEdit->centroid = boxEdit->centroid + boxEdit->basis[0] * translateEdit[0] + boxEdit->basis[1] * translateEdit[1] + boxEdit->basis[2] * translateEdit[2];
	boxEdit->cenref = boxEdit->centroid + M_rotationZ * offset;
	boxEdit->basis = M_rotationZ * boxEdit->basis;

	// render the record
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			if (i_class == i_classCurrent && i_box == i_boxCurrent)
				continue;
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
		}
	}
	drawBox(boxEdit, colorList[i_classCurrent], 6);
	debugRenderer->endRenderJob();
	updateSceneView();
}

void PCICL3DRepetitionGroundTruth2015::rotateZ(int32 step)
{
	rotateEdit[2] = float32(step) * 0.02f * (float32)M_PI * 2;
	Matrix3f M_rotationZ;
	M_rotationZ[0] = makeVector3f(cos(rotateEdit[2]), sin(rotateEdit[2]), 0);
	M_rotationZ[1] = makeVector3f(-sin(rotateEdit[2]), cos(rotateEdit[2]), 0);
	M_rotationZ[2] = makeVector3f(0, 0, 1);

	if (i_classCurrent < 0)
		return;

	Box * boxEdit = new Box;
	*boxEdit = *boxRecord[i_classCurrent][i_boxCurrent];
	Vector3f offset = boxEdit->cenref - boxEdit->centroid;
	boxEdit->centroid = boxEdit->centroid + boxEdit->basis[0] * translateEdit[0] + boxEdit->basis[1] * translateEdit[1] + boxEdit->basis[2] * translateEdit[2];
	boxEdit->cenref = boxEdit->centroid + M_rotationZ * offset;
	boxEdit->basis = M_rotationZ * boxEdit->basis;

	// render the record
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			if (i_class == i_classCurrent && i_box == i_boxCurrent)
				continue;
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
		}
	}
	drawBox(boxEdit, colorList[i_classCurrent], 6);
	debugRenderer->endRenderJob();
	updateSceneView();
}

// semantic labeling
void PCICL3DRepetitionGroundTruth2015::semanticlabelassign(int32 l)
{
	// grab the current class
	if (i_classCurrent > -1)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_classCurrent].size(); i_box++)
		{
			boxRecord[i_classCurrent][i_box]->label = l;
		}
	}
	else
	{
		warning("No box is selected\n");
	}
}

// rendering
void PCICL3DRepetitionGroundTruth2015::renderrigid()
{
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
		}
	}
	debugRenderer->endRenderJob();
}

void PCICL3DRepetitionGroundTruth2015::rendersemantic()
{
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[boxRecord[i_class][i_box]->label], linewidth);
		}
	}
	debugRenderer->endRenderJob();
}

// misc
void PCICL3DRepetitionGroundTruth2015::computeboxTemplate()
{
	if (pcFeature->getNumPoints() == 0)
	{
		warning("PCICLGroundTruth2015::computeboxTemplate() - no pcFeature point cloud.");
		return;
	}
	else
	{

		if (flag_manualcreateclass)
		{
			// find all points of the same label and show its bounding box
			AAT flagsAAT = pcFeature->getAAT("flags");
			AAT posAAT = pcFeature->getAAT("position");
			PCA3f pointPCA;
			card32 numPoints = 0;
			Vector3f cenref;
			std::vector<Vector3f> pointsProj;
			pointsProj.resize(0);
			for (mpcard i = 0; i < pcFeature->getNumPoints(); i++)
			{
				if (pcFeature->getPointSet()->get1i(i, flagsAAT) != 0)
				{
					Vector3f pos = pcFeature->getPointSet()->get3f(i, posAAT);
					pointPCA.addPoint(pos);
					if (numPoints == 0)
						cenref = pos;
					numPoints += 1;
					pointsProj.push_back(pos);
				}
			}
            if (0 == numPoints) return;

			Matrix3f pcabasis;
			Vector3f pcaeigenValues;
			Vector3f pcacentroid;
			pointPCA.analyze(pcaeigenValues, pcabasis, pcacentroid);
			pcabasis = invertMatrix(pcabasis);

			std::vector<float32> xProj;
			std::vector<float32> yProj;
			std::vector<float32> zProj;
			xProj.resize(numPoints);
			yProj.resize(numPoints);
			zProj.resize(numPoints);
			for (mpcard i = 0; i < numPoints; i++)
			{
				pointsProj[i] = pcabasis * (pointsProj[i] - pcacentroid);
				xProj[i] = pointsProj[i][0];
				yProj[i] = pointsProj[i][1];
				zProj[i] = pointsProj[i][2];
			}

			float32 xMin = *min_element(xProj.begin(), xProj.end());
			float32 xMax = *max_element(xProj.begin(), xProj.end());
			float32 yMin = *min_element(yProj.begin(), yProj.end());
			float32 yMax = *max_element(yProj.begin(), yProj.end());
			float32 zMin = *min_element(zProj.begin(), zProj.end());
			float32 zMax = *max_element(zProj.begin(), zProj.end());
			Matrix3f basis;
			Vector3f coeffs;
			Vector3f centroid;
			pcabasis = invertMatrix(pcabasis);
			basis[0] = pcabasis[0];
			basis[1] = pcabasis[1];
			basis[2] = pcabasis[2];
			centroid = pcacentroid;
			coeffs = makeVector3f((xMax - xMin) * 0.5f, (yMax - yMin) * 0.5f, (zMax - zMin) * 0.5f);

			// radiusref
			Matrix3f inversebasis = invertMatrix(basis);
			Vector3f offset = inversebasis * (cenref - centroid);

			Vector3f coeffsref = makeVector3f(abs(coeffs[0]) + abs(offset[0]), abs(coeffs[1]) + abs(offset[1]), abs(coeffs[2]) + abs(offset[2]));
			float32 radiusref = coeffsref.getNorm();

			radiusref = coeffs.getNorm();
			cenref = centroid;

			boxTemplate->basis = basis;
			boxTemplate->coeffs = coeffs;
			boxTemplate->centroid = centroid;
			boxTemplate->cenref = cenref;
			boxTemplate->radiusref = radiusref;

			boxCurrent->basis = basis;
			boxCurrent->coeffs = coeffs;
			boxCurrent->centroid = centroid;
			boxCurrent->cenref = cenref;
			boxCurrent->radiusref = radiusref;
		}
		else
		{
			// find all points of the same label and show its bounding box
			AAT flagsAAT = pcFeature->getAAT("flags");
			AAT posAAT = pcFeature->getAAT("position");
			PCA2f pointPCA;
			float32 min_z = 10000000.0f;
			float32 max_z = -10000000.0f;
			card32 numPoints = 0;
			Vector3f cenref;
			std::vector<Vector2f> pointsProj;
			pointsProj.resize(0);
			for (mpcard i = 0; i < pcFeature->getNumPoints(); i++)
			{
				if (pcFeature->getPointSet()->get1i(i, flagsAAT) != 0)
				{
					Vector3f pos = pcFeature->getPointSet()->get3f(i, posAAT);
					pointPCA.addPoint(makeVector2f(pos[0], pos[1]));
					if (pos[2] < min_z)
						min_z = pos[2];
					if (pos[2] > max_z)
						max_z = pos[2];
					if (numPoints == 0)
						cenref = pos;
					numPoints += 1;
					pointsProj.push_back(makeVector2f(pos[0], pos[1]));
				}
			}

			Matrix2f pca2basis;
			Vector2f pca2eigenValues;
			Vector2f pca2centroid;
			pointPCA.analyze(pca2eigenValues, pca2basis, pca2centroid);
			pca2basis = invertMatrix(pca2basis);

			std::vector<float32> xProj;
			std::vector<float32> yProj;
			xProj.resize(numPoints);
			yProj.resize(numPoints);
			for (mpcard i = 0; i < numPoints; i++)
			{
				pointsProj[i] = pca2basis * (pointsProj[i] - pca2centroid);
				xProj[i] = pointsProj[i][0];
				yProj[i] = pointsProj[i][1];
			}

			float32 xMin = *min_element(xProj.begin(), xProj.end());
			float32 xMax = *max_element(xProj.begin(), xProj.end());
			float32 yMin = *min_element(yProj.begin(), yProj.end());
			float32 yMax = *max_element(yProj.begin(), yProj.end());
			Matrix3f basis;
			Vector3f coeffs;
			Vector3f centroid;
			pca2basis = invertMatrix(pca2basis);
			basis[0] = makeVector3f(pca2basis[0][0], pca2basis[0][1], 0);
			basis[1] = makeVector3f(pca2basis[1][0], pca2basis[1][1], 0);
			basis[2] = makeVector3f(0, 0, 1);
			centroid = makeVector3f(pca2centroid[0], pca2centroid[1], (min_z + max_z) * 0.5f);
			coeffs = makeVector3f((xMax - xMin) * 0.5f, (yMax - yMin) * 0.5f, max_z - centroid[2]);

			// radiusref
			Matrix3f inversebasis = invertMatrix(basis);
			Vector3f offset = inversebasis * (cenref - centroid);

			Vector3f coeffsref = makeVector3f(abs(coeffs[0]) + abs(offset[0]), abs(coeffs[1]) + abs(offset[1]), abs(coeffs[2]) + abs(offset[2]));
			float32 radiusref = coeffsref.getNorm();
			
			boxTemplate->basis = basis;
			boxTemplate->coeffs = coeffs;
			boxTemplate->centroid = centroid;
			boxTemplate->cenref = cenref;
			boxTemplate->radiusref = radiusref;

			boxCurrent->basis = basis;
			boxCurrent->coeffs = coeffs;
			boxCurrent->centroid = centroid;
			boxCurrent->cenref = cenref;
			boxCurrent->radiusref = radiusref;
		}

		debugRenderer->beginRenderJob_OneFrame("", 0, true);
		for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
		{
			for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
			{
				float32 linewidth;
				if (i_box == 0)
				{
					linewidth = 8;
				}
				else
				{
					linewidth = 2;
				}
				drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
			}
		}
		drawBox(boxTemplate, colorList[i_classCurrent], 2);
		debugRenderer->endRenderJob();
	}
}

void PCICL3DRepetitionGroundTruth2015::computeboxCurrent()
{
	boxCurrent = new Box;

	if (pcFeature->getNumPoints() == 0)
	{
		warning("PCICLGroundTruth2015::computeboxTemplate() - no pcFeature point cloud.");
		return;
	}
	else
	{

		// find all points of the same label and show its bounding box
		AAT flagsAAT = pcFeature->getAAT("flags");
		AAT posAAT = pcFeature->getAAT("position");
		PCA3f pointPCA;
		card32 numPoints = 0;
		Vector3f cenref;
		std::vector<Vector3f> pointsProj;
		pointsProj.resize(0);
		for (mpcard i = 0; i < pcFeature->getNumPoints(); i++)
		{
			if (pcFeature->getPointSet()->get1i(i, flagsAAT) != 0)
			{
				Vector3f pos = pcFeature->getPointSet()->get3f(i, posAAT);
				pointPCA.addPoint(pos);
				if (numPoints == 0)
					cenref = pos;
				numPoints += 1;
				pointsProj.push_back(pos);
			}
		}

		Matrix3f pcabasis;
		Vector3f pcaeigenValues;
		Vector3f pcacentroid;
		pointPCA.analyze(pcaeigenValues, pcabasis, pcacentroid);
		pcabasis = invertMatrix(pcabasis);

		std::vector<float32> xProj;
		std::vector<float32> yProj;
		std::vector<float32> zProj;
		xProj.resize(numPoints);
		yProj.resize(numPoints);
		zProj.resize(numPoints);
		for (mpcard i = 0; i < numPoints; i++)
		{
			pointsProj[i] = pcabasis * (pointsProj[i] - pcacentroid);
			xProj[i] = pointsProj[i][0];
			yProj[i] = pointsProj[i][1];
			zProj[i] = pointsProj[i][2];
		}

		float32 xMin = *min_element(xProj.begin(), xProj.end());
		float32 xMax = *max_element(xProj.begin(), xProj.end());
		float32 yMin = *min_element(yProj.begin(), yProj.end());
		float32 yMax = *max_element(yProj.begin(), yProj.end());
		float32 zMin = *min_element(zProj.begin(), zProj.end());
		float32 zMax = *max_element(zProj.begin(), zProj.end());
		Matrix3f basis;
		Vector3f coeffs;
		Vector3f centroid;
		pcabasis = invertMatrix(pcabasis);
		basis[0] = pcabasis[0];
		basis[1] = pcabasis[1];
		basis[2] = pcabasis[2];
		centroid = pcacentroid;
		coeffs = makeVector3f((xMax - xMin) * 0.5f, (yMax - yMin) * 0.5f, (zMax - zMin) * 0.5f);

		float32 radiusref = coeffs.getNorm();

		boxCurrent->basis = basis;
		boxCurrent->coeffs = coeffs;
		boxCurrent->centroid = centroid;
		boxCurrent->cenref = centroid;
		boxCurrent->radiusref = radiusref;

		debugRenderer->beginRenderJob_OneFrame("", 0, true);
		for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
		{
			for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
			{
				float32 linewidth;
				if (i_box == 0)
				{
					linewidth = 8;
				}
				else
				{
					linewidth = 2;
				}
				drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
			}
		}
		drawBox(boxCurrent, colorList[i_classCurrent], 2);
		debugRenderer->endRenderJob();
	}
}

bool PCICL3DRepetitionGroundTruth2015::checkpointinbox(Vector3f pos2center, Matrix3f world2loc, Vector3f boundary)
{
	Vector3f pos = world2loc * (pos2center);
	if (abs(pos[0]) > boundary[0])
		return false;
	if (abs(pos[1]) > boundary[1])
		return false;
	if (abs(pos[2]) > boundary[2])
		return false;
	return true;
}

void PCICL3DRepetitionGroundTruth2015::localsnapping(Vector3f posRef, int32 numCandidate)
{
	boxCurrent = new Box;

	// find all candidates
	AAT POSITION = pcFeature->getAAT("position");
	std::vector<Vector3f> cenrefCandi;
	cenrefCandi.resize(numCandidate);
	knnItFeature->setSeekPointAndReset(posRef);
	for (mpcard i = 0; i < numCandidate; i++)
	{
		cenrefCandi[i] = knnItFeature->get3f(POSITION);
		knnItFeature->next();
	}

	float32 cutoffThreshold = 10.0f;
	std::vector<float32> distPoint2Plane;
	std::vector<int32> basisid;
	distPoint2Plane.resize(numCandidate);
	basisid.resize(numCandidate);
	Timer timer;
	debugOutput << "local snapping ";
	progressWindow->pushStep(true, "local snapping");
	for (mpcard i_p = 0; i_p < numCandidate; i_p++)
	{
		mpcard numAllPointsTarget;
		mpcard *allIndicesVoxel;
		Vector3f pos_query = cenrefCandi[i_p];
		spherequeryFeature->querry(pos_query, boxTemplate->radiusref, &allIndicesVoxel, numAllPointsTarget);
		if (numAllPointsTarget == 0)
		{
			distPoint2Plane[i_p] = cutoffThreshold;
			basisid[i_p] = 0;
			continue;
		}
		UnstructuredInCorePointCloud * pcTarget = new UnstructuredInCorePointCloud;
		pcTarget->clearAndSetup(pcTemplate->getDescr(), 0);
		std::vector<char> vbuffer_(pcTemplate->getDescr()->getSize());
		for (mpcard i_pv = 0; i_pv < numAllPointsTarget; i_pv++)
		{
			pcFeature->getPointSet()->getVertex(allIndicesVoxel[i_pv], &vbuffer_[0]);
			pcTarget->getPointSet()->addPoint(&vbuffer_[0]);
		}
		std::vector<float32> dist_Rt;
		dist_Rt.resize(m_sw_numBasis);

#ifdef USEOPENMP
		omp_set_dynamic(0);     // Explicitly disable dynamic teams
		omp_set_num_threads(NUMCORE); // Use customized threads for all consecutive parallel regions
#pragma omp parallel for schedule(static)
#endif
		for (int32 i_basis = 0; i_basis < m_sw_numBasis; i_basis++)
		{
			float32 oBinAngle = (float32)i_basis / (float32)m_sw_numBasis * (float32)M_PI * 2;
			Matrix3f M_rotationZ;
			M_rotationZ[0] = makeVector3f(cos(oBinAngle), sin(oBinAngle), 0);
			M_rotationZ[1] = makeVector3f(-sin(oBinAngle), cos(oBinAngle), 0);
			M_rotationZ[2] = makeVector3f(0, 0, 1);

			// rotate pcTemplate
			UnstructuredInCorePointCloud * pcTarget_copy = new UnstructuredInCorePointCloud;
			pcTarget_copy = dynamic_cast<UnstructuredInCorePointCloud *>(pcTarget->copy());
			UnstructuredInCorePointCloud * pcTemplate_Rt = new UnstructuredInCorePointCloud;
			pcTemplate_Rt = dynamic_cast<UnstructuredInCorePointCloud *>(pcTemplate->copy());
			for (mpcard i_pRt = 0; i_pRt < pcTemplate_Rt->getNumPoints(); i_pRt++)
			{
				Vector3f pos = M_rotationZ * (pcTemplate_Rt->getPointSet()->get3f(i_pRt, POSITION) - boxTemplate->cenref) + pos_query;
				pcTemplate_Rt->getPointSet()->set3f(i_pRt, POSITION, pos);
			}
			dist_Rt[i_basis] = computeAvgPointToPlaneDistance(pcTarget_copy, pcTemplate_Rt, cutoffThreshold, 4);
			delete pcTemplate_Rt;
			delete pcTarget_copy;
		}
		int32 bestBasis = std::distance(dist_Rt.begin(), min_element(dist_Rt.begin(), dist_Rt.end()));
		distPoint2Plane[i_p] = dist_Rt[bestBasis];
		basisid[i_p] = bestBasis;

		delete pcTarget;
		if (i_p % 2 == 0) {
			progressWindow->progress((float)i_p / (float)pcFeature->getNumPoints()*100.0f);
		}
	}
	progressWindow->popStep();
	debugOutput << "time: " << convertTimeToString(timer.getDeltaValue()) << "\n";

	int32 bestCandidate = std::distance(distPoint2Plane.begin(), min_element(distPoint2Plane.begin(), distPoint2Plane.end()));

	//// normalize matching score -----------------------------------------
	//float32 max_dist = 0.0f;
	//for (mpcard i_p = 0; i_p < numCandidate; i_p++)
	//{
	//	if (distPoint2Plane[i_p] < cutoffThreshold)
	//	{
	//		if (max_dist < distPoint2Plane[i_p])
	//			max_dist = distPoint2Plane[i_p];
	//	}
	//}
	//for (mpcard i_p = 0; i_p < numCandidate; i_p++)
	//{
	//	if (distPoint2Plane[i_p] > max_dist)
	//		distPoint2Plane[i_p] = max_dist;
	//}
	//DVectorF prob;
	//prob.setDim(numCandidate);
	//for (mpcard i_p = 0; i_p < numCandidate; i_p++)
	//{
	//	float32 color = exp(-(distPoint2Plane[i_p]) / (m_sw_sigma * m_sw_sigma));
	//	prob[i_p] = color;
	//}

	//debugRenderer->beginRenderJob_OneFrame("", 0, false);
	//for (mpcard i = 0; i < numCandidate; i++) {
	//	debugRenderer->addFastSphere(cenrefCandi[i], 0.1f, makeVector3f(prob[i], prob[i], prob[i]), false);
	//}
	//debugRenderer->endRenderJob();

	float32 oBinAngle = (float32)(basisid[bestCandidate]) / (float32)m_sw_numBasis * (float32)M_PI * 2;
	Matrix3f M_rotationZ;
	M_rotationZ[0] = makeVector3f(cos(oBinAngle), sin(oBinAngle), 0);
	M_rotationZ[1] = makeVector3f(-sin(oBinAngle), cos(oBinAngle), 0);
	M_rotationZ[2] = makeVector3f(0, 0, 1);
	debugRenderer->beginRenderJob_OneFrame("", 0, false);
	//debugRenderer->addFastSphere(cenrefCandi[bestCandidate], 0.25, makeVector3f(0, 1, 0), false);
	debugRenderer->addLine(cenrefCandi[bestCandidate] + M_rotationZ * (boxTemplate->centroid - boxTemplate->cenref), cenrefCandi[bestCandidate] + M_rotationZ * (boxTemplate->centroid - boxTemplate->cenref) + M_rotationZ * boxTemplate->basis[0] * boxTemplate->coeffs[0], makeVector4f(1, 1, 1, 1), makeVector4f(1, 0, 0, 1), 2, false, false);
	debugRenderer->addLine(cenrefCandi[bestCandidate] + M_rotationZ * (boxTemplate->centroid - boxTemplate->cenref), cenrefCandi[bestCandidate] + M_rotationZ * (boxTemplate->centroid - boxTemplate->cenref) + M_rotationZ * boxTemplate->basis[1] * boxTemplate->coeffs[1], makeVector4f(1, 1, 1, 1), makeVector4f(0, 1, 0, 1), 2, false, false);
	debugRenderer->addLine(cenrefCandi[bestCandidate] + M_rotationZ * (boxTemplate->centroid - boxTemplate->cenref), cenrefCandi[bestCandidate] + M_rotationZ * (boxTemplate->centroid - boxTemplate->cenref) + M_rotationZ * boxTemplate->basis[2] * boxTemplate->coeffs[2], makeVector4f(1, 1, 1, 1), makeVector4f(0, 0, 1, 1), 2, false, false);
	debugRenderer->endRenderJob();

	boxCurrent->basis = M_rotationZ * boxTemplate->basis;
	boxCurrent->centroid = cenrefCandi[bestCandidate] + M_rotationZ * (boxTemplate->centroid - boxTemplate->cenref);
	boxCurrent->coeffs = boxTemplate->coeffs;
	boxCurrent->cenref = cenrefCandi[bestCandidate];
	boxCurrent->radiusref = boxTemplate->radiusref;
}

void PCICL3DRepetitionGroundTruth2015::initializeParameters()
{
	m_unit_relRadiusFeature = m_unit_relRadiusFullres * 2;
}

// UI
void PCICL3DRepetitionGroundTruth2015::setPcName(std::string name)
{
	pcName = name;
}

void PCICL3DRepetitionGroundTruth2015::setDumpName(std::string name)
{
	dumpName = name;
}

void PCICL3DRepetitionGroundTruth2015::setRelRadiusFullres(float32 f)
{
	m_unit_relRadiusFullres = f;
}

// render
void PCICL3DRepetitionGroundTruth2015::drawCube(const Vector3f &pbegin, const Vector3f &pend, const Vector3f &color, const float &linewidth)
{
	Vector3f p1, p2, p3, p4;
	// face one 
	p1 = pbegin;
	p2 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
	p3 = makeVector3f(pend[0], pbegin[1], pend[2]);
	p4 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face two
	p1 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
	p2 = makeVector3f(pend[0], pbegin[1], pend[2]);
	p3 = makeVector3f(pend[0], pend[1], pend[2]);
	p4 = makeVector3f(pbegin[0], pend[1], pend[2]);
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face three
	p1 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
	p2 = makeVector3f(pbegin[0], pend[1], pend[2]);
	p3 = makeVector3f(pend[0], pend[1], pend[2]);
	p4 = makeVector3f(pend[0], pend[1], pbegin[2]);
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face four
	p1 = makeVector3f(pbegin[0], pbegin[1], pbegin[2]);
	p2 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
	p3 = makeVector3f(pend[0], pend[1], pbegin[2]);
	p4 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face five
	p1 = pbegin;
	p2 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
	p3 = makeVector3f(pbegin[0], pend[1], pend[2]);
	p4 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face six
	p1 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
	p2 = makeVector3f(pend[0], pbegin[1], pend[2]);
	p3 = makeVector3f(pend[0], pend[1], pend[2]);
	p4 = makeVector3f(pend[0], pend[1], pbegin[2]);
	drawQuad(p1, p2, p3, p4, color, linewidth);
}

void PCICL3DRepetitionGroundTruth2015::drawQuad(const Vector3f &p1, const Vector3f &p2, const Vector3f &p3, const Vector3f &p4, const Vector3f &color, const float &linewidth)
{
	debugRenderer->addLine(p1, p2, color, color, linewidth, false, 0);
	debugRenderer->addLine(p2, p3, color, color, linewidth, false, 0);
	debugRenderer->addLine(p3, p4, color, color, linewidth, false, 0);
	debugRenderer->addLine(p4, p1, color, color, linewidth, false, 0);
}

void PCICL3DRepetitionGroundTruth2015::drawCubeTR(const Vector3f &pbegin, const Vector3f &pend, const Vector3f &color, const float &linewidth, Vector3f T, Matrix3f R)
{
	Vector3f p1, p2, p3, p4;
	// face one 
	p1 = pbegin;
	p2 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
	p3 = makeVector3f(pend[0], pbegin[1], pend[2]);
	p4 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
	p1 = R * p1 + T;
	p2 = R * p2 + T;
	p3 = R * p3 + T;
	p4 = R * p4 + T;
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face two
	p1 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
	p2 = makeVector3f(pend[0], pbegin[1], pend[2]);
	p3 = makeVector3f(pend[0], pend[1], pend[2]);
	p4 = makeVector3f(pbegin[0], pend[1], pend[2]);
	p1 = R * p1 + T;
	p2 = R * p2 + T;
	p3 = R * p3 + T;
	p4 = R * p4 + T;
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face three
	p1 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
	p2 = makeVector3f(pbegin[0], pend[1], pend[2]);
	p3 = makeVector3f(pend[0], pend[1], pend[2]);
	p4 = makeVector3f(pend[0], pend[1], pbegin[2]);
	p1 = R * p1 + T;
	p2 = R * p2 + T;
	p3 = R * p3 + T;
	p4 = R * p4 + T;
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face four
	p1 = makeVector3f(pbegin[0], pbegin[1], pbegin[2]);
	p2 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
	p3 = makeVector3f(pend[0], pend[1], pbegin[2]);
	p4 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
	p1 = R * p1 + T;
	p2 = R * p2 + T;
	p3 = R * p3 + T;
	p4 = R * p4 + T;
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face five
	p1 = pbegin;
	p2 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
	p3 = makeVector3f(pbegin[0], pend[1], pend[2]);
	p4 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
	p1 = R * p1 + T;
	p2 = R * p2 + T;
	p3 = R * p3 + T;
	p4 = R * p4 + T;
	drawQuad(p1, p2, p3, p4, color, linewidth);

	// face six
	p1 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
	p2 = makeVector3f(pend[0], pbegin[1], pend[2]);
	p3 = makeVector3f(pend[0], pend[1], pend[2]);
	p4 = makeVector3f(pend[0], pend[1], pbegin[2]);
	p1 = R * p1 + T;
	p2 = R * p2 + T;
	p3 = R * p3 + T;
	p4 = R * p4 + T;
	drawQuad(p1, p2, p3, p4, color, linewidth);
}

void PCICL3DRepetitionGroundTruth2015::drawBox(Box * box, Vector3f color, float32 lindwidth)
{
	drawCubeTR(box->coeffs, -box->coeffs, color, lindwidth, box->centroid, box->basis);
	debugRenderer->addLine(box->centroid, box->centroid + box->basis[0] * abs(box->coeffs[0]), makeVector4f(1, 1, 1, 1), makeVector4f(1, 0, 0, 1), lindwidth, false, false);
	debugRenderer->addLine(box->centroid, box->centroid + box->basis[1] * abs(box->coeffs[1]), makeVector4f(1, 1, 1, 1), makeVector4f(0, 1, 0, 1), lindwidth, false, false);
	debugRenderer->addLine(box->centroid, box->centroid + box->basis[2] * abs(box->coeffs[2]), makeVector4f(1, 1, 1, 1), makeVector4f(0, 0, 1, 1), lindwidth, false, false);
	debugRenderer->addFastSphere(box->cenref, m_unit_relRadiusFeature * 100, makeVector3f(1, 0, 1), false);
}

void PCICL3DRepetitionGroundTruth2015::generateColor(std::vector<Vector3f> & colorList, card32 num)
{
	colorList.resize(std::max<card32>(num, 10));
	colorList[0] = makeVector3f(102.0f / 255.0f, 153.0f / 255.0f, 255.0f / 255.0f);
	colorList[1] = makeVector3f(255.0f / 255.0f, 127.0f / 255.0f, 102.0f / 255.0f);
	colorList[2] = makeVector3f(102.0f / 255.0f, 255.0f / 255.0f, 127.0f / 255.0f);
	colorList[3] = makeVector3f(102.0f / 255.0f, 230.0f / 255.0f, 255.0f / 255.0f);
	colorList[4] = makeVector3f(255.0f / 255.0f, 204.0f / 255.0f, 102.0f / 255.0f);
	colorList[5] = makeVector3f(230.0f / 255.0f, 255.0f / 255.0f, 102.0f / 255.0f);
	colorList[6] = makeVector3f(102.0f / 255.0f, 255.0f / 255.0f, 204.0f / 255.0f);
	colorList[7] = makeVector3f(255.0f / 255.0f, 102.0f / 255.0f, 153.0f / 255.0f);
	colorList[8] = makeVector3f(204.0f / 255.0f, 102.0f / 255.0f, 255.0f / 255.0f);
	colorList[9] = makeVector3f(153.0f / 255.0f, 255.0f / 255.0f, 102.0f / 255.0f);
	if (num > 10) {
		for (mpcard i = 10; i < num; i++) {
			colorList[i] = makeVector3f(rnd01(), rnd01(), rnd01());
		}
	}
}

// UI inherited rendering functions
void PCICL3DRepetitionGroundTruth2015::keyDown(GeneralKey key)
{
}

void PCICL3DRepetitionGroundTruth2015::keyUp(GeneralKey key)
{
	if (key.getkeyAlphaNumKey() == 'c' || key.getkeyAlphaNumKey() == 'C')
	{
		debugOutput << "keyUpCleanSelection() called \n";
		keyUpCleanboxCurrent();
	}
	
	if (key.getkeyAlphaNumKey() == 's' || key.getkeyAlphaNumKey() == 'S')
	{
		debugOutput << "keyUpSaveboxCurrent() called \n";
		keyUpSaveboxCurrent();
	}

	if (key.getkeyAlphaNumKey() == 'd' || key.getkeyAlphaNumKey() == 'D')
	{
		debugOutput << "keyUpDeleteCurrent() called \n";
		keyUpDeleteCurrent();
	}

}

// customized key
void PCICL3DRepetitionGroundTruth2015::keyUpCleanboxCurrent()
{
	AAT FLAGSFullres = pcFullres->getAAT("flags");
	AAT FLAGSFeature = pcFeature->getAAT("flags");
	for (mpcard i = 0; i < pcFullres->getNumPoints(); i++)
	{
		pcFullres->getPointSet()->set1i(i, FLAGSFullres, 0);
	}
	pcFullres->clearAttachments();

	for (mpcard i = 0; i < pcFeature->getNumPoints(); i++)
	{
		pcFeature->getPointSet()->set1i(i, FLAGSFeature, 0);
	}
	//pcFeature->clearAttachments();
	boxCurrent = new Box;

	// render the current frame
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
		}
	}
	debugRenderer->endRenderJob();
}

void PCICL3DRepetitionGroundTruth2015::keyUpSaveboxCurrent()
{
	if (flag_editbox)
	{
		Matrix3f M_rotationZ;
		M_rotationZ[0] = makeVector3f(cos(rotateEdit[2]), sin(rotateEdit[2]), 0);
		M_rotationZ[1] = makeVector3f(-sin(rotateEdit[2]), cos(rotateEdit[2]), 0);
		M_rotationZ[2] = makeVector3f(0, 0, 1);

		Vector3f offset = boxRecord[i_classCurrent][i_boxCurrent]->cenref - boxRecord[i_classCurrent][i_boxCurrent]->centroid;
		boxRecord[i_classCurrent][i_boxCurrent]->centroid = boxRecord[i_classCurrent][i_boxCurrent]->centroid + boxRecord[i_classCurrent][i_boxCurrent]->basis[0] * translateEdit[0] + boxRecord[i_classCurrent][i_boxCurrent]->basis[1] * translateEdit[1] + boxRecord[i_classCurrent][i_boxCurrent]->basis[2] * translateEdit[2];
		boxRecord[i_classCurrent][i_boxCurrent]->cenref = boxRecord[i_classCurrent][i_boxCurrent]->centroid + M_rotationZ * offset;
		boxRecord[i_classCurrent][i_boxCurrent]->basis = M_rotationZ * boxRecord[i_classCurrent][i_boxCurrent]->basis;
		translateEdit = makeVector3f(0, 0, 0);
		rotateEdit = makeVector3f(0, 0, 0);
	}
	else{
		AAT FLAGSFullres = pcFullres->getAAT("flags");
		AAT FLAGSFeature = pcFeature->getAAT("flags");
		for (mpcard i = 0; i < pcFullres->getNumPoints(); i++)
		{
			pcFullres->getPointSet()->set1i(i, FLAGSFullres, 0);
		}
		pcFullres->clearAttachments();

		for (mpcard i = 0; i < pcFeature->getNumPoints(); i++)
		{
			pcFeature->getPointSet()->set1i(i, FLAGSFeature, 0);
		}

		// push boxCurrent into boxRecord[i_classCurrent]
		boxRecord[i_classCurrent].push_back(boxCurrent);
	}

	// render the record
	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
	{
		for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
		{
			float32 linewidth;
			if (i_box == 0)
			{
				linewidth = 8;
			}
			else
			{
				linewidth = 2;
			}
			drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
		}
	}
	debugRenderer->endRenderJob();

}

void PCICL3DRepetitionGroundTruth2015::keyUpDeleteCurrent()
{
	translateEdit = makeVector3f(0, 0, 0);
	rotateEdit = makeVector3f(0, 0, 0);

	if (i_classCurrent >= 0 && i_boxCurrent >= 0 && boxRecord[i_classCurrent].size() > i_boxCurrent)
	{
		boxRecord[i_classCurrent].erase(boxRecord[i_classCurrent].begin() + i_boxCurrent);
		i_classCurrent = -1;
		i_boxCurrent = -1;

		// render the record
		debugRenderer->beginRenderJob_OneFrame("", 0, true);
		for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
		{
			for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
			{
				float32 linewidth;
				if (i_box == 0)
				{
					linewidth = 8;
				}
				else
				{
					linewidth = 2;
				}
				drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
			}
		}
		debugRenderer->endRenderJob();
	}
	else{
		warning("PCICLGroundTruth2015::keyUpDeleteCurrent() - dangerous deletion.");
		debugOutput << "i_classCurrent: " << i_classCurrent << "\n";
		debugOutput << "i_boxCurrent: " << i_boxCurrent << "\n";
		debugOutput << "boxRecord[i_classCurrent].size(): " << boxRecord[i_classCurrent].size() << "\n";
		return;
	}
}

void PCICL3DRepetitionGroundTruth2015::mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState)
{
	start_x = x;
	start_y = y;
	if (buttonsState.getLeft()) leftMBdown = true;
	if (buttonsState.getRight()) rightMBdown = true;
	if (leftMBdown) {
		flag_editbox = false;
		if ((flag_createtemplate && !flag_manualdetect) || (flag_manualcreateclass && !flag_manualcreateobject) || (!flag_manualcreateclass && flag_manualcreateobject))
		{
			leftMBdownX = x;
			leftMBdownY = y;
			leftMBmoveX = x;
			leftMBmoveY = y;
			CmdObjChangeSelectionOnClick *newCmd = new CmdObjChangeSelectionOnClick();
			newCmd->setup(x, y, ST_ctrlPressed, ST_brushsize, ST_mode, (scene->getRootState()->staticState->camera), (scene->getRootState()->staticState->viewFrustum), w, h, ST_SurfaceMode);
			scene->getCommandHistory()->executeCommand(scene, newCmd, true);
		}
		else if (!flag_createtemplate && flag_manualdetect)
		{
			Vector3f clickPoint;
			if (guessClickPoint(x, y, 5, clickPoint)) {
				AAT POSITION = pcFeature->getAAT("position");
				knnItFeature->setSeekPointAndReset(clickPoint);
				int32 index = knnItFeature->getPointSetPointNumber();
				debugRenderer->beginRenderJob_OneFrame("", 0, false);
				//debugRenderer->addFastSphere(pcFeature->getPointSet()->get3f(index, POSITION), 0.25f, makeVector3f(1, 0, 0), true);
				debugRenderer->endRenderJob();
				localsnapping(pcFeature->getPointSet()->get3f(index, POSITION), m_localsnapping_numCandidate);
			}
		}
		else if ((!flag_createtemplate && !flag_manualdetect) || (!flag_manualcreateclass && !flag_manualcreateobject))
		{
			mouseDownPassToCam(x, y, buttonsState, modifiersState);
		}
	}

	if (rightMBdown) {
		flag_editbox = true;
		flag_createtemplate = false; // we are in selection mode, not creating
		flag_manualdetect = false;
		flag_manualcreateclass = false;
		flag_manualcreateobject = false;

		Vector3f clickPoint;
		if (guessClickPoint(x, y, 5, clickPoint)) {
			// find the box that is closet to clickPoint
			float32 min_dist = 100000;
			int32 min_class = -1;
			int32 min_box = -1;
			for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
			{
				for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
				{
					Vector3f v_diff = boxRecord[i_class][i_box]->centroid - clickPoint;
					float32 dist = v_diff.getSqrNorm();
					if (dist < min_dist)
					{
						min_dist = dist;
						min_class = i_class;
						min_box = i_box;
					}
				}
			}

			if (min_dist < 2)
			{
				boxCurrent = new Box;
				*boxCurrent = *boxRecord[min_class][min_box];
				boxTemplate = new Box;
				*boxTemplate = *boxRecord[min_class][0];
				i_classCurrent = min_class;
				i_boxCurrent = min_box;

				debugRenderer->beginRenderJob_OneFrame("", 0, true);
				for (mpcard i_class = 0; i_class < boxRecord.size(); i_class++)
				{
					for (mpcard i_box = 0; i_box < boxRecord[i_class].size(); i_box++)
					{
						float32 linewidth;
						if (i_box == 0)
						{
							linewidth = 8;
						}
						else
						{
							linewidth = 2;
						}
						drawBox(boxRecord[i_class][i_box], colorList[i_class], linewidth);
					}
				}

				drawBox(boxRecord[min_class][min_box], colorList[min_class], 16);
				debugRenderer->endRenderJob();
			}
		}
	}

	forget(buttonsState);
	forget(modifiersState);
}

void PCICL3DRepetitionGroundTruth2015::mouseMoved(int32 x, int32 y)
{
	leftMBmoveX = x;
	leftMBmoveY = y;
	if (leftMBdown) {
		if ((flag_createtemplate && !flag_manualdetect) || (flag_manualcreateclass && !flag_manualcreateobject) || !flag_manualcreateclass && flag_manualcreateobject)
		{
			CmdObjChangeSelectionOnClick *newCmd = new CmdObjChangeSelectionOnClick();
			newCmd->setup(x, y, ST_ctrlPressed, ST_brushsize, ST_mode, (scene->getRootState()->staticState->camera), (scene->getRootState()->staticState->viewFrustum), w, h, ST_SurfaceMode);
			scene->getCommandHistory()->executeCommand(scene, newCmd, true);
		}
		else if (!flag_createtemplate && flag_manualdetect)
		{

		}
		else if (!flag_createtemplate && !flag_manualdetect)
		{
			mouseMovedPassToCam(x, y);
		}
	}
}

void PCICL3DRepetitionGroundTruth2015::mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState)
{
	CmdObjSelectionOnArea *newCmd = new CmdObjSelectionOnArea();
	newCmd->setup(leftMBdownX, leftMBdownY, x, y, ST_ctrlPressed, ST_brushsize, ST_mode, (scene->getRootState()->staticState->camera), (scene->getRootState()->staticState->viewFrustum), w, h);
	scene->getCommandHistory()->executeCommand(scene, newCmd, true);
	if (leftMBdown) {
		if ((flag_createtemplate && !flag_manualdetect) || (flag_manualcreateclass && !flag_manualcreateobject))
		{
			computeboxTemplate();
		}
		else if (!flag_manualcreateclass && flag_manualcreateobject) {
			computeboxCurrent();
		}
		else if (!flag_createtemplate && flag_manualdetect)
		{

		}
		else if (!flag_createtemplate && !flag_manualdetect)
		{
			mouseUpPassToCam(x, y, buttonsState, modifiersState);
		}
	}
	leftMBdown = false;
	rightMBdown = false;
	leftMBdownX = 0;
	leftMBdownY = 0;
	leftMBmoveX = x;
	leftMBmoveY = y;
	forget(buttonsState);
	forget(modifiersState);
}

void PCICL3DRepetitionGroundTruth2015::mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState)
{
	mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}

void PCICL3DRepetitionGroundTruth2015::areaResize(card32 width, card32 height) {
	h = height;
	w = width;
	if (w == 0) { WidthToHeight = 0.0f; return; };
	WidthToHeight = ((float)height) / ((float)width);
}

void PCICL3DRepetitionGroundTruth2015::glDrawTool(GLContext *glContext) {
	forget(glContext);
	if ((ST_mode == SELMODE_BRUSHCIRCLE) || (ST_mode == SELMODE_AREARECTANGLE) || (ST_mode == SELMODE_BRUSHQUAD) || (ST_mode == SELMODE_AREACIRCLE))
	{
		// render cursor
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();
		glPushAttrib(GL_ALL_ATTRIB_BITS);
		glDisable(GL_LIGHTING);
		glLineWidth(1.0f);
		float32 posX = (leftMBmoveX - w*0.5f) / (w*0.5f);
		float32 posY = -((leftMBmoveY - h*0.5f) / (h*0.5f));
		if (ST_mode == SELMODE_BRUSHQUAD) {
			float32 delta = ST_brushsize / (h*0.5f);
			glColor4f(1.0f, 0.9f, 0.1f, 1.0f);
			drawRectangle(posX - delta*WidthToHeight, posY - delta, posX + delta*WidthToHeight, posY + delta);
		}
		glPopAttrib();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	};
}

void PCICL3DRepetitionGroundTruth2015::drawRectangle(float32 x1, float32 y1, float32 x2, float32 y2) {
	glBegin(GL_LINE_LOOP);
	glVertex2f(x2, y2);
	glVertex2f(x2, y1);
	glVertex2f(x1, y1);
	glVertex2f(x1, y2);
	glEnd();
};

void PCICL3DRepetitionGroundTruth2015::drawFilledRectangle(float32 x1, float32 y1, float32 x2, float32 y2) {
	glBegin(GL_QUADS);
	glVertex2f(x2, y2);
	glVertex2f(x2, y1);
	glVertex2f(x1, y1);
	glVertex2f(x1, y2);
	glEnd();
};

void PCICL3DRepetitionGroundTruth2015::drawCircle(float32 x1, float32 y1, float32 r) {
	float32 DEG2RAD = 3.141592654f / 180.0f;
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < 360; i += 10)
	{
		float32 degInRad = i*DEG2RAD;
		glVertex2f(x1 + cos(degInRad)*WidthToHeight*r, y1 + sin(degInRad)*r);
	}
	glEnd();
};

void PCICL3DRepetitionGroundTruth2015::drawFilledCircle(float32 x1, float32 y1, float32 r) {
	float32 DEG2RAD = 3.141592654f / 180.0f;
	glBegin(GL_POLYGON);
	for (int32 i = 0; i < 360; i += 10)
	{
		float32 degInRad = i*DEG2RAD;
		glVertex2f(x1 + cos(degInRad)*WidthToHeight*r, y1 + sin(degInRad)*r);
	}
	glEnd();
};

void PCICL3DRepetitionGroundTruth2015::renderSelectedObject()
{






	//debugOutput << "eigenValues: " << eigenValues << "\n";
	//debugOutput << "coordFrame: " << coordFrame << "\n";
	//debugOutput << "centroid: " << centroid << "\n";

	//BoundingBox3f* bbox = new BoundingBox3f;
	//bbox->lowerCorner = makeVector3f(10e10f, 10e10f, 10e10f);
	//bbox->upperCorner = makeVector3f(-10e10f, -10e10f, -10e10f);



	//PointSet* ps = new PointSet;
	//ps->clearAndSetup(1, numPoint, pcFullres->getDescr());
	//card32 count = 0;
	//for (mpcard i = 0; i < pcFullres->getNumPoints(); i++)
	//{
	//	if (pcFullres->getPointSet()->get1i(i, flagsAAT) != 0)
	//	{
	//		ps->set3f(count, posAAT, pcFullres->getPointSet()->get3f(i, posAAT));
	//		count += 1;
	//	}
	//}


	//basis[2] = makeVector3f(0, 0, 1);
	//// project ps to xy plane

	//coordFrame[0].normalize();
	//coordFrame[1].normalize();

	return;

	//DVectorF box;
	//box->setDim(6);
	//Vector3f bboxCenter;
	//Matrix3f coordFrame;
	//Vector3f eigenValues;
	//Vector3f centroid;
	//pointPCA.analyze(eigenValues, coordFrame, centroid);
	//coordFrame[0].normalize();
	//coordFrame[1].normalize();
	//coordFrame[2].normalize();
	//Matrix3f rotMatrix = invertMatrix(coordFrame);
	//bboxCenter = bbox->getCenter();
	//std::vector<float32> pos_x;
	//std::vector<float32> pos_y;
	//std::vector<float32> pos_z;
	//pos_x.resize(ps->getNumEntries());
	//pos_y.resize(ps->getNumEntries());
	//pos_z.resize(ps->getNumEntries());
	//for (int ii = 0; ii < ps->getNumEntries(); ii++) {
	//	Vector3f pos_ = rotMatrix * (ps->get3f(ii, posAAT) - bboxCenter);
	//	pos_x[ii] = pos_[0];
	//	pos_y[ii] = pos_[1];
	//	pos_z[ii] = pos_[2];
	//}
	//box[0] = *min_element(pos_x.begin(), pos_x.end());
	//box[1] = *min_element(pos_y.begin(), pos_y.end());
	//box[2] = *min_element(pos_z.begin(), pos_z.end());
	//box[3] = *max_element(pos_x.begin(), pos_x.end());
	//box[4] = *max_element(pos_y.begin(), pos_y.end());
	//box[5] = *max_element(pos_z.begin(), pos_z.end());
	//{
	//	debugRenderer->beginRenderJob_OneFrame("", 0, true);
	//	Vector3f pos = bboxCenter;
	//	Vector3f locX = coordFrame[0];
	//	Vector3f locY = coordFrame[1];
	//	Vector3f locZ = coordFrame[2];
	//	Vector3f pbegin = makeVector3f(box[0], box[1], box[2]);
	//	Vector3f pend = makeVector3f(box[3], box[4], box[5]);
	//	Matrix3f rf = makeMatrix3f(locX[0], locY[0], locZ[0], locX[1], locY[1], locZ[1], locX[2], locY[2], locZ[2]);
	//	drawCubeTR(pbegin, pend, makeVector3f(1, 0, 0), 2.0f, pos, rf);
	//	debugRenderer->addLine(pos, pos + locX * box[0], makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(1.0f, 0.0f, 0.0f), 2);
	//	debugRenderer->addLine(pos, pos + locY * box[1], makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(0.0f, 1.0f, 0.0f), 2);
	//	debugRenderer->addLine(pos, pos + locZ * box[2], makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(0.0f, 0.0f, 1.0f), 2);
	//	debugRenderer->endRenderJob();
	//}

	//delete ps;
}
