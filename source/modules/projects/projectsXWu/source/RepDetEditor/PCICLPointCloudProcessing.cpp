//---------------------------------------------------------------------------
#include "StdAfx.h"
#include <opencv\cxcore.h>
//---------------------------------------------------------------------------
#include "PCICLPointCloudProcessing.h"
#include "ProgressWindow.h"
#include "Monomials2ndOrder.h"
#include <agents.h>
#include "DynamicLinearAlgebraTools.h"
#include "PCCResampler.h"
#include "NormalEstimator.h"
#include "PointSetKNNQuery.h"
#include "AttachedIndexedOctree.h"
#include "LinearAlgebra.h"
#include "NormalUnifier.h"
#include "PCCNormalEstimatorPCA.h"
#include "IndexGraph.h"
#include "FifoStack.h"
#include "PriorityQueue.h"
#include "ConvexHull2D.h"

//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

IMPLEMENT_X4_CLASS(PCICLPointCloudProcessing, 0){
	BEGIN_CLASS_INIT(PCICLPointCloudProcessing);
}

Matrix3f NormalCov(const Vector3f &pos, float32 sigma, HierarchicalKNNIterator * hIt, AAT normalAAT)
{
	float inv_sigma_2 = 1.0f / sqr(sigma);
	hIt->setMaxDistanceToSeekPoint(2.0f * sigma);
	hIt->setSeekPointAndReset(pos);
	Matrix3f C = IDENTITY3F * 0;
	float summedWeight = 0;
	while (!hIt->atEnd())
	{
		Vector3f normal = hIt->get3f(normalAAT);
		float weight = exp(-hIt->getSquaredDistance() * inv_sigma_2);
		C += outerProduct(normal, normal) * weight;
		summedWeight += weight;
		hIt->next();
	}
	C /= summedWeight;
	return C;
}

PCICLPointCloudProcessing::PCICLPointCloudProcessing(void){

}

PCICLPointCloudProcessing::~PCICLPointCloudProcessing(void){

}

void PCICLPointCloudProcessing::keyDown(GeneralKey key) {
}

void PCICLPointCloudProcessing::mouseDown(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState){
	mouseDownPassToCam(x, y, buttonsState, modifiersState);
}

void PCICLPointCloudProcessing::mouseMoved(int32 x, int32 y){
	mouseMovedPassToCam(x, y);
}

void PCICLPointCloudProcessing::mouseUp(int32 x, int32 y, MouseButtons buttonsState, Modifiers modifiersState){
	mouseUpPassToCam(x, y, buttonsState, modifiersState);
}

void PCICLPointCloudProcessing::mouseWheelRotated(int32 rotatedDelta, Modifiers modifiersState){
	mouseWheelRotatedPassToCam(rotatedDelta, modifiersState);
}

void PCICLPointCloudProcessing::computeSmoothNormal(PointCloud* pc, float32 m_CV_relBaseSigmaVertex){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeSmoothNormal(PointCloud* pc, float32 m_CV_relBaseSigmaVertex)\n";
	AAT pcposAAT = pc->getAAT("position");
	AAT pcnormalAAT = pc->getAAT("normal");
	SceneObjectIterator *iter = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *ps = mIt->lockBlockReadWrite();

	BoundingBox3f bb = ps->getBoundingBox();
	const float32 bbDiagonal = bb.getDiagonalLength();
	debugOutput << bbDiagonal << "\n";
	PointSetRangeQuery query(ps, sqr(m_CV_relBaseSigmaVertex * bbDiagonal * 3));

	card32 index = 0;
	progressWindow->pushStep(true, "computeBase");
	std::vector<Vector3f> posBuffer;
	float kernelRadSqr = sqr(m_CV_relBaseSigmaVertex * bbDiagonal * 3.0f);

	mpcard numPoints = ps->getNumEntries();
	for (mpcard ptNum = 0; ptNum < numPoints; ptNum++)
	{
		Vector3f pos = ps->get3f(ptNum, pcposAAT);
		card32 numPoints;
		query.setTargetPoint(pos);
		int32 *pts = query.getAllIndices(numPoints);

		Vector3f pcaNormal;
		PCA<float, 3> pca;
		for (card32 i = 0; i < numPoints; i++) {
			Vector3f sample_pos = ps->get3f(pts[i], pcposAAT);
			float dist_2 = normQuad(sample_pos - pos);
			float w = exp(-dist_2 / (2 * m_CV_relBaseSigmaVertex * bbDiagonal * m_CV_relBaseSigmaVertex * bbDiagonal));
			pca.addPoint(sample_pos, w);
		}
		{
			Vector3f eigenValues, centroid;
			Matrix3f eigenVectors;
			pca.analyze(eigenValues, eigenVectors, centroid);
			if (eigenVectors[2] * ps->get3f(ptNum, pcnormalAAT) < 0.0f)
				eigenVectors *= -1.0f;
			pcaNormal = normalize(eigenVectors[2]);
			ps->set3f(ptNum, pcnormalAAT, pcaNormal);
		}
		index++;
		if (index % 1000 == 0) {
			progressWindow->progress((float)index / (float)pc->getNumPoints()*100.0f);
		}
	}
	progressWindow->popStep();

	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

void PCICLPointCloudProcessing::computeSmoothNormal(PointCloud* pc, float32 m_CV_relBaseSigmaVertex, float32 m_AbsDiagonal){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeSmoothNormal(PointCloud* pc, float32 m_CV_relBaseSigmaVertex)\n";
	AAT pcposAAT = pc->getAAT("position");
	AAT pcnormalAAT = pc->getAAT("normal");
	SceneObjectIterator *iter = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *ps = mIt->lockBlockReadWrite();

	PointSetRangeQuery query(ps, sqr(m_CV_relBaseSigmaVertex * m_AbsDiagonal * 3));

	card32 index = 0;
	progressWindow->pushStep(true, "computeBase");
	std::vector<Vector3f> posBuffer;
	float kernelRadSqr = sqr(m_CV_relBaseSigmaVertex * m_AbsDiagonal * 3.0f);

	mpcard numPoints = ps->getNumEntries();
	for (mpcard ptNum = 0; ptNum < numPoints; ptNum++)
	{
		Vector3f pos = ps->get3f(ptNum, pcposAAT);
		card32 numPoints;
		query.setTargetPoint(pos);
		int32 *pts = query.getAllIndices(numPoints);

		Vector3f pcaNormal;
		PCA<float, 3> pca;
		for (card32 i = 0; i < numPoints; i++) {
			Vector3f sample_pos = ps->get3f(pts[i], pcposAAT);
			float dist_2 = normQuad(sample_pos - pos);
			float w = exp(-dist_2 / (2 * m_CV_relBaseSigmaVertex * m_AbsDiagonal * m_CV_relBaseSigmaVertex * m_AbsDiagonal));
			pca.addPoint(sample_pos, w);
		}
		{
			Vector3f eigenValues, centroid;
			Matrix3f eigenVectors;
			pca.analyze(eigenValues, eigenVectors, centroid);
			if (eigenVectors[2] * ps->get3f(ptNum, pcnormalAAT) < 0.0f)
				eigenVectors *= -1.0f;
			pcaNormal = normalize(eigenVectors[2]);
			ps->set3f(ptNum, pcnormalAAT, pcaNormal);
		}
		index++;
		if (index % 1000 == 0) {
			progressWindow->progress((float)index / (float)pc->getNumPoints()*100.0f);
		}
	}
	progressWindow->popStep();

	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

void PCICLPointCloudProcessing::computeSmoothNormalParallel(PointCloud* pc, float32 m_CV_relBaseSigmaVertex, float32 m_AbsDiagonal){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeSmoothNormal(PointCloud* pc, float32 m_CV_relBaseSigmaVertex)\n";
	AAT pcposAAT = pc->getAAT("position");
	AAT pcnormalAAT = pc->getAAT("normal");
	SceneObjectIterator *iter = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *ps = mIt->lockBlockReadWrite();
	AttachedIndexedOctree * knnOctree = PointSetKNNQuery::createOctree(ps);
	float32 radius = m_CV_relBaseSigmaVertex * m_AbsDiagonal * 3.0f;
	mpcard numPoints = ps->getNumEntries();

#ifdef USEOPENMP
	omp_set_dynamic(0);     // Explicitly disable dynamic teams
	omp_set_num_threads(NUMCORE); // Use customized threads for all consecutive parallel regions
#pragma omp parallel for schedule(static)
#endif
	for (int ptNum = 0; ptNum < numPoints; ptNum++)
	{
		FastSphereQuerry * query = new FastSphereQuerry(knnOctree, ps);

		Vector3f pos = ps->get3f(ptNum, pcposAAT);
		mpcard numPoints;
		mpcard *pts;
		query->querry(pos, radius, &pts, numPoints);
		Vector3f pcaNormal;
		PCA<float, 3> pca;
		for (card32 i = 0; i < numPoints; i++) {
			Vector3f sample_pos = ps->get3f(pts[i], pcposAAT);
			float dist_2 = normQuad(sample_pos - pos);
			float w = exp(-dist_2 / (2 * m_CV_relBaseSigmaVertex * m_AbsDiagonal * m_CV_relBaseSigmaVertex * m_AbsDiagonal));
			pca.addPoint(sample_pos, w);
		}
		{
			Vector3f eigenValues, centroid;
			Matrix3f eigenVectors;
			pca.analyze(eigenValues, eigenVectors, centroid);
			if (eigenVectors[2] * ps->get3f(ptNum, pcnormalAAT) < 0.0f)
				eigenVectors *= -1.0f;
			pcaNormal = normalize(eigenVectors[2]);
			ps->set3f(ptNum, pcnormalAAT, pcaNormal);
		}
		delete query;
	}
	//progressWindow->popStep();

	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}


void PCICLPointCloudProcessing::computeCurvatureRobust(PointCloud* pc, float32 baseSigma, BOOL usePCANormal, float32 maxCurvature){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeCurvatureRobust(PointCloud* pc, float32 baseSigma, BOOL usePCANormal, float32 maxCurvature)\n";
	//// add channels for principle curvatures & directions
	checkAttribute(pc, "curv2", 2, VAD::DATA_FORMAT_FLOAT32);
	checkAttribute(pc, "tangent_u", 3, VAD::DATA_FORMAT_FLOAT32);
	checkAttribute(pc, "tangent_v", 3, VAD::DATA_FORMAT_FLOAT32);
	AAT posAAT = pc->getAAT("position");
	AAT normalAAT = pc->getAAT("normal");
	AAT flagsAAT = pc->getAAT("flags");
	AAT c2AAT = pc->getAAT("curv2");
	AAT tUAAT = pc->getAAT("tangent_u");
	AAT tVAAT = pc->getAAT("tangent_v");

	SceneObjectIterator *iter = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *ps = mIt->lockBlockReadWrite();

	BoundingBox3f bb = ps->getBoundingBox();
	const float32 bbDiagonal = bb.getDiagonalLength();
	PointSetRangeQuery query(ps, sqr(baseSigma * bbDiagonal * 3.0f));

	card32 index = 0;
	std::vector<Vector3f> posBuffer;
	float kernelRadSqr = sqr(baseSigma * bbDiagonal * 3.0f);
	progressWindow->pushStep(true, "Robust Curvature ...");
	mpcard numPoints = ps->getNumEntries();
	for (mpcard i_p = 0; i_p < numPoints; i_p++)
	{
		Vector3f pos = ps->get3f(i_p, posAAT);
		card32 numNN;
		query.setTargetPoint(pos);
		int32 *pts = query.getAllIndices(numNN);

		Vector3f normal;
		if (usePCANormal)
		{
			PCA<float, 3> pca;
			for (card32 i = 0; i < numNN; i++) {
				Vector3f sample_pos = ps->get3f(pts[i], posAAT);
				float dist_2 = normQuad(sample_pos - pos);
				float w = exp(-dist_2 / (2 * baseSigma * bbDiagonal *baseSigma * bbDiagonal));
				pca.addPoint(sample_pos, w);
			}
			{
				Vector3f eigenValues, centroid;
				Matrix3f eigenVectors;
				pca.analyze(eigenValues, eigenVectors, centroid);
				if (eigenVectors[2] * ps->get3f(i_p, normalAAT) < 0.0f)
					eigenVectors *= -1.0f;
				normal = normalize(eigenVectors[2]);
				ps->set3f(i_p, normalAAT, normal);
			}
		}
		else
		{
			normal = ps->get3f(i_p, normalAAT);
		}

		Matrix3f tangentSystem = calcTangentSystem(normal);
		Matrix3f invTangentSystem = tangentSystem.transpose();

		if (posBuffer.size() < numNN)
		{
			posBuffer.resize(numNN);
		}

		for (card32 i = 0; i < numNN; i++)
		{
			Vector3f tmp = ps->get3f(pts[i], posAAT) - pos;
			posBuffer[i] = invTangentSystem * tmp / (sqrt(2.0f) * baseSigma * bbDiagonal);
		}

		StaticVector<float32, 6> coeff;
		Monomials2ndOrder::computeCoeffs(&posBuffer[0], (card32)numNN, coeff, 1.0f);

		float b_u = coeff[1];
		float b_v = coeff[2];
		float A_uu = coeff[3];
		float A_uv = coeff[4];
		float A_vv = coeff[5];

		Matrix2f A = makeMatrix2f(A_uu, A_uv,
			A_uv, A_vv);
		Vector2f lambda;
		Matrix2f eVects;
		A.computeEigenStructure(lambda, eVects);

		if (lambda[0] < 0){
			lambda[0] = -1 * lambda[0];
			eVects[0] = makeVector2f(-1 * eVects[0][0], -1 * eVects[0][1]);
		}
		if (lambda[1] < 0){
			lambda[1] = -1 * lambda[1];
			eVects[1] = makeVector2f(-1 * eVects[1][0], -1 * eVects[1][1]);
		}

		float32 lambda1 = lambda[0];
		float32 lambda2 = lambda[1];

		Vector2f v1 = eVects[0];
		Vector2f v2 = eVects[1];

		if (fabs(lambda1) < fabs(lambda2)) {
			swap(lambda1, lambda2);
			swap(v1, v2);
		}

		Vector3f newTangentU = tangentSystem[0] * v1[0] + tangentSystem[1] * v1[1];
		//Vector3f newTangentV = newTangentU.crossProduct(normal);
		Vector3f newTangentV = normal.crossProduct(newTangentU);

		if (lambda1 > maxCurvature){
			lambda1 = 0;
			lambda2 = 0;
		}

		ps->set2f(i_p, c2AAT, makeVector2f(lambda1, lambda2));
		ps->set3f(i_p, tUAAT, newTangentU);
		ps->set3f(i_p, tVAAT, newTangentV);
		index++;

		if (index % 1000 == 0) {
			progressWindow->progress((float)index / (float)pc->getNumPoints()*100.0f);
		}
	}
	progressWindow->popStep();
	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

void PCICLPointCloudProcessing::computeCurvatureRobust(PointCloud* pc, float32 baseSigma, BOOL usePCANormal, float32 maxCurvature, float32 m_AbsDiagonal){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeCurvatureRobust(PointCloud* pc, float32 baseSigma, BOOL usePCANormal, float32 maxCurvature)\n";
	//// add channels for principle curvatures & directions
	checkAttribute(pc, "curv2", 2, VAD::DATA_FORMAT_FLOAT32);
	checkAttribute(pc, "tangent_u", 3, VAD::DATA_FORMAT_FLOAT32);
	checkAttribute(pc, "tangent_v", 3, VAD::DATA_FORMAT_FLOAT32);
	AAT posAAT = pc->getAAT("position");
	AAT normalAAT = pc->getAAT("normal");
	AAT flagsAAT = pc->getAAT("flags");
	AAT c2AAT = pc->getAAT("curv2");
	AAT tUAAT = pc->getAAT("tangent_u");
	AAT tVAAT = pc->getAAT("tangent_v");

	SceneObjectIterator *iter = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *ps = mIt->lockBlockReadWrite();

	PointSetRangeQuery query(ps, sqr(baseSigma * m_AbsDiagonal * 3.0f));

	card32 index = 0;
	std::vector<Vector3f> posBuffer;
	float kernelRadSqr = sqr(baseSigma * m_AbsDiagonal * 3.0f);
	progressWindow->pushStep(true, "Robust Curvature ...");
	mpcard numPoints = ps->getNumEntries();
	for (mpcard i_p = 0; i_p < numPoints; i_p++)
	{
		Vector3f pos = ps->get3f(i_p, posAAT);
		card32 numNN;
		query.setTargetPoint(pos);
		int32 *pts = query.getAllIndices(numNN);

		Vector3f normal;
		if (usePCANormal)
		{
			PCA<float, 3> pca;
			for (card32 i = 0; i < numNN; i++) {
				Vector3f sample_pos = ps->get3f(pts[i], posAAT);
				float dist_2 = normQuad(sample_pos - pos);
				float w = exp(-dist_2 / (2 * baseSigma * m_AbsDiagonal *baseSigma * m_AbsDiagonal));
				pca.addPoint(sample_pos, w);
			}
			{
				Vector3f eigenValues, centroid;
				Matrix3f eigenVectors;
				pca.analyze(eigenValues, eigenVectors, centroid);
				if (eigenVectors[2] * ps->get3f(i_p, normalAAT) < 0.0f)
					eigenVectors *= -1.0f;
				normal = normalize(eigenVectors[2]);
				ps->set3f(i_p, normalAAT, normal);
			}
		}
		else
		{
			normal = ps->get3f(i_p, normalAAT);
		}

		Matrix3f tangentSystem = calcTangentSystem(normal);
		Matrix3f invTangentSystem = tangentSystem.transpose();

		if (posBuffer.size() < numNN)
		{
			posBuffer.resize(numNN);
		}

		for (card32 i = 0; i < numNN; i++)
		{
			Vector3f tmp = ps->get3f(pts[i], posAAT) - pos;
			posBuffer[i] = invTangentSystem * tmp / (sqrt(2.0f) * baseSigma * m_AbsDiagonal);
		}

		StaticVector<float32, 6> coeff;
		Monomials2ndOrder::computeCoeffs(&posBuffer[0], (card32)numNN, coeff, 1.0f);

		float b_u = coeff[1];
		float b_v = coeff[2];
		float A_uu = coeff[3];
		float A_uv = coeff[4];
		float A_vv = coeff[5];

		Matrix2f A = makeMatrix2f(A_uu, A_uv,
			A_uv, A_vv);
		Vector2f lambda;
		Matrix2f eVects;
		A.computeEigenStructure(lambda, eVects);

		if (lambda[0] < 0){
			lambda[0] = -1 * lambda[0];
			eVects[0] = makeVector2f(-1 * eVects[0][0], -1 * eVects[0][1]);
		}
		if (lambda[1] < 0){
			lambda[1] = -1 * lambda[1];
			eVects[1] = makeVector2f(-1 * eVects[1][0], -1 * eVects[1][1]);
		}

		float32 lambda1 = lambda[0];
		float32 lambda2 = lambda[1];

		Vector2f v1 = eVects[0];
		Vector2f v2 = eVects[1];

		if (fabs(lambda1) < fabs(lambda2)) {
			swap(lambda1, lambda2);
			swap(v1, v2);
		}

		Vector3f newTangentU = tangentSystem[0] * v1[0] + tangentSystem[1] * v1[1];
		//Vector3f newTangentV = newTangentU.crossProduct(normal);
		Vector3f newTangentV = normal.crossProduct(newTangentU);

		if (lambda1 > maxCurvature){
			lambda1 = 0;
			lambda2 = 0;
		}

		ps->set2f(i_p, c2AAT, makeVector2f(lambda1, lambda2));
		ps->set3f(i_p, tUAAT, newTangentU);
		ps->set3f(i_p, tVAAT, newTangentV);
		index++;

		if (index % 1000 == 0) {
			progressWindow->progress((float)index / (float)pc->getNumPoints()*100.0f);
		}
	}
	progressWindow->popStep();
	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}


void PCICLPointCloudProcessing::computeCurvatureRobustParallel(PointCloud *pc, float32 baseSigma, float32 maxCurvature, float32 m_AbsDiagonal){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeCurvatureRobust(PointCloud* pc, float32 baseSigma, BOOL usePCANormal, float32 maxCurvature)\n";
	//// add channels for principle curvatures & directions
	checkAttribute(pc, "curv2", 2, VAD::DATA_FORMAT_FLOAT32);
	checkAttribute(pc, "tangent_u", 3, VAD::DATA_FORMAT_FLOAT32);
	checkAttribute(pc, "tangent_v", 3, VAD::DATA_FORMAT_FLOAT32);
	AAT posAAT = pc->getAAT("position");
	AAT normalAAT = pc->getAAT("normal");
	AAT flagsAAT = pc->getAAT("flags");
	AAT c2AAT = pc->getAAT("curv2");
	AAT tUAAT = pc->getAAT("tangent_u");
	AAT tVAAT = pc->getAAT("tangent_v");

	SceneObjectIterator *iter = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *ps = mIt->lockBlockReadWrite();

	AttachedIndexedOctree * knnOctree = PointSetKNNQuery::createOctree(ps);
	

	card32 index = 0;
	float32 radius = baseSigma * m_AbsDiagonal * 3.0f;
	mpcard numPoints = ps->getNumEntries();

#ifdef USEOPENMP
	omp_set_dynamic(0);     // Explicitly disable dynamic teams
	omp_set_num_threads(NUMCORE); // Use customized threads for all consecutive parallel regions
#pragma omp parallel for schedule(static)
#endif
	for (int i_p = 0; i_p < numPoints; i_p++)
	{
		std::vector<Vector3f> posBuffer;
		FastSphereQuerry * query = new FastSphereQuerry(knnOctree, ps);
		Vector3f pos = ps->get3f(i_p, posAAT);
		mpcard numNN;
		mpcard *pts;
		query->querry(pos, radius, &pts, numNN);

		//card32 numNN;
		//query.setTargetPoint(pos);
		//int32 *pts = query.getAllIndices(numNN);

		Vector3f normal  = ps->get3f(i_p, normalAAT);
		Matrix3f tangentSystem = calcTangentSystem(normal);
		Matrix3f invTangentSystem = tangentSystem.transpose();

		if (posBuffer.size() < numNN)
		{
			posBuffer.resize(numNN);
		}

		for (card32 i = 0; i < numNN; i++)
		{
			Vector3f tmp = ps->get3f(pts[i], posAAT) - pos;
			posBuffer[i] = invTangentSystem * tmp / (sqrt(2.0f) * baseSigma * m_AbsDiagonal);
		}

		StaticVector<float32, 6> coeff;
		Monomials2ndOrder::computeCoeffs(&posBuffer[0], (card32)numNN, coeff, 1.0f);

		float b_u = coeff[1];
		float b_v = coeff[2];
		float A_uu = coeff[3];
		float A_uv = coeff[4];
		float A_vv = coeff[5];

		Matrix2f A = makeMatrix2f(A_uu, A_uv,
			A_uv, A_vv);
		Vector2f lambda;
		Matrix2f eVects;
		A.computeEigenStructure(lambda, eVects);

		if (lambda[0] < 0){
			lambda[0] = -1 * lambda[0];
			eVects[0] = makeVector2f(-1 * eVects[0][0], -1 * eVects[0][1]);
		}
		if (lambda[1] < 0){
			lambda[1] = -1 * lambda[1];
			eVects[1] = makeVector2f(-1 * eVects[1][0], -1 * eVects[1][1]);
		}

		float32 lambda1 = lambda[0];
		float32 lambda2 = lambda[1];

		Vector2f v1 = eVects[0];
		Vector2f v2 = eVects[1];

		if (fabs(lambda1) < fabs(lambda2)) {
			swap(lambda1, lambda2);
			swap(v1, v2);
		}

		Vector3f newTangentU = tangentSystem[0] * v1[0] + tangentSystem[1] * v1[1];
		Vector3f newTangentV = normal.crossProduct(newTangentU);

		if (lambda1 > maxCurvature){
			lambda1 = 0;
			lambda2 = 0;
		}

		ps->set2f(i_p, c2AAT, makeVector2f(lambda1, lambda2));
		ps->set3f(i_p, tUAAT, newTangentU);
		ps->set3f(i_p, tVAAT, newTangentV);

		delete query;
	}

	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}


void PCICLPointCloudProcessing::computeCuratureKNN(PointCloud *pc, float32 maxCurvature, card32 numNN, float32 baseSigma, float32 diagonal){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeCuratureKNN(PointCloud* pc, float32 baseSigma, BOOL usePCANormal, float32 maxCurvature)\n";
	//// add channels for principle curvatures & directions
	checkAttribute(pc, "curv2", 2, VAD::DATA_FORMAT_FLOAT32);
	checkAttribute(pc, "tangent_u", 3, VAD::DATA_FORMAT_FLOAT32);
	checkAttribute(pc, "tangent_v", 3, VAD::DATA_FORMAT_FLOAT32);
	AAT posAAT = pc->getAAT("position");
	AAT normalAAT = pc->getAAT("normal");
	AAT flagsAAT = pc->getAAT("flags");
	AAT c2AAT = pc->getAAT("curv2");
	AAT tUAAT = pc->getAAT("tangent_u");
	AAT tVAAT = pc->getAAT("tangent_v");

	SceneObjectIterator* it = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	BasicPointCloudIterator* basicIt = dynamic_cast<BasicPointCloudIterator*>(it);
	ModifyablePointCloudIterator* modIt = dynamic_cast<ModifyablePointCloudIterator*>(it);
	HierarchicalKNNIterator* knnIt = new HierarchicalKNNIterator(pc, 32, NULL);
	PointSet *ps = modIt->lockBlockReadWrite();

	long pIndex = 0;
	progressWindow->pushStep(true, "computeCuratureKNN");

	while (!basicIt->atEnd()){
		const Vector3f& p = basicIt->get3f(posAAT);
		const Vector3f& normal = basicIt->get3f(normalAAT);
		knnIt->setSeekPointAndReset(p);
		knnIt->next();	// Skip self

		Matrix3f tangentSystem = calcTangentSystem(normal);
		Matrix3f invTangentSystem = tangentSystem.transpose();
		std::vector<Vector3f> posBuffer;
		posBuffer.resize(numNN);

		card32 numNeighbors = 0;
		bool converged = false;
		while (!converged){
			knnIt->next();
			const Vector3f& np = knnIt->get3f(posAAT);
			Vector3f tmp = np - p;
			posBuffer[numNeighbors] = invTangentSystem * tmp / (sqrt(2.0f) * baseSigma * diagonal); // here need to define baseSigma and bbDiagonal
			numNeighbors++;
			if (numNeighbors >= numNN)
				converged = true;
		}

		StaticVector<float32, 6> coeff;
		Monomials2ndOrder::computeCoeffs(&posBuffer[0], (card32)numNN, coeff, 1.0f);

		float b_u = coeff[1];
		float b_v = coeff[2];
		float A_uu = coeff[3];
		float A_uv = coeff[4];
		float A_vv = coeff[5];

		Matrix2f A = makeMatrix2f(A_uu, A_uv,
			A_uv, A_vv);
		Vector2f lambda;
		Matrix2f eVects;
		A.computeEigenStructure(lambda, eVects);

		if (lambda[0] < 0){
			lambda[0] = -1 * lambda[0];
			eVects[0] = makeVector2f(-1 * eVects[0][0], -1 * eVects[0][1]);
		}
		if (lambda[1] < 0){
			lambda[1] = -1 * lambda[1];
			eVects[1] = makeVector2f(-1 * eVects[1][0], -1 * eVects[1][1]);
		}

		float32 lambda1 = lambda[0];
		float32 lambda2 = lambda[1];

		Vector2f v1 = eVects[0];
		Vector2f v2 = eVects[1];

		if (fabs(lambda1) < fabs(lambda2)) {
			swap(lambda1, lambda2);
			swap(v1, v2);
		}

		Vector3f newTangentU = tangentSystem[0] * v1[0] + tangentSystem[1] * v1[1];
		Vector3f newTangentV = normal.crossProduct(newTangentU);

		if (lambda1 > maxCurvature){
			lambda1 = 0;
			lambda2 = 0;
		}

		ps->set2f(pIndex, c2AAT, makeVector2f(lambda1, lambda2));
		ps->set3f(pIndex, tUAAT, newTangentU);
		ps->set3f(pIndex, tVAAT, newTangentV);

		if (pIndex % 11111 == 0)
		{
			progressWindow->progress((float32)pIndex / (float32)pc->getNumPoints() *100.0f);
		}

		pIndex++;
		basicIt->next();
	}
	progressWindow->popSteps();

	delete it;
	delete knnIt;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

std::vector<std::vector<Vector3f>> PCICLPointCloudProcessing::computeRFUniformCurvature(PointCloud * pcFeature, card32 numOrientationBins){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeRFUniformCurvature(PointCloud * pcFeature, card32 numOrientationBins)\n";

	SceneObjectIterator *iter = pcFeature->createIterator(SOIT::CAP_BASIC_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	const PointSet *psFeature = bIt->lockBlockRead();

	AAT featposAAT = pcFeature->getAAT("position");
	AAT featlocXAAT = pcFeature->getAAT("locX");
	AAT featlocYAAT = pcFeature->getAAT("locY");
	AAT featlocZAAT = pcFeature->getAAT("locZ");

	std::vector<std::vector<Vector3f>> rf;
	rf.resize(pcFeature->getNumPoints());
	for (mpcard i = 0; i < pcFeature->getNumPoints(); i++){
		Vector3f pos = psFeature->get3f(i, featposAAT);
		Vector3f locX = psFeature->get3f(i, featlocXAAT);
		Vector3f locY = psFeature->get3f(i, featlocYAAT); // use the principle curvature of a larger area as the reference frame
		Vector3f locZ = psFeature->get3f(i, featlocZAAT);
		Matrix3f frameTangent;
		frameTangent[0] = locY;
		frameTangent[1] = locZ;
		frameTangent[2] = locX;

		rf[i].resize(numOrientationBins);
		for (mpcard j = 0; j < numOrientationBins; j++){
			float32 oBinAngle = (float32)j / (float32)numOrientationBins * (float32)M_PI;
			Vector3f newtU = frameTangent*((XAXIS_VECTOR3F*cos(oBinAngle) + YAXIS_VECTOR3F*sin(oBinAngle)));
			rf[i][j] = newtU;
		}
	}

	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
	return rf;
}

std::vector<DVectorF> PCICLPointCloudProcessing::computeFeaturefromRF(PointCloud* pcFeatureSingle, PointCloud* pcFeatureMulti, std::vector<std::vector<Vector3f>> rf){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeFeaturefromRF(PointCloud* pcFeatureSingle, PointCloud* pcFeatureMulti, std::vector<std::vector<Vector3f>> rf)\n";
	std::vector<DVectorF> idxSingle2Multi;
	idxSingle2Multi.resize(rf.size());

	SceneObjectIterator *iter = pcFeatureSingle->createIterator(SOIT::CAP_BASIC_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	const PointSet *psFeatureSingle = bIt->lockBlockRead();

	SceneObjectIterator *iterMulti = pcFeatureMulti->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iterMulti != NULL);
	BasicPointCloudIterator *bItMulti = dynamic_cast<BasicPointCloudIterator*>(iterMulti);
	ModifyablePointCloudIterator *mItMulti = dynamic_cast<ModifyablePointCloudIterator*>(iterMulti);
	PointSet *psFeatureMulti = mItMulti->lockBlockReadWrite();

	AAT featposAAT = pcFeatureSingle->getAAT("position");
	AAT featlocXAAT = pcFeatureSingle->getAAT("locX");
	AAT featlocYAAT = pcFeatureSingle->getAAT("locY");
	AAT featlocZAAT = pcFeatureSingle->getAAT("locZ");
	AAT featc2AAT = pcFeatureSingle->getAAT("curv2");

	AAT featnormalAAT = pcFeatureSingle->getAAT("normal");
	AAT feattangentUAAT = pcFeatureSingle->getAAT("tangent_u");
	AAT feattangentVAAT = pcFeatureSingle->getAAT("tangent_v");

	mpcard numFeaturesScratch = 0;
	for (mpcard i = 0; i < rf.size(); i++){
		numFeaturesScratch += rf[i].size();
	}

	numFeaturesScratch = numFeaturesScratch * 2; // the ambiguity of sign
	psFeatureMulti->clearAndSetup(1, numFeaturesScratch, pcFeatureSingle->getDescr());

	int32 count = 0;
	for (mpcard i = 0; i < rf.size(); i++){
		idxSingle2Multi[i].setDim(rf[i].size() * 2);
		for (mpcard j = 0; j < rf[i].size(); j++){
			psFeatureMulti->set3f(count, featposAAT, psFeatureSingle->get3f(i, featposAAT));
			psFeatureMulti->set3f(count, featlocXAAT, psFeatureSingle->get3f(i, featlocXAAT));
			psFeatureMulti->set3f(count, featlocYAAT, rf[i][j]);
			psFeatureMulti->set3f(count, featlocZAAT, psFeatureSingle->get3f(i, featlocXAAT).crossProduct(rf[i][j]));
			psFeatureMulti->set2f(count, featc2AAT, psFeatureSingle->get2f(i, featc2AAT));
			psFeatureMulti->set3f(count, featnormalAAT, psFeatureSingle->get3f(i, featnormalAAT));
			psFeatureMulti->set3f(count, feattangentUAAT, psFeatureSingle->get3f(i, feattangentUAAT));
			psFeatureMulti->set3f(count, feattangentVAAT, psFeatureSingle->get3f(i, feattangentVAAT));
			idxSingle2Multi[i][j * 2] = count;
			count += 1;

			psFeatureMulti->set3f(count, featposAAT, psFeatureSingle->get3f(i, featposAAT));
			psFeatureMulti->set3f(count, featlocXAAT, psFeatureSingle->get3f(i, featlocXAAT));
			Vector3f v = rf[i][j] / (-1);
			psFeatureMulti->set3f(count, featlocYAAT, v);
			psFeatureMulti->set3f(count, featlocZAAT, psFeatureSingle->get3f(i, featlocXAAT).crossProduct(v));
			psFeatureMulti->set2f(count, featc2AAT, psFeatureSingle->get2f(i, featc2AAT));
			psFeatureMulti->set3f(count, featnormalAAT, psFeatureSingle->get3f(i, featnormalAAT));
			psFeatureMulti->set3f(count, feattangentUAAT, psFeatureSingle->get3f(i, feattangentUAAT));
			psFeatureMulti->set3f(count, feattangentVAAT, psFeatureSingle->get3f(i, feattangentVAAT));
			idxSingle2Multi[i][j * 2 + 1] = count;
			count += 1;
		}
	}

	if (0)
	{
		debugRenderer->beginRenderJob_OneFrame("", 0, true);
		for (mpcard f = 0; f < 100; f++)
		{
			Vector3f pos_render = psFeatureMulti->get3f(f, featposAAT);
			Vector3f locX_render = psFeatureMulti->get3f(f, featlocXAAT);
			Vector3f locY_render = psFeatureMulti->get3f(f, featlocYAAT);
			Vector3f locZ_render = psFeatureMulti->get3f(f, featlocZAAT);
			debugRenderer->addLine(pos_render, pos_render + locX_render / 20, makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(1.0f, 0.0f, 0.0f), 2);
			debugRenderer->addLine(pos_render, pos_render + locY_render / 20, makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(0.0f, 1.0f, 0.0f), 2);
			debugRenderer->addLine(pos_render, pos_render + locZ_render / 20, makeVector3f(1.0f, 1.0f, 1.0f), makeVector3f(0.0f, 0.0f, 1.0f), 2);
		}
		debugRenderer->endRenderJob();
	}

	bItMulti->unlock();
	delete iterMulti;
	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
	return idxSingle2Multi;
}

void PCICLPointCloudProcessing::computeFeaturefromRFAAT(PointCloud* pcFeatureSingle, PointCloud* pcFeatureMulti, std::vector<std::vector<Vector3f>> rf){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeFeaturefromRFAAT(PointCloud* pcFeatureSingle, PointCloud* pcFeatureMulti, std::vector<std::vector<Vector3f>> rf)\n";
	card32 dim_idxSingle2MultiAAT = (card32)rf[0].size() * 2;
	
	SceneObjectIterator *iterMulti = pcFeatureMulti->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iterMulti != NULL);
	BasicPointCloudIterator *bItMulti = dynamic_cast<BasicPointCloudIterator*>(iterMulti);
	ModifyablePointCloudIterator *mItMulti = dynamic_cast<ModifyablePointCloudIterator*>(iterMulti);
	PointSet *psFeatureMulti = mItMulti->lockBlockReadWrite();

	mpcard numFeaturesScratch = 0;
	for (mpcard i = 0; i < rf.size(); i++){
		numFeaturesScratch += rf[i].size();
	}

	numFeaturesScratch = numFeaturesScratch * 2; // the ambiguity of sign
	psFeatureMulti->clearAndSetup(1, numFeaturesScratch, pcFeatureSingle->getDescr());

	checkAttribute(pcFeatureSingle, "idxSingle2Multi", dim_idxSingle2MultiAAT, VAD::DATA_FORMAT_CARD32);
	std::vector<DVectorF> idxSingle2Multi;
	idxSingle2Multi.resize(rf.size());

	SceneObjectIterator *iter = pcFeatureSingle->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *psFeatureSingle = mIt->lockBlockReadWrite();

	AAT featposAAT = psFeatureSingle->getAAT("position");
	AAT featlocXAAT = psFeatureSingle->getAAT("locX");
	AAT featlocYAAT = psFeatureSingle->getAAT("locY");
	AAT featlocZAAT = psFeatureSingle->getAAT("locZ");
	AAT featc2AAT = psFeatureSingle->getAAT("curv2");

	AAT featnormalAAT = psFeatureSingle->getAAT("normal");
	AAT feattangentUAAT = psFeatureSingle->getAAT("tangent_u");
	AAT feattangentVAAT = psFeatureSingle->getAAT("tangent_v");
	AAT featidxSingle2MultiAAT = psFeatureSingle->getAAT("idxSingle2Multi");

	card32 count = 0;
	for (mpcard i = 0; i < rf.size(); i++){
		idxSingle2Multi[i].setDim(rf[i].size() * 2);
		for (mpcard j = 0; j < rf[i].size(); j++){
			psFeatureMulti->set3f(count, featposAAT, psFeatureSingle->get3f(i, featposAAT));
			psFeatureMulti->set3f(count, featlocXAAT, psFeatureSingle->get3f(i, featlocXAAT));
			psFeatureMulti->set3f(count, featlocYAAT, rf[i][j]);
			psFeatureMulti->set3f(count, featlocZAAT, psFeatureSingle->get3f(i, featlocXAAT).crossProduct(rf[i][j]));
			psFeatureMulti->set2f(count, featc2AAT, psFeatureSingle->get2f(i, featc2AAT));
			psFeatureMulti->set3f(count, featnormalAAT, psFeatureSingle->get3f(i, featnormalAAT));
			psFeatureMulti->set3f(count, feattangentUAAT, psFeatureSingle->get3f(i, feattangentUAAT));
			psFeatureMulti->set3f(count, feattangentVAAT, psFeatureSingle->get3f(i, feattangentVAAT));
			idxSingle2Multi[i][j * 2] = count;
			count += 1;

			psFeatureMulti->set3f(count, featposAAT, psFeatureSingle->get3f(i, featposAAT));
			psFeatureMulti->set3f(count, featlocXAAT, psFeatureSingle->get3f(i, featlocXAAT));
			Vector3f v = rf[i][j] / (-1);
			psFeatureMulti->set3f(count, featlocYAAT, v);
			psFeatureMulti->set3f(count, featlocZAAT, psFeatureSingle->get3f(i, featlocXAAT).crossProduct(v));
			psFeatureMulti->set2f(count, featc2AAT, psFeatureSingle->get2f(i, featc2AAT));
			psFeatureMulti->set3f(count, featnormalAAT, psFeatureSingle->get3f(i, featnormalAAT));
			psFeatureMulti->set3f(count, feattangentUAAT, psFeatureSingle->get3f(i, feattangentUAAT));
			psFeatureMulti->set3f(count, feattangentVAAT, psFeatureSingle->get3f(i, feattangentVAAT));
			idxSingle2Multi[i][j * 2 + 1] = count;
			count += 1;
		}

		card8 *ptr = (card8*)psFeatureSingle->getDataPointer(i);
		ptr += featidxSingle2MultiAAT.getOffset();
		card32 *fptr = (card32*)ptr;
		for (int j = 0; j < (int)dim_idxSingle2MultiAAT; j++) {
			*fptr = idxSingle2Multi[i][j];
			//debugOutput << *fptr << "\n";
			fptr++;
		}
		//debugOutput << idxSingle2Multi[i] << "\n";
	}

	bItMulti->unlock();
	delete iterMulti;
	bIt->unlock();
	delete iter;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

void PCICLPointCloudProcessing::computeNormalPCAUnify(PointCloud * pc, float32 radiusResampler, card32 numNN, card32 numNNsampled){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeNormalPCAUnify(PointCloud * pc, float32 radiusResampler, card32 numNN, card32 numNNsampled)\n";
	AAT pospcAAT = pc->getAAT("position");
	checkAttribute(pc, "normal", 3, VAD::DATA_FORMAT_FLOAT32);
	
	SceneObjectIterator *iter = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *ps = mIt->lockBlockReadWrite();

	// resample 
	UnstructuredInCorePointCloud * pcsampled = new UnstructuredInCorePointCloud;
	pcsampled->setPointSet(dynamic_cast<PointSet*>(ps->copy()));
	PCCResampler::resamplePC(pcsampled, radiusResampler, false);

	// pca normal on both the fullres and the lowres 
	NormalEstimator::estimateNormals(pcsampled, numNNsampled, -1.0f, -1.0f, ENVIRONMENT_KNN, false);
	NormalUnifier::unifyNormalsHoppe(pcsampled->getPointSet(), numNNsampled);
	NormalEstimator::estimateNormals(pc, numNN, -1.0f, -1.0f, ENVIRONMENT_KNN, false);

	// check sign
	AAT normalpcAAT = pc->getAAT("normal");
	AAT normalpcsampledAAT = pcsampled->getAAT("normal");
	AttachedIndexedOctree * knnOctree_pcsampled;
	PointSetKNNQuery * knn_pcsampled;
	knnOctree_pcsampled = PointSetKNNQuery::createOctree(pcsampled->getPointSet());
	knn_pcsampled = new PointSetKNNQuery(pcsampled->getPointSet(), knnOctree_pcsampled, NULL_VECTOR3F, pospcAAT);

	for (int32 i = 0; i < pc->getNumPoints(); i++){
		Vector3f pos = ps->get3f(i, pospcAAT);
		knn_pcsampled->setTargetPoint(pos);
		card32 index;{
			bool found;
			knn_pcsampled->getNextNearestPointIndex(index, found);
			if (!found) throw EInvalidState("point set empty");
		}
		Vector3f normalpc = ps->get3f(i, normalpcAAT);
		Vector3f normalpcsampled = pcsampled->getPointSet()->get3f(index, normalpcsampledAAT);

		if (normalpc*normalpcsampled < 0){
			ps->set3f(i, normalpcAAT, -normalpc);
		}
	}

	bIt->unlock();
	delete iter;
	delete knn_pcsampled;
	delete knnOctree_pcsampled;
	delete pcsampled;
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

void PCICLPointCloudProcessing::computeNormalPCACamera(PointCloud *pc, card32 numNN){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeNormalPCACamera(PointCloud *pc, card32 numNN)\n";
	NormalEstimator::estimateNormals(pc, numNN, -1.0f, -1.0f, ENVIRONMENT_KNN, false);
	PCCNormalEstimatorPCA::unifyNormalsByCameraPosition(pc);
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

void PCICLPointCloudProcessing::computeNormalPCA(PointCloud *pc, card32 numNN){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeNormalPCA(PointCloud *pc, card32 numNN)\n";
	NormalEstimator::estimateNormals(pc, numNN, -1.0f, -1.0f, ENVIRONMENT_KNN, false);
	debugOutput << "Time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

// compute slippage
void PCICLPointCloudProcessing::computeSlippageTranslation(PointCloud * pc, float32 m_Slippage_relBaseSigma, float32 m_AbsDiagonal){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeSlippageTranslation(PointCloud * pc, float32 m_Slippage_relBaseSigma, float32 m_AbsDiagonal)\n";
	float32 sigma = m_Slippage_relBaseSigma * m_AbsDiagonal;
	checkAttribute(pc, "slippage", 2, VAD::DATA_FORMAT_FLOAT32);
	AAT posAAT = pc->getAAT("position");
	AAT normalAAT = pc->getAAT("normal");
	AAT slippageAAT = pc->getAAT("slippage");

	SceneObjectIterator *iter = pc->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC);
	x4Assert(iter != NULL);
	BasicPointCloudIterator *bIt = dynamic_cast<BasicPointCloudIterator*>(iter);
	ModifyablePointCloudIterator *mIt = dynamic_cast<ModifyablePointCloudIterator*>(iter);
	PointSet *ps = mIt->lockBlockReadWrite();

	HierarchicalKNNIterator hIt(pc, 32, NULL);
	progressWindow->pushStep(true, "compute slippage");
	for (mpcard i = 0; i < pc->getNumPoints(); i++)
	{
		Vector3f pos = ps->get3f(i, posAAT);
		Matrix3f normalCov = NormalCov(pos, sigma, &hIt, normalAAT);
		Matrix3f eigenVectors;
		Vector3f eigenValues;
		normalCov.computeEigenStructure(eigenValues, eigenVectors);
		ps->set2f(i, slippageAAT, makeVector2f(fabs(eigenValues[2]), fabs(eigenValues[1])));
		if (i % 2 == 0) {
			progressWindow->progress((float)i / (float)pc->getNumPoints()*100.0f);
		}
	}
	progressWindow->popStep();
	bIt->unlock();
	delete iter;
}

// remove outlier
void PCICLPointCloudProcessing::removeOutlier_dist2KNN(PointCloud * pc, card32 k, float32 sigma_outlier){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::computeSlippageTranslation(PointCloud * pc, float32 m_Slippage_relBaseSigma, float32 m_AbsDiagonal)\n";
	//checkAttribute(pc, "meandist", 1, VAD::DATA_FORMAT_FLOAT32);
	AAT POSITION = pc->getAAT("position");
	AAT FLAGS = pc->getAAT("flags");
	//AAT MEANDIST = pc->getAAT("meandist");


	SceneObjectIterator* it = pc->createIterator(SOIT(SOIT::CAP_BASIC_PC | SOIT::CAP_MODIFYABLE_PC | SOIT::CAP_DYNAMIC_PC));
	BasicPointCloudIterator* basicIt = dynamic_cast<BasicPointCloudIterator*>(it);
	ModifyablePointCloudIterator* modIt = dynamic_cast<ModifyablePointCloudIterator*>(it);
	DynamicPointCloudIterator* dynIt = dynamic_cast<DynamicPointCloudIterator*>(it);
	ModifyableKNNIterator* knnIt = new ModifyableKNNIterator(pc, 32, NULL);

	if (basicIt == NULL){
		debugOutput << "Cannot create basic iterator.\n"; debugOutput.flush();
		return;
	}

	if (modIt == NULL){
		debugOutput << "Cannot create modifyable iterator.\n"; debugOutput.flush();
		return;
	}

	if (dynIt == NULL){
		debugOutput << "Cannot create dynamic iterator.\n"; debugOutput.flush();
		return;
	}

	progressWindow->pushStep(true, "Reset");
	basicIt->reset();
	unsigned long pointIndex = 0;
	while (!basicIt->atEnd()){
		const Vector3f& p = basicIt->get3f(POSITION);
		if (_isnan(p[0]) || _isnan(p[1]) || _isnan(p[2])){
			debugOutput << "Resampler: point cloud contains NAN point ... deleting point\n"; debugOutput.flush();
			dynIt->deleteCurrent();
		}
		if (!_finite(p[0]) || !_finite(p[1]) || !_finite(p[2])){
			debugOutput << "Resampler: point cloud contains INF point ... deleting point\n"; debugOutput.flush();
			dynIt->deleteCurrent();
		}
		modIt->set1i(FLAGS, 0);
		basicIt->next();
		pointIndex++;
		if (pointIndex % 1000 == 0)
			progressWindow->progress(float32(pointIndex) / float32(pc->getNumPoints()) * 100);
	}
	bool aborted;
	progressWindow->popStep(&aborted);

	card32 numPointStart = pc->getNumPoints();
	std::vector<float32> mean_dist_array;
	mean_dist_array.resize(numPointStart);

	// Mark all delete points
	progressWindow->pushStep(true, "compute mean distance");
	basicIt->reset();
	pointIndex = 0;
	while (!basicIt->atEnd()){
		const Vector3f& p = basicIt->get3f(POSITION);
		knnIt->setSeekPointAndReset(p);
		knnIt->next();			// Skip self
		float32 count = 0;
		float32 dist = 0;
		while (count < k){
			const Vector3f& np = knnIt->get3f(POSITION);
			float sqrD = (p - np) * (p - np);
			dist += sqrt(sqrD);
			knnIt->next();
			count += 1;
		}
		mean_dist_array[pointIndex] = dist / count;
		basicIt->next();
		pointIndex++;
		if (pointIndex % 1000 == 0)
			progressWindow->progress(float32(pointIndex) / float32(pc->getNumPoints()) * 100);
	}
	progressWindow->popStep(&aborted);

	// compute global mean & std
	float32 mean_global = 0;
	float32 var_global = 0;
	float32 std_global = 0;
	for (mpcard i = 0; i < numPointStart; i++)
	{
		mean_global += mean_dist_array[i];
	}
	mean_global = mean_global * (1 / float32(numPointStart));

	for (mpcard i = 0; i < numPointStart; i++)
	{
		var_global += (mean_dist_array[i] - mean_global) * (mean_dist_array[i] - mean_global);
	}
	var_global = var_global * (1 / float32(numPointStart));
	std_global = sqrt(var_global);

	// filter out the points with large statistics error
	float32 mean_max = mean_global + std_global * sigma_outlier;
	basicIt->reset();
	pointIndex = 0;
	while (!basicIt->atEnd()){
		if (mean_dist_array[pointIndex] > mean_max)
			modIt->set1i(FLAGS, 1);
		basicIt->next();
		pointIndex++;
	}

	// Delete marked points
	progressWindow->pushStep(true, "Deleting");
	basicIt->reset();
	pointIndex = 0;
	card32 numDeleted = 0;
	while (!basicIt->atEnd()){
		if (basicIt->get1i(FLAGS) == 1){
			dynIt->deleteCurrent();
			numDeleted++;
		}
		else
			basicIt->next();
		++pointIndex;
		if (pointIndex % 1000 == 0)
			progressWindow->progress(100.0f * float32(pointIndex) / float32(pc->getNumPoints()));
	}
	progressWindow->popStep(&aborted);

	delete knnIt;
	basicIt->unlock();
	delete it;
}

// tools
void PCICLPointCloudProcessing::computeUnify(PointSet * ps, string name_attribute, card32 seed, card32 numNN){
	Timer timer;
	//debugOutput << "PCICLPointCloudProcessing::computeUnify(PointSet * ps, string name_attribute, card32 seed, card32 numNN)\n";
	AAT POSITION = ps->getAAT("position");
	AAT ATTRIBUTE = ps->getAAT(name_attribute);
	AAT FLAGS = ps->getAAT("flags");

	// Reset all flags
	for (unsigned int pIndex = 0; pIndex < ps->getNumEntries(); pIndex++){
		ps->set1i(pIndex, FLAGS, ps->get1i(pIndex, FLAGS) & (0xFFFFFFFF ^ NORMAL_ESTIMATOR_FLAG));
	}

	// Compute neighbor graph, assign confidence, remember best

	IGGraph* riemannian = new IGGraph();
	riemannian->createNodes(ps->getNumEntries());
	AttachedIndexedOctree* octree = PointSetKNNQuery::createOctree(ps, 32);
	PointSetKNNQuery knn(ps, octree, makeVector3f(100000, 0, 0), POSITION);

	// Set std::vector with largest z coordinate
	knn.setTargetPoint(makeVector3f(0, 0, 100000));
	card32 nIndex;
	bool found;
	knn.getNextNearestPointIndex(nIndex, found);
	Vector3f rootPoint = ps->get3f(nIndex, POSITION);
	riemannian->setRoot(nIndex);

	//create stack
	FifoStack<card32> stack;
	stack.push(nIndex);

	//Build riemannian graph
	int pointcounter = 0;
	int knncounter = 0;
	while (!stack.empty()){
		pointcounter++;
		const card32 currentIndex = stack.pop();
		Vector3f current = ps->get3f(currentIndex, POSITION);
		//calculate average point distance
		knn.setTargetPoint(current);
		float avgDist = 0;
		knncounter = 0;
		while (knncounter++ < (int)numNN){
			knn.getNextNearestPointIndex(nIndex, found);
			Vector3f neighbor = ps->get3f(nIndex, POSITION);
			avgDist += (current - neighbor).getSqrNorm();
		}
		avgDist /= float32(numNN);
		Vector3f n_i, n_j;
		knn.setTargetPoint(current);
		ps->set1i(currentIndex, FLAGS, ps->get1i(currentIndex, FLAGS) | NORMAL_ESTIMATOR_FLAG);
		n_i = ps->get3f(currentIndex, ATTRIBUTE);
		knncounter = 0;
		float costs;
		while (knncounter < (int)numNN){
			knn.getNextNearestPointIndex(nIndex, found);
			Vector3f neighbor = ps->get3f(nIndex, POSITION);
			n_j = ps->get3f(nIndex, ATTRIBUTE);
			costs = (1 - abs(n_i * n_j) * exp(-pow((current - neighbor).getSqrNorm(), 2) / pow(avgDist, 2)));
			//costs = (1 - abs(n_i * n_j));
			riemannian->createEdge(currentIndex, nIndex, costs);
			if (!(ps->get1i(nIndex, FLAGS) & NORMAL_ESTIMATOR_FLAG)){
				stack.push(nIndex);
				ps->set1i(nIndex, FLAGS, ps->get1i(nIndex, FLAGS) | NORMAL_ESTIMATOR_FLAG);
			}
			knncounter++;
		}
	}

	// Build minimal spanning tree
	IGGraph* mst = new IGGraph();
	mst->createNodes(riemannian->getNodeCount());
	mst->setRoot(riemannian->getRoot());
	PriorityQueue<float, IGEdge*>* pq = new PriorityQueue<float, IGEdge*>();

	IGNode* root = riemannian->getNode(riemannian->getRoot());
	root->setFlag(true);
	list<IGEdge*>::iterator it = root->getEdgeIterator();
	for (unsigned i = 0; i < root->getEdgeCount(); i++){
		IGEdge* current = *it;
		pq->insert(current->getCosts(), current);
		it++;
	}

	IGEdge* currentEdge;
	IGNode* currentNode;
	IGNode* targetNode;
	int connectedNodes = 0;

	while (!pq->isEmpty()){
		currentEdge = pq->pop_first();
		currentNode = riemannian->getNode(currentEdge->getStartNode());
		targetNode = riemannian->getNode(currentEdge->getTargetNode());
		if (targetNode->isFlagSet()) continue;
		mst->createEdge(currentNode->getIndex(), targetNode->getIndex(), currentEdge->getCosts());
		list<IGEdge*>::iterator it = targetNode->getEdgeIterator();
		for (unsigned i = 0; i < targetNode->getEdgeCount(); i++){
			if (!riemannian->getNode((*it)->getTargetNode())->isFlagSet()) pq->insert((*it)->getCosts(), *it);
			it++;
		}
		targetNode->setFlag(true);
		connectedNodes++;
	}

	// Start growing with best
	root = mst->getNode(mst->getRoot());
	root->setFlag(true);
	card32 currentIndex = root->getIndex();
	knn.setTargetPoint(ps->get3f(currentIndex, POSITION));
	Vector3f attribute = ps->get3f(currentIndex, ATTRIBUTE);
	if (attribute * makeVector3f(1, 0, 0) < 0){
		ps->set3f(currentIndex, ATTRIBUTE, -attribute);
	}

	currentNode = root;
	list<IGEdge*>* dfs = new list<IGEdge*>();
	it = root->getEdgeIterator();
	for (unsigned i = 0; i < root->getEdgeCount(); i++){
		dfs->push_front(*it);
		it++;
	}

	/***********************************
	* FOR TESTING ONLY! SHUFFLES NORMALS
	************************************/
	/*
	for (int i=0;i<mst->getNodeCount();i++){
	knnIt->setSeekPointAndReset(ps->get3f(i,POSITION));
	if(i%2==0) knnIt->set3f(NORMAL,-(knnIt->get3f(NORMAL)));
	}
	*/

	//Traversing MST in depth first order
	int nodeCounter = 0;
	while (!dfs->empty()){
		currentEdge = dfs->front();
		dfs->pop_front();
		currentNode = mst->getNode(currentEdge->getStartNode());
		targetNode = mst->getNode(currentEdge->getTargetNode());
		if (!targetNode->isFlagSet()){
			Vector3f currentAttribute, targetAttribute;
			card32 currentIndex = currentNode->getIndex();
			knn.setTargetPoint(ps->get3f(currentIndex, POSITION));
			currentAttribute = ps->get3f(currentIndex, ATTRIBUTE);
			card32 targetIndex = targetNode->getIndex();
			knn.setTargetPoint(ps->get3f(targetIndex, POSITION));
			targetAttribute = ps->get3f(targetIndex, ATTRIBUTE);
			if (currentAttribute * targetAttribute < 0) ps->set3f(targetIndex, ATTRIBUTE, -targetAttribute);
			targetNode->setFlag(true);
			nodeCounter++;
			it = targetNode->getEdgeIterator();
			for (unsigned i = 0; i<targetNode->getEdgeCount(); i++){
				if (!mst->getNode((*it)->getTargetNode())->isFlagSet()) dfs->push_front(*it);
				it++;
			}
		}
	}

	delete pq;
	delete riemannian;
	delete mst;
	delete dfs;
	delete octree;
	//debugOutput << "computeUnify time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

// sampling from mesh
void PCICLPointCloudProcessing::sampleMeshRandomly(UnstructuredInCoreTriangleMesh *sourceMesh, UnstructuredInCorePointCloud * pc, card64 expNumPoints, bool createTangentDiscs,
	float32 tangentDiscOversampling, bool surfaceNormalSmoothing, bool trackTriangleID){
	Timer timer;
	debugOutput << "PCICLPointCloudProcessing::sampleMeshRandomly(UnstructuredInCoreTriangleMesh *sourceMesh, UnstructuredInCorePointCloud * pc, card64 expNumPoints, bool createTangentDiscs, float32 tangentDiscOversampling, bool surfaceNormalSmoothing, bool trackTriangleID)\n";
	if (!sourceMesh) {
		throw X4Exception("X4::sampleMeshRandomly() - source is no TriangleMeshPointCloud.");
	}
	UnstructuredInCorePointCloud *sourceAsPC = dynamic_cast<UnstructuredInCorePointCloud*>(sourceMesh);
	x4Assert(sourceAsPC);

	if (!pc) {
		throw X4Exception("X4::sampleMeshRandomly() - pc is no PointCloud.");
	}

	// append necessary fields to destPointCloud
	if (!pc->getDescr()->equal(sourceAsPC->getDescr()))
	{
		const VertexDescriptor * vd = sourceAsPC->getDescr();
		for (card32 i = 0; i < vd->getNumAttributes(); i++)
		{
			checkAttribute(pc, vd->getAttribute(i).name, vd->getAttribute(i).numComponents, vd->getAttribute(i).dataFormat);
		}
	}

	AAT faceIDXAAT;
	if (trackTriangleID)
	{
		faceIDXAAT = pc->getAAT("faceidx");
	}

	// vertex descriptors, AATS...
	const VertexDescriptor *sourceVertexDescr = sourceAsPC->getDescr();
	const VertexDescriptor *destVertexDescr = pc->getDescr();
	AAT SOURCE_POS = sourceAsPC->getAAT("position");
	AAT TANGENT_U;
	if (pc->providesAttribute("tangent_u")) {
		TANGENT_U = pc->getAAT("tangent_u");
	}
	else {
		if (createTangentDiscs) throw X4Exception("X4::sampleMeshRandomly() - cannot create tangent discs, attribute channel is missing.");
	}
	AAT TANGENT_V;
	if (pc->providesAttribute("tangent_v")) {
		TANGENT_V = pc->getAAT("tangent_v");
	}
	else {
		if (createTangentDiscs) throw X4Exception("X4::sampleMeshRandomly() - cannot create tangent discs, attribute channel is missing.");
	}
	AAT normalAAT;
	if (surfaceNormalSmoothing)
	{
		normalAAT = sourceAsPC->getAAT("normal");
	}

	if (!sourceMesh->getTriangleDescr()->providesAttribute("index")) {
		throw X4Exception("X4::sampleMeshRandomly() - source mesh pc does not provide index channel.");
	}
	AAT SOURCE_INDEX = sourceMesh->getTriangleDescr()->getToken("index");

	// create iterators
	SceneObjectIterator *sourceMeshIt = sourceMesh->createIterator(SOIT::CAP_BASIC_PC | SOIT::CAP_TRIANGLE_MESH);
	BasicPointCloudIterator *sourceMeshIt_basic = dynamic_cast<BasicPointCloudIterator*>(sourceMeshIt);
	SceneObjectIterator *destIt = pc->createIterator(SOIT::CAP_DYNAMIC_PC);
	DynamicPointCloudIterator *destIt_dyn = dynamic_cast<DynamicPointCloudIterator*>(destIt);
	BasicPointCloudIterator *desbasicIt = dynamic_cast<BasicPointCloudIterator*>(destIt);
	ModifyablePointCloudIterator *destmodIt = dynamic_cast<ModifyablePointCloudIterator*>(destIt);
	sourceMeshIt_basic->lockBlockRead();
	destmodIt->lockBlockReadWrite();

	const PointSet *vertices = sourceAsPC->getPointSet();

	//progressWindow->pushStep(true, "area calc", 0.5f);
	//progressWindow->pushStep(true, "sampling", 0.5f);
	// step 1: calculate overall area...
	float64 area = 0;
	card32 numTriangles = sourceMesh->getNumTriangles();
	card32 actTriangle = 0;
	bool aborted = false;
	//while (!sourceMeshIt_basic->atEnd() && !aborted) {
	while (!sourceMeshIt_basic->atEnd()) {
		Vector3i indices = sourceMeshIt_basic->get3i(SOURCE_INDEX);
		Vector3f pos[3];
		pos[0] = vertices->get3f(indices[0], SOURCE_POS);
		pos[1] = vertices->get3f(indices[1], SOURCE_POS);
		pos[2] = vertices->get3f(indices[2], SOURCE_POS);

		float32 thisarea = 0.5f*norm((pos[1] - pos[0]).crossProduct(pos[2] - pos[0]));

		area += (float64)thisarea;

		sourceMeshIt_basic->next();

		// progress
		//aborted = !progressWindow->progress((float)actTriangle++ / (float)numTriangles * 100.0f);

	}
	sourceMeshIt_basic->reset();
	//progressWindow->popStep(&aborted);

	float64 areaPerPoint = area / expNumPoints;
	float32 tangentSize = sqrt(areaPerPoint * tangentDiscOversampling);
	
	// step 2: sampling...
	char *destBuffer = new char[destVertexDescr->getSize()];
	//typedef const char *constCharPtr;
	const char **vbuff = new const char*[3];
	card32 actPoint = 0;
	actTriangle = 0;
	while (!sourceMeshIt_basic->atEnd()) {
		Vector3i indices = sourceMeshIt_basic->get3i(SOURCE_INDEX);
		Vector3f pos[3];
		pos[0] = vertices->get3f(indices[0], SOURCE_POS);
		pos[1] = vertices->get3f(indices[1], SOURCE_POS);
		pos[2] = vertices->get3f(indices[2], SOURCE_POS);

		Vector3f tNormal = (pos[1] - pos[0]).crossProduct(pos[2] - pos[0]);
		float64 thisarea = 0.5f*norm(tNormal);

		float64 numPoints = thisarea / areaPerPoint;

		card64 numPointsInt = floor(numPoints);
		float32 remPts = (float32)(numPoints - numPointsInt);

		if (remPts > rnd01()) numPointsInt++;

		for (card64 pnum = 0; pnum < numPointsInt; pnum++) {

			vbuff[0] = vertices->getDataPointer(indices[0]);
			vbuff[1] = vertices->getDataPointer(indices[1]);
			vbuff[2] = vertices->getDataPointer(indices[2]);

			float32 u = rnd01();
			float32 v = rnd01();
			if (u + v > 1) {
				float32 nu = 1.0f - v;
				v = 1.0f - u;
				u = nu;
			}

			float32 factors[3];
			factors[0] = u;
			factors[1] = v;
			factors[2] = 1.0f - u - v;

			sourceVertexDescr->interpolate(vbuff, factors, 3, destBuffer);

			if (surfaceNormalSmoothing)
			{
				// never smooth surface
			}

			if (createTangentDiscs) {
				// naver do this 
			}

			destIt_dyn->add(destBuffer);
			
			if (trackTriangleID)
			{
				destmodIt->set1u(faceIDXAAT, actTriangle);
			}
			desbasicIt->next();
		}
		sourceMeshIt_basic->next();
		actTriangle += 1;
	}

	sourceMeshIt_basic->unlock();
	desbasicIt->unlock();
	delete[] vbuff;
	delete[] destBuffer;
	delete sourceMeshIt;
	delete destIt;
	debugOutput << "Robust Curvature time: " << convertTimeToString(timer.getDeltaValue()) << "\n";
}

// clip mesh by RegularVolume
void PCICLPointCloudProcessing::LinefromVoxel(Vector3f pbegin, Vector3f pend, int32 idx, Line3f &l){
	Vector3f p1, p2;
	switch (idx) {
	case 0:
		p1 = pbegin;
		p2 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
		break;
	case 1:
		p1 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
		p2 = makeVector3f(pend[0], pbegin[1], pend[2]);
		break;
	case 2:
		p1 = makeVector3f(pend[0], pbegin[1], pend[2]);
		p2 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
		break;
	case 3:
		p1 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
		p2 = pbegin;
		break;
	case 4:
		p1 = makeVector3f(pend[0], pbegin[1], pbegin[2]);
		p2 = makeVector3f(pend[0], pend[1], pbegin[2]);
		break;
	case 5:
		p1 = makeVector3f(pend[0], pbegin[1], pend[2]);
		p2 = makeVector3f(pend[0], pend[1], pend[2]);
		break;
	case 6:
		p1 = makeVector3f(pbegin[0], pbegin[1], pend[2]);
		p2 = makeVector3f(pbegin[0], pend[1], pend[2]);
		break;
	case 7:
		p1 = makeVector3f(pbegin[0], pbegin[1], pbegin[2]);
		p2 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
		break;
	case 8:
		p1 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
		p2 = makeVector3f(pend[0], pend[1], pbegin[2]);
		break;
	case 9:
		p1 = makeVector3f(pend[0], pend[1], pbegin[2]);
		p2 = makeVector3f(pend[0], pend[1], pend[2]);
		break;
	case 10:
		p1 = makeVector3f(pend[0], pend[1], pend[2]);
		p2 = makeVector3f(pbegin[0], pend[1], pend[2]);
		break;
	case 11:
		p1 = makeVector3f(pbegin[0], pend[1], pend[2]);
		p2 = makeVector3f(pbegin[0], pend[1], pbegin[2]);
		break;
	default:
		;
	}
	//debugOutput<<"idx: "<<idx<<"p1: "<<p1<<", p2: "<<p2<<"\n";
	l = Line3f(p2 - p1, p1);
}

void PCICLPointCloudProcessing::facefromVoxel(Vector3f pbegin, Vector3f pend, std::vector<Plane3f> &p){
	Vector3f normal;
	Plane3f plane;

	// face one
	normal = makeVector3f(0, -1, 0);
	p[0] = Plane3f(normal, pbegin);

	// face two
	normal = makeVector3f(0, 1, 0);
	p[1] = Plane3f(normal, pend);

	// face three
	normal = makeVector3f(0, 0, -1);
	p[2] = Plane3f(normal, pbegin);

	// face four
	normal = makeVector3f(0, 0, 1);
	p[3] = Plane3f(normal, pend);

	// face five
	normal = makeVector3f(-1, 0, 0);
	p[4] = Plane3f(normal, pbegin);

	// face six
	normal = makeVector3f(1, 0, 0);
	p[5] = Plane3f(normal, pend);
}

void PCICLPointCloudProcessing::edgesfromTriangle(std::vector<Vector3f> p, std::vector<Line3f> & edges){
	// edge one
	edges[0] = Line3f(p[1] - p[0], p[0]);
	// edge two
	edges[1] = Line3f(p[2] - p[1], p[1]);
	// edge three
	edges[2] = Line3f(p[0] - p[2], p[2]);
}

void PCICLPointCloudProcessing::clipMeshbyRegularVolume(UnstructuredInCoreTriangleMesh * mesh, RegularVolume * volume, UnstructuredInCoreTriangleMesh * meshClipped, float32 clip_min_vertexdist_pos, float32 clip_min_vertexdist_neg, vector<vector<int>> & idx_voxel2triangleClipped){
	debugOutput << "PCICLPointCloudProcessing::clipMeshbyRegularVolume()\n";

	int nVoxelsXYZ_input = volume->getWidth() * volume->getHeight() * volume->getDepth();
	Vector3f model_lowerCorner = volume->getBoundingBox().lowerCorner;
	Vector3f model_upperCorner = volume->getBoundingBox().upperCorner;

	bool render_flag_render_clip = false;
	VertexArray va_points(mesh);
	VertexArray va_triangles(mesh->getTriangles());
	//AAT positionAAT = pcFullres->getAAT("position");
	vector<vector<int>> idx_voxel2triangleOri;
	idx_voxel2triangleOri.resize(nVoxelsXYZ_input);

	vector<Vector3f> lowerTriangle;
	vector<Vector3f> upperTriangle;
	card32 numFaces = va_triangles.getNumElements();

	{
		float32 min_x = 1000;
		float32 max_x = -1000;
		float32 min_y = 1000;
		float32 max_y = -1000;
		float32 min_z = 1000;
		float32 max_z = -1000;
		for (mpcard i = 0; i < numFaces; i++) {
			Vector3f p0, p1, p2;
			Vector3i tri = va_triangles.getIndex3i(i);
			p0 = va_points.getPosition3f(tri[0]);
			p1 = va_points.getPosition3f(tri[1]);
			p2 = va_points.getPosition3f(tri[2]);
			Vector3f v0 = makeVector3f(p0[0], p1[0], p2[0]);
			Vector3f v1 = makeVector3f(p0[1], p1[1], p2[1]);
			Vector3f v2 = makeVector3f(p0[2], p1[2], p2[2]);
			min_x = *min_element(v0.begin(), v0.end());
			max_x = *max_element(v0.begin(), v0.end());
			min_y = *min_element(v1.begin(), v1.end());
			max_y = *max_element(v1.begin(), v1.end());
			min_z = *min_element(v2.begin(), v2.end());
			max_z = *max_element(v2.begin(), v2.end());
			lowerTriangle.push_back(makeVector3f(min_x, min_y, min_z));
			upperTriangle.push_back(makeVector3f(max_x, max_y, max_z));
		}
	}

	{
		float32 min_x = 1000;
		float32 max_x = -1000;
		float32 min_y = 1000;
		float32 max_y = -1000;
		float32 min_z = 1000;
		float32 max_z = -1000;
		for (int i = 0; i < nVoxelsXYZ_input; i++)
		{
			// get the bounding box of the current voxel
			Vector3i loc = volume->transformIndexToWHD(i);
			float32 wf_lower = (float32)loc[0] / (float32)volume->getWidth();
			float32 hf_lower = (float32)loc[1] / (float32)volume->getHeight();
			float32 df_lower = (float32)loc[2] / (float32)volume->getDepth();
			Vector3f v_lower = makeVector3f(wf_lower, hf_lower, df_lower);
			v_lower = v_lower.componentProduct(model_upperCorner - model_lowerCorner) + model_lowerCorner;
			float32 wf_upper = (float32)(loc[0] + 1) / (float32)volume->getWidth();
			float32 hf_upper = (float32)(loc[1] + 1) / (float32)volume->getHeight();
			float32 df_upper = (float32)(loc[2] + 1) / (float32)volume->getDepth();
			Vector3f v_upper = makeVector3f(wf_upper, hf_upper, df_upper);
			v_upper = v_upper.componentProduct(model_upperCorner - model_lowerCorner) + model_lowerCorner;
			float32 x_normVoxel = abs(v_lower[0] - v_upper[0]);
			float32 y_normVoxel = abs(v_lower[1] - v_upper[1]);
			float32 z_normVoxel = abs(v_lower[2] - v_upper[2]);

			// for every triangle, test if it intersect with the voxel
			for (int i_f = 0; i_f < (int)numFaces; i_f++)
			{
				float32 x_normTriangle = abs(lowerTriangle[i_f][0] - upperTriangle[i_f][0]);
				float32 y_normTriangle = abs(lowerTriangle[i_f][1] - upperTriangle[i_f][1]);
				float32 z_normTriangle = abs(lowerTriangle[i_f][2] - upperTriangle[i_f][2]);
				Vector4f x_range = makeVector4f(v_lower[0], v_upper[0], lowerTriangle[i_f][0], upperTriangle[i_f][0]);
				Vector4f y_range = makeVector4f(v_lower[1], v_upper[1], lowerTriangle[i_f][1], upperTriangle[i_f][1]);
				Vector4f z_range = makeVector4f(v_lower[2], v_upper[2], lowerTriangle[i_f][2], upperTriangle[i_f][2]);
				min_x = *min_element(x_range.begin(), x_range.end());
				max_x = *max_element(x_range.begin(), x_range.end());
				min_y = *min_element(y_range.begin(), y_range.end());
				max_y = *max_element(y_range.begin(), y_range.end());
				min_z = *min_element(z_range.begin(), z_range.end());
				max_z = *max_element(z_range.begin(), z_range.end());
				float32 x_normOverlap = abs(min_x - max_x);
				float32 y_normOverlap = abs(min_y - max_y);
				float32 z_normOverlap = abs(min_z - max_z);
				if ((x_normOverlap <= x_normTriangle + x_normVoxel) & (y_normOverlap <= y_normTriangle + y_normVoxel) & (z_normOverlap <= z_normTriangle + z_normVoxel))
				{
					idx_voxel2triangleOri[i].push_back(i_f);
				}
			}
		}
	}

	// clip triangles by the voxels
	int32 total_nPoints = 0;
	int32 total_nFaces = 0;
	std::vector<Vector3f> Triangles;
	std::vector<Vector3f> TrianglesNormal;
	std::vector<Vector3i> Triangles2Voxel;
	Triangles.resize(0);
	TrianglesNormal.resize(0);
	Triangles2Voxel.resize(0);

	const char **vbuff = new const char*[3];
	for (int i_voxel = 0; i_voxel < nVoxelsXYZ_input; i_voxel++)//for (int i_voxel = 0; i_voxel < nVoxelsXYZ_input; i_voxel++)//for (int i = 0; i < nVoxelsXYZ_input; i++)//
	{
		if (render_flag_render_clip)
		{
			debugRenderer->beginRenderJob_OneFrame("", i_voxel, true);
		}
		if (idx_voxel2triangleOri[i_voxel].size() > 0)
		{
			Vector3i loc = volume->transformIndexToWHD(i_voxel);
			float32 wf_lower = (float32)loc[0] / (float32)volume->getWidth();
			float32 hf_lower = (float32)loc[1] / (float32)volume->getHeight();
			float32 df_lower = (float32)loc[2] / (float32)volume->getDepth();
			Vector3f v_lower = makeVector3f(wf_lower, hf_lower, df_lower);
			v_lower = v_lower.componentProduct(model_upperCorner - model_lowerCorner) + model_lowerCorner;
			float32 wf_upper = (float32)(loc[0] + 1) / (float32)volume->getWidth();
			float32 hf_upper = (float32)(loc[1] + 1) / (float32)volume->getHeight();
			float32 df_upper = (float32)(loc[2] + 1) / (float32)volume->getDepth();
			Vector3f v_upper = makeVector3f(wf_upper, hf_upper, df_upper);
			v_upper = v_upper.componentProduct(model_upperCorner - model_lowerCorner) + model_lowerCorner;

			// set up the six faces for the voxel
			std::vector<Plane3f> faces;
			faces.resize(6);
			facefromVoxel(v_lower, v_upper, faces);
			for (int i_triangle = 0; i_triangle < idx_voxel2triangleOri[i_voxel].size(); i_triangle++)
			{

				Plane3f faceTriangle;
				std::vector<Vector3f> p;
				p.resize(3);
				// get the current triangle 
				Vector3i tri = va_triangles.getIndex3i(idx_voxel2triangleOri[i_voxel][i_triangle]);

				// three vertices
				card32 num_nodeTriangleClipped = 0;
				for (mpcard j = 0; j < 3; j++){
					p[j] = va_points.getPosition3f(tri[j]);
					// vertices that are inside of the voxels are added to the list of node
					if ((p[j][0] > (v_lower[0] - clip_min_vertexdist_pos)) & (p[j][0] < (v_upper[0] + clip_min_vertexdist_pos)) & (p[j][1] > (v_lower[1] - clip_min_vertexdist_pos)) & (p[j][1] < (v_upper[1] + clip_min_vertexdist_pos)) & (p[j][2] > (v_lower[2] - clip_min_vertexdist_pos)) & (p[j][2] < (v_upper[2] + clip_min_vertexdist_pos)))
					{
						num_nodeTriangleClipped += 1;
					}
				}
				std::vector<Vector3f> nodeTriangleClipped;
				nodeTriangleClipped.resize(num_nodeTriangleClipped);
				card32 count = 0;
				for (mpcard j = 0; j < 3; j++){
					// vertices that are inside of the voxels are added to the list of node
					if ((p[j][0] > (v_lower[0] - clip_min_vertexdist_pos)) & (p[j][0] < (v_upper[0] + clip_min_vertexdist_pos)) & (p[j][1] > (v_lower[1] - clip_min_vertexdist_pos)) & (p[j][1] < (v_upper[1] + clip_min_vertexdist_pos)) & (p[j][2] > (v_lower[2] - clip_min_vertexdist_pos)) & (p[j][2] < (v_upper[2] + clip_min_vertexdist_pos)))
					{
						nodeTriangleClipped[count] = p[j];
						count += 1;
					}
				}

				// compute coordinate frame (longest edge is always dim 0)
				Vector3f longestEdge = p[1] - p[0];
				for (mpcard j = 1; j<3; j++)
				{
					if (norm(p[(j + 1) % 3] - p[j]) > norm(longestEdge))
						longestEdge = p[(j + 1) % 3] - p[j];
				}

				Vector3f normal = normalize((p[1] - p[0]).crossProduct(p[2] - p[0]));
				Matrix3f frame;
				frame[0] = normalize(longestEdge);
				frame[1] = normalize(normal.crossProduct(longestEdge));
				frame[2] = normal;
				Matrix3f invFrame = invertMatrix(frame);
				float32 det_invFrame = invFrame.getDeterminant();

				if ((det_invFrame != 0) && (det_invFrame == det_invFrame)) // check det for zero or NaN
				{
					Matrix3f M_projection2world = frame;
					Vector3f frameCenter = p[0];
					faceTriangle = Plane3f(normal, p[0]);
					std::vector<Line3f> edges;
					edges.resize(3);
					edgesfromTriangle(p, edges);
					// for each edge in the triangle, intersect with all six faces in the cube
					for (int i_e = 0; i_e < 3; i_e++)
					{
						Vector3f color_edge = makeVector3f(1.0f, 1.0f, 0.0f);
						if (render_flag_render_clip){
							debugRenderer->addLine(edges[i_e].getPoint(), edges[i_e].getPoint() + edges[i_e].getDirection(), color_edge, color_edge, 0.01f);
						}
						for (int i_f = 0; i_f < 6; i_f++)
						{
							// intersect edges[i_e] with faces[i_f]
							Vector3f intersectionPoint;
							bool flag_inter = intersectPlaneLine(faces[i_f], edges[i_e], intersectionPoint);
							if (flag_inter)
							{
								// test if the intersection lies between the beginning and the end
								double dist_1 = (intersectionPoint - edges[i_e].getPoint()).getNorm() + (intersectionPoint - (edges[i_e].getPoint() + edges[i_e].getDirection())).getNorm();
								double dist_2 = edges[i_e].getDirection().getNorm();
								if (abs(dist_1 - dist_2) < clip_min_vertexdist_pos)
								{
									nodeTriangleClipped.push_back(intersectionPoint);
								}
							}
						}
					}
					std::vector<Line3f>().swap(edges);

					// for each edge in the cube, intersect with the triangle face
					for (int i_l = 0; i_l < 12; i_l++)
					{
						Line3f l;
						LinefromVoxel(v_lower, v_upper, i_l, l);
						Vector3f intersectionPoint;
						bool flag_inter = intersectPlaneLine(faceTriangle, l, intersectionPoint);
						if (render_flag_render_clip){
							debugRenderer->addLine(l.getPoint(), l.getPoint() + l.getDirection(), makeVector3f(1, 0, 0), makeVector3f(1, 0, 0), 0.01f);
						}

						if (flag_inter)
						{
							//see if the intersection is inside of the triangle using barycentric coordinates
							Vector3f v0 = p[2] - p[0];
							Vector3f v1 = p[1] - p[0];
							Vector3f v2 = intersectionPoint - p[0];
							float dot00 = v0 * v0;
							float dot01 = v0 * v1;
							float dot02 = v0 * v2;
							float dot11 = v1 * v1;
							float dot12 = v1 * v2;
							float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
							float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
							float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
							if ((u > 0) && (v > 0) && (u + v < 1)){
								nodeTriangleClipped.push_back(intersectionPoint);
							}
						}
					}

					// remove nodes that are outside of the voxel, add triangle vertices that is inside of the voxel 
					for (int i_n = (int)nodeTriangleClipped.size() - 1; i_n > -1; i_n--){
						if ((nodeTriangleClipped[i_n][0] > (v_lower[0] - clip_min_vertexdist_pos)) & (nodeTriangleClipped[i_n][0] < (v_upper[0] + clip_min_vertexdist_pos)) & (nodeTriangleClipped[i_n][1] > (v_lower[1] - clip_min_vertexdist_pos)) & (nodeTriangleClipped[i_n][1] < (v_upper[1] + clip_min_vertexdist_pos)) & (nodeTriangleClipped[i_n][2] > (v_lower[2] - clip_min_vertexdist_pos)) & (nodeTriangleClipped[i_n][2] < (v_upper[2] + clip_min_vertexdist_pos))){
						}
						else{
							nodeTriangleClipped[i_n] = nodeTriangleClipped.back(); // remove
							nodeTriangleClipped.pop_back();
						}
					}
					// remove the redundant points (too close) from the last element, search forward
					for (int i_n = (int)nodeTriangleClipped.size() - 1; i_n > 0; i_n--)
					{
						for (int j_n = i_n - 1; j_n > -1; j_n--)
						{
							// compute the norm of the difference between nodeTriangleClipped[i_n] and nodeTriangleClipped[j_n]
							Vector3f diff_v = nodeTriangleClipped[i_n] - nodeTriangleClipped[j_n];
							float32 diff = diff_v.getNorm();
							//
							if (diff < clip_min_vertexdist_neg)
							{
								nodeTriangleClipped[i_n] = nodeTriangleClipped.back();
								nodeTriangleClipped.pop_back();
								break;
							}
						}
					}
					if (nodeTriangleClipped.size() >= 3)
					{
						// project points onto the triangle plane
						ConvexHull2D* hull = new ConvexHull2D((card32)nodeTriangleClipped.size() + 1); // plus one is very important to prevent exception thrown due to the HACK for adding the pivot point !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						for (int i_p = 0; i_p < (int)nodeTriangleClipped.size(); i_p++)
						{
							Vector2f pf = shrink3To2(invFrame*(nodeTriangleClipped[i_p] - frameCenter));
							hull->addPoint(pf);
						}
						hull->computeConvexHull();

						card32 numHullPoints = hull->getNumHullPoints();
						std::vector<Vector3f> nodeHull;
						nodeHull.resize(numHullPoints);
						// project point on the hull back into 3D
						for (card32 i_h = 0; i_h < numHullPoints; i_h++){
							Vector2f pf = hull->getHullPoint(i_h);
							Vector3f pf3D = makeVector3f(pf[0], pf[1], 0);
							nodeHull[i_h] = M_projection2world * pf3D + frameCenter;
						}
						for (int i_t = 0; i_t < (int)numHullPoints - 1; i_t++)
						{
							total_nPoints = total_nPoints + 3;
							Triangles.push_back(nodeHull[numHullPoints - 1]);
							Triangles.push_back(nodeHull[i_t]);
							Triangles.push_back(nodeHull[i_t + 1]);
							TrianglesNormal.push_back(normal);
							Triangles2Voxel.push_back(makeVector3i(total_nPoints - 3, total_nPoints - 2, total_nPoints - 1));
							// add triangle list to voxels
							idx_voxel2triangleClipped[i_voxel].push_back(total_nPoints / 3 - 1);
							if (render_flag_render_clip){
								debugRenderer->addLine(nodeHull[numHullPoints - 1], nodeHull[i_t], makeVector4f(0, 1, 0, 0), makeVector4f(0, 1, 0, 0), 3, false, 0);
								debugRenderer->addLine(nodeHull[i_t], nodeHull[i_t + 1], makeVector4f(0, 1, 0, 0), makeVector4f(0, 1, 0, 0), 3, false, 0);
								debugRenderer->addLine(nodeHull[i_t + 1], nodeHull[numHullPoints - 1], makeVector4f(0, 1, 0, 0), makeVector4f(0, 1, 0, 0), 3, false, 0);
							}
						}
						std::vector<Vector3f>().swap(nodeHull);
						delete hull;
					}

					if (render_flag_render_clip)
					{
						debugOutput << "\n*****************************\n";
						for (int i = 0; i < nodeTriangleClipped.size(); i++)
						{
							debugOutput << nodeTriangleClipped[i] << "\n";
							debugRenderer->addSphere(nodeTriangleClipped[i], 0.002f, makeVector3f(0.3f, 0.9f, 0.3f));
						}
						debugOutput << "*****************************\n";
					}
				}
				std::vector<Vector3f>().swap(nodeTriangleClipped);
				std::vector<Vector3f>().swap(p);
			} // for (int i_triangle = 0; i_triangle < 3; i_triangle++)
		} // if (idx_voxel2triangleOri[i_voxel].size() > 0)
		else{

		}

		if (render_flag_render_clip)
		{
			debugRenderer->endRenderJob();
		}
	}

	// add Triangles to mesh_points and mesh_indices
	total_nFaces = total_nPoints / 3;
	VertexDescriptor * vd_points = createDefaultVertexDescriptor(true, true, true, true, false);
	VertexDescriptor * vd_face = new VertexDescriptor;
	vd_face->pushAttrib(VAD("index", 3, VAD::DATA_FORMAT_INT32));
	meshClipped->clearAndSetup(vd_points, total_nPoints, vd_face, total_nFaces);
	delete vd_points;
	delete vd_face;
	VertexArray mesh_pointsClipped(meshClipped->getPointSet());
	VertexArray mesh_indicesClipped(meshClipped->getTriangles());
	Vector3f color = makeVector3f(0.75, 0.75, 0.75);
	// insert data into mesh_points and mesh_indices
	for (int i_f = 0; i_f < total_nFaces; i_f++){
		mesh_pointsClipped.setPosition3f(i_f * 3, Triangles[i_f * 3]);
		mesh_pointsClipped.setPosition3f(i_f * 3 + 1, Triangles[i_f * 3 + 1]);
		mesh_pointsClipped.setPosition3f(i_f * 3 + 2, Triangles[i_f * 3 + 2]);
		mesh_pointsClipped.setColor3f(i_f * 3, color);
		mesh_pointsClipped.setColor3f(i_f * 3 + 1, color);
		mesh_pointsClipped.setColor3f(i_f * 3 + 2, color);
		mesh_pointsClipped.setNormal3f(i_f * 3 + 2, TrianglesNormal[i_f]);
		mesh_pointsClipped.setNormal3f(i_f * 3 + 2, TrianglesNormal[i_f]);
		mesh_pointsClipped.setNormal3f(i_f * 3 + 2, TrianglesNormal[i_f]);
		mesh_indicesClipped.setIndex3i(i_f, Triangles2Voxel[i_f]);
	}
}

// ICP
bool PCICLPointCloudProcessing::computeICP_R2(PointSet * ps_source, PointSet * ps_target, const Vector3f &pos_ref_source, Vector3f &pos_ref_target, HierarchicalKNNIterator * m_hIt_source, HierarchicalKNNIterator * m_hIt_target, float32 m_MedPointDistance, int32 m_MaxNumIterations, float32 m_OutlierDistance, float32 m_ConvergeDistance, Vector3f &v_axis, Matrix3f &rotation){
	AAT pos_sourceAAT = ps_source->getAAT("position");
	AAT normal_sourceAAT = ps_source->getAAT("normal");
	AAT pos_targetAAT = ps_target->getAAT("position");
	AAT normal_targetAAT = ps_target->getAAT("normal");

	// collect surfels in ps_source
	std::vector<Surfel> surfels_source;
	float summed_weight = 0;
	for (mpcard i = 0; i < ps_source->getNumEntries(); i++)
	{
		Vector3f pos = ps_source->get3f(i, pos_sourceAAT);
		Surfel surfel;
		surfel.pos = pos;
		surfel.normal = ps_source->get3f(i, normal_sourceAAT);
		surfel.weight = 1.0f;//exp( - dist_2 * inv_sigma_2 );
		surfels_source.push_back(surfel);
	}

	if (surfels_source.size() < 10)
		return false;

	float residual = 0;
	float movement = 0;
	std::vector<Vector3f> points_target(surfels_source.size());
	std::vector<Vector3f> points_source(surfels_source.size());
	std::vector<Vector3f> points_target_proj(surfels_source.size());
	std::vector<Vector3f> points_source_proj(surfels_source.size());

	for (int32 pass = 0; pass < m_MaxNumIterations; pass++)
	{
		residual = 0;
		summed_weight = 0;
		Vector3f centroid2 = NULL_VECTOR3F;
		Vector3f centroid1 = NULL_VECTOR3F;

		Vector3f centroid2_proj = NULL_VECTOR3F;
		Vector3f centroid1_proj = NULL_VECTOR3F;

		// calculate corresponding points
		for (int32 i = 0; i < surfels_source.size(); i++)
		{
			// transform surfel by current transformation
			Surfel surfelTransformed;
			surfelTransformed.pos = rotation * (surfels_source[i].pos - pos_ref_source) + pos_ref_target;
			surfelTransformed.normal = rotation * surfels_source[i].normal;

			// seek closest point
			m_hIt_target->setSeekPointAndReset(surfelTransformed.pos);
			Vector3f closestPoint = m_hIt_target->get3f(pos_targetAAT);
			Vector3f closestPointNormal = m_hIt_target->get3f(normal_targetAAT);
			surfels_source[i].dynamicWeight = pow(closestPointNormal * surfelTransformed.normal, 2.0f) *
				surfels_source[i].weight;

			points_source[i] = surfelTransformed.pos;
			points_target[i] = surfelTransformed.pos - closestPointNormal * (closestPointNormal * (surfelTransformed.pos - closestPoint));

			//project onto v_axis and pos_ref_target
			points_source_proj[i] = points_source[i] - v_axis *
				(v_axis * (points_source[i] - pos_ref_target));
			points_target_proj[i] = points_target[i] - v_axis *
				(v_axis * (points_target[i] - pos_ref_target));

			float projectedDistance = norm(points_source[i] - points_target[i]);
			surfels_source[i].dynamicWeight *= (projectedDistance > m_OutlierDistance*m_MedPointDistance) ? 0.0f : 1.0f;

			// current residual
			residual += projectedDistance * surfels_source[i].dynamicWeight;

			// centroids
			centroid1 += surfels_source[i].pos * surfels_source[i].dynamicWeight;
			centroid2 += points_target[i] * surfels_source[i].dynamicWeight;
			centroid1_proj += points_source_proj[i] * surfels_source[i].dynamicWeight;
			centroid2_proj += points_target_proj[i] * surfels_source[i].dynamicWeight;
			summed_weight += surfels_source[i].dynamicWeight;
		}

		
		if (summed_weight < 1e-20f)
			return false;

		centroid1 /= summed_weight;
		centroid2 /= summed_weight;
		centroid1_proj /= summed_weight;
		centroid2_proj /= summed_weight;
		residual /= summed_weight;

		Matrix3f crossCovariance = IDENTITY3F * 0;
		for (unsigned i = 0; i < surfels_source.size(); i++)
		{
			crossCovariance += outerProduct(points_source_proj[i] - centroid1_proj, points_target_proj[i] - centroid2_proj) * surfels_source[i].dynamicWeight;
		}

		// Prepare OpenCV Datasructure CvMat
		Matrix3f U, V;
		Vector3f eigenValues;
		CvMat cvMatInput, cvMat_U, cvMat_W, cvMat_Vt;
		cvInitMatHeader(&cvMatInput, 3, 3, CV_32FC1, crossCovariance.data());
		cvInitMatHeader(&cvMat_W, 3, 1, CV_32FC1, eigenValues.data());
		cvInitMatHeader(&cvMat_U, 3, 3, CV_32FC1, U.data());
		cvInitMatHeader(&cvMat_Vt, 3, 3, CV_32FC1, V.data());

		// Do SVD
		cvSVD(&cvMatInput, &cvMat_W, &cvMat_U, &cvMat_Vt, CV_SVD_MODIFY_A | CV_SVD_U_T);

		rotation = V.transpose()*U.transpose() * rotation;
		Vector3f oldPoint = pos_ref_target;
		pos_ref_target = rotation * (pos_ref_source - centroid1) + centroid2;
		movement = norm(pos_ref_target - oldPoint);

		//debugRenderer->beginRenderJob_OneFrame("", pass, false);
		////for (int32 i = 0; i < points_target.size(); i++)
		////{
		////	debugRenderer->addFastSphere(points_source[i], 0.0125f, makeVector3f(1, 0, 0), false);
		////}
		////debugRenderer->addFastSphere(centroid1, 0.05f, makeVector3f(1, 0, 0), false);

		////for (int32 i = 0; i < points_target.size(); i++)
		////{
		////	debugRenderer->addFastSphere(points_target[i], 0.0125f, makeVector3f(0, 1, 0), false);
		////}
		////debugRenderer->addFastSphere(centroid2, 0.05f, makeVector3f(0, 1, 0), false);

		////for (int32 i = 0; i < points_target.size(); i++)
		////{
		////	if (surfels_source[i].dynamicWeight > 0)
		////	{
		////		debugRenderer->addLine(points_source[i], points_target[i], makeVector4f(1, 1, 1, 1), makeVector4f(1, 1, 1, 1), 1, false, false);
		////	}
		////}

		//for (int32 i = 0; i < points_target_proj.size(); i++)
		//{
		//	debugRenderer->addFastSphere(points_source_proj[i], 0.0125f, makeVector3f(1, 0, 0), false);
		//}
		//debugRenderer->addFastSphere(centroid1_proj, 0.05f, makeVector3f(1, 0, 0), false);

		//for (int32 i = 0; i < points_target_proj.size(); i++)
		//{
		//	debugRenderer->addFastSphere(points_target_proj[i], 0.0125f, makeVector3f(0, 1, 0), false);
		//}
		//debugRenderer->addFastSphere(centroid2_proj, 0.05f, makeVector3f(0, 1, 0), false);

		//for (int32 i = 0; i < points_target_proj.size(); i++)
		//{
		//	if (surfels_source[i].dynamicWeight > 0)
		//	{
		//		debugRenderer->addLine(points_source_proj[i], points_target_proj[i], makeVector4f(1, 1, 1, 1), makeVector4f(1, 1, 1, 1), 1, false, false);
		//	}
		//}


		////for (int32 j = 0; j < surfels_source.size(); j++)
		////{
		////	Surfel s = surfels_source[j];
		////	Vector3f p = rotation * (s.pos - centroid1) + centroid2;
		////	debugRenderer->addFastSphere(p, 0.0125f, makeVector3f(1, 1, 0), false);
		////}
		//debugRenderer->endRenderJob();

		if (movement < m_ConvergeDistance)
			return true;
	}
	return false;
}

bool PCICLPointCloudProcessing::computeICP_R3(PointSet * ps_source, PointSet * ps_target, const Vector3f &pos_ref_source, Vector3f &pos_ref_target, HierarchicalKNNIterator * m_hIt_source, HierarchicalKNNIterator * m_hIt_target, float32 m_MedPointDistance, int32 m_MaxNumIterations, float32 m_OutlierDistance, float32 m_ConvergeDistance, Matrix3f &rotation){

	AAT pos_sourceAAT = ps_source->getAAT("position");
	AAT normal_sourceAAT = ps_source->getAAT("normal");
	AAT pos_targetAAT = ps_target->getAAT("position");
	AAT normal_targetAAT = ps_target->getAAT("normal");
	
	// collect surfels in ps_source
	std::vector<Surfel> surfels_source;
	float summed_weight = 0;
	for (mpcard i = 0; i < ps_source->getNumEntries(); i++)
	{
		Vector3f pos = ps_source->get3f(i, pos_sourceAAT);
		Surfel surfel;
		surfel.pos = pos;
		surfel.normal = ps_source->get3f(i, normal_sourceAAT);
		surfel.weight = 1.0f;//exp( - dist_2 * inv_sigma_2 );
		surfels_source.push_back(surfel);
	}

	if (surfels_source.size() < 10)
		return false;

	float residual = 0;
	float movement = 0;
	std::vector<Vector3f> points_target(surfels_source.size());
	std::vector<Vector3f> points_source(surfels_source.size());


	for (int32 pass = 0; pass < m_MaxNumIterations; pass++)
	{
		residual = 0;
		summed_weight = 0;
		Vector3f centroid2 = NULL_VECTOR3F;
		Vector3f centroid1 = NULL_VECTOR3F;
		
		// calculate corresponding points
		for (int32 i = 0; i < surfels_source.size(); i++)
		{
			// transform surfel by current transformation
			Surfel surfelTransformed;
			surfelTransformed.pos = rotation * (surfels_source[i].pos - pos_ref_source) + pos_ref_target;
			surfelTransformed.normal = rotation * surfels_source[i].normal;
			points_source[i] = surfelTransformed.pos;

			// seek closest point
			m_hIt_target->setSeekPointAndReset(surfelTransformed.pos);
			Vector3f closestPoint = m_hIt_target->get3f(pos_targetAAT);
			Vector3f closestPointNormal = m_hIt_target->get3f(normal_targetAAT);
			surfels_source[i].dynamicWeight = pow(closestPointNormal * surfelTransformed.normal, 2.0f) *
				surfels_source[i].weight;
			//project onto surfel
				Vector3f projectedPoint = surfelTransformed.pos - closestPointNormal *
				(closestPointNormal * (surfelTransformed.pos - closestPoint));


			float projectedDistance = norm(surfelTransformed.pos - projectedPoint);
			points_target[i] = projectedPoint;
			surfels_source[i].dynamicWeight *= (projectedDistance > m_OutlierDistance*m_MedPointDistance) ? 0.0f : 1.0f;

			// current residual
			residual += projectedDistance * surfels_source[i].dynamicWeight;

			// centroids
			centroid1 += surfels_source[i].pos * surfels_source[i].dynamicWeight;
			centroid2 += projectedPoint * surfels_source[i].dynamicWeight;
			summed_weight += surfels_source[i].dynamicWeight;
		}

		//centroid1 = pos_ref_source;
		//centroid2 = pos_ref_target;

		if (summed_weight < 1e-20f)
			return false;

		centroid1 /= summed_weight;
		centroid2 /= summed_weight;
		residual /= summed_weight;


		//debugOutput << "centroid1: " << centroid1 << ", centroid2: " << centroid2 << ", residual:" << residual << "\n";
		// estimate new transformation
		Matrix3f crossCovariance = IDENTITY3F * 0;
		for (unsigned i = 0; i < surfels_source.size(); i++)
		{
			crossCovariance += outerProduct(surfels_source[i].pos - centroid1, points_target[i] - centroid2) * surfels_source[i].dynamicWeight;
		}

		// Prepare OpenCV Datasructure CvMat
		Matrix3f U, V;
		Vector3f eigenValues;
		CvMat cvMatInput, cvMat_U, cvMat_W, cvMat_Vt;
		cvInitMatHeader(&cvMatInput, 3, 3, CV_32FC1, crossCovariance.data());
		cvInitMatHeader(&cvMat_W, 3, 1, CV_32FC1, eigenValues.data());
		cvInitMatHeader(&cvMat_U, 3, 3, CV_32FC1, U.data());
		cvInitMatHeader(&cvMat_Vt, 3, 3, CV_32FC1, V.data());

		// Do SVD
		cvSVD(&cvMatInput, &cvMat_W, &cvMat_U, &cvMat_Vt, CV_SVD_MODIFY_A | CV_SVD_U_T);

		rotation = V.transpose()*U.transpose();
		Vector3f oldPoint = pos_ref_target;
		pos_ref_target = rotation * (pos_ref_source - centroid1) + centroid2;
		movement = norm(pos_ref_target - oldPoint);

		//// compute the determinant
		////float32 dtm = determinant(rotation);
		////debugOutput << "dtm: " << dtm << "\n";
		//debugRenderer->beginRenderJob_OneFrame("", pass, false);
		//for (int32 i = 0; i < points_target.size(); i++)
		//{
		//	debugRenderer->addFastSphere(points_source[i], 0.0125f, makeVector3f(1, 0, 0), false);
		//}
		//debugRenderer->addFastSphere(centroid1, 0.05f, makeVector3f(1, 0, 0), false);

		//for (int32 i = 0; i < points_target.size(); i++)
		//{
		//	debugRenderer->addFastSphere(points_target[i], 0.0125f, makeVector3f(0, 1, 0), false);
		//}
		//debugRenderer->addFastSphere(centroid2, 0.05f, makeVector3f(0, 1, 0), false);

		//for (int32 i = 0; i < points_target.size(); i++)
		//{
		//	if (surfels_source[i].dynamicWeight > 0)
		//	{
		//		debugRenderer->addLine(points_source[i], points_target[i], makeVector4f(1, 1, 1, 1), makeVector4f(1, 1, 1, 1), 1, false, false);
		//	}
		//}
		//for (int32 j = 0; j < surfels_source.size(); j++)
		//{
		//	Surfel s = surfels_source[j];
		//	Vector3f p = rotation * (s.pos - centroid1) + centroid2;
		//	debugRenderer->addFastSphere(p, 0.0125f, makeVector3f(1, 1, 0), false);
		//}
		//debugRenderer->endRenderJob();

		if (movement < m_ConvergeDistance)
			return true;
	}

	return false;
}