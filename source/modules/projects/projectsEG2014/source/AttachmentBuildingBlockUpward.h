#ifndef AttachmentBuildingBlockUpward_H
#define  AttachmentBuildingBlockUpward_H
//---------------------------------------------------------------------------
#include "projectsEG2014.h"
//---------------------------------------------------------------------------
#include "Persistent.h"
#include "AttachedData.h"
#include "DynamicLinearAlgebra.h"
#include "SparseLinearAlgebra.inline.h"

//---------------------------------------------------------------------------
namespace X4{

	class PROJECTSEG2014_API AttachmentBuildingBlockUpward : public AttachedData {
		X4_CLASS(AttachmentBuildingBlockUpward)

	public:
		// feature
		card32 numFeature;

		// segment
		card32 numSegmentation;

		// clique
		card32 numCliqe;
		std::vector<DVectorF> clique;
		DVectorF idxTemplate2Feature;

		// buiding block
		card32 numBB;
		DVectorF idxBB2Clique;
		DMatrixF CooC_M_dist;

		// supervision
		card32 numTemplate;
		std::vector<DVectorF> userTemplateFeature;
		std::vector<DVectorF> idxKey2Feature; // for each PM, stores the index from all parts to pcFeature
		std::vector<Vector3f> pm_bbox_lowerCorner;
		std::vector<Vector3f> pm_bbox_upperCorner;
		std::vector<Vector3f> pm_bboxCenter;
		std::vector<DVectorF> pm_coordFrame;

		//// pattern
		//card32 numPattern;
		//std::vector<DVectorF> patterns;

		AttachmentBuildingBlockUpward(void);
		~AttachmentBuildingBlockUpward(void);

		virtual void read(InputObjectStream *s);
		virtual void write(OutputObjectStream *s) const;
	};


}


#endif