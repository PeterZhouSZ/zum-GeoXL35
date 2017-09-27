//---------------------------------------------------------------------------
#ifndef CLResultsBoxGroundTruth2015H
#define CLResultsBoxGroundTruth2015H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"

namespace X4 {

    class PROJECTSXWU_API CLResultsBoxGroundTruth2015 : public Persistent {
		X4_CLASS(CLResultsBoxGroundTruth2015)
	public:
		card32 numClass;
		DVectorI sizeClass;
		std::vector<std::vector<Vector3f>> objCenter;
		std::vector<std::vector<Matrix3f>> objFrame;
		std::vector<std::vector<Vector3f>> objDim;
		std::vector<int32> objLabel;
	private:
		
	public:
		CLResultsBoxGroundTruth2015();
		virtual void assign(const Object* obj, X4CopyContext *context = NULL);
		~CLResultsBoxGroundTruth2015();

		virtual void read(InputObjectStream *s);
		virtual void write(OutputObjectStream *s) const;


	};
}

#endif
