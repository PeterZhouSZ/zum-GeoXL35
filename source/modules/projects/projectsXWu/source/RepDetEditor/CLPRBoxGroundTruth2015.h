//---------------------------------------------------------------------------
#ifndef CLPRBoxGroundTruth2015H
#define CLPRBoxGroundTruth2015H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"

namespace X4 {

	class PROJECTSXWU_API CLPRBoxGroundTruth2015 : public Persistent {
		X4_CLASS(CLPRBoxGroundTruth2015)
	public:
		card32 numPR;
		std::vector<float32> recall;
		std::vector<float32> prec;
	private:
		
	public:
		CLPRBoxGroundTruth2015();
		virtual void assign(const Object* obj, X4CopyContext *context = NULL);
		~CLPRBoxGroundTruth2015();

		virtual void read(InputObjectStream *s);
		virtual void write(OutputObjectStream *s) const;


	};
}

#endif
