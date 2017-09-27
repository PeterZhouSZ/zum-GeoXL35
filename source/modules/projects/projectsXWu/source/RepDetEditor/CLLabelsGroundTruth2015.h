//---------------------------------------------------------------------------
#ifndef CLLabelsGroundTruth2015H
#define CLLabelsGroundTruth2015H
//---------------------------------------------------------------------------
#include "CommonHdrXWu.h"

namespace X4 {

	class PROJECTSXWU_API CLLabelsGroundTruth2015 : public Persistent {
		X4_CLASS(CLLabelsGroundTruth2015)
	public:
		card32 numClass;
		std::vector<std::string> label;
	private:
		
	public:
		CLLabelsGroundTruth2015();
		virtual void assign(const Object* obj, X4CopyContext *context = NULL);
		~CLLabelsGroundTruth2015();

		virtual void read(InputObjectStream *s);
		virtual void write(OutputObjectStream *s) const;
	};
}

#endif
