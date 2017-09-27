//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "CLPRBoxGroundTruth2015.h"
//---------------------------------------------------------------------------
//#include "CopyObjectProperties.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

IMPLEMENT_X4_CLASS(CLPRBoxGroundTruth2015, 0) {
	BEGIN_CLASS_INIT(CLPRBoxGroundTruth2015);
	ADD_CARD32_PROP(numPR, 0)
}

CLPRBoxGroundTruth2015::CLPRBoxGroundTruth2015() 
{
	numPR = 0;
	recall.resize(numPR);
	prec.resize(numPR);
}

void CLPRBoxGroundTruth2015::assign(const Object* obj, X4CopyContext *context) {
	copyObjectProperties(obj, this);
}

CLPRBoxGroundTruth2015::~CLPRBoxGroundTruth2015() {

}

void CLPRBoxGroundTruth2015::read(InputObjectStream *s){
	Persistent::read(s);
	recall.resize(numPR);
	for (mpcard i = 0; i < numPR; i++)
	{
		s->read(recall[i]);
	}
	prec.resize(numPR);
	for (mpcard i = 0; i < numPR; i++)
	{
		s->read(prec[i]);
	}
}

void CLPRBoxGroundTruth2015::write(OutputObjectStream *s) const{
	Persistent::write(s);
	for (mpcard i = 0; i < numPR; i++)
	{
		s->write(recall[i]);
	}
	for (mpcard i = 0; i < numPR; i++)
	{
		s->write(prec[i]);
	}
}


