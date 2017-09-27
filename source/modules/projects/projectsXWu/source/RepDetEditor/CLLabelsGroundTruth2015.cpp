//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "CLLabelsGroundTruth2015.h"
//---------------------------------------------------------------------------
//#include "CopyObjectProperties.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

IMPLEMENT_X4_CLASS( CLLabelsGroundTruth2015, 0 ) {
	BEGIN_CLASS_INIT( CLLabelsGroundTruth2015 );
	ADD_CARD32_PROP(numClass, 0)
}

CLLabelsGroundTruth2015::CLLabelsGroundTruth2015() 
{
	numClass = 0;;
	label.resize(numClass);
}

void CLLabelsGroundTruth2015::assign(const Object* obj, X4CopyContext *context) {
	copyObjectProperties(obj, this);
}

CLLabelsGroundTruth2015::~CLLabelsGroundTruth2015() {

}

void CLLabelsGroundTruth2015::read(InputObjectStream *s){
	Persistent::read(s);
	label.resize(numClass);
	for (mpcard i = 0; i < numClass; i++)
	{
		s->readString(label[i]);
	}
}

void CLLabelsGroundTruth2015::write(OutputObjectStream *s) const{
	Persistent::write(s);
	for (mpcard i = 0; i < numClass; i++)
	{
		s->writeString(label[i]);
	}
}
