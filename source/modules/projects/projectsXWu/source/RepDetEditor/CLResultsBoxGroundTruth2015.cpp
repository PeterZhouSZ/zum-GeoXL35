//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "CLResultsBoxGroundTruth2015.h"
//---------------------------------------------------------------------------
//#include "CopyObjectProperties.h"
//---------------------------------------------------------------------------
using namespace X4;
//---------------------------------------------------------------------------

IMPLEMENT_X4_CLASS( CLResultsBoxGroundTruth2015, 0 ) {
	BEGIN_CLASS_INIT( CLResultsBoxGroundTruth2015 );
	ADD_CARD32_PROP(numClass, 0)


}

CLResultsBoxGroundTruth2015::CLResultsBoxGroundTruth2015() 
{
	numClass = 0;;
	sizeClass.setDim(numClass);
	objCenter.resize(numClass);
	objFrame.resize(numClass);;
	objDim.resize(numClass);
	objLabel.resize(numClass);
}

void CLResultsBoxGroundTruth2015::assign(const Object* obj, X4CopyContext *context) {
	copyObjectProperties(obj, this);
}

CLResultsBoxGroundTruth2015::~CLResultsBoxGroundTruth2015() {

}

void CLResultsBoxGroundTruth2015::read(InputObjectStream *s){
	Persistent::read(s);
	sizeClass.read(s);
	objCenter.resize(numClass);
	for (mpcard i = 0; i < numClass; i++)
	{
		objCenter[i].resize(sizeClass[i]);
		for (mpcard j = 0; j < sizeClass[i]; j++)
		{
			objCenter[i][j].read(s);
		}
	}

	objFrame.resize(numClass);
	for (mpcard i = 0; i < numClass; i++)
	{
		objFrame[i].resize(sizeClass[i]);
		for (mpcard j = 0; j < sizeClass[i]; j++)
		{
			for (mpcard k = 0; k < 3; k++)
			{
				objFrame[i][j][k].read(s);
			}
		}
	}

	objDim.resize(numClass);
	for (mpcard i = 0; i < numClass; i++)
	{
		objDim[i].resize(sizeClass[i]);
		for (mpcard j = 0; j < sizeClass[i]; j++)
		{
			objDim[i][j].read(s);
		}
	}

	objLabel.resize(numClass);
	for (mpcard i = 0; i < numClass; i++)
	{
		s->read(objLabel[i]);
	}

}

void CLResultsBoxGroundTruth2015::write(OutputObjectStream *s) const{
	Persistent::write(s);
	sizeClass.write(s);
	for (mpcard i = 0; i < numClass; i++)
	{
		for (mpcard j = 0; j < sizeClass[i]; j++)
		{
			objCenter[i][j].write(s);
		}
	}

	for (mpcard i = 0; i < numClass; i++)
	{
		for (mpcard j = 0; j < sizeClass[i]; j++)
		{
			for (mpcard k = 0; k < 3; k++)
			{
				objFrame[i][j][k].write(s);
			}
		}
	}
	for (mpcard i = 0; i < numClass; i++)
	{
		for (mpcard j = 0; j < sizeClass[i]; j++)
		{
			objDim[i][j].write(s);
		}
	}

	for (mpcard i = 0; i < numClass; i++)
	{
		s->write(objLabel[i]);
	}
}


