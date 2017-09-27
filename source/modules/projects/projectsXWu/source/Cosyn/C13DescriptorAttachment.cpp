//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "C13DescriptorAttachment.h"
//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
//---------------------------------------------------------------------------

IMPLEMENT_CLASS( C13DescriptorAttachment, 0 )
{
    BEGIN_CLASS_INIT( C13DescriptorAttachment );
    ADD_CARD32_PROP(level, 0);
    ADD_FLOAT32_PROP(scale, 0);
}

void C13DescriptorAttachment::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const C13DescriptorAttachment *other = dynamic_cast<const C13DescriptorAttachment*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}


void NAMESPACE_VERSION::C13DescriptorAttachment::read( InputObjectStream *s )
{
    AttachedData::read(s);
    pcaBasis.read(s);
    eigValue.read(s);
    descriptor.read(s);
    s->read(level);
    s->read(scale);
    lowerCorner.read(s);
    upperCorner.read(s);

    mpcard listsize;
    s->readMaxPlatformCard(listsize);
    featureSupports.resize(listsize);
    for (card32 i = 0; i<featureSupports.size(); ++i) {
        mpcard entrysize;
        s->readMaxPlatformCard(entrysize);
        featureSupports[i].resize(entrysize);
        for (card32 j = 0; j < featureSupports[i].size(); ++j) {
            s->read(featureSupports[i][j]);
        }	
    }
}

void NAMESPACE_VERSION::C13DescriptorAttachment::write( OutputObjectStream *s ) const
{
    AttachedData::write(s);
    pcaBasis.write(s);
    eigValue.write(s);
    descriptor.write(s);
    s->write(level);
    s->write(scale);
    lowerCorner.write(s);
    upperCorner.write(s);

    s->writeMaxPlatformCard(featureSupports.size());
    for (card32 i = 0; i<featureSupports.size(); ++i) {
        s->writeMaxPlatformCard(featureSupports[i].size());
        for (card32 j = 0; j < featureSupports[i].size(); ++j) {
            s->write(featureSupports[i][j]);
        }	
    }
}
