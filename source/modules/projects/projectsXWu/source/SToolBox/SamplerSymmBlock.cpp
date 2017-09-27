#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "SToolBox/SamplerSymmBlock.h"
//---------------------------------------------------------------------------
#include "PropertyTableProperty.h"
#include "SeparatorClassProperty.h"
//---------------------------------------------------------------------------

namespace {
    int DR_FRAME = 0;
}

IMPLEMENT_CLASS( PointSymmBlock, 0 ) 
{
    BEGIN_CLASS_INIT( PointSymmBlock );
    INIT_PROPERTY_TABLE();
}
IMPLEMENT_CLASS( PointSymmBlockVec, 0 ) 
{
    BEGIN_CLASS_INIT( PointSymmBlockVec );
    INIT_PROPERTY_TABLE();
    ADD_OBJECT_LIST_PROP( blockvec_, 0, PointSymmBlock::getClass() );
}

void PointSymmBlock::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const PointSymmBlock *other = dynamic_cast<const PointSymmBlock*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void PointSymmBlock::read( InputObjectStream *s )
{
    AttachedData::read(s);
    s->read(elem_);

    mpcard listsize;
    s->readMaxPlatformCard(listsize);
    pindx_.resize(listsize);
    for (card32 i = 0; i < pindx_.size(); ++i) {
        s->read(pindx_[i]);
    }

    s->readMaxPlatformCard(listsize);
    knots_.resize(listsize);
    for (card32 i = 0; i < knots_.size(); ++i) {
        s->read(knots_[i].symm_);
        s->read(knots_[i].ordx_);
    }
}

void PointSymmBlock::write( OutputObjectStream *s ) const
{
    AttachedData::write(s);
    s->write(elem_);

    s->writeMaxPlatformCard(pindx_.size());
    for (card32 i = 0; i < pindx_.size(); ++i) {
        s->write(pindx_[i]);
    }

    s->writeMaxPlatformCard(knots_.size());
    for (card32 i = 0; i < knots_.size(); ++i) {
        s->write(knots_[i].symm_);
        s->write(knots_[i].ordx_);
    }
}

//--------------------------------------------------------------------------------------

IMPLEMENT_CLASS( LineSymmBlock, 0 ) 
{
    BEGIN_CLASS_INIT( LineSymmBlock );
    INIT_PROPERTY_TABLE();
}
IMPLEMENT_CLASS( LineSymmBlockVec, 0 ) 
{
    BEGIN_CLASS_INIT( LineSymmBlockVec );
    INIT_PROPERTY_TABLE();
    ADD_OBJECT_LIST_PROP( blockvec_, 0, LineSymmBlock::getClass() );
}

void LineSymmBlock::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const LineSymmBlock *other = dynamic_cast<const LineSymmBlock*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}

void LineSymmBlock::read( InputObjectStream *s )
{
    AttachedData::read(s);
    s->read(elem_);

    mpcard listsize;
    s->readMaxPlatformCard(listsize);
    pindx_.resize(listsize);
    for (card32 i = 0; i < pindx_.size(); ++i) {
        s->read(pindx_[i]);
    }

    s->readMaxPlatformCard(listsize);
    knots_.resize(listsize);
    for (card32 i = 0; i < knots_.size(); ++i) {
        std::deque<card32>& vec = knots_[i];
        s->readMaxPlatformCard(listsize);
        vec.resize(listsize);
        for (card32 j = 0; j < vec.size(); ++j) {
            s->read(vec[j]);
        }
    }
}

void LineSymmBlock::write( OutputObjectStream *s ) const
{
    AttachedData::write(s);
    s->write(elem_);

    s->writeMaxPlatformCard(pindx_.size());
    for (card32 i = 0; i < pindx_.size(); ++i) {
        s->write(pindx_[i]);
    }

    s->writeMaxPlatformCard(knots_.size());
    for (card32 i = 0; i < knots_.size(); ++i) {
        const std::deque<card32>& vec = knots_[i];
        s->writeMaxPlatformCard(vec.size());
        for (card32 j = 0; j < vec.size(); ++j) {
            s->write(vec[j]);
        }
    }
}
