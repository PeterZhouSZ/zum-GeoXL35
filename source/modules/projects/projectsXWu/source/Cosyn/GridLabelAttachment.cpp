//---------------------------------------------------------------------------
#include "StdAfx.h"
//---------------------------------------------------------------------------
#include "GridLabelAttachment.h"
#include "Util\ColorSchemer.hpp"
//---------------------------------------------------------------------------
#include "CopyObjectProperties.h"
#include "VariableArrayClassProperty.h"
//---------------------------------------------------------------------------

IMPLEMENT_CLASS( GridLabelAttachment , 0 )
{
    BEGIN_CLASS_INIT( GridLabelAttachment );
    ADD_VECTOR3F_VAR_ARRAY_PROP( cell_cen, 0 );
    ADD_CARD32_VAR_ARRAY_PROP( labels, 0 );
    ADD_FLOAT32_PROP(cell_size, 0);
}

void GridLabelAttachment::VisualizeGrid(void)
{
    debugRenderer->beginRenderJob_OneFrame("visualize_grid_", DebugRenderer::DR_FRAME++);
    const size_t num_var = cell_cen.size();
    for (size_t variable = 0; variable < num_var; ++variable) {
        const unsigned& label = labels[variable];
        const Vector3f& ccen = cell_cen[variable];

        BoundingBox3f bb;
        bb.lowerCorner = bb.upperCorner = ccen;
        bb.addBorder(cell_size / 2.0f);
        debugRenderer->addBoundingBox(bb, makeVector3f(0.2f, 0.2f, 0.2f), 1);

        Matrix3f orientation; 
        orientation.setIdentity();
        const float scale = (cell_size * 0.5f);
        const Vector3f color = (label == 0) ?
            makeVector3f(0.f, 0.f, 0.f) :
            ColorSchemer::GetColor(label-1);
        debugRenderer->addCenteredBox(ccen, orientation,
            makeVector3f(scale,scale,scale), color);		
    }
    debugRenderer->endRenderJob();
}

void GridLabelAttachment::assign( const Object* obj, COPY_CONTEXT *context ) 
{
    const GridLabelAttachment *other = dynamic_cast<const GridLabelAttachment*>(obj);
    if (other) {
        copyObjectProperties( obj, this );
    } else {
        pAssert(false);
    }
}


void GridLabelAttachment::read( InputObjectStream *s )
{
    AttachedData::read(s);
}

void GridLabelAttachment::write( OutputObjectStream *s ) const
{
    AttachedData::write(s);
}
