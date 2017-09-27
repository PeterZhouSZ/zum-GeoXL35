//----------------------------------------------------------------------
#include "StdAfx.h"
//----------------------------------------------------------------------
#include "HandleConstraints.h"
//----------------------------------------------------------------------
//----------------------------------------------------------------------

HandleConstraints::HandleConstraints(float32 const weight)
    : weight_(weight)
{}

void HandleConstraints::BuildConstraints(
    SymmSpaceSolver::Ptr receiver,
    const Handle3DList::Ptr& handleList
    )
{
    const bool debug_output = false;

    if (debug_output) debugRenderer->beginRenderJob_OneFrame("handle_constraint_", DebugRenderer::DR_FRAME++);
    for (Handle3DGizmo& handle : handleList->getHandles()) {
        const std::deque<unsigned>& points = handle.getPoints();
        const Matrix4f transformation = handle.getTransformation();
        for (const unsigned& index : points) {
            receiver->addSoftConstraint(
                index,
                transformation,
                weight_
                );
        }
    }
    if (debug_output) debugRenderer->endRenderJob();
}
